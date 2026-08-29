#!/usr/bin/env python3
import json, math, os, tempfile, time
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None
try:
    from unitree_api.msg import Request as UnitreeRequest
except ImportError:
    UnitreeRequest = None

RUNTIME=Path(os.environ.get('PATROL_RUNTIME','/project/runtime'))
DATA=Path(os.environ.get('PATROL_DATA','/project/patrol_data'))

def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=Path(str(path)+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')
    os.replace(temporary,path)

def rpy_from_q(q):
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    sinp=max(-1.0,min(1.0,2*(q.w*q.y-q.z*q.x)))
    pitch=math.asin(sinp)
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,pitch,yaw

def rotation_from_q(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]],dtype=np.float64)

def rpy_from_rotation(r):
    pitch=math.asin(max(-1.0,min(1.0,-float(r[2,0]))))
    return math.atan2(float(r[2,1]),float(r[2,2])),pitch,math.atan2(float(r[1,0]),float(r[0,0]))

class Bridge(Node):
    def __init__(self):
        super().__init__('patrol_bridge')
        self.declare_parameter('odom_topic',os.environ.get('ODOM_TOPIC','/odom'))
        self.declare_parameter('map_topic','/map')
        self.pose={'x':0.0,'y':0.0,'z':0.0,'roll':0.0,'pitch':0.0,'yaw':0.0}; self.odom_seen=False; self.map_seen=False; self.cloud_map_seen=False; self.last_goal=None; self.nav_status='idle'; self.goal_handle=None
        # Do not execute a goal file left by an earlier mapping/offline session.
        try:self.last_goal=json.loads((RUNTIME/'goal.json').read_text(encoding='utf-8')).get('id')
        except (OSError,ValueError):pass
        self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.on_odom,10)
        self.odom_pub=self.create_publisher(Odometry,'/go2/odom',10)
        sensor_qos=rclpy.qos.qos_profile_sensor_data
        self.cloud_pub=self.create_publisher(PointCloud2,'/go2/cloud_base',sensor_qos)
        self.create_subscription(PointCloud2,'/utlidar/cloud_base',self.on_cloud,sensor_qos)
        # The GO2 L1 publishes a changing, sparse partial scan (roughly 400-500
        # points per message). Accumulate a short window in odom coordinates,
        # then compensate it back into the current base frame. This gives the
        # 2-D converter one coherent front-FOV scan instead of many fragments.
        self.cloud_window_seconds=float(os.environ.get('GO2_CLOUD_WINDOW','0.8'))
        self.cloud_batches=deque(); self.latest_odom_pose=None
        self.tf_broadcaster=TransformBroadcaster(self)
        self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.on_map,1)
        self.cloud_map_topic=os.environ.get('CLOUD_MAP_TOPIC','/Laser_map')
        self.cloud_map_max_points=max(2000,int(os.environ.get('CLOUD_MAP_MAX_POINTS','24000')))
        self.cloud_map_last_write=0.0;self.latest_cloud_map=None;self.last_cloud_save_request=None
        # A save request is one-shot. Ignore a request left on disk by an
        # earlier bridge session so a restart can never overwrite a saved map.
        try:self.last_cloud_save_request=json.loads((RUNTIME/'cloud_save_request.json').read_text(encoding='utf-8')).get('id')
        except (OSError,ValueError):pass
        level_roll=float(os.environ.get('CLOUD_LEVEL_ROLL','-0.030788'))
        level_pitch=float(os.environ.get('CLOUD_LEVEL_PITCH','0.621767'))
        cr,sr=math.cos(level_roll),math.sin(level_roll);cp,sp=math.cos(level_pitch),math.sin(level_pitch)
        self.cloud_level_rotation=np.array([[cp,0.0,sp],[sr*sp,cr,-sr*cp],[-cr*sp,sr,cr*cp]],dtype=np.float32)
        self.create_subscription(PointCloud2,self.cloud_map_topic,self.on_cloud_map,sensor_qos)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.sport_pub=self.create_publisher(UnitreeRequest,'/api/sport/request',10) if UnitreeRequest else None
        self.teleop_was_moving=False; self.teleop_last_publish=0.0; self.sport_publish_count=0; self.sport_last_api_id=None
        self.nav_motion_enabled=os.environ.get('ENABLE_NAV_MOTION','0')=='1'
        self.nav_was_moving=False;self.nav_last_received=0.0;self.nav_last_command=None
        if self.nav_motion_enabled:self.create_subscription(Twist,'/cmd_vel',self.on_cmd_vel,10)
        self.nav_client=ActionClient(self,NavigateToPose,'navigate_to_pose') if NavigateToPose else None
        self.create_timer(.20,self.publish_accumulated_cloud)
        self.create_timer(.25,self.tick); self.create_timer(.25,self.cloud_save_tick); self.create_timer(.05,self.teleop_tick); self.create_timer(.05,self.nav_watchdog); self.create_timer(.5,self.write_state)
        self.get_logger().info('Patrol bridge ready, runtime='+str(RUNTIME))
    def on_odom(self,msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        raw_roll,raw_pitch,raw_yaw=rpy_from_q(q)
        level_position=self.cloud_level_rotation@np.array([p.x,p.y,p.z],dtype=np.float32)
        roll,pitch,yaw=rpy_from_rotation(self.cloud_level_rotation@rotation_from_q(q))
        self.pose={'x':float(level_position[0]),'y':float(level_position[1]),'z':float(level_position[2]),'roll':roll,'pitch':pitch,'yaw':yaw}
        self.latest_odom_pose=(p.x,p.y,p.z,raw_yaw)
        self.odom_seen=True
        now=self.get_clock().now().to_msg()
        msg.header.stamp=now
        self.odom_pub.publish(msg)
        # GO2 publishes nav_msgs/Odometry but does not expose /tf to the external DDS
        # participant. Re-publish the same pose as odom -> base_link for SLAM/Nav2.
        transform=TransformStamped()
        transform.header=msg.header
        transform.header.frame_id=msg.header.frame_id or 'odom'
        transform.child_frame_id=msg.child_frame_id or 'base_link'
        transform.transform.translation.x=p.x
        transform.transform.translation.y=p.y
        transform.transform.translation.z=p.z
        transform.transform.rotation=q
        self.tf_broadcaster.sendTransform(transform)
    def on_cloud(self,msg):
        if self.latest_odom_pose is None:return
        points=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)))
        if points.dtype.names:
            points=np.column_stack((points['x'],points['y'],points['z'])).astype(np.float32)
        else:points=np.asarray(points,dtype=np.float32).reshape(-1,3)
        if not len(points):return
        x,y,z,yaw=self.latest_odom_pose;c=math.cos(yaw);s=math.sin(yaw)
        odom_points=np.empty_like(points)
        odom_points[:,0]=x+c*points[:,0]-s*points[:,1]
        odom_points[:,1]=y+s*points[:,0]+c*points[:,1]
        odom_points[:,2]=z+points[:,2]
        now=time.monotonic();self.cloud_batches.append((now,odom_points))
        while self.cloud_batches and now-self.cloud_batches[0][0]>self.cloud_window_seconds:self.cloud_batches.popleft()
    def publish_accumulated_cloud(self):
        if self.latest_odom_pose is None or not self.cloud_batches:return
        now=time.monotonic()
        while self.cloud_batches and now-self.cloud_batches[0][0]>self.cloud_window_seconds:self.cloud_batches.popleft()
        if not self.cloud_batches:return
        points=np.vstack([batch for _,batch in self.cloud_batches])
        x,y,z,yaw=self.latest_odom_pose;c=math.cos(yaw);s=math.sin(yaw)
        dx=points[:,0]-x;dy=points[:,1]-y
        base_points=np.empty_like(points)
        base_points[:,0]=c*dx+s*dy
        base_points[:,1]=-s*dx+c*dy
        base_points[:,2]=points[:,2]-z
        header=Header();header.frame_id='base_link';header.stamp=self.get_clock().now().to_msg()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,base_points.tolist()))
    def on_map(self,msg):
        width,height=msg.info.width,msg.info.height
        stride=max(1,math.ceil(max(width/400,height/400)))
        cells=[]
        for y in range(0,height,stride):
            row=y*width
            for x in range(0,width,stride): cells.append(int(msg.data[row+x]))
        atomic_json(RUNTIME/'map.json',{'available':True,'width':math.ceil(width/stride),'height':math.ceil(height/stride),'resolution':msg.info.resolution*stride,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],'cells':cells,'updated_at':time.time()})
        self.map_seen=True
    def cloud_xyz(self,msg):
        fields={field.name:field.offset for field in msg.fields}
        if not all(name in fields for name in ('x','y','z')) or msg.point_step<=0:return np.empty((0,3),dtype=np.float32)
        endian='>' if msg.is_bigendian else '<'
        # Some FAST-LIO Laser_map messages briefly report a width larger than
        # the populated byte buffer while the map is being rebuilt. Bound the
        # view by the real buffer so one partial frame cannot kill the bridge.
        last_offset=max(fields[name] for name in ('x','y','z'))+4
        available=max(0,(len(msg.data)-last_offset)//msg.point_step+1)
        count=min(msg.width*msg.height,available)
        if count<=0:return np.empty((0,3),dtype=np.float32)
        axes=[np.ndarray((count,),dtype=endian+'f4',buffer=msg.data,offset=fields[name],strides=(msg.point_step,)) for name in ('x','y','z')]
        points=np.column_stack(axes).astype(np.float32,copy=False)
        return points[np.isfinite(points).all(axis=1)]
    def on_cloud_map(self,msg):
        now=time.monotonic()
        # Laser_map can contain hundreds of thousands of points. Keep the full
        # float32 map for PCD saving, but throttle/decimate the browser snapshot.
        try:points=self.cloud_xyz(msg)
        except (ValueError,BufferError) as exc:
            self.get_logger().warning('Discard malformed Laser_map frame: '+str(exc));return
        if not len(points):return
        points=points@self.cloud_level_rotation.T
        self.latest_cloud_map=points
        self.cloud_map_seen=True
        if now-self.cloud_map_last_write<0.8:return
        self.cloud_map_last_write=now
        stride=max(1,math.ceil(len(points)/self.cloud_map_max_points))
        shown=np.round(points[::stride][:self.cloud_map_max_points],3)
        low=shown.min(axis=0);high=shown.max(axis=0)
        atomic_json(RUNTIME/'cloud.json',{'available':True,'frame':'map_level','source':self.cloud_map_topic,
            'total_points':int(len(points)),'shown_points':int(len(shown)),'bounds':[low.tolist(),high.tolist()],
            'points':shown.reshape(-1).tolist(),'updated_at':time.time()})
    def cloud_save_tick(self):
        request=read_json_file(RUNTIME/'cloud_save_request.json')
        if not request or request.get('id')==self.last_cloud_save_request:return
        self.last_cloud_save_request=request.get('id')
        response={'id':self.last_cloud_save_request,'ok':False,'updated_at':time.time()}
        try:
            if self.latest_cloud_map is None or not len(self.latest_cloud_map):raise RuntimeError('尚未收到MID360三维地图')
            name=''.join(c for c in str(request.get('name','')) if c.isalnum() or c in '_-' or '\u4e00'<=c<='\u9fff')[:40]
            if not name:name=time.strftime('map_%Y%m%d_%H%M%S')
            target=DATA/'maps'/(name+'.pcd');target.parent.mkdir(parents=True,exist_ok=True)
            points=np.asarray(self.latest_cloud_map,dtype='<f4')
            header=(f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
                    f'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n'
                    f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode('ascii')
            temporary=Path(str(target)+'.tmp')
            with open(temporary,'wb') as handle:handle.write(header);handle.write(points.tobytes(order='C'))
            os.replace(temporary,target)
            response.update(ok=True,name=name,pcd=str(target),points=int(len(points)),bytes=target.stat().st_size)
        except Exception as exc:response['error']=str(exc)
        atomic_json(RUNTIME/'cloud_save_response.json',response)
    def tick(self):
        try: goal=json.loads((RUNTIME/'goal.json').read_text(encoding='utf-8'))
        except (OSError,ValueError): return
        if goal.get('id')==self.last_goal:return
        self.last_goal=goal.get('id'); self.send_goal(goal)
    def sport_request(self,api_id,parameter=''):
        if not self.sport_pub:return
        request=UnitreeRequest();request.header.identity.api_id=api_id;request.parameter=parameter
        self.sport_pub.publish(request)
        self.sport_publish_count+=1;self.sport_last_api_id=api_id
    def stop_motion(self):
        self.sport_request(1003)
        self.teleop_was_moving=False;self.nav_was_moving=False
    def on_cmd_vel(self,msg):
        if not self.nav_motion_enabled:return
        vx=max(-0.10,min(0.10,float(msg.linear.x)))
        vy=max(-0.08,min(0.08,float(msg.linear.y)))
        vyaw=max(-0.25,min(0.25,float(msg.angular.z)))
        def effective(value,minimum):
            if abs(value)<=0.005:return 0.0
            return math.copysign(max(abs(value),minimum),value)
        vx=effective(vx,0.05);vy=effective(vy,0.05);vyaw=effective(vyaw,0.10)
        self.nav_last_command={'vx':vx,'vy':vy,'vyaw':vyaw}
        self.nav_last_received=time.monotonic()
        moving=any(abs(value)>0.005 for value in (vx,vy,vyaw))
        if moving:
            self.sport_request(1008,json.dumps({'x':vx,'y':vy,'z':vyaw},separators=(',',':')))
            self.nav_was_moving=True
        elif self.nav_was_moving:self.stop_motion()
    def nav_watchdog(self):
        if self.nav_was_moving and time.monotonic()-self.nav_last_received>0.25:self.stop_motion()
    def teleop_tick(self):
        if not self.sport_pub:return
        try:command=json.loads((RUNTIME/'teleop.json').read_text(encoding='utf-8'))
        except (OSError,ValueError):
            if self.teleop_was_moving:self.stop_motion()
            return
        age=time.time()-float(command.get('updated_at',0))
        vx=max(-0.30,min(0.30,float(command.get('vx',0))))
        vy=max(-0.30,min(0.30,float(command.get('vy',0))))
        vyaw=max(-0.50,min(0.50,float(command.get('vyaw',0))))
        moving=age<=0.35 and any(abs(value)>0.001 for value in (vx,vy,vyaw))
        now=time.monotonic()
        if moving and now-self.teleop_last_publish>=0.09:
            self.sport_request(1008,json.dumps({'x':vx,'y':vy,'z':vyaw},separators=(',',':')))
            self.teleop_last_publish=now;self.teleop_was_moving=True
        elif not moving and self.teleop_was_moving:self.stop_motion()
    def pose_msg(self,goal):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(goal['x']);p.pose.position.y=float(goal['y']);p.pose.position.z=float(goal.get('z',0));yaw=float(goal.get('yaw',0));p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
    def send_goal(self,goal):
        pose=self.pose_msg(goal)
        if not self.nav_client or not self.nav_client.wait_for_server(timeout_sec=.5):
            self.goal_pub.publish(pose);self.nav_status='published';self.get_logger().warning('Nav2 action unavailable; published /goal_pose')
            return
        request=NavigateToPose.Goal();request.pose=pose;self.nav_status='sending'
        future=self.nav_client.send_goal_async(request);future.add_done_callback(self.goal_response)
    def goal_response(self,future):
        self.goal_handle=future.result()
        if not self.goal_handle.accepted:self.nav_status='failed';return
        self.nav_status='navigating';result=self.goal_handle.get_result_async();result.add_done_callback(self.goal_result)
    def goal_result(self,future):
        status=future.result().status
        self.nav_status='succeeded' if status==4 else ('canceled' if status==5 else 'failed')
    def write_state(self):
        topics=[name for name,_ in self.get_topic_names_and_types()]
        atomic_json(RUNTIME/'robot_state.json',{'updated_at':time.time(),'pose':self.pose,'odom_online':self.odom_seen,'nav_status':self.nav_status,'map_online':self.map_seen,'cloud_map_online':self.cloud_map_seen,'topics':topics,'teleop_bridge':{'available':self.sport_pub is not None,'moving':self.teleop_was_moving,'publish_count':self.sport_publish_count,'last_api_id':self.sport_last_api_id},'nav_motion_bridge':{'enabled':self.nav_motion_enabled,'moving':self.nav_was_moving,'last_cmd_age':time.monotonic()-self.nav_last_received if self.nav_last_received else None,'last_command':self.nav_last_command}})

def read_json_file(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return None

def main():
    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.stop_motion();node.nav_status='stopped';node.write_state();node.destroy_node();rclpy.shutdown()
