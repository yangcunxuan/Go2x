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
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
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
        self.pose={'x':0.0,'y':0.0,'z':0.0,'roll':0.0,'pitch':0.0,'yaw':0.0}; self.odom_seen=False; self.map_seen=False; self.cloud_map_seen=False; self.odom_last_seen=0.0; self.cloud_map_last_seen=0.0; self.last_goal=None; self.nav_status='idle'; self.goal_handle=None
        self.localization_sane=True;self.localization_error=None;self.last_level_sample=None;self.sane_recovery_since=None;self.sane_recoverable=True
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
        # Plan A: in localization mode the localization_manager publishes the
        # dynamic map_level <- camera_init TF; in mapping mode the static
        # leveling TF covers it. The web pose/cloud follow the live TF with
        # the env-based level transform as last resort.
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.on_map,1)
        self.cloud_map_topic=os.environ.get('CLOUD_MAP_TOPIC','/Laser_map')
        self.cloud_map_max_points=max(2000,int(os.environ.get('CLOUD_MAP_MAX_POINTS','24000')))
        self.cloud_map_last_write=0.0;self.latest_cloud_map=None;self.last_cloud_save_request=None
        # Navigation mode runs FAST-LIO with Laser_map publishing disabled
        # (accumulated-map serialisation costs seconds of CPU per publish and
        # starves the stack on small machines). The browser still needs a live
        # cloud to pick patrol points on, so keep a rolling window of the cheap
        # per-scan /cloud_registered frames as the cloud.json source fallback.
        self.scan_window_seconds=float(os.environ.get('GO2_SCAN_WINDOW','10.0'))
        self.scan_window_max_points=int(os.environ.get('GO2_SCAN_WINDOW_MAX_POINTS','600000'))
        self.scan_window=deque(); self.scan_window_last_write=0.0
        self.create_subscription(PointCloud2,os.environ.get('CLOUD_SCAN_TOPIC','/cloud_registered'),self.on_cloud_registered,sensor_qos)
        # A save request is one-shot. Ignore a request left on disk by an
        # earlier bridge session so a restart can never overwrite a saved map.
        try:self.last_cloud_save_request=json.loads((RUNTIME/'cloud_save_request.json').read_text(encoding='utf-8')).get('id')
        except (OSError,ValueError):pass
        level_roll=float(os.environ.get('CLOUD_LEVEL_ROLL','-0.030788'))
        level_pitch=float(os.environ.get('CLOUD_LEVEL_PITCH','0.621767'))
        level_yaw=float(os.environ.get('MAP_LEVEL_YAW','0'))
        self.robot_yaw_offset=float(os.environ.get('ROBOT_YAW_OFFSET','0'))
        cr,sr=math.cos(level_roll),math.sin(level_roll);cp,sp=math.cos(level_pitch),math.sin(level_pitch)
        base_level_rotation=np.array([[cp,0.0,sp],[sr*sp,cr,-sr*cp],[-cr*sp,sr,cr*cp]],dtype=np.float32)
        cy,sy=math.cos(level_yaw),math.sin(level_yaw)
        yaw_rotation=np.array([[cy,-sy,0.0],[sy,cy,0.0],[0.0,0.0,1.0]],dtype=np.float32)
        self.cloud_level_rotation=yaw_rotation@base_level_rotation
        self.cloud_level_translation=np.array([
            float(os.environ.get('MAP_LEVEL_X','0')),
            float(os.environ.get('MAP_LEVEL_Y','0')),
            float(os.environ.get('MAP_LEVEL_Z','0'))],dtype=np.float32)
        # Laser_map is a multi-megabyte accumulated cloud. BEST_EFFORT starts
        # dropping every frame once UDP fragmentation grows, even though the
        # 10 Hz raw LiDAR stream remains healthy. Reliable + depth 1 preserves
        # only the newest complete map without building a callback backlog.
        map_qos=rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2,self.cloud_map_topic,self.on_cloud_map,map_qos)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.sport_pub=self.create_publisher(UnitreeRequest,'/api/sport/request',10) if UnitreeRequest else None
        self.teleop_was_moving=False; self.teleop_last_publish=0.0; self.sport_publish_count=0; self.sport_last_api_id=None
        self.nav_motion_enabled=os.environ.get('ENABLE_NAV_MOTION','0')=='1'
        self.nav_was_moving=False;self.nav_last_received=0.0;self.nav_last_command=None
        if self.nav_motion_enabled:self.create_subscription(Twist,'/cmd_vel',self.on_cmd_vel,10)
        self.nav_client=ActionClient(self,NavigateToPose,'navigate_to_pose') if NavigateToPose else None
        self.create_timer(.20,self.publish_accumulated_cloud)
        self.recovery_last_write=0.0;self.recovery_last_cloud_time=0.0
        self.create_timer(.25,self.tick); self.create_timer(.25,self.cloud_save_tick); self.create_timer(5.0,self.recovery_tick); self.create_timer(.05,self.teleop_tick); self.create_timer(.05,self.nav_watchdog); self.create_timer(.5,self.write_state)
        self.get_logger().info('Patrol bridge ready, runtime='+str(RUNTIME))
    def map_level_transform(self):
        """(4x4 matrix, source) for map_level <- camera_init. source is 'tf'
        (live, from the localization manager or the mapping static TF) or
        'env' (the fixed level transform, last resort). The caller resets its
        jump-guard baseline when the source changes — a global relocalization
        jump is legitimate, not odometry divergence (P0 audit #3)."""
        try:
            tf=self.tf_buffer.lookup_transform('map_level','camera_init',Time())
            tr=tf.transform
            matrix=np.eye(4,dtype=np.float64)
            matrix[:3,:3]=rotation_from_q(tr.rotation)
            matrix[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
            return matrix,'tf'
        except Exception:
            matrix=np.eye(4,dtype=np.float64)
            matrix[:3,:3]=self.cloud_level_rotation
            matrix[:3,3]=self.cloud_level_translation
            return matrix,'env'
        

    def on_odom(self,msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        raw_roll,raw_pitch,raw_yaw=rpy_from_q(q)
        matrix,tf_source=self.map_level_transform()
        if tf_source!=getattr(self,'_tf_source',''):
            # First fix or a switch between env/static and live TF: the
            # absolute pose legitimately jumps by the size of the global
            # correction. Reset the guard baseline instead of latching.
            self._tf_source=tf_source
            self.last_level_sample=None
        level_position=(matrix@np.array([p.x,p.y,p.z,1.0],dtype=np.float64))[:3]
        roll,pitch,yaw=rpy_from_rotation(matrix[:3,:3]@rotation_from_q(q))
        display_yaw=math.atan2(math.sin(yaw+self.robot_yaw_offset),math.cos(yaw+self.robot_yaw_offset))
        sample_time=time.time()
        if self.last_level_sample is not None:
            previous,previous_time=self.last_level_sample;dt=sample_time-previous_time
            planar=float(np.linalg.norm(level_position[:2]-previous[:2]));vertical=abs(float(level_position[2]-previous[2]))
            unstable=0.02<=dt<=1.0 and ((planar>.12 and planar/dt>1.0) or (vertical>.12 and vertical/dt>.8))
            if unstable and self.localization_sane:
                self.localization_sane=False
                # A planar jump larger than 1 m is treated as real divergence
                # (kidnapped robot / drift), not posture: latched until the
                # bridge restarts. Posture-sized jumps recover automatically.
                self.sane_recoverable=planar<=1.0
                self.localization_error=f'FAST-LIO位姿突变: dt={dt:.3f}s planar={planar:.3f}m vertical={vertical:.3f}m'
                self.get_logger().error(self.localization_error)
            # Body posture changes (stand up / lie down) legitimately move the
            # lidar tens of centimetres and can trip the jump guard; a guard
            # that latches forever turns that into a permanent navigation
            # lockout. Recover once the pose stream is stable again for 3 s.
            if not self.localization_sane:
                if not self.sane_recoverable or unstable or planar/dt>0.15 or vertical/dt>0.1:
                    self.sane_recovery_since=None
                else:
                    if self.sane_recovery_since is None:self.sane_recovery_since=sample_time
                    elif sample_time-self.sane_recovery_since>=3.0:
                        self.localization_sane=True;self.localization_error=None;self.sane_recovery_since=None
                        self.get_logger().info('FAST-LIO位姿已连续3秒稳定，定位保护自动恢复')
        self.last_level_sample=(level_position.copy(),sample_time)
        self.pose={'x':float(level_position[0]),'y':float(level_position[1]),'z':float(level_position[2]),'roll':roll,'pitch':pitch,'yaw':display_yaw}
        self.latest_odom_pose=(p.x,p.y,p.z,raw_yaw)
        self.odom_seen=True
        self.odom_last_seen=time.time()
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
        self.publish_scan_window_cloud()
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
    def on_cloud_registered(self,msg):
        # Per-scan registered cloud in camera_init; apply the same fixed
        # level transform as Laser_map. Decimate each frame so the window
        # stays bounded even in feature-dense scenes.
        try:points=self.cloud_xyz(msg)
        except (ValueError,BufferError):return
        if not len(points):return
        if len(points)>4000:points=points[::math.ceil(len(points)/4000)]
        matrix=self.map_level_transform()
        points=np.dot(points, matrix[:3,:3].T) + matrix[:3,3]
        now=time.monotonic()
        self.scan_window.append((now,points))
        while self.scan_window and now-self.scan_window[0][0]>self.scan_window_seconds:
            self.scan_window.popleft()
        while len(self.scan_window)>1:
            total=sum(len(p) for _,p in self.scan_window)
            if total<=self.scan_window_max_points:break
            self.scan_window.popleft()
    def publish_scan_window_cloud(self):
        # Laser_map owns cloud.json while it is fresh (mapping mode); when it
        # is disabled (navigation mode) the rolling scan window takes over so
        # the web view stays live for point picking without the map cost.
        if time.time()-self.cloud_map_last_seen<2.0 or not self.scan_window:return
        now=time.monotonic()
        if now-self.scan_window_last_write<1.0:return
        self.scan_window_last_write=now
        points=np.vstack([batch for _,batch in self.scan_window])
        span=points.max(axis=0)-points.min(axis=0)
        if span[0]>float(os.environ.get('CLOUD_MAX_XY_SPAN','250')) or span[1]>float(os.environ.get('CLOUD_MAX_XY_SPAN','250')) or span[2]>float(os.environ.get('CLOUD_MAX_Z_SPAN','40')):
            self.get_logger().warning('Discard divergent scan window: span='+str(span.tolist()));return
        stride=max(1,math.ceil(len(points)/self.cloud_map_max_points))
        shown=np.round(points[::stride][:self.cloud_map_max_points],3)
        low=shown.min(axis=0);high=shown.max(axis=0)
        atomic_json(RUNTIME/'cloud.json',{'available':True,'frame':'map_level','source':'scan_window',
            'total_points':int(len(points)),'shown_points':int(len(shown)),'bounds':[low.tolist(),high.tolist()],
            'points':shown.reshape(-1).tolist(),'updated_at':time.time()})
    def on_cloud_map(self,msg):
        now=time.monotonic()
        # Laser_map can contain hundreds of thousands of points. Keep the full
        # float32 map for PCD saving, but throttle/decimate the browser snapshot.
        try:points=self.cloud_xyz(msg)
        except (ValueError,BufferError) as exc:
            self.get_logger().warning('Discard malformed Laser_map frame: '+str(exc));return
        if not len(points):return
        points=points@self.cloud_level_rotation.T+self.cloud_level_translation
        low=points.min(axis=0);high=points.max(axis=0);span=high-low
        max_xy_span=float(os.environ.get('CLOUD_MAX_XY_SPAN','250'))
        max_z_span=float(os.environ.get('CLOUD_MAX_Z_SPAN','40'))
        max_abs=float(os.environ.get('CLOUD_MAX_ABS_COORD','500'))
        if span[0]>max_xy_span or span[1]>max_xy_span or span[2]>max_z_span or np.max(np.abs(points))>max_abs:
            self.get_logger().error(
                f'Reject divergent Laser_map: points={len(points)} span={span.tolist()} max_abs={float(np.max(np.abs(points))):.2f}')
            return
        self.latest_cloud_map=points
        self.cloud_map_seen=True
        self.cloud_map_last_seen=time.time()
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
            self.write_pcd(target,points)
            # NumPy twin for the localization stack: exact float32 dump of the
            # same map_level cloud, loaded with np.load(..., mmap_mode="r").
            npy=target.with_suffix('.npy')
            # np.save appends ".npy" to any filename that does not end with
            # it — pass an open file handle so the .tmp name is respected.
            with open(str(npy)+'.tmp','wb') as handle:
                np.save(handle,np.asarray(points,dtype='<f4'))
            os.replace(str(npy)+'.tmp',npy)
            response.update(ok=True,name=name,pcd=str(target),npy=str(npy),points=int(len(points)),bytes=target.stat().st_size)
        except Exception as exc:response['error']=str(exc)
        atomic_json(RUNTIME/'cloud_save_response.json',response)
    def write_pcd(self,target,points):
        header=(f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
                f'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n'
                f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode('ascii')
        temporary=Path(str(target)+'.tmp')
        with open(temporary,'wb') as handle:handle.write(header);handle.write(np.asarray(points,dtype='<f4').tobytes(order='C'))
        os.replace(temporary,target)
    def recovery_tick(self):
        if self.latest_cloud_map is None or not len(self.latest_cloud_map):return
        if self.cloud_map_last_seen<=self.recovery_last_cloud_time:return
        try:
            target=RUNTIME/'mapping_recovery_latest.pcd'
            self.write_pcd(target,self.latest_cloud_map)
            self.recovery_last_cloud_time=self.cloud_map_last_seen;self.recovery_last_write=time.time()
            atomic_json(RUNTIME/'mapping_recovery.json',{'available':True,'pcd':str(target),'points':int(len(self.latest_cloud_map)),
                'cloud_updated_at':self.cloud_map_last_seen,'saved_at':self.recovery_last_write})
        except Exception as exc:self.get_logger().warning('Recovery PCD write failed: '+str(exc))
    def tick(self):
        # Legacy Nav2 goal path: with the CMU planner stack goal.json is
        # consumed by goal_relay (planner container) instead. Publishing
        # /goal_pose here feeds pathFollower a map_level pose it misreads as
        # camera_init and latches a bogus goal, freezing navigation. Enable
        # only when the old Nav2 stack is actually running.
        if os.environ.get('ENABLE_LEGACY_NAV2_GOAL','0')!='1':return
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
        p=PoseStamped();p.header.frame_id='map_level';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(goal['x']);p.pose.position.y=float(goal['y']);p.pose.position.z=float(goal.get('z',0));yaw=float(goal.get('yaw',0));p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
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
        now=time.time()
        atomic_json(RUNTIME/'robot_state.json',{'updated_at':now,'pose':self.pose,'odom_online':bool(self.odom_seen and now-self.odom_last_seen<2.0),'odom_age':now-self.odom_last_seen if self.odom_last_seen else None,'nav_status':self.nav_status,'map_online':self.map_seen,'cloud_map_online':bool(self.cloud_map_seen and now-self.cloud_map_last_seen<3.0),'cloud_map_age':now-self.cloud_map_last_seen if self.cloud_map_last_seen else None,'localization_sane':self.localization_sane,'localization_error':self.localization_error,'topics':topics,'teleop_bridge':{'available':self.sport_pub is not None,'moving':self.teleop_was_moving,'publish_count':self.sport_publish_count,'last_api_id':self.sport_last_api_id},'nav_motion_bridge':{'enabled':self.nav_motion_enabled,'moving':self.nav_was_moving,'last_cmd_age':time.monotonic()-self.nav_last_received if self.nav_last_received else None,'last_command':self.nav_last_command}})

def read_json_file(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return None

def main():
    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.stop_motion();node.nav_status='stopped';node.write_state();node.destroy_node();rclpy.shutdown()
