#!/usr/bin/env python3
import json, math, os, tempfile, time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
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

def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=Path(str(path)+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')
    os.replace(temporary,path)

def yaw_from_q(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Bridge(Node):
    def __init__(self):
        super().__init__('patrol_bridge')
        self.declare_parameter('odom_topic',os.environ.get('ODOM_TOPIC','/odom'))
        self.declare_parameter('map_topic','/map')
        self.pose={'x':0.0,'y':0.0,'yaw':0.0}; self.odom_seen=False; self.map_seen=False; self.last_goal=None; self.nav_status='idle'; self.goal_handle=None
        self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.on_odom,10)
        self.odom_pub=self.create_publisher(Odometry,'/go2/odom',10)
        self.cloud_pub=self.create_publisher(PointCloud2,'/go2/cloud_base',10)
        self.create_subscription(PointCloud2,'/utlidar/cloud_base',self.on_cloud,10)
        self.tf_broadcaster=TransformBroadcaster(self)
        self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.on_map,1)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.sport_pub=self.create_publisher(UnitreeRequest,'/api/sport/request',10) if UnitreeRequest else None
        self.teleop_was_moving=False; self.teleop_last_publish=0.0; self.sport_publish_count=0; self.sport_last_api_id=None
        self.nav_motion_enabled=os.environ.get('ENABLE_NAV_MOTION','0')=='1'
        self.nav_was_moving=False;self.nav_last_received=0.0
        if self.nav_motion_enabled:self.create_subscription(Twist,'/cmd_vel',self.on_cmd_vel,10)
        self.nav_client=ActionClient(self,NavigateToPose,'navigate_to_pose') if NavigateToPose else None
        self.create_timer(.25,self.tick); self.create_timer(.05,self.teleop_tick); self.create_timer(.05,self.nav_watchdog); self.create_timer(.5,self.write_state)
        self.get_logger().info('Patrol bridge ready, runtime='+str(RUNTIME))
    def on_odom(self,msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        self.pose={'x':p.x,'y':p.y,'yaw':yaw_from_q(q)}
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
        msg.header.stamp=self.get_clock().now().to_msg()
        self.cloud_pub.publish(msg)
    def on_map(self,msg):
        width,height=msg.info.width,msg.info.height
        stride=max(1,math.ceil(max(width/400,height/400)))
        cells=[]
        for y in range(0,height,stride):
            row=y*width
            for x in range(0,width,stride): cells.append(int(msg.data[row+x]))
        atomic_json(RUNTIME/'map.json',{'available':True,'width':math.ceil(width/stride),'height':math.ceil(height/stride),'resolution':msg.info.resolution*stride,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],'cells':cells,'updated_at':time.time()})
        self.map_seen=True
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
        vx=max(-0.20,min(0.20,float(msg.linear.x)))
        vy=max(-0.15,min(0.15,float(msg.linear.y)))
        vyaw=max(-0.40,min(0.40,float(msg.angular.z)))
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
        p=PoseStamped();p.header.frame_id='map_level';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(goal['x']);p.pose.position.y=float(goal['y']);yaw=float(goal.get('yaw',0));p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
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
        atomic_json(RUNTIME/'robot_state.json',{'updated_at':time.time(),'pose':self.pose,'odom_online':self.odom_seen,'nav_status':self.nav_status,'map_online':self.map_seen,'topics':topics,'teleop_bridge':{'available':self.sport_pub is not None,'moving':self.teleop_was_moving,'publish_count':self.sport_publish_count,'last_api_id':self.sport_last_api_id},'nav_motion_bridge':{'enabled':self.nav_motion_enabled,'moving':self.nav_was_moving,'last_cmd_age':time.monotonic()-self.nav_last_received if self.nav_last_received else None}})

def main():
    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.stop_motion();node.destroy_node();rclpy.shutdown()
