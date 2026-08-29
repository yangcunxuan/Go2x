#!/usr/bin/env python3
import json, math, os, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState

RUNTIME=Path(os.environ.get('PATROL_RUNTIME','/project/runtime'))
# Forward component substituted into rotate-only Move requests.  Measured
# 2026-08-29: this firmware ignores pure in-place rotation; a 0.10 m/s arc
# yields a ~0.15 m turn radius (near-in-place) and reliable yaw response.
ROT_ARC_X=float(os.environ.get('GO2_ROT_ARC_X','0.10'))

def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=Path(str(path)+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')
    os.replace(temporary,path)

class Go2StateBridge(Node):
    def __init__(self):
        super().__init__('go2_state_bridge')
        self.low=None;self.sport=None;self.low_at=0.0;self.sport_at=0.0;self.last_action=None
        self.nav_was_moving=False;self.nav_last_publish=0.0;self.nav_publish_count=0
        self.nav_last_parameter=None;self.nav_block_reason='no_command';self.nav_joystick_acquired_at=0.0
        self.nav_pulse_started=0.0;self.nav_gap_until=0.0;self.nav_cadence=os.environ.get('GO2_NAV_CADENCE','continuous')
        self.nav_lie_at=time.monotonic()
        self.request_id=int(time.time()*1000000)
        try:self.last_action=json.loads((RUNTIME/'go2_action.json').read_text()).get('id')
        except (OSError,ValueError):pass
        qos=rclpy.qos.qos_profile_sensor_data
        self.create_subscription(LowState,'/lf/lowstate',self.on_low,qos)
        self.create_subscription(SportModeState,'/lf/sportmodestate',self.on_sport,qos)
        self.publisher=self.create_publisher(Request,'/api/sport/request',10)
        self.create_timer(.10,self.action_tick);self.create_timer(.05,self.nav_tick);self.create_timer(.50,self.write_state)
        self.get_logger().info('GO2 state bridge ready (DDS domain 0)')
    def on_low(self,msg):self.low=msg;self.low_at=time.time()
    def on_sport(self,msg):self.sport=msg;self.sport_at=time.time()
    def request(self,api_id,parameter=''):
        # GO2 de-duplicates requests by identity.id.  A constant default ID
        # makes a streamed Move command run only for its first short lease.
        self.request_id+=1
        request=Request();request.header.identity.id=self.request_id
        request.header.identity.api_id=api_id;request.parameter=parameter
        return request
    def action_tick(self):
        try:action=json.loads((RUNTIME/'go2_action.json').read_text(encoding='utf-8'))
        except (OSError,ValueError):return
        if action.get('id')==self.last_action:return
        self.last_action=action.get('id');api_id=int(action.get('api_id',0))
        if api_id not in (1003,1004,1005):return
        request=self.request(api_id)
        self.publisher.publish(request)
        atomic_json(RUNTIME/'go2_action_result.json',{'id':self.last_action,'ok':True,'api_id':api_id,'sent_at':time.time()})
    def nav_safe(self, require_localization=True):
        if time.time()-self.low_at>=2 or time.time()-self.sport_at>=2 or not self.low or not self.sport:
            self.nav_block_reason='state_stale';return False
        if require_localization:
            try:localization=json.loads((RUNTIME/'robot_state.json').read_text(encoding='utf-8'))
            except (OSError,ValueError):localization={}
            if localization.get('localization_sane') is False:
                self.nav_block_reason='localization_invalid';return False
        temperatures=[float(x.temperature) for x in self.low.motor_state if float(x.temperature)>0]
        if not temperatures or max(temperatures)>=85:self.nav_block_reason='temperature';return False
        if float(self.sport.body_height)<0.18:
            self.nav_lie_at=time.monotonic();self.nav_block_reason='not_standing';return False
        # A stand-up transition crosses the 0.18 m gate long before the gait
        # is ready; Move commands during the ~8 s settle window make the body
        # lunge without stepping (measured 2026-08-29). Hold motion until the
        # dog has been continuously standing for 8 s.
        if self.nav_lie_at and time.monotonic()-self.nav_lie_at<8.0:
            self.nav_block_reason='standing_settle';return False
        try:
            if float(self.sport.bms_state.soc)<15:self.nav_block_reason='low_battery';return False
        except (AttributeError,TypeError,ValueError):pass
        self.nav_block_reason=None;return True
    def nav_stop(self):
        if self.nav_was_moving:
            self.publisher.publish(self.request(1003))
        self.nav_joystick_acquired_at=0.0
        self.nav_was_moving=False;self.nav_pulse_started=0.0;self.nav_gap_until=0.0
    def nav_tick(self):
        try:command=json.loads((RUNTIME/'go2_nav_command.json').read_text(encoding='utf-8'))
        except (OSError,ValueError):self.nav_stop();return
        fresh=time.time()-float(command.get('updated_at',0))<=.30
        if not fresh:self.nav_block_reason='command_stale';self.nav_stop();return
        if int(command.get('api_id',0))!=1008:self.nav_block_reason='not_move_command';self.nav_stop();return
        source=str(command.get('source','navigation'))
        if source not in ('navigation','teleop'):
            self.nav_block_reason='invalid_source';self.nav_stop();return
        # Autonomous navigation depends on a sane map pose.  Manual web
        # control is body-relative and must remain available when FAST-LIO is
        # stopped; it still requires fresh GO2 state, safe temperature and a
        # standing robot, and the web server blocks it while Nav2 is running.
        if not self.nav_safe(require_localization=(source=='navigation')):self.nav_stop();return
        now=time.monotonic()
        try:
            motion=json.loads(str(command.get('parameter','{}')))
            rotate_only=(abs(float(motion.get('z',0)))>.005 and
                         abs(float(motion.get('x',0)))<=.005 and
                         abs(float(motion.get('y',0)))<=.005)
        except (ValueError,TypeError):
            rotate_only=False
        # GO2 firmware facts measured 2026-08-29 (see 项目信息/07 doc):
        #  - With BIG identity ids, continuous Move streams are ignored and
        #    only a pulsed Move/Stop cadence advances (the original nav
        #    workaround).
        #  - With SMALL incrementing ids, continuous streams are accepted —
        #    verified for rotation (+101 deg) and empirically for translation
        #    by smooth manual teleop driving.  Continuous is now the default
        #    for BOTH sources: the pulsed cadence made navigation visibly
        #    stop-and-go.  Set GO2_NAV_CADENCE=pulse to restore the old
        #    workaround; in pulse mode rotate-only requests are rewritten as
        #    tight arcs because Move(0,0,z) is ignored on that cadence.
        if self.nav_cadence=='pulse':
            if rotate_only:
                try:z=max(-1.0,min(1.0,float(motion.get('z',0))))
                except (ValueError,TypeError):z=0.0
                command=dict(command)
                command['parameter']=json.dumps({'x':ROT_ARC_X,'y':0.0,'z':z},separators=(',',':'))
            if now<self.nav_gap_until:
                self.nav_block_reason='pulse_gap';return
            if self.nav_was_moving and now-self.nav_pulse_started>=.30:
                self.publisher.publish(self.request(1003));self.nav_was_moving=False
                self.nav_gap_until=now+.12;self.nav_block_reason='pulse_gap';return
        if now-self.nav_last_publish<.06:return
        request=self.request(1008,str(command.get('parameter','')))
        self.publisher.publish(request);self.nav_last_publish=now
        if not self.nav_was_moving:self.nav_pulse_started=now
        self.nav_was_moving=True
        self.nav_publish_count+=1;self.nav_last_parameter=request.parameter
    def write_state(self):
        now=time.time();result={'updated_at':now,'lowstate_online':now-self.low_at<2,'sportstate_online':now-self.sport_at<2}
        if self.low:
            b=self.low.bms_state;i=self.low.imu_state
            motors=[float(x.temperature) for x in self.low.motor_state if float(x.temperature)>0]
            result.update(battery_soc=int(b.soc),battery_current_ma=int(b.current),battery_cycles=int(b.cycle),
                voltage_v=float(self.low.power_v),current_a=float(self.low.power_a),imu_rpy=[float(x) for x in i.rpy],
                gyro=[float(x) for x in i.gyroscope],accelerometer=[float(x) for x in i.accelerometer],
                imu_temperature_c=float(i.temperature),max_motor_temperature_c=max(motors) if motors else None,
                foot_force=[int(x) for x in self.low.foot_force])
        if self.sport:
            s=self.sport
            result.update(mode=int(s.mode),gait_type=int(s.gait_type),body_height=float(s.body_height),
                velocity=[float(x) for x in s.velocity],yaw_speed=float(s.yaw_speed),sport_position=[float(x) for x in s.position],
                range_obstacle=[float(x) for x in s.range_obstacle])
        result['nav_relay']={'moving':self.nav_was_moving,'publish_count':self.nav_publish_count,
            'last_parameter':self.nav_last_parameter,'block_reason':self.nav_block_reason,
            'joystick_overridden':bool(self.nav_joystick_acquired_at)}
        atomic_json(RUNTIME/'go2_state.json',result)

def main():
    rclpy.init();node=Go2StateBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.nav_stop();node.destroy_node();rclpy.shutdown()
