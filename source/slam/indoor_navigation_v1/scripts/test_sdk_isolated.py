"""Real ROS NavCmd serialization/stop test in LOCALHOST-only domains 188/189.

No robot-domain participant is created. Injected SDK state is test-only.
"""
import argparse,json,os,sys,time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='189' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
    raise SystemExit('Requires ROS_DOMAIN_ID=189 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
if os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE') or os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'):
    raise SystemExit('Robot DDS profile must be unset for isolated test')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));from indoor.bootstrap import ROOT
from s10nav.util import write
sys.path.insert(0,str(ROOT/'tests'));from test_control import inputs,simple_route
from indoor.control import Guard
from sdk_gate import SdkGate,Q
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from drdds.msg import MotionInfo,StdStatus,NavCmd

class TestGate(SdkGate):
    def service_watch(self):
        while not self.stop.is_set():self.service=(time.monotonic(),False);self.stop.wait(.1)

def main():
    a=argparse.Namespace(execute=True,sensor_domain=188,sdk_domain=189)
    n=TestGate(a);assert n.native_ctx.get_domain_id()==189 and n.sensor_ctx.get_domain_id()==188
    n.guard=Guard(simple_route());source=Node('isolated_indoor_snapshot',context=n.sensor_ctx)
    fake=Node('isolated_robot_sdk',context=n.native_ctx);n.sensor_executor.add_node(source);n.native_executor.add_node(fake)
    loc=source.create_publisher(String,'/wym/indoor/localization',1)
    motion=fake.create_publisher(MotionInfo,'/MOTION_INFO',Q);charge=fake.create_publisher(StdStatus,'/CHARGE_STATUS',Q)
    received=[];fake.create_subscription(NavCmd,'/NAV_CMD',lambda m:received.append((time.monotonic(),m.data.x_vel,m.data.y_vel,m.data.yaw_vel)),Q)
    start=time.monotonic();last=0;attempt=False;stopped=None
    try:
        while time.monotonic()-start<8:
            now=time.monotonic();elapsed=now-start
            if now-last>.03:
                last=now;msg=MotionInfo();msg.header.stamp=fake.get_clock().now().to_msg();msg.data.motion_state.state=17;msg.data.gait_state.gait=0x3002;motion.publish(msg)
                c=StdStatus();c.state=0;c.error_code=0;charge.publish(c)
                # Healthy observations until 5 seconds; then no localization
                # messages while SDK feedback/deadman keep arriving.
                if elapsed<5:
                    snap=inputs(now)[0];m=String();m.data=json.dumps(snap);loc.publish(m)
            n.spin_once()
            if elapsed>3.5 and not attempt:
                attempt=True;n.key('a');assert n.guard.armed,'Unable to arm isolated gate'
            if 3.5<elapsed<6:n.key(' ')
            if elapsed>5.6 and stopped is None:stopped=not n.guard.armed
    finally:
        n.sensor_executor.remove_node(source);n.native_executor.remove_node(fake);source.destroy_node();fake.destroy_node();n.close()
    active=[v for v in received if 3.8<=v[0]-start<5]
    outage=[v for v in received if 5.5<=v[0]-start<7.5]
    result=dict(domains=[188,189],native_domain_created=False,messages=len(received),nonzero_before_outage=sum(any(v[1:]) for v in active),
        outage_messages=len(outage),nonzero_after_deadline=sum(any(v[1:]) for v in outage),latched_stop=stopped,
        bounds_pass=all(0<=v[1]<=.10001 and v[2]==0 and abs(v[3])<=.20001 for v in received),robot_commands_sent=False)
    result['passed']=result['nonzero_before_outage']>0 and len(outage)>10 and result['nonzero_after_deadline']==0 and stopped and result['bounds_pass']
    write(ROOT/'results/sdk_isolated.json',result);print(json.dumps(result),flush=True)
    assert result['passed'],result
if __name__=='__main__':main()
