"""Actual GOAI policy + bridge + synthetic sensors, loopback-only domains 187/188."""
import json,math,os,signal,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from goai_route import RouteBridge,TOPIC
from indoor.bootstrap import ROOT
from s10nav.util import read,sha
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data as Q
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from drdds.msg import JointsData,JointsDataCmd,ImuData,Steer

assert os.environ['ROS_DOMAIN_ID']=='187' and os.environ['ROS_LOCALHOST_ONLY']=='1'
assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST'
assert os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']==os.environ['FASTDDS_DEFAULT_PROFILES_FILE']
ns={'f':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'};doc=ET.parse(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'])
assert [v.text for v in doc.findall('.//f:interfaceWhiteList/f:address',ns)]==['127.0.0.1']
assert doc.find('.//f:useBuiltinTransports',ns).text=='false'

contexts=[Context(),Context()]
for ctx,domain in zip(contexts,(187,188)):rclpy.init(context=ctx,domain_id=domain)
native=Node('goai_route_fixture',context=contexts[0]);sensor=Node('goai_localization_fixture',context=contexts[1]);executors=[]
for n,c in zip((native,sensor),contexts):
    ex=SingleThreadedExecutor(context=c);ex.add_node(n);executors.append(ex)
jpub=native.create_publisher(JointsData,'/JOINTS_DATA',Q);ipub=native.create_publisher(ImuData,'/IMU_DATA',Q)
spub=native.create_publisher(Steer,'/STEER',Q);kpub=native.create_publisher(String,'/GAMEPAD_KEY',Q)
apub=native.create_publisher(String,'/wym/goai/nav_authority',1);lpub=sensor.create_publisher(String,'/wym/indoor/localization',1)
status={};velocities=[];joint_count=0;nonce=0;nonce_counter=0;authority_seq=0;loc_seq=0;obstacle=False;bridge=None;process=None;last_feed=-1.;last_loc=-1.
map_hold=None;safety_hold=None
signs=[1,1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,-1,1,-1]
offsets=[-35,-145,156,0,35,-145,156,0,-35,145,-156,0,35,145,-156,0]
q=[.05,-.35,.65,0,-.05,-.35,.65,0,.05,.35,-.65,0,-.05,.35,-.65,0]
joints=JointsData();imu=ImuData();imu.data.acc_z=9.81;steer=Steer()
for i,m in enumerate(joints.data.joints_data):
    m.position=float((q[i]-math.radians(offsets[i]))*signs[i]);m.motion_temp=30.;m.driver_temp=30.;m.status_word=1

def joint_cb(m):
    global joint_count
    joint_count+=1
    for a,b in zip(m.data.joints_data,joints.data.joints_data):
        if a.kp>0:b.position=a.position

def status_cb(m):status.clear();status.update(json.loads(m.data))
def velocity_cb(m):velocities.append((time.monotonic(),m.header.stamp.sec*1000000000+m.header.stamp.nanosec,m.header.frame_id,m.twist.linear.x,m.twist.linear.y,m.twist.angular.z))
native.create_subscription(JointsDataCmd,'/JOINTS_CMD',joint_cb,Q);native.create_subscription(String,'/wym/goai/status',status_cb,Q)
native.create_subscription(TwistStamped,TOPIC,velocity_cb,Q)
asset_id=sha(ROOT/'assets/manifest.json');boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
pose=[*read(ROOT/'assets/route.json')['waypoints'][0]['xyz'],0.]

def pump(seconds,localization=True,authority=True):
    global last_feed,last_loc,loc_seq,authority_seq
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        now=time.monotonic()
        if now-last_feed>=.005:
            stamp=native.get_clock().now().to_msg();joints.header.stamp=stamp;imu.header.stamp=stamp;steer.header.stamp=stamp
            jpub.publish(joints);ipub.publish(imu);spub.publish(steer);last_feed=now
        if now-last_loc>=.02:
            if localization:
                measured=now-.02 if map_hold is None else map_hold
                clearance_t=now-.02 if safety_hold is None else safety_hold
                loc_seq+=1;m=String();m.data=json.dumps(dict(schema='goai.localization.continuity.v2',mono=now,seq=loc_seq,pose=pose,
                    health=dict(age_s=now-measured,valid=now-measured<.35,single_lidar=False,healthy_lidars=['front','rear']),
                    generation=[1,0],front_fresh=True,rear_fresh=True,perception_mono=dict(front=clearance_t,rear=clearance_t),obstacle=obstacle,error=None,
                    solution=dict(mode='TRACKING',measurement_mono=measured,estimate_mono=now-.005,measured_pose=pose,pose=pose,
                        generation=[1,0],velocity_body=[0,0,0],reason=''),
                    safety_records={s:dict(mono=clearance_t,valid=True,blocked=obstacle,epoch=0) for s in ('front','rear')},
                    asset_id=asset_id,boot_id=boot_id,run_id='isolated-fixture'))
                lpub.publish(m)
            if authority:
                authority_seq+=1;m=String();m.data=f'P {nonce} {authority_seq}' if nonce else f'R 0 {authority_seq}';apub.publish(m)
            last_loc=now
        for ex in executors:ex.spin_once(timeout_sec=.001)
        if bridge:bridge.spin_once()

def key(value):m=String();m.data=value;kpub.publish(m)
def require(value,why):
    if not value:raise AssertionError(why+' '+json.dumps(dict(goai=status,route=None if bridge is None else bridge.guard.last_command)))
def select():
    global nonce,nonce_counter
    bridge.command('ROUTE-READY');require(bridge.guard.pending,'route intent accepted')
    nonce_counter+=1;nonce=nonce_counter;pump(.3);key('G12_KEY_C');pump(.7)
    require(status.get('source')=='navigation' and bridge.guard.armed,'actual GOAI selected nav')
    require(any(abs(x)>1e-5 for x in status.get('selected_velocity',[])),'actual GOAI consumed route velocity')

report={};logs=ROOT/'results/goai_route';logs.mkdir(parents=True,exist_ok=True)
log=(logs/'actual-goai-isolated.log').open('w')
try:
    exe='/home/wym/s10_goai_ws/install/s10_goai_deploy/lib/s10_goai_deploy/goai_node'
    process=subprocess.Popen([exe,'--ros-args','-p','allow_actuation:=true','-p','sdk_mode_confirmed:=true','-p','g12_event_input:=true'],stdout=log,stderr=subprocess.STDOUT)
    bridge=RouteBridge(NS(execute=False,sensor_domain=188,control_domain=187));pump(3)
    require(bridge.sent==0 and native.count_publishers(TOPIC)==0 and joint_count==0,'monitor creates no control publisher');report['monitor_no_commands']=True
    bridge.close();bridge=None
    bridge=RouteBridge(NS(execute=True,sensor_domain=188,control_domain=187));pump(1)
    require(all(v[3:]==(0.,0.,0.) for v in velocities),'execute starts with zeros')
    key('G12_KEY_C');pump(4.6);require(status.get('mode')=='hold','synthetic stand')
    key('G12_KEY_A');pump(3);require(status.get('mode')=='policy','synthetic manual policy')
    require(bridge.nonzero==0,'no autonomous start');report['zero_until_explicit_route_and_C']=True
    select();report['actual_goai_consumes_route_twist']=True
    require(all(v[2]=='base_link' and 0<=v[3]<=.1 and v[4]==0 and abs(v[5])<=.2 for v in velocities),'message contract')
    require(all(b[1]>a[1] for a,b in zip(velocities,velocities[1:])),'strictly increasing stamps');report['twist_frame_limits_monotonic']=True
    map_hold=time.monotonic()-.02;pump(.31)
    require(bridge.guard.armed and bridge.guard.robust_limits[0]<=.04,'short map gap keeps bounded navigation')
    pump(.13);require(bridge.guard.armed and bridge.guard.robust_limits[0]<=.02,'prediction-only velocity cap')
    map_hold=None;pump(.25);require(bridge.guard.armed,'short gap recovery requires no new permit')
    require(bridge.guard.robust_limits[0]<=.04,'recovery speed hysteresis');pump(.6)
    report['short_map_gap_degrades_without_disarm']=True
    safety_hold=time.monotonic()-.02;pump(.28)
    require(bridge.guard.armed and bridge.guard.robust_limits==(.02,0.),'short clearance gap forbids turning')
    safety_hold=None;pump(.65);require(bridge.guard.armed,'short clearance gap recovers');report['short_clearance_gap_no_turn']=True
    obstacle=True;pump(.55);require(status['selected_velocity']==[0.,0.,0.] and not bridge.guard.armed,'obstacle zero')
    obstacle=False;pump(2.3);require(status['selected_velocity']==[0.,0.,0.] and not bridge.guard.armed,'no auto recovery')
    bridge.command('ROUTE-READY');require(not bridge.guard.pending,'must return to manual first');report['obstacle_stop_latched']=True
    key('G12_KEY_A');nonce=0;pump(2.4);select();require(bridge.guard.target==1,'resume preserves target')
    report['explicit_rearm_preserves_order']=True
    map_hold=time.monotonic()-.02;pump(.95)
    require(status['selected_velocity']==[0.,0.,0.] and not bridge.guard.armed,'fresh estimator heartbeat cannot hide expired map measurement')
    map_hold=None;pump(.6);require(not bridge.guard.armed,'long map gap recovery stays latched')
    report['expired_map_with_live_heartbeat_latched']=True
    key('G12_KEY_A');nonce=0;pump(2.4);select()
    pump(.6,localization=False);require(status['selected_velocity']==[0.,0.,0.] and not bridge.guard.armed,'localization timeout zero')
    pump(.5);require(not bridge.guard.armed,'localization recovery no automatic resume');report['localizer_loss_latched']=True
    key('G12_KEY_A');nonce=0;pump(2.4);select()
    pump(.7,authority=False);require(status['selected_velocity']==[0.,0.,0.] and not bridge.guard.armed,'G12 permit timeout zero');report['authority_loss_actual_goai_zero']=True
    # Bridge disappears while actual GOAI is selected. Its independent input
    # deadline must expire even if an authority fixture keeps publishing.
    key('G12_KEY_A');nonce=0;pump(2.4);select()
    before=len(velocities);bridge.close();bridge=None;pump(.6)
    require(status['selected_velocity']==[0.,0.,0.] and status['source']=='stopped','bridge exit + GOAI watchdog')
    report['bridge_exit_independent_goai_deadline']=True
    report.update(physical_motion=False,domains=[187,188],joint_frames_synthetic=joint_count,velocity_messages=len(velocities),passed=True)
finally:
    if bridge:bridge.close()
    if process and process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=8)
    log.close()
    for ex in executors:ex.shutdown()
    for n in (native,sensor):n.destroy_node()
    for c in contexts:c.shutdown()
    (logs/'isolated_result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
