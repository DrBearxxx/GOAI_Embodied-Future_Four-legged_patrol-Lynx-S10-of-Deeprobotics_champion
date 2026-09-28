"""One bounded, authorized standing test. No axes, velocity or joint publishers.

Uses existing STAND/POLICY/LIE operator requests; never forges G12 centered
evidence. Existing GOAI health/ownership guards remain authoritative.
"""
import collections,datetime,json,math,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data as Q
from drdds.msg import JointsData,ImuData
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]
PIN='010f038ed1203b230100000000001903'
BOOT='3bb27b1d-28a6-41b6-afc4-81793f7e2b4f'
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==BOOT,'New boot: re-audit first'
out=ROOT/'results'/('standing-retest-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir(parents=True,exist_ok=False)
subprocess.run(['python3',str(ROOT/'scripts/verify_goai_overlay.py')],check=True)
before=set((ROOT/'live_logs').glob('*.jsonl'))
rclpy.init();node=rclpy.create_node('wym_authorized_standing_retest',enable_rosout=False,start_parameter_services=False)
status={};joint={};body_imu={};operator_stop=False;phase='waiting_real_center';started=False;children=[];handles=[]
last_print=-1.;last_row=-1.;history=[];requests=[];failure=None;standing_verified=False;policy_verified=False
log=(out/'telemetry.jsonl').open('x',buffering=1)
direction=[1,1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,-1,1,-1]
offsets=[math.radians(v) for v in [-35,-145,156,0,35,-145,156,0,-35,145,-156,0,35,145,-156,0]]
neutral=[.05,-.35,.65,0,-.05,-.35,.65,0,.05,.35,-.65,0,-.05,.35,-.65,0]

def joints(m):
    q=[x.position*d+o for x,d,o in zip(m.data.joints_data,direction,offsets)]
    for i in range(16):
        if i%4 in (1,2):
            limit=math.radians(140 if i%4==1 else 164)
            if q[i]<-limit:q[i]+=2*math.pi
            elif q[i]>limit:q[i]-=2*math.pi
    joint.update(mono=time.monotonic(),q=q,max_stand_error=max(abs(q[i]-neutral[i]) for i in range(16) if i%4!=3))
def imu(m):
    body_imu.update(mono=time.monotonic(),roll_deg=m.data.roll,pitch_deg=m.data.pitch,yaw_deg=m.data.yaw)
def key(m):
    global operator_stop
    if started and m.data in ('G12_KEY_A','G12_KEY_B','G12_KEY_C','G12_KEY_D','G20_KEY_R1','G20_KEY_R2'):
        operator_stop=True
subs=[node.create_subscription(String,'/wym/goai/status',lambda m:status.update(json.loads(m.data)),Q),
      node.create_subscription(JointsData,'/JOINTS_DATA',joints,Q),node.create_subscription(ImuData,'/IMU_DATA',imu,Q),
      node.create_subscription(String,'/GAMEPAD_KEY',key,Q)]
# This is an operator posture-command entry, not sensor/controller spoofing.
pub=node.create_publisher(String,'/GAMEPAD_KEY',Q)

def healthy():
    return (status and 0<=time.monotonic()-status.get('mono',-1e9)<.25 and status.get('backend')=='goai_default'
        and status.get('allow_actuation') is True and status.get('healthy') is True and status.get('fault') is False
        and status.get('ownership') is True and status.get('passive_native_gid')==PIN and status.get('foreign_commands')==0
        and status.get('other_joint_publishers')==0 and status.get('manual_publishers')==1
        and status.get('nav_publishers')==0 and status.get('nav_permit_publishers')==0
        and node.count_publishers('/wym/goai/status')==1)
def centered():
    axes=status.get('steer_axes',[])
    return len(axes)==3 and all(math.isfinite(v) and abs(v)<.03 for v in axes)
def pump(seconds,require_healthy=False):
    global last_print,last_row
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.005);now=time.monotonic()
        if require_healthy:
            if operator_stop:raise RuntimeError('OPERATOR_TAKEOVER')
            if not healthy():raise RuntimeError('GOAI_HEALTH_OR_OWNERSHIP_LOST')
            if not centered():raise RuntimeError('REAL_JOYSTICK_MOVED')
            if status.get('mode') in ('damping','disarmed'):raise RuntimeError('CONTROLLER_STOPPED')
        if now-last_row>=.1:
            row=dict(mono=now,phase=phase,status=dict(status),joint=dict(joint),body_imu=dict(body_imu))
            log.write(json.dumps(row)+'\n');history.append(row);last_row=now
        if now-last_print>=5:
            print(json.dumps(dict(phase=phase,mode=status.get('mode'),healthy=status.get('healthy'),ownership=status.get('ownership'),
                manual_ready=status.get('manual_ready'),steer_packets=status.get('steer_packets'),
                stand_error=joint.get('max_stand_error'),selected_velocity=status.get('selected_velocity'))),flush=True);last_print=now
def request(label):
    assert label in ('STAND','POLICY','LIE')
    if not healthy():raise RuntimeError('NO_FRESH_HEALTH_FOR_POSTURE_REQUEST')
    m=String();m.data=label;pub.publish(m);requests.append(dict(label=label,mono=time.monotonic()))
    print('OPERATOR_REQUEST',label,flush=True)
def wait_for(predicate,timeout,strict=False):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        pump(.03,strict)
        if predicate():return
    raise RuntimeError('TIMEOUT_'+phase)
def child(script,seconds,label):
    handle=(out/(label+'.stdout')).open('x');handles.append(handle)
    p=subprocess.Popen(['bash',str(ROOT/script),'--seconds',str(seconds)],cwd=ROOT,stdout=handle,stderr=subprocess.STDOUT)
    children.append(p)
    return p

try:
    print('WAITING: actual G12 centered packet required; no stand until real input passes existing gate.',flush=True)
    wait_for(lambda:healthy() and status.get('mode')=='disarmed' and status.get('manual_ready') is True
        and status.get('steer_packets',0)>0 and centered(),90)
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:args=[Path(v.decode()).name for v in p.read_bytes().split(b'\0') if v]
        except (OSError,UnicodeDecodeError):continue
        assert not set(args)&{'goai_route.py','localize_mp.py','localize.py'},'Existing navigation/localizer; no duplicate launched'
    phase='standing';request('STAND');started=True
    wait_for(lambda:status.get('mode') in ('standing','hold'),2)
    wait_for(lambda:status.get('mode')=='hold' and time.monotonic()-joint.get('mono',-1e9)<.1 and joint['max_stand_error']<.25,8,True)
    phase='stand_settle';pump(1.5,True)
    assert status['mode']=='hold' and joint['max_stand_error']<.25,'Measured stand not stable'
    standing_verified=True
    assert status['manual_ready'] and centered(),'Actual centered entry expired'
    phase='policy_start';request('POLICY');wait_for(lambda:status.get('mode')=='policy',2,True)
    pump(3.5,True);assert status['mode']=='policy' and all(abs(v)<1e-6 for v in status['selected_velocity'])
    policy_verified=True
    phase='standing_sensor_warmup';localizer=child('run_goai_localizer.sh',73,'localizer');pump(8,True)
    phase='standing_observation';route=child('run_goai_route.sh',60,'route')
    end=time.monotonic()+62
    while time.monotonic()<end:
        pump(.1,True)
        if localizer.poll() is not None:raise RuntimeError('LOCALIZER_EXITED_EARLY')
        if status['mode']!='policy' or any(abs(v)>1e-6 for v in status['selected_velocity']):raise RuntimeError('NONZERO_OR_UNEXPECTED_CONTROL')
        if route.poll() is not None:
            if route.returncode!=0:raise RuntimeError('ROUTE_MONITOR_FAILED')
            break
    route.wait(timeout=3)
except (Exception,KeyboardInterrupt) as exc:
    failure=str(exc) or type(exc).__name__;print('TEST_STOP',failure,flush=True)
finally:
    for p in children:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
    for f in handles:f.close()
    # Never kill the locomotion process while it is supporting the robot.
    try:
        phase='safe_return';pump(.3)
        if started and not operator_stop and healthy() and status.get('mode') in ('standing','hold','policy'):
            request('LIE');wait_for(lambda:status.get('mode')=='disarmed' and status.get('fault') is False,9)
        pump(.2)
    except Exception as exc:failure=(failure+'; ' if failure else '')+'RETURN:'+str(exc)
    phase='finished';pump(.2);log.close()
    summary=dict(output=str(out),standing_verified=standing_verified,zero_velocity_policy_verified=policy_verified,failure=failure,
        operator_takeover=operator_stop,posture_requests=requests,final_goai_status=dict(status),final_joint=dict(joint),
        navigation_commands_sent=0,axes_or_joint_messages_published_by_test=0,
        physical_posture_motion_requested=started,children_exit_codes=[p.returncode for p in children],
        new_logs=[str(p) for p in sorted(set((ROOT/'live_logs').glob('*.jsonl'))-before)])
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
if failure:raise SystemExit(2)
