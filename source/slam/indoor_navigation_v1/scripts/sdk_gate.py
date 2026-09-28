"""S10 native /NAV_CMD transport. Default monitor-only; no mode/joint commands.

Execution is interactive and separately gated by SDK ownership, fresh measured
feedback, healthy localization, operator arming, and a short keyboard lease.
It does NOT replace the robot-side receiver watchdog or physical emergency stop.
"""
import argparse,json,math,os,select,subprocess,sys,termios,threading,time,tty,fcntl
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT
from indoor.control import Guard,zero
from s10nav.util import read,write,sha
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from drdds.msg import NavCmd,MotionInfo,StdStatus

Q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)

class SdkGate:
    def __init__(self,a):
        for f,h in read(ROOT/'assets/manifest.json')['files'].items():
            if sha(ROOT/'assets'/f)!=h:raise ValueError('asset_hash_mismatch:'+f)
        self.a=a;self.begin=time.monotonic();self.guard=Guard(read(ROOT/'assets/route.json'))
        self.snapshot=None;self.feedback=None;self.motion_stamp=None;self.charge=None;self.pub=None;self.sent=0;self.nonzero=0
        self.service=(None,None);self.stop=threading.Event();self.last_log=-1.;self.last_state=None
        self.sensor_ctx=Context();self.native_ctx=Context();rclpy.init(context=self.sensor_ctx,domain_id=a.sensor_domain);rclpy.init(context=self.native_ctx,domain_id=a.sdk_domain)
        self.name=f'wym_indoor_sdk_{os.getpid()}'
        self.sensor=Node(self.name+'_diagnostic',context=self.sensor_ctx,enable_rosout=False,start_parameter_services=False)
        self.native=Node(self.name,context=self.native_ctx,enable_rosout=False,start_parameter_services=False)
        self.sensor_executor=SingleThreadedExecutor(context=self.sensor_ctx);self.sensor_executor.add_node(self.sensor)
        self.native_executor=SingleThreadedExecutor(context=self.native_ctx);self.native_executor.add_node(self.native)
        self.sensor.create_subscription(String,'/wym/indoor/localization',self.localization,1)
        self.native.create_subscription(MotionInfo,'/MOTION_INFO',self.motion,Q)
        self.native.create_subscription(StdStatus,'/CHARGE_STATUS',self.charging,Q)
        self.native.create_subscription(NavCmd,'/NAV_CMD',self.nav_observe,Q)
        self.nav_samples=0;self.nav_latest=None
        (ROOT/'live_logs').mkdir(exist_ok=True);self.log=(ROOT/'live_logs'/f'sdk-{time.time_ns()}.jsonl').open('x',buffering=1)
        self.thread=threading.Thread(target=self.service_watch,daemon=True);self.thread.start()
        self.native.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def service_watch(self):
        while not self.stop.is_set():
            try:
                v=subprocess.run(['systemctl','is-active','rl_deploy.service'],capture_output=True,text=True,timeout=.8)
                # Only known inactive/failed states are safe. Errors and a
                # missing systemd connection are NOT evidence of ownership.
                active=v.stdout.strip() not in ['inactive','failed']
            except Exception:active=True
            self.service=(time.monotonic(),active);self.stop.wait(.7)

    def localization(self,m):
        try:
            obj=json.loads(m.data)
            if len(m.data)>50000:raise ValueError('oversized_snapshot')
            self.snapshot=obj
        except Exception:self.snapshot=None;self.guard.stop('INVALID_LOCALIZATION_MESSAGE')

    def charging(self,m):self.charge=dict(mono=time.monotonic(),state=int(m.state),error=int(m.error_code))

    def motion(self,m):
        stamp=(int(m.header.stamp.sec),int(m.header.stamp.nanosec));values=[m.data.vel_x,m.data.vel_y,m.data.vel_yaw,m.data.height]
        if not all(math.isfinite(v) for v in values) or self.motion_stamp is not None and stamp<=self.motion_stamp:
            self.feedback=None;self.motion_stamp=stamp;return
        self.motion_stamp=stamp
        c=self.charge or dict(mono=-1,state=-1,error=-1)
        self.feedback=dict(mono=time.monotonic(),state=int(m.data.motion_state.state),gait=int(m.data.gait_state.gait),
            speed=math.hypot(m.data.vel_x,m.data.vel_y),yaw_speed=float(m.data.vel_yaw),height=float(m.data.height),
            charge=c['state'],charge_error=c['error'],charge_mono=c['mono'],native_stamp=list(stamp))

    def nav_observe(self,m):
        self.nav_samples+=1;self.nav_latest=dict(x=float(m.data.x_vel),y=float(m.data.y_vel),yaw=float(m.data.yaw_vel))

    def environment(self):
        t,active=self.service
        pub=self.native.get_publishers_info_by_topic('/NAV_CMD');subs=self.native.get_subscriptions_info_by_topic('/NAV_CMD')
        joints=self.native.get_publishers_info_by_topic('/JOINTS_CMD')
        return dict(mono=t if t is not None else -1,rl_service_active=active is not False,
            other_nav_publishers=sum(ep.node_name!=self.name for ep in pub),
            nav_subscribers=sum(ep.node_name!=self.name for ep in subs),
            other_named_joint_publishers=[ep.node_name for ep in joints if not ep.node_name.startswith('_CREATED_BY_BARE_DDS_APP_')])

    def send(self,command):
        if self.pub is None:return
        v=[float(command[k]) for k in ('vx','vy','wz')]
        if not all(math.isfinite(x) for x in v) or not 0<=v[0]<=.100001 or v[1]!=0 or abs(v[2])>.200001:
            self.guard.stop('COMMAND_RANGE_ERROR');v=[0.,0.,0.]
        m=NavCmd();m.header.frame_id=self.sent;m.header.stamp=self.native.get_clock().now().to_msg()
        m.data.x_vel,m.data.y_vel,m.data.yaw_vel=v;self.pub.publish(m);self.sent+=1;self.nonzero+=int(any(v))

    def tick(self):
        now=time.monotonic();env=self.environment();cmd=self.guard.tick(now,self.snapshot,self.feedback,env);self.send(cmd)
        status=dict(t=now,elapsed_s=now-self.begin,execute=self.a.execute,command=cmd,feedback=self.feedback,environment=env,
            nav_observed_count=self.nav_samples,nav_observed_latest=self.nav_latest,sent=self.sent,nonzero_sent=self.nonzero)
        self.log.write(json.dumps(status)+'\n')
        if cmd['state']!=self.last_state or now-self.last_log>=2:
            print(json.dumps(status),flush=True);self.last_log=now;self.last_state=cmd['state']

    def key(self,k):
        now=time.monotonic()
        if k in ('q','\x03','\x1b'):self.guard.stop('OPERATOR_STOP');self.stop.set()
        elif k=='s':self.guard.stop('OPERATOR_STOP')
        elif k=='a' and self.a.execute:
            ok,reason=self.guard.arm(now);print('ARM',ok,reason,flush=True)
            if ok and self.pub is None:self.pub=self.native.create_publisher(NavCmd,'/NAV_CMD',Q)
        elif k==' ':self.guard.renew_deadman(now)

    def spin_once(self):
        self.sensor_executor.spin_once(timeout_sec=.002);self.native_executor.spin_once(timeout_sec=.002)

    def close(self):
        self.stop.set();self.guard.stop('SHUTDOWN')
        if self.pub is not None:
            # Best-effort stop burst. SIGKILL, network loss and power failure
            # need an independently verified receiver-side timeout.
            for _ in range(10):self.send(zero('SHUTDOWN'));time.sleep(.1)
        write(ROOT/'live_logs'/f'sdk-exit-{time.time_ns()}.json',dict(execute=self.a.execute,sent=self.sent,nonzero_sent=self.nonzero,
            reached=self.guard.reached,complete=self.guard.complete,nav_samples=self.nav_samples,feedback=self.feedback,environment=self.environment()))
        self.log.close();self.native_executor.shutdown();self.sensor_executor.shutdown()
        self.native.destroy_node();self.sensor.destroy_node();self.native_ctx.shutdown();self.sensor_ctx.shutdown()

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--seconds',type=float,default=10)
    p.add_argument('--onsite-confirmed',action='store_true');p.add_argument('--navigation-mode-confirmed',action='store_true')
    p.add_argument('--receiver-stop-verified',action='store_true');p.add_argument('--sensor-domain',type=int,default=88);p.add_argument('--sdk-domain',type=int,default=0)
    a=p.parse_args()
    if a.execute and (not all([a.onsite_confirmed,a.navigation_mode_confirmed,a.receiver_stop_verified]) or not sys.stdin.isatty()):
        p.error('Execution requires an interactive onsite operator, confirmed navigation mode, and a physically verified receiver-side stop timeout. Monitor requires no flags.')
    # One indoor bridge per user/host, even across multiple terminals.
    lock=open('/tmp/wym-indoor-sdk.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    n=SdkGate(a);old=None
    if a.execute:old=termios.tcgetattr(sys.stdin);tty.setcbreak(sys.stdin.fileno());print('a: ARM; SPACE: renew 0.4 s motion lease; s: STOP; q: STOP+QUIT. Stops latch; a explicitly reassociates current segment.',flush=True)
    try:
        while not n.stop.is_set() and (not a.seconds or time.monotonic()-n.begin<a.seconds):
            n.spin_once()
            if old is not None and select.select([sys.stdin],[],[],0)[0]:
                k=os.read(sys.stdin.fileno(),1).decode(errors='ignore')
                if not k:n.guard.stop('TERMINAL_EOF');break
                n.key(k)
    except KeyboardInterrupt:pass
    finally:
        if old is not None:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old)
        n.close()
if __name__=='__main__':main()
