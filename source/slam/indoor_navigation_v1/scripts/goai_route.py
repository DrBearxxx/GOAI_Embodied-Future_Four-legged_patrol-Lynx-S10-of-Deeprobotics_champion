"""Indoor route -> GOAI TwistStamped. Default read-only. Never joint/SDK commands."""
import argparse,fcntl,json,math,os,select,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT
from indoor.goai_control import GoaiRoute
from indoor.control import zero
from indoor.transport import JsonIngress
from s10nav.util import read,sha
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

TOPIC='/wym/goai/nav_cmd_vel'
Q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
SUB_Q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)


class RouteBridge:
    def __init__(self,a):
        self.a=a;self.pub=None;self.closed=False;self.sent=0;self.nonzero=0;self.last_stamp=0
        self.start=time.monotonic();self.last_print=-1.;self.last_reason=None;self.error=None;self.env=None;self.graph_time=-1.;self.suspended=False
        assets=ROOT/'assets'
        for f,h in read(assets/'manifest.json')['files'].items():
            if sha(assets/f)!=h:raise ValueError('asset_hash_mismatch:'+f)
        self.guard=GoaiRoute(read(assets/'route.json'),sha(assets/'manifest.json'),Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        self.loc=JsonIngress();self.feedback=JsonIngress();self.name=f'wym_goai_route_{os.getpid()}'
        self.contexts=[Context(),Context()]
        for ctx,domain in zip(self.contexts,(a.sensor_domain,a.control_domain)):rclpy.init(context=ctx,domain_id=domain)
        self.sensor=Node(self.name+'_localization',context=self.contexts[0],enable_rosout=False,start_parameter_services=False)
        self.control=Node(self.name,context=self.contexts[1],enable_rosout=False,start_parameter_services=False)
        self.executors=[]
        for n,c in zip((self.sensor,self.control),self.contexts):
            ex=SingleThreadedExecutor(context=c);ex.add_node(n);self.executors.append(ex)
        self.sensor.create_subscription(String,'/wym/indoor/localization',lambda m,i:self.input(self.loc,m,i,self.sensor,'/wym/indoor/localization'),SUB_Q)
        self.control.create_subscription(String,'/wym/goai/status',lambda m,i:self.input(self.feedback,m,i,self.control,'/wym/goai/status'),SUB_Q)
        if a.execute:self.pub=self.control.create_publisher(TwistStamped,TOPIC,Q)
        logs=ROOT/'live_logs';logs.mkdir(exist_ok=True)
        self.log=(logs/f'goai-route-{time.time_ns()}.jsonl').open('x',encoding='utf-8',buffering=1)
        self.control.create_timer(.04,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        print('EXECUTE: initial ZERO stream; ROUTE-READY -> NAV-READY -> C' if a.execute else 'MONITOR ONLY: no velocity/authority/joint publishers',flush=True)

    def input(self,ingress,m,info,node,topic):
        endpoints=node.get_publishers_info_by_topic(topic)
        if len(endpoints)!=1:
            ingress.value=None;ingress.error='AMBIGUOUS_INPUT_PUBLISHERS';self.guard.stop(ingress.error);return
        if not ingress.receive(m.data,info,time.time_ns(),time.monotonic(),bytes(endpoints[0].endpoint_gid)):self.guard.stop(ingress.error)

    def graph(self,now):
        if now-self.graph_time<.1:return self.env
        publishers=self.control.get_publishers_info_by_topic(TOPIC)
        own_count=sum(p.node_name==self.name for p in publishers)
        other=len(publishers) if self.pub is None else (len(publishers)-1 if own_count==1 else max(1,len(publishers)))
        self.env=dict(mono=now,status_publishers=self.control.count_publishers('/wym/goai/status'),
            localization_publishers=self.sensor.count_publishers('/wym/indoor/localization'),
            other_nav_publishers=other,
            nav_subscribers=self.control.count_subscribers(TOPIC),
            authority_publishers=self.control.count_publishers('/wym/goai/nav_authority'))
        self.graph_time=now;return self.env

    def send(self,c):
        if self.pub is None:return
        stamp=self.control.get_clock().now()
        if stamp.nanoseconds<=self.last_stamp:
            self.guard.stop('ROS_CLOCK_REGRESSION');self.error='ROS_CLOCK_REGRESSION'
            # Do not manufacture a new stamp. GOAI's independent .25 s
            # command deadline takes over if the system clock cannot advance.
            return
        self.last_stamp=stamp.nanoseconds
        v=[float(c[k]) for k in ('vx','vy','wz')]
        if not all(math.isfinite(x) for x in v) or not 0<=v[0]<=.1 or v[1]!=0 or abs(v[2])>.2:
            self.guard.stop('COMMAND_RANGE_ERROR');v=[0.,0.,0.]
        m=TwistStamped();m.header.frame_id='base_link';m.header.stamp=stamp.to_msg()
        m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=v
        self.pub.publish(m);self.sent+=1;self.nonzero+=int(any(v))

    def tick(self):
        now=time.monotonic()
        was_armed=self.guard.armed
        try:
            env=self.graph(now);c=self.guard.tick(now,self.loc.get(now),self.feedback.get(now),env)
            if self.error:self.guard.stop(self.error);c=zero(self.error)
        except Exception as exc:
            self.error='BRIDGE_EXCEPTION:'+repr(exc);self.guard.stop(self.error);c=zero(self.error)
        f=self.feedback.get(now)
        # Normal zero velocities are slew-limited by GOAI. On a route safety
        # stop, send one immediate zero then withdraw the stream while nav is
        # selected: the independent .25 s GOAI deadline latches a hard zero
        # target. Do not keep renewing a smooth-deceleration command forever.
        if self.suspended and f and f['source'] in ('manual','stopped'):self.suspended=False
        if not self.suspended:
            self.send(c)
            if not self.guard.armed and (was_armed or (f and f['source']=='navigation')):self.suspended=True
        status=dict(mono=now,execute=self.a.execute,command=c,environment=self.env,
            localization=self.loc.get(now),goai=self.feedback.get(now),sent=self.sent,nonzero_sent=self.nonzero,
            errors=[self.loc.error,self.feedback.error,self.error],stream_suspended=self.suspended)
        self.log.write(json.dumps(status,allow_nan=False)+'\n')
        reason=(c['state'],c.get('target'),c.get('pending'),c.get('ready'))
        if reason!=self.last_reason or now-self.last_print>=2:
            print(json.dumps(dict(command=c,sent=self.sent,nonzero_sent=self.nonzero)),flush=True)
            self.last_print=now;self.last_reason=reason

    def command(self,line):
        cmd=line.strip()
        if cmd in ('STOP','QUIT'):
            self.guard.stop('OPERATOR_STOP');self.send(zero('OPERATOR_STOP'))
            if cmd=='QUIT':self.closed=True
        elif cmd=='ROUTE-READY' and self.a.execute:
            # Refresh all guards at command time, not at the last timer tick.
            self.tick();print('ROUTE',*self.guard.prepare(time.monotonic()),flush=True)
        elif cmd:print('Commands: ROUTE-READY, STOP, QUIT. Data recovery never starts motion.',flush=True)

    def spin_once(self):
        for ex in self.executors:ex.spin_once(timeout_sec=.001)

    def close(self):
        self.guard.stop('SHUTDOWN')
        for _ in range(3):self.send(zero('SHUTDOWN'));time.sleep(.02)
        self.log.write(json.dumps(dict(exit=True,sent=self.sent,nonzero_sent=self.nonzero,
            reached=self.guard.reached,complete=self.guard.complete))+'\n');self.log.close()
        for ex in self.executors:ex.shutdown()
        for node in (self.sensor,self.control):node.destroy_node()
        for ctx in self.contexts:ctx.shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--onsite-confirmed',action='store_true')
    p.add_argument('--seconds',type=float,default=0);p.add_argument('--sensor-domain',type=int,default=88);p.add_argument('--control-domain',type=int,default=0)
    a=p.parse_args()
    if a.execute and (not a.onsite_confirmed or not sys.stdin.isatty()):p.error('Execution requires onsite confirmation and interactive terminal')
    if a.sensor_domain==a.control_domain:p.error('Sensor and control domains must remain separate')
    lock=open('/tmp/wym-indoor-goai-route.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    n=RouteBridge(a)
    try:
        while not n.closed and (not a.seconds or time.monotonic()-n.start<a.seconds):
            n.spin_once()
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                line=sys.stdin.readline()
                if not line:break
                n.command(line)
    except KeyboardInterrupt:pass
    finally:n.close()

if __name__=='__main__':main()
