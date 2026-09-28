"""Explicitly enabled Orin adapter. No learned-policy or factory control on import.

Cross-family transitions use a prepared standing transfer or the resting path.
ACKs are never treated as confirmation. No factory TLS/configuration writes.
"""
import datetime,json,math,os,signal,socket,struct,subprocess,time,fcntl
from pathlib import Path
from core import MODES,NATIVE,WAYPOINT,waypoint
from wire import decode,basic_status,enable_kernel_timestamp,received_monotonic,integer
from standing_handover import StandingHandover
from damping import DampingControl
from policy_recovery import PolicyRecovery

NATIVE_PHASES=('WAIT_STAND','WAIT_GAIT','WAIT_INPUT_MODE','WAIT_REST')
VERSION='1.18-step-move'
NEAR_ZERO_LINEAR=.10  # m/s, measured planar speed; not an exact-zero comparison
NEAR_ZERO_YAW=.15     # rad/s, measured yaw rate

class Hardware(PolicyRecovery,DampingControl,StandingHandover):
    backend='hardware'
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        from drdds.msg import MotionInfo
        from s10_waypoint_deploy.msg import WaypointCommand
        from dds_ownership import RosDdsOwnership
        self.lock=open('/tmp/wym-s10-g12-gateway.lock','w')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.rclpy=rclpy;rclpy.init();self.node=rclpy.create_node('wym_g12_gateway')
        self.String=String;self.WaypointCommand=WaypointCommand
        self.wp=self.node.create_publisher(WaypointCommand,'/wym/s10_waypoint/command',1)
        self.mode=self.node.create_publisher(String,'/wym/s10_waypoint/mode',1)
        self.motion=None;self.motion_at=-1e9;self.policy={};self.policy_at=-1e9
        self.basic={};self.basic_at=-1e9;self.proc=None;self.log=None
        self.requested='none';self.confirmed='none';self.phase='IDLE';self.since=0;self.detail='运动未解锁'
        self.last_hb=-1e9;self.last_send=-1e9;self.counter=0;self.owned=False
        self.last_audit=-1e9;self.audit={'ok':False,'reason':'WAIT_DDS'}
        self.pause_axes=(0.,0.,0.)
        self.native_input_kind='velocity'
        self.gait_sent_at=None;self.gait_reply=None;self.stand_sent_at=None
        self.native_continuity=False
        self.waypoint_continuity=False
        self.operations=[];self.replies=[]
        self.init_handover()
        self.rx=dict(datagrams=0,basic=0,unrelated=0,rejected=0,last_error=None,last_magic=None)
        self.tx=dict(heartbeat=0,operations=0,axes=0,nonzero_axes=0)
        self.last_reply=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        def motion(msg):
            self.motion=msg.data;self.motion_at=time.monotonic()
            if self.motion_at-getattr(self,'last_motion_trace',-1e9)>=.02:
                self.last_motion_trace=self.motion_at
                self.record('motion_feedback',feedback_mono=self.motion_at,motion_state=self.ms(),gait=self.gs(),
                    height=self.motion.height,velocity=dict(x=self.motion.vel_x,y=self.motion.vel_y,yaw=self.motion.vel_yaw))
            if self.phase.startswith('HANDOVER_'):self.handover_motion_states.add(self.ms())
        def policy(msg):self.policy_feedback(msg.data,time.monotonic())
        self.subs=[self.node.create_subscription(MotionInfo,'/MOTION_INFO',motion,qos),self.node.create_subscription(String,'/wym/s10_waypoint/status',policy,qos)]
        self.dds=RosDdsOwnership(self.node)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('10.21.33.102',0));self.sock.connect(('10.21.33.103',30004));self.sock.setblocking(False)
        self.timestamp_mode=enable_kernel_timestamp(self.sock)
        from zero_guardian import Guardian
        self.guardian=Guardian(self.sock)
        from flight_log import FlightLog
        self.flight=FlightLog(Path(__file__).resolve().parents[1]/'logs/actuation-trace.jsonl')
        self.trace_context={}
    def record(self,event,**fields):
        flight=getattr(self,'flight',None)
        if flight:flight.emit(event,context=getattr(self,'trace_context',{}),**fields)
    def send(self,kind,command,items):
        if command==0x110002:
            from unified_control import validate_velocity
            validate_velocity([items["X"],items["Y"],items["Yaw"]])
        elif kind==0x100001 and command==0x100002:
            from unified_control import validate_axes
            validate_axes([items['X'],items['Y'],items['Yaw']])
        body=json.dumps({'PatrolDevice':dict(Type=kind,Command=command,Time=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),Items=items)},separators=(',',':'),allow_nan=False).encode()
        header=struct.pack('<4sHHBBB5s',b'\xeb\x91\xeb\x90',len(body),self.counter%65536,1,self.counter%256,1,bytes(5));self.counter+=1
        try:self.sock.send(header+body)
        except OSError as exc:
            self.record('wire_error',type=hex(kind),command=hex(command),items=items,error=str(exc));raise
        if kind!=0x100064:
            self.last_wire=dict(type=hex(kind),command=hex(command),items=dict(items),sequence=(self.counter-1)%65536,sent_mono=time.monotonic())
            self.record('wire_tx',**self.last_wire,byte_count=len(header)+len(body),
                input_kind=getattr(self,'native_input_kind',None),
                usage_mode=getattr(self,'basic',{}).get('ControlUsageMode'))
        if kind==0x100064 and command==5:self.tx['heartbeat']+=1
        elif kind==0x100001 and command in (0x110002,0x100002):
            self.tx['axes']+=1;self.tx['nonzero_axes']+=int(any(items.values()))
        else:
            self.tx['operations']+=1
            self.operations.append(dict(type=kind,command=command,items=items,mono=time.monotonic()))
            self.operations=self.operations[-16:]
            print('OPERATION '+json.dumps(dict(type=hex(kind),command=hex(command),items=items)),flush=True)
    def read(self,now):
        for _ in range(32):
            try:data,anc,flags,_=self.sock.recvmsg(65536,128)
            except BlockingIOError:break
            self.rx['datagrams']+=1
            try:
                # Sample AFTER recv/callbacks. Robot Time may have another epoch
                # or coarse resolution; it cannot renew or expire this lease.
                arrived=received_monotonic(anc,flags,time.time(),time.monotonic())
                d=decode(data);self.rx['last_magic']=d['magic']
                b=basic_status(d)
                if b is not None:
                    self.basic=b;self.basic_at=arrived;self.rx['basic']+=1
                    self.record('basic_feedback',feedback_mono=arrived,basic=b)
                else:
                    self.rx['unrelated']+=1
                    if 'ErrorCode' in d['items']:
                        code=integer(d['items']['ErrorCode'],'ERROR_CODE',-2147483648)
                        self.last_reply=dict(type=d['type'],command=d['command'],error_code=code,mono=arrived,message=d['message'])
                        if d['type'] in (0x100001,0x300001) and d['command']==0x300002:
                            # Firmware may reply after the transition deadline.
                            # Keep this as the latest wire reply, not proof of
                            # acceptance or correlation to a rapidly retargeted gait.
                            self.last_gait_response=dict(self.last_reply)
                        if d['command']!=5:
                            self.replies.append(dict(self.last_reply));self.replies=self.replies[-16:]
                            self.record('operation_reply',reply=self.last_reply)
                        if (self.phase=='WAIT_GAIT' and self.gait_sent_at is not None and arrived>=self.gait_sent_at
                                and d['type'] in (0x100001,0x300001) and d['command']==0x300002):
                            self.gait_reply=dict(self.last_reply)
            except (ValueError,KeyError,TypeError,struct.error) as e:
                self.rx['rejected']+=1;self.rx['last_error']=str(e)
    def ms(self):return self.motion.motion_state.state if self.motion else -1
    def gs(self):return self.motion.gait_state.gait if self.motion else -1
    def stopped(self):return self.motion and all(math.isfinite(v) for v in (self.motion.vel_x,self.motion.vel_y,self.motion.vel_yaw)) and math.hypot(self.motion.vel_x,self.motion.vel_y)<=NEAR_ZERO_LINEAR and abs(self.motion.vel_yaw)<=NEAR_ZERO_YAW
    def resting(self):return self.stopped() and self.motion.height<.22 and (self.ms() in (0,4) or (self.ms()==100 and self.policy.get('mode','disarmed')=='disarmed'))
    def process_conflict(self):
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():continue
            if self.proc and int(entry.name)==self.proc.pid:continue
            if self.proc:
                try:
                    if os.getpgid(int(entry.name))==self.proc.pid:continue
                except ProcessLookupError:continue
            try:name=(entry/'comm').read_text().strip()
            except OSError:continue
            if name in ('rl_deploy','blind_node','waypoint_node'):return name
        return None
    def audit_now(self,now):
        graph=self.dds.graph()
        if self.proc and self.proc.poll() is None:
            graph['/JOINTS_CMD']=[e for e in graph.get('/JOINTS_CMD',[]) if e['name']!='wym_s10_waypoint_deploy']
        checked_at=time.monotonic()
        self.audit=self.dds.audit_state.audit(graph,checked_at,self.dds.rmw)
        conflict=self.process_conflict()
        if conflict:self.audit=dict(ok=False,reason='COMPETING_PROCESS:'+conflict)
        # The command topics are single-source; don't share with the js0 bridge.
        for topic in ('/wym/s10_waypoint/command','/wym/s10_waypoint/mode'):
            if any(e.node_name!='wym_g12_gateway' for e in self.node.get_publishers_info_by_topic(topic)):
                self.audit=dict(ok=False,reason='COMPETING_WAYPOINT_SOURCE')
        self.last_audit=checked_at
    def health_reason(self,now,check_policy=True):
        if not self.guardian.healthy(now):return 'ZERO_GUARDIAN_NOT_READY_OR_LATCHED'
        if not self.audit['ok']:return self.audit.get('reason') or 'DDS_AUDIT_FAILED'
        if not 0<=now-self.last_audit<=.5:return 'DDS_AUDIT_STALE'
        if not 0<=now-self.motion_at<=.3:return 'MOTION_TELEMETRY_STALE'
        if not 0<=now-self.basic_at<=2.5:return 'BASIC_STATUS_STALE'
        if self.basic.get('HES')!=0:return 'HARD_STOP_ACTIVE'
        if self.basic.get('Charge')!=0:return 'CHARGING'
        if self.basic.get('Sleep',True):return 'SLEEP_ACTIVE'
        if self.proc and check_policy:
            if self.proc.poll() is not None:return 'POLICY_PROCESS_EXITED'
            if not 0<=now-self.policy_at<=1.6:return 'POLICY_STATUS_STALE'
            if self.policy.get('fault')=='1':return 'POLICY_FAULT:'+self.policy.get('fault_reason','unknown')
            if self.policy.get('frozen')=='1':return 'POLICY_FROZEN'
            if self.policy.get('fault')!='0' or self.policy.get('frozen')!='0':return 'POLICY_STATUS_INCOMPLETE'
        return None
    def healthy(self,now):return self.health_reason(now) is None
    def guard(self,now):
        if not self.healthy(now):raise ValueError('实机反馈/控制所有权未通过：'+str(self.health_reason(now)))
    def wake(self,now):
        reason=self.health_reason(now)
        if reason not in (None,'SLEEP_ACTIVE'):raise ValueError('唤醒前检查失败：'+reason)
        if self.proc or self.phase.startswith('HANDOVER_'):raise ValueError('控制器交接中，不能唤醒')
        if not self.basic.get('Sleep'):
            self.detail='机器人已唤醒；请选择策略';return
        if not self.stopped() or self.ms() not in (0,2,4):raise ValueError('等待静止趴下反馈后唤醒')
        self.pause();self.quiesce_native_input(now)
        self.owned=False;self.requested='none';self.confirmed='none'
        self.phase='WAIT_WAKE';self.since=now
        # Factory APK CommandId.SLEEP_MODE_MSG_ID + changeSleepModeMsg(false).
        # Wake does not send posture, gait, SDK ownership, or velocity commands.
        self.send(0x100002,0x400002,dict(Sleep=False))
        self.detail='正在唤醒；等待休眠状态解除，不自动起立'
    def advance_wake(self,now):
        if self.basic_at>self.since and 0<=now-self.basic_at<=2.5 and self.basic.get('Sleep') is False:
            self.phase='IDLE';self.detail='已唤醒；请选择策略并按需起立'
        elif now-self.since>8:self.fail_transition('唤醒未确认；不自动重发')
    def pause(self):
        # HOLD keeps the waypoint balance policy running. Never damp a standing
        # robot just because the operator's network went away.
        self.pause_axes=(0.,0.,0.)
        self.native_continuity=False
        self.waypoint_continuity=False
        if self.phase=='DAMPING_RESET':
            self.phase='DAMPING_UNCONFIRMED';self.detail='已取消阻尼后的起立；运动仍锁定'
        if self.phase.startswith('HANDOVER_') and self.handover_sdk_at is None:
            # Before the ownership request is sent, Stop can safely cancel the
            # remaining transaction. After sending it, finish into a stable HOLD
            # instead of abandoning support midway. Neither path enables MOVE.
            if self.policy.get('handover')=='prepared':self.command('handover_cancel:'+self.policy['handover_token'])
            if self.policy.get('handover')=='await_native':self.command('handover_cancel_release:'+self.policy['handover_token'])
            self.fail_transition('已取消尚未提交的站立接管；旧控制器保持，运动锁定')
    def command(self,word):
        self.mode.publish(self.String(data=word))
    def fail_transition(self,detail):
        self.phase='FAILED';self.confirmed='none';self.detail=detail
        self.stand_sent_at=None
        print('TRANSITION_FAILED '+detail,flush=True)
    def request_gait(self,now):
        # A lying robot must first complete an explicit stand request. Never
        # mistake a pre-stand gait request or an ACK for the resulting gait.
        self.gait_reply=None;self.gait_sent_at=now;self.phase='WAIT_GAIT';self.since=now
        self.send(0x100001,0x300002,dict(GaitParam=MODES[self.requested]))
        self.detail=f'已站立；已申请步态 {MODES[self.requested]:#x}，等待实机确认'
    def prepare_native(self,mode,now,force_gait=False):
        self.native_continuity=False
        self.native_axes_inhibited=False
        self.requested=mode;self.confirmed='none';self.owned=True;self.since=now
        self.gait_sent_at=None;self.gait_reply=None;self.stand_sent_at=None
        self.phase='WAIT_STAND';self.detail='模式已选择；点击起立后自动申请步态'
        if force_gait:self.request_gait(now)
        else:self.advance_native(now)
    def native_mode(self):
        return 0 if getattr(self,'native_input_kind','velocity')=='native_axes' else 1
    def native_command(self):
        return 0x100002 if self.native_mode()==0 else 0x110002
    def set_native_input_kind(self,kind,now):
        if kind not in ('velocity','native_axes'):raise ValueError('INVALID_NATIVE_INPUT_KIND')
        if kind==getattr(self,'native_input_kind','velocity'):return
        # Clear the previous channel before changing its units or usage mode.
        if self.native_output_active():
            self.send(0x100001,self.native_command(),dict(X=0.,Y=0.,Yaw=0.,Z=0.,Roll=0.,Pitch=0.))
        self.native_input_kind=kind
    def native_gait_confirmed(self,now):
        self.confirmed=self.requested
        mode=self.native_mode();label='原厂摇杆接口' if mode==0 else '导航速度接口'
        # Gait/posture returns firmware to Mode=0. Manual control uses that
        # original interface; navigation configures Mode=1 after the gait.
        if self.basic.get('ControlUsageMode')==mode and (mode==0 or self.gait_sent_at is None):
            self.phase='READY';self.detail='所选步态及'+label+'已就绪';return
        self.phase='WAIT_INPUT_MODE';self.since=now
        self.send(0x100002,0x500002,dict(Mode=mode))
        self.detail='步态已确认；正在切换'+label
    def advance_native(self,now):
        if not self.healthy(now):
            # Never issue an operation using invalid feedback. A single late
            # sample doesn't irreversibly cancel a user's stand/selection.
            deadline=(self.gait_sent_at+5 if self.phase=='WAIT_GAIT' else
                      self.since+5 if self.phase=='WAIT_INPUT_MODE' else
                      self.since+20 if self.phase=='WAIT_REST' else
                      self.stand_sent_at+20 if self.stand_sent_at is not None else None)
            if deadline is not None and now>deadline:self.fail_transition('等待有效反馈超时；可重新选择模式')
            return
        if self.phase=='WAIT_STAND':
            if self.ms()==17 and (self.stand_sent_at is None or self.motion_at>self.stand_sent_at):
                self.stand_sent_at=None
                if self.gs()==MODES[self.requested]:
                    self.native_gait_confirmed(now)
                else:self.request_gait(now)
            elif self.stand_sent_at is not None and now-self.stand_sent_at>20:
                self.fail_transition('起立反馈超时；不会重复起立，请检查状态')
        elif self.phase=='WAIT_GAIT':
            target=MODES[self.requested]
            if self.motion_at>self.gait_sent_at and self.ms()==17 and self.gs()==target:
                self.native_gait_confirmed(now)
            elif self.gait_reply and self.gait_reply['error_code']!=0:
                self.fail_transition(f'步态请求被拒绝 ErrorCode={self.gait_reply["error_code"]}；目标 {target:#x} / 实际 {self.gs():#x}；可重新选择')
            elif now-self.gait_sent_at>5:
                self.fail_transition(f'步态确认超时；目标 {target:#x} / 实际 {self.gs():#x}；可重新选择，不自动重发')
        elif self.phase=='WAIT_INPUT_MODE':
            mode=self.native_mode();label='原厂摇杆接口' if mode==0 else '导航速度接口'
            rejected=next((r for r in reversed(self.replies) if r['type'] in (0x100002,0x300002)
                and r['command']==0x500002 and r['mono']>=self.since and r['error_code']!=0),None)
            if self.basic_at>self.since and self.motion_at>self.since and self.basic.get('ControlUsageMode')==mode:
                if self.ms()==17 and self.gs()==MODES[self.requested]:
                    self.phase='READY';self.detail='所选步态及'+label+'已就绪'
                else:self.fail_transition('输入模式切换后步态发生变化；请重新选择策略')
            elif rejected:
                self.phase='FAILED';self.detail=f'步态已确认；{label}请求被拒绝 ErrorCode={rejected["error_code"]}'
            elif now-self.since>5:
                self.phase='FAILED';self.detail=f'步态已确认；{label}模式未就绪，实际 Mode={self.basic.get("ControlUsageMode")}'
        elif self.phase=='WAIT_REST':
            if self.motion_at>self.since and self.ms() in (0,4):
                self.phase='WAIT_STAND';self.stand_sent_at=None
                self.detail='已趴下；保留所选模式，下次起立后自动申请步态'
            elif now-self.since>20:self.fail_transition('趴下反馈超时；请检查状态')
    def select(self,mode,now):
        # Selecting a model while explicitly damped cannot produce motion.
        # A present hardware fault still blocks arming; a historical one does not.
        if self.proc and self.policy.get('mode')=='damping' and self.health_reason(now)=='POLICY_FROZEN':
            pass
        else:self.guard(now)
        self.control_interrupted=False
        if self.phase not in ('IDLE','READY','FAILED','WAIT_STAND','WAIT_GAIT','WAIT_INPUT_MODE','WAIT_PROFILE','DAMPING','DAMPING_UNCONFIRMED'):raise ValueError('姿态/控制器交接中，请等待当前动作完成')
        if not self.proc and mode in NATIVE and self.ms() in (0,2,4,17):
            # APK changeGaitEnable/sendGaitType* has no speed or lie prerequisite.
            # This changes gait within the factory controller, not SDK ownership.
            self.prepare_native(mode,now);return
        if not self.proc and mode in WAYPOINT and self.ms()==100:
            # SDK is already entered (e.g. a gateway restart). Load disarmed;
            # never resend an ownership transition just to choose an actor.
            self.requested=mode;self.confirmed='none';self.owned=True
            self.quiesce_native_input(now);self.launch_policy(now);return
        if self.proc and self.ms()==100 and mode in WAYPOINT and self.policy.get('mode') in ('policy','hold','disarmed','damping'):
            if self.policy.get('mode')=='policy' and self.policy.get('live_profile_switch')!='1':
                raise ValueError('策略节点尚未更新到直接切换版；未发送 return/stand')
            self.requested=mode;self.since=now
            # Explicit target names are retry-safe; even selecting the current
            # profile must cancel an earlier pending selection on the node.
            if self.policy.get('live_profile_switch')=='1':self.command('select_profile:'+mode)
            elif self.policy.get('profile')!=mode:self.command('cycle_policy')
            self.confirmed='none';self.phase='WAIT_PROFILE'
            self.detail='直接切换 waypoint 模型；保持当前控制状态，不回 stand'
            return
        if (self.proc and self.ms()==100 and mode in NATIVE and self.policy.get('mode')=='damping'
                and self.stopped() and self.motion.height<.22):
            self.command('reset');self.recovery_target=mode;self.phase='RECOVERY_SELECT';self.since=now
            self.detail='已趴下；结束旧策略后切换所选模式';return
        if not self.stopped():raise ValueError('等待近零速度：平面速度 ≤0.10 m/s、转向角速度 ≤0.15 rad/s')
        if mode in WAYPOINT and self.ms()==17 and (not self.proc or self.policy.get('mode')=='disarmed'):
            self.begin_standing_waypoint(mode,now);return
        if self.proc and mode in NATIVE and self.ms()==100 and self.policy.get('mode') in ('policy','hold'):
            self.begin_standing_native(mode,now);return
        if self.proc and mode in NATIVE and self.ms() in (0,4,17) and self.policy.get('mode')=='disarmed':
            # An expired, never-armed preload is not an active SDK owner. Reap
            # it before restoring native input; do not force another SDK cycle.
            self.requested=mode;self.confirmed='none';self.handover_source='native'
            os.killpg(self.proc.pid,signal.SIGINT)
            self.handover_phase('REAP',now,'清理未起控的预加载节点；保留原厂控制');return
        if not self.resting():raise ValueError('等待稳定站姿或趴下反馈后再交接')
        if self.proc and self.policy.get('mode')!='disarmed':raise ValueError('先按趴下，等待策略完全结束')
        self.requested=mode;self.confirmed='none';self.since=now;self.owned=True
        if self.proc:
            os.killpg(self.proc.pid,signal.SIGINT);self.phase='RELEASE_POLICY'
        else:self.begin_sdk_switch(now)
    def can_live_select(self,mode,now):
        if mode in WAYPOINT:
            return (self.owned and self.requested in WAYPOINT and self.proc
                    and self.phase in ('READY','WAIT_PROFILE','FAILED')
                    and self.policy.get('mode')=='policy' and self.policy.get('live_profile_switch')=='1'
                    and self.waypoint_input_ready(now))
        return (mode in NATIVE and self.requested in NATIVE and not self.proc
                and self.phase in ('READY','WAIT_GAIT','WAIT_INPUT_MODE','FAILED')
                and self.native_input_ready(now))
    def select_live(self,mode,now):
        if not self.can_live_select(mode,now):raise ValueError('当前不是已接管的同一控制器')
        if mode in WAYPOINT:
            self.select(mode,now);self.waypoint_continuity=True
        else:
            self.guard(now)
            # Always issue the latest target, including A -> B -> A before
            # feedback for B arrives. Do not mistake old A feedback for success.
            self.prepare_native(mode,now,force_gait=True);self.native_continuity=True
        if self.phase=='WAIT_GAIT':self.detail='已申请原厂步态，等待实际步态更新'
    def native_input_ready(self,now):
        if not self.native_output_active() or self.proc or not self.healthy(now):return False
        if self.ms()!=17:return False
        continuous=getattr(self,'native_continuity',False) and self.gs() in tuple(MODES[m] for m in NATIVE)
        # Direct factory gait changes retain the current input stream. Firmware
        # can reset Mode=0 during a navigation gait change; keep sending the
        # correctly typed velocity command while the Mode=1 request is applied.
        if continuous and self.phase in ('WAIT_GAIT','WAIT_INPUT_MODE'):
            return self.basic.get('ControlUsageMode') in (0,1)
        if self.phase=='WAIT_INPUT_MODE' or self.basic.get('ControlUsageMode')!=self.native_mode():return False
        return self.ready(now) or (getattr(self,'native_continuity',False)
                                  and self.gs() in tuple(MODES[m] for m in NATIVE))
    def live_input_lost(self,now):
        if getattr(self,"control_interrupted",False):return True
        return ((getattr(self,'native_continuity',False) and not self.native_input_ready(now)) or
                (getattr(self,'waypoint_continuity',False) and not self.waypoint_input_ready(now)))
    def waypoint_input_ready(self,now):
        if not self.proc or not self.owned or not self.healthy(now) or self.ms()!=100:return False
        return self.ready(now) or (getattr(self,'waypoint_continuity',False)
            and self.policy.get('mode')=='policy' and self.policy.get('profile') in WAYPOINT)
    def begin_sdk_switch(self,now):
        # Resting fallback. Standing handover uses the prepared protocol instead.
        if self.requested in WAYPOINT:
            self.quiesce_native_input(now);self.phase='WAIT_SDK_QUIET';self.since=now
            self.detail='停止原厂输入通道后再请求 SDK';return
        self.send(0x100005,0x300002,dict(SDKEnable=self.requested in WAYPOINT,Frequency=200))
        self.phase='WAIT_SDK';self.since=now
    def quiesce_native_input(self,now):
        # Gate BOTH the normal sender and its independent zero guardian before
        # SDKEnable. Zero-valued native joystick packets are still operations.
        self.native_axes_inhibited=True;self.guardian.quiesce(now)
    def launch_policy(self,now):
        if self.process_conflict():raise ValueError('其他运控进程仍存在')
        launcher=Path('/home/wym/s10_g12_damping_ws/g12_waypoints.sh')
        if not launcher.is_file():raise ValueError('站立接管策略节点尚未安装')
        logs=Path(__file__).resolve().parent/'logs';logs.mkdir(exist_ok=True)
        self.log=(logs/('policy-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.log')).open('x')
        env=dict(os.environ,S10_G12_HANDOVER='YES')
        self.proc=subprocess.Popen(['bash',str(launcher),self.requested],stdout=self.log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        self.policy={};self.policy_at=-1e9;self.phase='WAIT_PROFILE';self.since=now
    def ready(self,now):
        if self.confirmed=='none' or not self.healthy(now):return False
        if self.confirmed in WAYPOINT:return self.ms()==100 and self.policy.get('profile')==self.confirmed and self.policy.get('mode') in ('hold','policy')
        return self.phase=='READY' and self.ms()==17 and self.gs()==MODES[self.confirmed] and self.basic.get('ControlUsageMode')==self.native_mode()
    def run(self,now):
        if not self.ready(now):raise ValueError(self.not_ready_reason(now))
        if self.confirmed in WAYPOINT and self.policy.get('mode')=='hold':self.command('policy')
    def not_ready_reason(self,now):
        if not self.healthy(now):return '实机反馈/控制所有权未通过：'+str(self.health_reason(now))
        if self.phase=='FAILED':return self.detail
        if self.requested in NATIVE:
            if self.ms()!=17:return '尚未站立；请先点击起立'
            return f'步态未确认：目标 {MODES[self.requested]:#x} / 实际 {self.gs():#x}'
        return '请先选择模式；stand 就绪后自动进入策略控制'
    def posture(self,action,now):
        self.control_interrupted=False
        if action=='stand' and self.stand_from_damping(now):return
        self.guard(now)
        if self.phase.startswith('HANDOVER_'):raise ValueError('站立接管中；保持零输入，等待当前接管反馈')
        if not self.stopped():raise ValueError('尚未静止')
        if self.proc:
            allowed=('disarmed',) if action=='stand' else ('hold','policy')
            if self.policy.get('mode') not in allowed:raise ValueError('策略当前不允许该姿态切换')
            self.command(action)
        elif self.owned and self.requested in NATIVE and self.ms()!=100:
            if self.phase not in ('WAIT_STAND','WAIT_GAIT','WAIT_INPUT_MODE','WAIT_REST','READY','FAILED'):raise ValueError('等待原厂控制接管确认')
            if action=='stand' and self.ms()==17:return
            if action=='stand' and self.stand_sent_at is not None and self.phase=='WAIT_STAND':return
            if action=='lie' and self.ms() in (0,4):
                self.phase='WAIT_STAND';self.confirmed='none';self.stand_sent_at=None;return
            if action=='stand' and self.ms() not in (0,2,4):raise ValueError('起立过程中，请等待反馈')
            if action=='lie' and self.ms()!=17:raise ValueError('姿态变化中，请等待反馈')
            self.confirmed='none';self.gait_sent_at=None;self.gait_reply=None;self.since=now
            self.phase='WAIT_STAND' if action=='stand' else 'WAIT_REST'
            self.stand_sent_at=now if action=='stand' else None
            self.detail='正在起立；完成后自动申请所选步态' if action=='stand' else '正在趴下；已取消待处理步态'
            self.send(0x100001,0x200002,dict(MotionParam=1 if action=='stand' else 4))
        else:raise ValueError('请先切换并确认控制模式')
    def native_output_active(self):
        return not self.native_axes_inhibited and self.owned and self.ms()!=100 and (self.requested in NATIVE or self.handover_source=='native')
    def publish_waypoints(self,axes):
        move,points=waypoint(axes)
        msg=self.WaypointCommand();msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id='base_heading';msg.move=move
        for p,(x,y) in zip(msg.points,points):p.x=x;p.y=y;p.z=0.
        self.wp.publish(msg)
        self.record('waypoint_tx',move=move,points=points,axes=list(axes))
    def tick(self,axes,now):
        for _ in range(8):self.rclpy.spin_once(self.node,timeout_sec=0)
        now=time.monotonic()
        self.reconcile_policy(now)
        self.guardian.pulse(self.native_output_active(),now,self.native_command())
        self.read(now)
        now=time.monotonic()
        if now-self.last_hb>1:self.send(0x100064,5,{});self.last_hb=now
        if now-self.last_audit>.2:self.audit_now(now)
        now=time.monotonic()
        if self.phase in ('RECOVERY_REAP','RECOVERY_SELECT'):
            pass
        elif self.phase=='WAIT_WAKE':
            self.advance_wake(now)
        elif self.phase.startswith('DAMPING'):
            self.advance_damping(now)
        elif self.phase.startswith('HANDOVER_'):
            self.advance_handover(now)
        elif self.phase in NATIVE_PHASES:
            self.advance_native(now)
        elif self.phase not in ('IDLE','READY','FAILED'):
            if now-self.since>20:self.fail_transition('切换超时；不自动重试')
            elif self.phase=='RELEASE_POLICY':
                if self.proc.poll() is not None:
                    self.proc=None
                    if self.log:self.log.close();self.log=None
                    self.phase='WAIT_RELEASE_GRAPH';self.since=now
            elif self.phase=='WAIT_RELEASE_GRAPH':
                if self.audit['ok']:self.begin_sdk_switch(now)
            elif self.phase=='WAIT_SDK_QUIET' and self.healthy(now) and self.guardian.quiesced(now):
                if not self.resting():self.fail_transition('趴下状态变化；未请求 SDK')
                else:
                    self.send(0x100005,0x300002,dict(SDKEnable=True,Frequency=200))
                    self.phase='WAIT_SDK';self.since=now
            elif self.phase=='WAIT_SDK' and now-self.motion_at<.3:
                if self.requested in WAYPOINT:
                    if self.ms()==100:self.launch_policy(now)
                elif self.ms() in (0,4):self.prepare_native(self.requested,now)
            elif self.phase=='WAIT_PROFILE' and self.since<self.policy_at<=now and now-self.policy_at<1.6:
                if self.policy.get('profile')==self.requested and self.policy.get('mode') in ('disarmed','hold','policy','damping'):
                    self.confirmed=self.requested;self.phase='DAMPING' if self.policy.get('mode')=='damping' else 'READY' 
                    self.detail='模型已直接切换；保持策略控制' if self.policy.get('mode')=='policy' else '模型已就绪；stand 完成后自动接管摇杆'
        if now-self.last_send>=.05:
            if self.proc and self.proc.poll() is None:
                self.publish_waypoints(axes if self.waypoint_input_ready(now) else (0.,0.,0.))
            if self.native_output_active():
                x,y,w=axes if self.native_input_ready(now) else (0.,0.,0.)
                self.send(0x100001,self.native_command(),dict(X=x,Y=y,Yaw=w,Z=0.,Roll=0.,Pitch=0.))
            self.last_send=now
    def status(self,now):
        feedback_confirmed=self.confirmed
        if getattr(self,'native_continuity',False) and self.native_input_ready(now):
            feedback_confirmed=next((m for m in NATIVE if MODES[m]==self.gs()),'none')
        if getattr(self,'waypoint_continuity',False) and self.waypoint_input_ready(now):
            feedback_confirmed=self.policy.get('profile','none')
        if not self.healthy(now):feedback_confirmed='none'
        elif feedback_confirmed in WAYPOINT and (self.ms()!=100 or self.policy.get('profile')!=feedback_confirmed):feedback_confirmed='none'
        elif feedback_confirmed in NATIVE and (self.ms()!=17 or self.gs()!=MODES[feedback_confirmed]):feedback_confirmed='none'
        actual_policy=next((m for m in NATIVE if MODES[m]==self.gs()),'none') if self.healthy(now) and self.ms()==17 else feedback_confirmed
        response=getattr(self,'last_gait_response',None)
        response=response if response and self.gait_sent_at is not None and response['mono']>=self.gait_sent_at else None
        return dict(backend=self.backend,confirmed=feedback_confirmed,actual_policy=actual_policy,gait_response=response,
                    phase=self.phase,requested=self.requested,motion_state=self.ms(),
                    live_input_ready=self.native_input_ready(now) or self.waypoint_input_ready(now),
                    health_reason=self.health_reason(now),sleeping=self.basic.get('Sleep'),detail=self.detail,
                    damping=self.damping_feedback(now),damping_available=True,
                    robot=f'状态 {self.ms()}  步态 {self.gs():#x}  策略 {self.policy.get("mode","-")} / {self.policy.get("profile","-")}  {self.detail}  所有权 {self.audit.get("reason") or "ok"}')
    def diagnostics(self,now):
        def age(t):return round((now-t)*1000,2) if math.isfinite(t) and t>0 else None
        return dict(version=VERSION,health=self.health_reason(now) or 'ok',
                    requested=self.requested,owned=self.owned,native_input_kind=getattr(self,'native_input_kind','velocity'),
                    native_command=hex(self.native_command()),
                    state=self.ms(),gait=self.gs(),basic=self.basic,
                    motion_age_ms=age(self.motion_at),basic_age_ms=age(self.basic_at),
                    policy_age_ms=age(self.policy_at),audit_age_ms=age(self.last_audit),
                    timestamp_mode=self.timestamp_mode,rx=dict(self.rx),tx=dict(self.tx),
                    recording=self.flight.status() if getattr(self,'flight',None) else None,last_wire=getattr(self,'last_wire',None),
                    fault_semantics=self.policy.get('fault_semantics','legacy'),
                    control_interrupted=getattr(self,'control_interrupted',False),
                    last_reply=self.last_reply,gait_reply=self.gait_reply,operations=list(self.operations),replies=list(self.replies),
                    handover_source=self.handover_source,handover_token=self.handover_token,
                    handover_motion_states=sorted(self.handover_motion_states),native_axes_inhibited=self.native_axes_inhibited,
                    native_continuity=getattr(self,'native_continuity',False),
                    waypoint_continuity=getattr(self,'waypoint_continuity',False),
                    velocity=None if self.motion is None else dict(x=self.motion.vel_x,y=self.motion.vel_y,yaw=self.motion.vel_yaw),
                    near_zero_limits=dict(linear=NEAR_ZERO_LINEAR,yaw=NEAR_ZERO_YAW),policy=dict(self.policy),dds=self.audit)
    def close(self):
        if self.proc and self.proc.poll() is None:
            # Leave the owned policy running with a final HOLD; its independent
            # command watchdog handles gateway death. Never kill a supporting node.
            # Shutdown must never advance a pending SDK/mode transition.
            self.publish_waypoints((0.,0.,0.))
            if self.policy.get('mode')=='policy':self.command('return')
        elif self.native_output_active():
            for _ in range(3):self.send(0x100001,self.native_command(),dict(X=0.,Y=0.,Yaw=0.,Z=0.,Roll=0.,Pitch=0.))
        try:self.guardian.close();self.node.destroy_node();self.rclpy.shutdown();self.sock.close()
        finally:
            if getattr(self,'flight',None):self.flight.close()
