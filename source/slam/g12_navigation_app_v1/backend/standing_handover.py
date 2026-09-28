"""Tokenized, feedback-driven standing handover; never kills a supporting node."""
import secrets,signal,os

PREFIX='HANDOVER_'
class StandingHandover:
    def init_handover(self):
        self.handover_source=None;self.handover_token='none';self.handover_sdk_at=None
        self.native_axes_inhibited=False;self.handover_motion_states=set()
    def handover_phase(self,phase,now,detail):
        self.phase=PREFIX+phase;self.since=now;self.detail=detail
    def begin_standing_waypoint(self,mode,now):
        self.requested=mode;self.confirmed='none';self.owned=True
        self.handover_source='native';self.handover_token=secrets.token_hex(12);self.handover_sdk_at=None
        self.handover_motion_states=set()
        if not self.proc:self.launch_policy(now)
        self.handover_phase('PRELOAD',now,'原厂保持站立；预加载 waypoint，不退出原厂控制')
    def begin_standing_native(self,mode,now):
        if self.policy.get('gateway_handover')!='1':raise ValueError('当前策略没有站立接管接口，请使用新版 G12 策略节点')
        self.requested=mode;self.confirmed='none';self.owned=True
        self.handover_source='waypoint';self.handover_sdk_at=None
        self.handover_motion_states=set()
        self.handover_token=secrets.token_hex(12)
        if self.policy.get('handover')=='await_native':
            self.command('handover_cancel_release:'+self.policy['handover_token'])
            self.handover_phase('CANCEL_RELEASE',now,'确认仍由 SDK 保持站姿，再重新申请原厂接管')
        else:
            if self.policy.get('mode')=='policy':self.command('return')
            self.handover_phase('RETURN',now,'waypoint 回到标准站姿并持续保持；不卸力')
    def handover_policy(self,now):
        return (self.proc and self.proc.poll() is None and 0<=now-self.policy_at<1.6
                and self.policy.get('fault')=='0' and self.policy.get('frozen')=='0')
    def handover_matches(self,phase,now):
        return (self.handover_policy(now) and self.policy_at>self.since and
                self.policy.get('handover_token')==self.handover_token and self.policy.get('handover')==phase)
    def advance_handover(self,now):
        phase=self.phase.removeprefix(PREFIX)
        limit={'PRELOAD':25,'PROFILE':5,'PREPARE':5,'COMMIT':5,'QUIESCE':2,'SDK':7,'HOLD':12,
               'RETURN':12,'RELEASE':5,'NATIVE':7,'REAP':5,'GRAPH':5,'CANCEL_RELEASE':5}[phase]
        if now-self.since>limit:
            self.fail_transition('站立接管超时（'+phase+'）；实际状态='+str(self.ms())+
                '，策略接管='+self.policy.get('handover','unknown')+'，预检='+self.policy.get('preflight','unknown')+
                '；不强停支撑策略、不自动重试');return
        if phase=='REAP':
            if self.proc.poll() is not None:
                self.proc=None;self.policy={};self.policy_at=-1e9
                if self.log:self.log.close();self.log=None
                self.handover_phase('GRAPH',now,'原厂持续保持站立；等待旧发布者退出发现列表')
            return
        # PRELOAD/REAP intentionally have no fresh child status yet. All robot
        # feedback and exclusivity checks remain required before new operations.
        if self.health_reason(now,check_policy=False) is not None:return
        valid=self.handover_policy(now)
        if phase in ('SDK','NATIVE') and self.handover_sdk_at is not None:
            rejected=next((r for r in reversed(self.replies) if r['type'] in (0x100005,0x300005)
                and r['command']==0x300002 and r['mono']>=self.handover_sdk_at and r['error_code']!=0),None)
            if rejected:
                self.fail_transition('SDK 交接请求被拒绝 ErrorCode='+str(rejected['error_code'])+'；保留旧控制器');return
        if phase=='PRELOAD':
            if (valid and self.policy.get('gateway_handover')=='1' and self.policy.get('handover_prepare_ready')=='1'
                    and self.policy.get('mode')=='disarmed' and self.policy.get('handover') in ('idle','failed','released')
                    and self.ms()==17 and self.stopped()):
                if self.policy.get('profile')!=self.requested:
                    self.command('cycle_policy');self.handover_phase('PROFILE',now,'预加载节点切换目标模型');return
                self.command('handover_prepare:'+self.handover_token)
                self.handover_phase('PREPARE',now,'模型已加载；建立待命发布者，尚无电机输出')
        elif phase=='PROFILE' and valid and self.policy_at>self.since and self.policy.get('profile')==self.requested:
            self.handover_phase('PRELOAD',now,'目标模型已加载，继续准备接管')
        elif phase=='PREPARE' and self.handover_matches('prepared',now):
            if self.ms()!=17 or not self.stopped():self.fail_transition('原厂站姿变化；未切入 SDK');return
            self.command('handover_commit:'+self.handover_token)
            self.handover_phase('COMMIT',now,'准备就绪；等待节点确认接管门控')
        elif phase=='COMMIT' and self.handover_matches('await_sdk',now):
            if self.ms()!=17 or not self.stopped():self.fail_transition('原厂站姿变化；未切入 SDK');return
            self.quiesce_native_input(now)
            self.handover_phase('QUIESCE',now,'已停止原厂摇杆发送；等待零轴看门狗退出原厂通道')
        elif phase=='QUIESCE' and self.guardian.quiesced(now):
            if not self.handover_matches('await_sdk',now):return
            if self.ms()!=17 or not self.stopped():self.fail_transition('速度或原厂状态变化；未切入 SDK');return
            self.send(0x100005,0x300002,dict(SDKEnable=True,Frequency=200));self.handover_sdk_at=now
            self.handover_phase('SDK',now,'已请求 SDK；待命节点收到新 SDK 状态后立即保持实测站姿')
        elif phase=='SDK' and valid and self.policy.get('handover')=='active' and self.policy.get('handover_token')==self.handover_token and self.ms()==100 and self.motion_at>self.handover_sdk_at:
            self.handover_phase('HOLD',now,'SDK 已接管；平滑过渡到 waypoint 标准站姿')
        elif phase=='HOLD' and valid and self.policy.get('mode')=='hold' and self.stopped() and self.ms()==100:
            self.confirmed=self.requested;self.phase='READY';self.handover_source=None
            self.detail='站立接管已确认；自动进入策略并接管摇杆'
        elif phase=='CANCEL_RELEASE' and valid and self.policy.get('handover')=='active':
            self.handover_phase('RETURN',now,'SDK 仍保持站姿；准备重新申请原厂接管')
        elif phase=='RETURN' and valid and self.policy.get('mode')=='hold' and self.ms()==100 and self.stopped():
            self.command('handover_release:'+self.handover_token)
            self.handover_phase('RELEASE',now,'保持 waypoint 站姿；准备等待原厂实际接管')
        elif phase=='RELEASE' and self.handover_matches('await_native',now):
            self.send(0x100005,0x300002,dict(SDKEnable=False,Frequency=200));self.handover_sdk_at=now
            self.handover_phase('NATIVE',now,'已请求原厂接管；不先停策略、不发送阻尼')
        elif phase=='NATIVE' and self.handover_matches('released',now) and self.ms()==17 and self.motion_at>self.handover_sdk_at:
            # The child's release acknowledgment guarantees no supporting output
            # and no shutdown damping. Only now may its process be reaped.
            if self.policy.get('mode')!='disarmed' or self.policy.get('command_publisher')!='0':return
            os.killpg(self.proc.pid,signal.SIGINT)
            self.handover_source='native';self.handover_phase('REAP',now,'原厂站立已确认；清理已释放的策略进程')
        elif phase=='GRAPH':
            self.handover_source=None;self.prepare_native(self.requested,now)
