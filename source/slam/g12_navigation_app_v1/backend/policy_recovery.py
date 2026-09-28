"""Current policy conditions and retirement of a node that released ownership.

No robot transport on import. Clearing an alarm never issues stand/run/SDKEnable.
"""
import os
import signal

class PolicyRecovery:
    def policy_feedback(self,text,now):
        self.policy=dict(x.split('=',1) for x in text.split() if '=' in x)
        self.policy_at=now
        signature=tuple(self.policy.get(k) for k in
            ('fault','frozen','fault_event_seq','mode','native_released'))
        changed=signature!=getattr(self,'last_policy_trace_signature',None)
        if changed or now-getattr(self,'last_policy_trace',-1e9)>=1.:
            self.record('policy_feedback',feedback_mono=now,policy=dict(self.policy))
            self.last_policy_trace=now;self.last_policy_trace_signature=signature

    def reconcile_policy(self,now):
        if getattr(self,'recovery_reaping',False):
            if self.proc and self.proc.poll() is None:return
            if self.log:self.log.close();self.log=None
            self.proc=None;self.policy={};self.policy_at=-1e9
            self.recovery_reaping=False;self.phase='IDLE'
            self.detail='底盘已恢复原厂控制；旧策略节点已退出，可直接选择模式'
            self.record('policy_retired',reason='native_owner')
            return
        if not self.proc or not 0<=now-self.policy_at<1.6:return
        # Only this node's explicit release acknowledgment permits process exit.
        # Expected tokenized handovers continue through their existing protocol.
        if (not self.phase.startswith('HANDOVER_') and self.policy.get('native_released')=='1'
                and self.policy.get('mode')=='disarmed' and self.policy.get('command_publisher')=='0'
                and 0<=now-self.motion_at<.3 and self.ms() in (0,2,4,17)):
            self.record('policy_retiring',reason='native_owner',policy=dict(self.policy))
            self.pause_axes=(0.,0.,0.);self.native_continuity=False;self.waypoint_continuity=False
            self.owned=False;self.confirmed='none';self.requested='none'
            self.control_interrupted=True
            self.handover_source=None;self.handover_sdk_at=None
            self.quiesce_native_input(now)
            if self.proc.poll() is None:
                try:os.killpg(self.proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
            self.recovery_reaping=True;self.phase='RECOVERY_REAP'
            self.detail='底盘已退出 SDK；清理已释放输出的旧策略节点'
            return
        if self.phase=='RECOVERY_SELECT':
            if self.policy_at>self.since and self.policy.get('mode')=='disarmed' and self.policy.get('fault')=='0' and self.policy.get('frozen')=='0':
                target=self.recovery_target;self.phase='DAMPING'
                self.select(target,now)
            elif now-self.since>5:self.fail_transition('当前异常尚未恢复；模式选择未执行，可在恢复后重新选择')
            return
        if self.ms()!=100 or self.policy.get('mode')!='damping':return
        if self.phase in ('DAMPING_RESET','DAMPING_RESET_FAILED','WAIT_PROFILE'):return
        self.native_continuity=False;self.waypoint_continuity=False
        self.control_interrupted=True;self.confirmed='none';self.phase='DAMPING'
        if self.policy.get('fault')=='1':
            self.detail='当前异常：'+self.policy.get('fault_reason','unknown')
        elif self.policy.get('frozen')=='1':
            self.detail='人工阻尼已生效；可选择模型或点击起立'
        else:
            self.detail='当前异常已清除；保持停止，可选择模型或点击起立'
