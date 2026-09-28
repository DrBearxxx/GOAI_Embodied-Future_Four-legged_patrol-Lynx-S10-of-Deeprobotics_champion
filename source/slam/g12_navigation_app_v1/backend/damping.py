"""Explicit unload action. No sockets, ROS, or automatic robot actions on import."""
from core import NATIVE,WAYPOINT

class DampingControl:
    def damping(self,now):
        # Supersede all gateway transactions before sending anything. Do not use
        # the normal health/near-zero gate: damping must remain available after
        # a policy fault, RC failure, or a stalled mode switch.
        self.pause_axes=(0.,0.,0.)
        self.native_continuity=False;self.waypoint_continuity=False
        self.confirmed='none';self.phase='DAMPING_WAIT';self.since=now
        self.gait_sent_at=None;self.gait_reply=None;self.stand_sent_at=None
        self.handover_source=None;self.handover_sdk_at=None;self.handover_token='none'
        self.damping_at=now;self.damping_route='native';self.damping_sent=False
        self.quiesce_native_input(now)
        live=self.proc is not None and self.proc.poll() is None
        if live:
            # This also cancels a prepared/committed handover and queued actor
            # change on the node. Do not kill a node in order to unload it.
            self.command('damping')
        if live and self.ms()==100:
            self.damping_route='waypoint';self.damping_sent=True
        self.detail='阻尼已请求；等待实际反馈。阻尼会卸力，不保持站姿'

    def damping_feedback(self,now):
        after=getattr(self,'damping_at',float('inf'))
        if not 0<=now-self.motion_at<.3 or self.motion_at<=after:return 'none'
        if self.ms()==2:return 'native'
        if (self.ms()==100 and self.proc and self.proc.poll() is None
                and 0<=now-self.policy_at<1.6 and self.policy_at>after
                and self.policy.get('mode')=='damping' and self.policy.get('command_publisher')=='1'
                and self.policy.get('frozen')=='1'):
            return 'waypoint'
        return 'none'

    def advance_damping(self,now):
        if self.phase=='DAMPING_RESET':
            if (self.proc and self.proc.poll() is None and self.policy_at>self.since
                    and 0<=now-self.policy_at<1.6 and self.policy.get('mode')=='disarmed'
                    and self.policy.get('fault')=='0' and self.policy.get('frozen')=='0'
                    and self.ms()==100 and self.healthy(now)):
                # Only an explicit Stand action enters this phase. Reset itself
                # never arms; do not re-use a late reset ACK after Stop/reconnect.
                self.command('stand');self.confirmed=self.requested
                self.phase='READY';self.detail='已解除阻尼；按起立指令执行，完成后接管摇杆'
            elif now-self.since>5:
                self.phase='DAMPING_RESET_FAILED';self.detail='解除阻尼未确认；未发送起立，请检查策略预检'
            return
        if self.phase=='DAMPING_WAIT':
            if not self.damping_sent and self.guardian.quiesced(now):
                # Original APK: motionDampingMsg() -> MOTION_MSG_ID, param 2.
                # No SDK toggle, stand, lie, gait, or reset command accompanies it.
                self.send(0x100001,0x200002,dict(MotionParam=2));self.damping_sent=True
            if self.damping_sent and self.damping_feedback(now)!='none':
                self.phase='DAMPING';self.detail='阻尼已生效；摇杆锁定，点击起立才能重新起控'
            elif now-self.since>5:
                self.phase='DAMPING_UNCONFIRMED';self.detail='阻尼未确认；未自动重发。必要时使用机器人硬急停'
        elif self.phase=='DAMPING_UNCONFIRMED' and self.damping_sent and self.damping_feedback(now)!='none':
            self.phase='DAMPING';self.detail='已收到阻尼反馈；运动仍锁定'

    def stand_from_damping(self,now):
        if not self.phase.startswith('DAMPING') and not (self.proc and self.policy.get('mode')=='damping'):return False
        if self.proc:
            reason=self.health_reason(now,check_policy=False)
            if reason:raise ValueError('起立前检查失败：'+reason)
            if (self.proc.poll() is not None or not 0<=now-self.policy_at<1.6 or self.ms()!=100
                    or self.policy.get('mode') not in ('damping','disarmed')):
                raise ValueError('策略未处于可恢复的 SDK 阻尼状态；未发送起立')
            self.requested=self.policy.get('profile',self.requested)
            if self.requested not in WAYPOINT:raise ValueError('未确认 waypoint 模型')
            self.command('reset');self.phase='DAMPING_RESET';self.since=now
            self.detail='按起立指令解除策略阻尼；等待节点预检后起立';return True
        self.guard(now)
        if self.ms() not in (0,2,4):raise ValueError('尚未收到可起立的原厂状态')
        if self.requested not in NATIVE:self.requested='basic'
        self.owned=True;self.native_axes_inhibited=False
        self.phase='WAIT_STAND';self.stand_sent_at=None
        return False  # continue the regular native Stand path
