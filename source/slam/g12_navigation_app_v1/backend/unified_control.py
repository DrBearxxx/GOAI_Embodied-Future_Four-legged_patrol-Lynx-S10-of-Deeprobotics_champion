"""Single robot writer; navigation uses m/s, manual control uses original RC axes."""
import math
from core import MODES, NATIVE, WAYPOINT, Monitor as BaseMonitor, Simulator as BaseSimulator

OFFICIAL = ('basic', 'stairs', 'platform')
POLICY_SWITCH_PAUSE_S = .5


def validate_velocity(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 3 or
        any(type(x) not in (int, float) or not math.isfinite(x) for x in values)):
        raise ValueError('INVALID_VELOCITY')
    return tuple(values)


def validate_axes(values):
    if (not isinstance(values,(list,tuple)) or len(values)!=3 or
        any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in values)):
        raise ValueError('INVALID_RC_AXES')
    return tuple(values)


class Controller:
    def __init__(self, adapter):
        self.adapter=adapter; self.enabled=False; self.axes=(0.,0.,0.)
        self.last_input=-1e9; self.last_seq=-1; self.session=None
        self.reason='运动锁定'; self.auto_pending=False; self.input_kind='velocity'
        self.native_input_kind='velocity'
        self.policy_switch=None; self.policy_switch_error=None; self.output=(0.,0.,0.)

    def record_switch(self,event,**fields):
        record=getattr(self.adapter,'record',None)
        if record:record(event,**fields)

    def stop(self, reason):
        if self.policy_switch:
            self.record_switch('policy_switch_cancelled',policy=self.policy_switch['policy'],reason=reason)
            self.policy_switch_error=reason
        self.policy_switch=None;self.output=(0.,0.,0.)
        self.enabled=False; self.axes=(0.,0.,0.); self.reason=reason
        self.adapter.pause()

    def new_session(self, session):
        if session != self.session:
            self.stop('新会话；需显式恢复'); self.session=session; self.last_seq=-1

    def accept(self, request, now):
        action=request.get('action','')
        if request.get('stop') is True or action=='pause':
            self.stop('已暂停'); return
        if action=='damping':
            self.stop('已请求阻尼'); self.adapter.damping(now); return
        if action=='wake':
            self.stop('已请求唤醒；运动保持锁定'); self.adapter.wake(now); return
        seq=request.get('sample_seq'); age=request.get('sample_age_ms')
        if (type(seq) is not int or seq<=self.last_seq or type(age) not in (float,int)
            or not math.isfinite(age) or not 0<=age<=200 or request.get('fresh') is not True):
            self.stop('输入过期或重复'); return
        values=request.get('axes'); kind=request.get('input_kind')
        if kind=='velocity': values=validate_velocity(values)
        elif kind in ('waypoint_axes','native_axes'):values=validate_axes(values)
        else: raise ValueError('INPUT_KIND_REQUIRED')
        self.last_seq=seq; self.last_input=now; self.input_kind=kind
        requested=getattr(self.adapter,'requested',getattr(self.adapter,'selected','none'))
        if action.startswith('select:'):
            mode=action[7:]
            if mode not in MODES: raise ValueError('UNKNOWN_POLICY')
            if mode in NATIVE:
                if kind not in ('velocity','native_axes'):raise ValueError('NATIVE_INPUT_KIND_REQUIRED')
                if kind!=self.native_input_kind:
                    self.stop('切换导航 / 人工输入')
                    setter=getattr(self.adapter,'set_native_input_kind',None)
                    if setter:setter(kind,now)
                    self.native_input_kind=kind
            live=getattr(self.adapter,'can_live_select',lambda m,t:False)
            # The single writer sends zero for a short dwell BEFORE selecting.
            # Keep the stream armed for same-controller switches, but gate its
            # output until the existing adapter feedback says the target is ready.
            previous=self.policy_switch
            moving_switch=self.enabled and live(mode,now)
            if previous or moving_switch or (requested in MODES and mode!=requested):
                if not moving_switch:self.stop('策略切换中')
                self.policy_switch_error=None
                zero_since=previous['zero_since'] if previous and previous['stage']=='pause' else None
                self.policy_switch=dict(policy=mode,input_kind=kind,stage='pause',zero_since=zero_since)
                self.output=(0.,0.,0.);self.reason='原地暂停 0.5 秒后切换策略'
                self.record_switch('policy_switch_pause',policy=mode,input_kind=kind,pause_s=POLICY_SWITCH_PAUSE_S)
                return
            else:
                self.stop('策略切换中');self.policy_switch_error=None; self.adapter.select(mode,now)
            self.reason='已请求策略；等待实机反馈'; return
        if action in ('stand','lie'):
            self.stop('姿态操作'); self.adapter.posture(action,now); return
        if action=='run':
            if any(values): raise ValueError('RESUME_REQUIRES_ZERO_INPUT')
            if self.policy_switch or not self.adapter.ready(now): raise ValueError('POLICY_NOT_CONFIRMED')
            self.adapter.run(now); self.enabled=True; self.reason='控制已启用'
        elif action: raise ValueError('UNKNOWN_ACTION')
        if self.enabled:
            expected=self.native_input_kind if requested in NATIVE else 'waypoint_axes'
            if kind!=expected: self.stop('策略与输入单位不匹配'); return
            self.axes=tuple(values)

    def tick(self, now):
        if self.enabled and now-self.last_input>.35: self.stop('输入超时；已锁定')
        if self.enabled and not self.adapter.healthy(now): self.stop('实机反馈失效')
        if self.enabled and getattr(self.adapter,'live_input_lost',lambda t:False)(now):
            self.stop('控制状态已离开')
        switch=self.policy_switch
        # Evaluate feedback before tick can receive timestamps newer than now.
        if switch and switch['stage']=='feedback' and self.adapter.ready(now):
            self.record_switch('policy_switch_completed',policy=switch['policy'],enabled=self.enabled)
            self.policy_switch=None;switch=None
            self.reason='控制已启用' if self.enabled else '策略已就绪'
        if switch and switch['stage']=='pause' and switch['zero_since'] is not None and now-switch['zero_since']>=POLICY_SWITCH_PAUSE_S:
            mode=switch['policy']
            live=getattr(self.adapter,'can_live_select',lambda m,t:False)
            if self.enabled and live(mode,now):self.adapter.select_live(mode,now)
            else:self.adapter.select(mode,now)
            switch['stage']='feedback'
            self.reason='原地切换策略，完成后自动继续'
            self.record_switch('policy_switch_requested',policy=mode,zero_duration_s=now-switch['zero_since'])
        self.output=self.axes if self.enabled and not switch else (0.,0.,0.)
        self.adapter.tick(self.output,now)
        if switch and switch['stage']=='pause' and switch['zero_since'] is None:
            # Start the dwell only after a zero tick reached the adapter.
            switch['zero_since']=now
        if switch and switch['stage']=='feedback':
            if getattr(self.adapter,'phase','')=='FAILED':
                self.stop(getattr(self.adapter,'detail','策略切换失败'))

    def status(self, now):
        switch=self.policy_switch
        requested=switch['policy'] if switch else getattr(self.adapter,'requested',getattr(self.adapter,'selected','none'))
        kind='waypoint_axes' if requested in WAYPOINT else self.native_input_kind
        state=dict(self.adapter.status(now), enabled=self.enabled, ready=not switch and self.adapter.ready(now),
                    healthy=self.adapter.healthy(now), reason=self.reason,
                    requested=requested,policy_switch_paused=bool(switch),
                    policy_switch_error=self.policy_switch_error,
                    policy_switch=None if not switch else dict(switch),policy_switch_pause_s=POLICY_SWITCH_PAUSE_S,
                    input_kind=kind, output_units='normalized' if kind!='velocity' else 'm/s,m/s,rad/s',
                    output=list(self.output))
        if switch:
            state['detail']=self.reason if switch['stage']=='pause' else '原地切换；'+state.get('detail','')
            if switch['stage']=='pause':state['phase']='WAIT_SWITCH_PAUSE'
        return state


class Monitor(BaseMonitor):
    pass


class Simulator(BaseSimulator):
    def status(self, now):
        return dict(super().status(now), confirmed=self.selected if self.ready(now) else 'none',
                    requested=self.selected, velocity={'x':self.output[0],'y':self.output[1],'yaw':self.output[2]})
