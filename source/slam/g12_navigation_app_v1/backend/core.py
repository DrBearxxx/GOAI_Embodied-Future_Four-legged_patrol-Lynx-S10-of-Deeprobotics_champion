"""Single-owner control state. No robot I/O in this module."""
import math
MODES={'low':None,'high':None,'low_v5':None,'high_v2':None,'basic':0x3002,'stairs':0x3003,'platform':0x1002,
       'basic_normal':0x1001,'stairs_normal':0x1003,'step_move':0xF002}
WAYPOINT=tuple(name for name,gait in MODES.items() if gait is None)
NATIVE=tuple(name for name,gait in MODES.items() if gait is not None)
DISTANCES=(.5,1.,2.,3.5,5.)

def waypoint(axes):
    x,y,turn=axes
    if not any(axes): return False,[(0.,0.)]*5
    heading=(math.atan2(y,x) if x or y else 0.)+.35*turn
    c=.4*turn
    points=[((math.sin(heading+c*s)-math.sin(heading))/c,
             (math.cos(heading)-math.cos(heading+c*s))/c) if abs(c)>1e-6 else
            (s*math.cos(heading),s*math.sin(heading)) for s in DISTANCES]
    return True,points

class Controller:
    def __init__(self,adapter):
        self.adapter=adapter;self.enabled=False;self.axes=(0.,0.,0.)
        self.last_input=-1e9;self.last_seq=-1;self.reason='等待配对；运动未解锁'
        self.session=None;self.auto_pending=False
    def stop(self,reason):
        self.enabled=False;self.auto_pending=False;self.axes=(0.,0.,0.);self.reason=reason
        self.adapter.pause()
    def new_session(self,session):
        if session!=self.session:
            self.stop('新连接；松杆后选择模式或恢复控制');self.session=session;self.last_seq=-1
    def accept(self,request,now):
        axes=request.get('axes');seq=request.get('sample_seq');age=request.get('sample_age_ms')
        if not isinstance(axes,list) or len(axes)!=3 or not all(type(v) in (int,float) and math.isfinite(v) and abs(v)<=1 for v in axes):
            raise ValueError('非法摇杆输入')
        if type(seq)!=int or type(age) not in (int,float) or not math.isfinite(age):raise ValueError('非法输入序号')
        action=request.get('action','')
        if not isinstance(action,str):raise ValueError('非法操作')
        # Explicit touchscreen unload intent is independent of RC freshness and
        # centering. The server still requires an authenticated one-shot ticket.
        if action=='damping':
            self.stop('已请求阻尼；等待实际反馈，不自动恢复运动')
            self.adapter.damping(now);return
        fresh=request.get('fresh') is True and 0<=age<=200 and seq>=self.last_seq
        if request.get('stop') is True or action=='pause':
            self.stop('已停止；松杆后选择模式或恢复控制');return
        if not fresh:
            self.stop('摇杆数据过期 / 重复；运动已锁定');return
        if seq==self.last_seq:
            if tuple(axes)!=self.axes or now-self.last_input>.20:
                self.stop('摇杆样本未更新；运动锁定');return
        else:
            self.last_seq=seq;self.last_input=now;self.axes=tuple(axes)
        if action.startswith('select:'):
            mode=action[7:]
            if mode not in MODES:raise ValueError('未知模式')
            # Factory gait selection does not transfer controller ownership.
            # Keep an already-live operator stream through its async feedback.
            live=getattr(self.adapter,'can_live_select',lambda mode,now:False)
            if self.enabled and live(mode,now):
                self.adapter.select_live(mode,now)
                self.auto_pending=False
                self.reason=('waypoint 模型直接切换；保持 HOLD/MOVE，不回 stand' if mode in WAYPOINT else
                             '原厂步态切换；摇杆保持响应，颜色按实际反馈更新')
                return
            if any(axes):raise ValueError('先松杆，再切换模式')
            self.stop('切换模式；自动暂停输入')
            self.adapter.select(mode,now)
            self.auto_pending=self.adapter.backend!='monitor'
            self.reason='正在切换；stand 就绪后自动启用摇杆，无需另按启用'
        elif action=='run':
            if any(axes):raise ValueError('启用前必须松杆')
            if not self.adapter.ready(now):
                explain=getattr(self.adapter,'not_ready_reason',None)
                raise ValueError(explain(now) if explain else '实机状态未确认或尚未完成起立')
            self.adapter.run(now);self.auto_pending=False;self.enabled=True;self.reason='摇杆已启用；松杆 HOLD'
        elif action in ('stand','lie'):
            if any(axes):raise ValueError('先松杆，再进行姿态操作')
            self.stop('姿态操作；自动暂停输入')
            self.adapter.posture(action,now)
            self.auto_pending=action=='stand' and self.adapter.backend!='monitor'
            self.reason='正在起立；完成后自动进入控制' if action=='stand' else '正在趴下；运动锁定'
        elif action:raise ValueError('未知操作')
    def tick(self,now):
        if (self.enabled or self.auto_pending) and now-self.last_input>.40:self.stop('控制链路超时；零指令并锁定')
        if self.enabled and not self.adapter.healthy(now):self.stop('实机状态失效；零指令并锁定')
        if self.enabled and getattr(self.adapter,'live_input_lost',lambda now:False)(now):
            self.stop('当前控制状态已离开；零指令，松杆后重新选择或恢复控制')
        # Intent belongs to this explicit selection/stand operation only, never
        # a reconnect or process restart. Evaluate last tick's feedback before
        # adapter.tick receives newer timestamps than this `now` snapshot.
        if self.auto_pending:
            if getattr(self.adapter,'phase',None) in ('FAILED','DAMPING_RESET_FAILED'):
                self.stop(getattr(self.adapter,'detail','切换失败；运动锁定'))
            elif self.adapter.healthy(now) and self.adapter.ready(now):
                if any(self.axes):self.reason='已就绪；松杆后自动接管摇杆'
                elif 0<=now-self.last_input<=.20:
                    self.adapter.run(now);self.auto_pending=False;self.enabled=True
                    self.reason='已自动进入控制；摇杆已启用，松杆 HOLD'
        self.adapter.tick(self.axes if self.enabled else (0.,0.,0.),now)
    def status(self,now):
        return dict(self.adapter.status(now),enabled=self.enabled,auto_pending=self.auto_pending,reason=self.reason)

class Monitor:
    backend='monitor'
    def __init__(self):self.selected='none'
    def pause(self):pass
    def select(self,mode,now):self.selected=mode
    def ready(self,now):return False
    def healthy(self,now):return False
    def run(self,now):raise ValueError('只读后端')
    def posture(self,action,now):raise ValueError('只读后端：不发送机器人指令')
    def damping(self,now):raise ValueError('只读后端：不发送阻尼指令')
    def wake(self,now):raise ValueError('只读后端：不发送唤醒指令')
    def tick(self,axes,now):pass
    def status(self,now):return dict(backend=self.backend,confirmed='none',phase='MONITOR',robot='选中 '+self.selected+'；仅检测输入，不切换实机')

class Simulator(Monitor):
    backend='simulation'
    def __init__(self):super().__init__();self.standing=False;self.output=(0.,0.,0.);self.damped=False
    def ready(self,now):return self.selected!='none' and self.standing
    def healthy(self,now):return True
    def run(self,now):pass
    def wake(self,now):pass
    def posture(self,action,now):self.standing=action=='stand';self.damped=False
    def damping(self,now):self.standing=False;self.output=(0.,0.,0.);self.damped=True
    def tick(self,axes,now):self.output=axes
    def status(self,now):return dict(backend=self.backend,confirmed=self.selected,phase='SIMULATION',robot='协议模拟器（非运动仿真）；不会连接机器人')
