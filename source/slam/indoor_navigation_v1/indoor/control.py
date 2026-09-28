"""Deterministic, fail-closed 10 m trial controller. No ROS or hardware side effects."""
import math
import numpy as np

def zero(reason,**extra):return dict(vx=0.,vy=0.,wz=0.,state=reason,**extra)

class Guard:
    def __init__(self,route):
        self.route=route;self.xyz=np.array([p['xyz'] for p in route['waypoints']],float)
        if len(self.xyz)<2 or not np.isfinite(self.xyz).all():raise ValueError('invalid_route')
        self.target=0;self.armed=False;self.complete=False;self.reason='DISARMED';self.good_since=None
        self.generation=None;self.last_pose=None;self.last_seq=-1;self.deadline=-1.;self.last_tick=None
        self.last_command=zero('DISARMED');self.latest={};self.reached=[]

    def stop(self,reason='OPERATOR_STOP'):
        self.armed=False;self.reason=reason;self.deadline=-1.;self.last_command=zero(reason)

    def renew_deadman(self,now):
        if self.armed:self.deadline=now+.4

    def backend_check(self,now,feedback,env):
        """Native SDK contract. Alternative backends must check real feedback."""
        if not feedback or not -.02<=now-feedback['mono']<=.3:return 'SDK_FEEDBACK_STALE'
        if feedback['state']!=17 or feedback['gait']!=0x3002:return 'SDK_NOT_FLAT_RL'
        if feedback['charge']!=0 or feedback['charge_error']!=0 or not -.02<=now-feedback['charge_mono']<=1.5:return 'CHARGE_NOT_IDLE'
        if not math.isfinite(feedback['speed']) or feedback['speed']>.5:return 'UNEXPECTED_BODY_SPEED'
        if not env or not -.02<=now-env['mono']<=1.5:return 'OWNERSHIP_CHECK_STALE'
        if env['other_nav_publishers']!=0:return 'COMPETING_NAV_PUBLISHERS'
        if env['rl_service_active'] or env['other_named_joint_publishers']:return 'COMPETING_JOINT_CONTROLLER'
        if env['nav_subscribers']<1:return 'NO_SDK_RECEIVER'
        return None

    def _corridor(self,p):
        if self.target==0:return float(np.linalg.norm(p-self.xyz[0])),float(abs(p[2]-self.xyz[0,2]))
        a,b=self.xyz[self.target-1:self.target+1];v=b-a
        u=float(np.clip((p-a)@v/max(v@v,1e-10),0.,1.));q=a+u*v
        return float(np.linalg.norm((p-q)[:2])),float(abs(p[2]-q[2]))

    def localization_check(self,now,snapshot):
        h=snapshot['health'];age=h['age_s'];mono=snapshot['mono']
        if age is None or not math.isfinite(age) or age<0 or not h['valid'] or age+max(0.,now-mono)>.35:return 'LOCALIZATION_LOST'
        if h['single_lidar'] or set(h['healthy_lidars'])!={'front','rear'}:return 'DUAL_LIDAR_REQUIRED'
        if not snapshot['front_fresh'] or not snapshot['rear_fresh']:return 'PERCEPTION_STALE'
        if snapshot['obstacle']:return 'OBSTACLE_STOP'
        return None

    def goal_reached(self,snapshot,distance):return distance<=.18
    def velocity_limits(self,snapshot):return .10,.20

    def check(self,now,snapshot,feedback,env):
        try:
            if not snapshot:return 'NO_LOCALIZATION'
            mono=float(snapshot['mono']);seq=int(snapshot['seq']);p=np.array(snapshot['pose'],float)
            if p.shape!=(4,) or not np.isfinite(p).all() or not math.isfinite(mono):return 'INVALID_POSE'
            if not -.02<=now-mono<=.25:return 'STALE_LOCALIZER'
            local_reason=self.localization_check(now,snapshot)
            if local_reason:return local_reason
            if snapshot.get('error'):return 'LOCALIZER_EXCEPTION'
            backend_reason=self.backend_check(now,feedback,env)
            if backend_reason:return backend_reason
            if seq<self.last_seq:return 'LOCALIZER_RESTART'
            if seq!=self.last_seq:
                if self.last_pose is not None and self.armed:
                    old_t,old_p=self.last_pose;dt=mono-old_t
                    angle=abs(math.atan2(math.sin(p[3]-old_p[3]),math.cos(p[3]-old_p[3])))
                    if dt<=0 or np.linalg.norm(p[:3]-old_p[:3])>.15+.35*dt or angle>.20+.4*dt:return 'POSE_JUMP'
                self.last_pose=(mono,p.copy());self.last_seq=seq
            if self.armed and snapshot['generation']!=self.generation:return 'RELOCALIZATION_REQUIRES_REARM'
            cross,height=self._corridor(p[:3])
            if cross>(.25 if self.target==0 else .45) or height>.22:return 'OUTSIDE_CURRENT_ROUTE_SEGMENT'
            if self.target and self.route['edges'][self.target-1]['warnings']:return 'ROUTE_REQUIRES_REVIEW'
            return None
        except (KeyError,TypeError,ValueError,OverflowError):return 'INVALID_INPUT'

    def tick(self,now,snapshot,feedback,env):
        reason=self.check(now,snapshot,feedback,env)
        self.latest=dict(snapshot=snapshot,feedback=feedback,env=env)
        if reason:
            self.good_since=None
            if self.armed:self.stop(reason)
            self.reason=reason
            return zero(reason,target=self.target,armed=False,complete=self.complete)
        if self.good_since is None:self.good_since=now
        if self.complete:return zero('COMPLETE',target=self.target,armed=False,complete=True)
        if not self.armed:return zero(self.reason,target=self.target,armed=False,ready=now-self.good_since>=2.)
        if now>self.deadline:
            self.stop('DEADMAN_EXPIRED');return zero(self.reason,target=self.target,armed=False)
        p=np.array(snapshot['pose']);goal=self.xyz[self.target];dist=np.linalg.norm(p[:3]-goal)
        if self.goal_reached(snapshot,dist):
            self.reached.append(self.target)
            if self.target==len(self.xyz)-1:
                self.complete=True;self.stop('COMPLETE');return zero('COMPLETE',target=self.target,armed=False,complete=True)
            self.target+=1;return zero('NEXT_POINT',target=self.target,armed=True)
        d=goal[:2]-p[:2];angle=math.atan2(math.sin(math.atan2(d[1],d[0])-p[3]),math.cos(math.atan2(d[1],d[0])-p[3]))
        # No sidestepping/backward travel. Face the next point; stop translation
        # for a sharp turn. Safety stops bypass acceleration limiting.
        max_vx,max_wz=self.velocity_limits(snapshot)
        vx=min(max_vx,dist*.35)*max(0.,math.cos(angle)) if abs(angle)<.55 else 0.
        wz=float(np.clip(angle,-max_wz,max_wz));dt=.1 if self.last_tick is None else min(.1,max(0.,now-self.last_tick))
        vx=min(vx,self.last_command['vx']+.1*dt)
        self.last_tick=now;self.last_command=dict(vx=float(vx),vy=0.,wz=wz,state='FOLLOW',target=self.target,armed=True)
        return self.last_command

    def arm(self,now):
        if self.complete:return False,'COMPLETE_RESTART_REQUIRED'
        if not self.latest:return False,'NO_DATA'
        reason=self.check(now,**self.latest)
        if reason:return False,reason
        if self.good_since is None or now-self.good_since<2:return False,'WAIT_2S_STABLE'
        self.armed=True;self.reason='ARMED_WAIT_DEADMAN';self.generation=self.latest['snapshot']['generation']
        self.deadline=now+.4;self.last_tick=now;self.last_command=zero('ARMED')
        return True,'ARMED'
