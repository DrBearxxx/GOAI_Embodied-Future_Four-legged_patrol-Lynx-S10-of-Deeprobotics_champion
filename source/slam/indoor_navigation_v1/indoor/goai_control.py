"""GOAI route gate: explicit route intent + fresh G12 permit, no SDK emulation."""
import math
import numpy as np
from .control import Guard,zero
from .continuity import safety_policy


class GoaiRoute(Guard):
    def __init__(self,route,asset_id,boot_id):
        super().__init__(route)
        self.asset_id=asset_id;self.boot_id=boot_id
        self.pending=False;self.pending_until=-1.;self.active_nonce=None
        self.used_nonces=set();self.localizer_run=None;self.pending_generation=None;self.observed_generation=None
        self.intent_reason='ROUTE_DISARMED';self.last_cycle=None
        self.robust_limits=(.1,.2);self.solution_mode='LEGACY';self.strong_since=None;self.recover_after=-1.
        self.measurement_age=float('inf')

    def stop(self,reason='OPERATOR_STOP'):
        super().stop(reason)
        self.pending=False;self.active_nonce=None;self.good_since=None
        self.intent_reason=reason
        self.strong_since=None

    def localization_check(self,now,snapshot):
        if snapshot.get('schema')!='goai.localization.continuity.v2':return super().localization_check(now,snapshot)
        s=snapshot['solution'];age=now-s['measurement_mono']
        self.measurement_age=age
        measured=np.array(s['measured_pose'],float)
        velocity=np.array(s['velocity_body'],float)
        if velocity.shape!=(3,) or not np.isfinite(velocity).all() or np.linalg.norm(velocity)>.5:return 'UNEXPECTED_ESTIMATED_BODY_SPEED'
        if (measured.shape!=(4,) or not np.isfinite(measured).all() or
                s['estimate_mono']<s['measurement_mono'] or
                s['generation']!=snapshot['generation'] or s['pose']!=snapshot['pose']):return 'INVALID_SOLUTION'
        if s['mode'] not in ('TRACKING','DEGRADED','PREDICT_ONLY'):return 'LOCALIZATION_LOST:'+s.get('reason','')
        imu_age=now-s['estimate_mono']
        if not 0<=age<=.55 or not 0<=imu_age<=.08:return 'PREDICTION_EXPIRED'
        xy=.04+.08*age+.15*age*age;yaw=.035+.10*age+.20*age*age
        if xy>.14 or yaw>.18 or np.linalg.norm(np.array(snapshot['pose'][:3])-measured[:3])>.15:return 'PREDICTION_BUDGET_EXHAUSTED'
        safety=safety_policy(now,snapshot['safety_records'])
        if safety['mode']=='STOP':return safety['reason']
        mode=s['mode']
        if age>.35 or imu_age>.04:mode='PREDICT_ONLY'
        elif age>.25 and mode=='TRACKING':mode='DEGRADED'
        caps={'TRACKING':(.1,.2),'DEGRADED':(.04,.10),'PREDICT_ONLY':(.02,.06)}[mode]
        if mode!='TRACKING' or safety['mode']!='FRESH':self.recover_after=now+.5
        if now<self.recover_after:caps=(min(caps[0],.04),min(caps[1],.10))
        self.robust_limits=(min(caps[0],safety['max_vx']),min(caps[1],safety['max_wz']))
        if imu_age>.04:self.robust_limits=(min(self.robust_limits[0],.02),0.)
        self.solution_mode=mode if safety['mode']=='FRESH' else 'CAUTIOUS'
        # Keep remaining uncertainty inside the existing route corridor.
        cross,height=self._corridor(np.array(snapshot['pose'][:3]))
        if self.target and (cross+xy>.45 or height+xy>.22):return 'UNCERTAINTY_OUTSIDE_CORRIDOR'
        return None

    def velocity_limits(self,snapshot):
        return self.robust_limits if snapshot.get('schema')=='goai.localization.continuity.v2' else super().velocity_limits(snapshot)

    def goal_reached(self,snapshot,distance):
        if snapshot.get('schema')!='goai.localization.continuity.v2':return super().goal_reached(snapshot,distance)
        s=snapshot['solution']
        return (self.solution_mode in ('TRACKING','DEGRADED') and self.measurement_age<=.25
            and distance<=.18 and np.linalg.norm(np.array(s['measured_pose'][:3])-self.xyz[self.target])<=.18)

    def backend_check(self,now,f,e):
        if not f or not -.02<=now-f['mono']<=.25:return 'GOAI_STATUS_STALE'
        if f['backend']!='goai_default' or f['allow_actuation'] is not True:return 'GOAI_NOT_HARDWARE_POLICY'
        if f['healthy'] is not True or f['fault'] is not False:return 'GOAI_UNHEALTHY'
        if f['mode']!='policy':return 'GOAI_NOT_POLICY'
        if f['ownership'] is not True or f['other_joint_publishers']!=0 or f['foreign_commands']!=0 or f['joint_receivers']<1:return 'GOAI_OWNERSHIP_LOST'
        if f['source'] not in ('manual','navigation'):return 'GOAI_SOURCE_STOPPED'
        if not e or not -.02<=now-e['mono']<=.25:return 'GRAPH_STALE'
        if e['status_publishers']!=1 or e['localization_publishers']!=1:return 'AMBIGUOUS_INPUT_PUBLISHERS'
        if e['other_nav_publishers']!=0 or e['nav_subscribers']<1:return 'GOAI_NAV_OWNERSHIP'
        if e['authority_publishers']>1:return 'COMPETING_AUTHORITY'
        if self.armed and (f['source']!='navigation' or f['nav_permit_active'] is not True or
                f['nav_permit_nonce']!=self.active_nonce or e['authority_publishers']!=1):return 'G12_TAKEOVER_OR_PERMIT_LOST'
        return None

    def check(self,now,snapshot,feedback,env):
        try:
            if not snapshot:return 'NO_LOCALIZATION'
            if snapshot['asset_id']!=self.asset_id or snapshot['boot_id']!=self.boot_id:return 'MAP_OR_CLOCK_ID_MISMATCH'
            run=snapshot['run_id']
            if not isinstance(run,str) or not run:return 'INVALID_LOCALIZER_RUN'
            if run!=self.localizer_run:
                was_running=self.armed or self.pending
                self.localizer_run=run;self.last_seq=-1;self.last_pose=None;self.good_since=None
                if was_running:return 'LOCALIZER_RESTART'
            if snapshot['generation']!=self.observed_generation:
                self.observed_generation=list(snapshot['generation']);self.good_since=None
                if self.armed or self.pending:return 'RELOCALIZATION_REQUIRES_REARM'
            if self.last_pose and snapshot['seq']==self.last_seq and snapshot['mono']!=self.last_pose[0]:return 'REPEATED_POSE_RESTAMPED'
            if self.last_pose and snapshot['seq']!=self.last_seq and snapshot['mono']<=self.last_pose[0]:return 'POSE_TIME_REGRESSION'
            if snapshot.get('schema')!='goai.localization.continuity.v2':
                for side in ('front','rear'):
                    t=snapshot['perception_mono'][side]
                    if not math.isfinite(t) or not 0<=now-t<=.25:return 'PERCEPTION_STALE'
                h=snapshot['health']
                if h['valid'] is not True or h['single_lidar'] is not False:return 'LOCALIZATION_LOST'
                if snapshot['front_fresh'] is not True or snapshot['rear_fresh'] is not True or snapshot['obstacle'] is not False:return 'PERCEPTION_UNSAFE'
            if self.pending and snapshot['generation']!=self.pending_generation:return 'RELOCALIZATION_REQUIRES_REARM'
            return super().check(now,snapshot,feedback,env)
        except (KeyError,TypeError,ValueError,OverflowError):return 'INVALID_INPUT'

    def prepare(self,now):
        """Only a fresh operator line in manual mode can prepare a route."""
        if self.complete:return False,'COMPLETE_RESTART_REQUIRED'
        if self.armed or self.pending:return False,'ALREADY_PREPARED'
        if not self.latest:return False,'NO_DATA'
        reason=self.check(now,**self.latest)
        if reason:return False,reason
        f=self.latest['feedback']
        if f['source']!='manual' or f['manual_ready'] is not True or f['nav_ready'] is not True:return False,'A_MANUAL_CENTER_ZERO_REQUIRED'
        if f['nav_permit_active'] or f['nav_permit_available']:return False,'PREPARE_BEFORE_NAV_READY'
        if self.good_since is None or now-self.good_since<2:return False,'WAIT_2S_STABLE'
        if self.latest['snapshot'].get('schema')=='goai.localization.continuity.v2' and (self.strong_since is None or now-self.strong_since<2):return False,'WAIT_2S_FRESH_MAP_AND_COVERAGE'
        self.pending=True;self.pending_until=now+30.;self.pending_generation=self.latest['snapshot']['generation']
        self.intent_reason='WAIT_NAV_READY_AND_C'
        return True,self.intent_reason

    def tick(self,now,snapshot,feedback,env):
        if self.last_cycle is not None and (now<=self.last_cycle or now-self.last_cycle>.20):
            self.stop('ROUTE_LOOP_STALLED')
        self.last_cycle=now
        reason=self.check(now,snapshot,feedback,env)
        self.latest=dict(snapshot=snapshot,feedback=feedback,env=env)
        if reason:
            self.stop(reason)
            return zero(reason,target=self.target,complete=self.complete,armed=False)
        if snapshot.get('schema')=='goai.localization.continuity.v2':
            strong=(self.solution_mode=='TRACKING' and self.measurement_age<=.25 and safety_policy(now,snapshot['safety_records'])['mode']=='FRESH')
            if not strong:self.strong_since=None
            elif self.strong_since is None:self.strong_since=now
        if self.pending:
            if now>self.pending_until:self.stop('ROUTE_ENTRY_EXPIRED')
            elif feedback['source']=='navigation':
                nonce=feedback['nav_permit_nonce']
                if snapshot.get('schema')=='goai.localization.continuity.v2' and (self.strong_since is None or now-self.strong_since<2):
                    self.stop('FRESH_MAP_AND_COVERAGE_REQUIRED_FOR_ENTRY')
                elif (feedback['nav_permit_active'] is not True or type(nonce) is not int or nonce<=0 or
                        nonce in self.used_nonces or env['authority_publishers']!=1):self.stop('FRESH_G12_PERMIT_REQUIRED')
                else:
                    self.active_nonce=nonce
                    ok,why=super().arm(now)
                    if not ok:self.stop(why)
                    else:self.pending=False;self.used_nonces.add(nonce);self.intent_reason='FOLLOW'
        if self.armed:self.renew_deadman(now)
        c=super().tick(now,snapshot,feedback,env)
        if (snapshot.get('schema')=='goai.localization.continuity.v2' and not self.armed and not self.pending and
                (self.strong_since is None or now-self.strong_since<2)):
            c['ready']=False
        if not self.armed and not self.complete:
            c['state']='WAIT_NAV_READY_AND_C' if self.pending else ('ROUTE_READY' if c.get('ready') else 'WAIT_2S_STABLE')
        c['pending']=self.pending;c['intent']=self.intent_reason
        c['navigation_quality']=self.solution_mode
        return c
