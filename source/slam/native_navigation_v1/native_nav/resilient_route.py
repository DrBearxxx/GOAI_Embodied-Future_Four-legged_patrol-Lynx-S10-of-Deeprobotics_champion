"""Persistent mission: bounded perception faults HOLD zero, hard faults disarm.

HOLD retains an already-granted gateway lease by sending fresh zero commands.
It never sends ARM. An expired/revoked gateway lease cannot be auto-reacquired.
"""
import math
import numpy as np
from .route import NativeRoute
from .recovery import RecoveryWindow,angle_distance
from indoor.control import Guard,zero
from indoor.continuity import safety_policy
from .odometry_bridge import MAX_MAP_AGE,MAX_TRAVEL,budgets


SOFT_REASONS={'NO_LOCALIZATION','STALE_LOCALIZER','PREDICTION_EXPIRED','PREDICTION_BUDGET_EXHAUSTED',
              'NO_SAFETY_COVERAGE','SAFETY_COVERAGE_EXPIRED','OBSTACLE_OR_INVALID_CLOUD','INVALID_SAFETY_RECORD',
              'UNCERTAINTY_OUTSIDE_CORRIDOR','MAP_REACQUISITION','POSE_JUMP','RECOVERY_OUTSIDE_STOPPED_ENVELOPE',
              'INVALID_INPUT','INVALID_POSE','INVALID_SEQUENCE','INVALID_GENERATION','INVALID_SOLUTION',
              'UNEXPECTED_ESTIMATED_BODY_SPEED','POSE_TIME_REGRESSION','REPEATED_POSE_RESTAMPED'}


class ResilientRoute(NativeRoute):
    def __init__(self,route,asset_id,boot_id):
        super().__init__(route,asset_id,boot_id)
        self.holding=False;self.hold_reason=None;self.hold_anchor=None;self.hold_started=None
        self.window=RecoveryWindow();self.window_ready=False
        self.hold_count=0;self.resume_count=0;self.reassociation_notices=0

    def stop(self,reason='OPERATOR_STOP'):
        super().stop(reason)
        self.holding=False;self.hold_reason=None;self.hold_anchor=None;self.hold_started=None
        if hasattr(self,'window'):self.window.clear()
        self.window_ready=False

    def _hold(self,now,reason):
        if not self.holding:
            self.holding=True;self.hold_started=now;self.hold_count+=1
            self.hold_anchor=None if self.last_pose is None else self.last_pose[1].copy()
        self.hold_reason=reason;self.window.clear();self.window_ready=False
        self.last_command=zero('HOLD:'+reason);self.last_tick=now

    @staticmethod
    def stationary(feedback):
        try:
            f=feedback['feedback'];speed=float(f['speed']);wz=float(f['wz'])
            return math.isfinite(speed) and 0<=speed<=.03 and math.isfinite(wz) and abs(wz)<=.05
        except (KeyError,TypeError,ValueError,OverflowError):return False

    def localization_check(self,now,snapshot):
        s=snapshot['solution']
        residual=s.get('filter_residual_m',0.);z_residual=s.get('filter_residual_z_m',0.);yaw_residual=s.get('filter_residual_yaw_rad',0.)
        if not 0<=residual<=.10 or not 0<=z_residual<=residual or not 0<=yaw_residual<=.061:return 'INVALID_SOLUTION'
        if residual or yaw_residual:
            age=now-s['measurement_mono'];cross,height=self._corridor(np.asarray(snapshot['pose'][:3]))
            xy=.04+.08*age+.15*age*age
            if s.get('mode')=='ODOM_BRIDGE':xy,_=budgets(age,s['bridge']['path_length_m'],s['bridge']['turn_rad'])
            if self.target and (cross+xy+residual>.45 or height+xy+z_residual>.22):return 'UNCERTAINTY_OUTSIDE_CORRIDOR'
            if .035+.10*min(age,.55)+.20*min(age,.55)**2+yaw_residual>.18:return 'PREDICTION_BUDGET_EXHAUSTED'
        if s.get('mode')!='ODOM_BRIDGE':return super().localization_check(now,snapshot)
        age=now-s['measurement_mono'];imu_age=now-s['estimate_mono'];b=s['bridge']
        measured=np.asarray(s['measured_pose'],float);pose=np.asarray(s['pose'],float);v=np.asarray(s['velocity_body'],float)
        if (s['generation']!=snapshot['generation'] or s['pose']!=snapshot['pose'] or measured.shape!=(4,) or
                pose.shape!=(4,) or v.shape!=(3,) or not np.isfinite(np.r_[measured,pose,v]).all() or
                np.linalg.norm(v)>.5):return 'INVALID_SOLUTION'
        if not .55<age<=MAX_MAP_AGE or not 0<=imu_age<=.08:return 'PREDICTION_EXPIRED'
        if (b.get('schema')!='s10.measured_odometry_bridge.v1' or b['extrapolated'] is not False or
                type(b['samples']) is not int or b['samples']<4 or type(b['vio_epoch']) is not int or
                b['start_mono']!=s['measurement_mono'] or b['end_mono']!=s['estimate_mono'] or
                b['end_mono']<=b['start_mono']):return 'INVALID_SOLUTION'
        travel=b['path_length_m'];turn=b['turn_rad']
        if (not 0<=travel<=MAX_TRAVEL or not 0<=turn<=.30 or not 0<=b['rotation_disagreement_rad']<=.10 or
                not 0<=b['max_imu_gap_s']<=.035 or np.linalg.norm(pose[:3]-measured[:3])>MAX_TRAVEL):return 'PREDICTION_BUDGET_EXHAUSTED'
        xy,yaw=budgets(age,travel,turn)
        if xy>.14 or yaw>.18:return 'PREDICTION_BUDGET_EXHAUSTED'
        safety=safety_policy(now,snapshot['safety_records'])
        if safety['mode']=='STOP':return safety['reason']
        cross,height=self._corridor(pose[:3])
        if self.target and (cross+xy>.45 or height+xy>.22):return 'UNCERTAINTY_OUTSIDE_CORRIDOR'
        self.measurement_age=age;self.solution_mode='ODOM_BRIDGE';self.recover_after=now+.5
        self.robust_limits=(min(.03,safety['max_vx']),0. if imu_age>.04 else min(.06,safety['max_wz']))
        return None

    def check(self,now,snapshot,feedback,env):
        try:
            # Control authority is never hidden behind a perception HOLD.
            reason=self.backend_check(now,feedback,env)
            if reason:return reason
            if not snapshot:return 'NO_LOCALIZATION'
            if not isinstance(snapshot,dict):return 'INVALID_INPUT'
            if snapshot.get('schema')!='goai.localization.continuity.v2':return 'CONTINUITY_V2_REQUIRED'
            if snapshot['asset_id']!=self.asset_id or snapshot['boot_id']!=self.boot_id:return 'MAP_OR_CLOCK_ID_MISMATCH'
            run=snapshot['run_id'];generation=snapshot['generation'];seq=snapshot['seq'];mono=snapshot['mono']
            if not isinstance(run,str) or not run:return 'INVALID_LOCALIZER_RUN'
            if (not isinstance(generation,list) or len(generation)!=2 or
                    any(type(v) is not int or v<0 for v in generation)):return 'INVALID_GENERATION'
            if type(seq) is not int or seq<0:return 'INVALID_SEQUENCE'
            p=np.asarray(snapshot['pose'],float)
            if p.shape!=(4,) or not np.isfinite(p).all() or not math.isfinite(mono):return 'INVALID_POSE'
            if self.localizer_run is not None and run!=self.localizer_run:return 'LOCALIZER_RESTART'
            if self.last_pose and (seq<self.last_seq or mono<self.last_pose[0]):return 'POSE_TIME_REGRESSION'
            if self.last_pose and seq==self.last_seq and mono!=self.last_pose[0]:return 'REPEATED_POSE_RESTAMPED'
            if not -.02<=now-mono<=.25:return 'STALE_LOCALIZER'
            if snapshot.get('error'):return 'LOCALIZER_EXCEPTION'
            reason=self.localization_check(now,snapshot)
            if reason:return reason
            if self.armed and self.last_pose and seq!=self.last_seq:
                old_t,old_p=self.last_pose;dt=mono-old_t
                if dt<=0:return 'POSE_TIME_REGRESSION'
                # The tolerated displacement must not grow without bound while
                # measurements are missing. Keep the previous short-step budget.
                dt=min(dt,.55)
                if np.linalg.norm(p[:3]-old_p[:3])>.15+.35*dt or angle_distance(p[3],old_p[3])>.20+.4*dt:return 'POSE_JUMP'
            if self.holding and self.hold_anchor is not None:
                if np.linalg.norm(p[:3]-self.hold_anchor[:3])>.20 or angle_distance(p[3],self.hold_anchor[3])>.25:
                    return 'RECOVERY_OUTSIDE_STOPPED_ENVELOPE'
            cross,height=self._corridor(p[:3])
            if cross>(.25 if self.target==0 else .45) or height>.22:return 'OUTSIDE_CURRENT_ROUTE_SEGMENT'
            if self.target and self.route['edges'][self.target-1]['warnings']:return 'ROUTE_REQUIRES_REVIEW'
            return None
        except (KeyError,TypeError,ValueError,OverflowError,IndexError):return 'INVALID_INPUT'

    def _result(self,command):
        return dict(command,mission_state='HOLD' if self.holding else 'COMPLETE' if self.complete else 'RUNNING' if self.armed else 'WAIT_ARM',
                    hold_reason=self.hold_reason,hold_count=self.hold_count,resume_count=self.resume_count,
                    reassociation_notices=self.reassociation_notices,navigation_quality=self.solution_mode)

    def tick(self,now,snapshot,feedback,env):
        if not math.isfinite(now) or (self.last_cycle is not None and (now<self.last_cycle or now-self.last_cycle>.20)):
            self.stop('ROUTE_LOOP_STALLED');self.last_cycle=now
            return self._result(zero(self.reason,target=self.target,armed=False,ready=False))
        if self.last_cycle==now:
            return self._result(zero('DUPLICATE_CYCLE',target=self.target,armed=self.armed,ready=False))
        self.last_cycle=now
        self.latest=dict(snapshot=snapshot,feedback=feedback,env=env)
        # A new session/runtime may be adopted only while disarmed. No old
        # operator intent or lease is carried across a reconnect/restart.
        if not self.armed:
            if feedback and feedback.get('session')!=self.gateway_session:self.gateway_session=None;self.window.clear()
            if isinstance(snapshot,dict) and snapshot.get('run_id')!=self.localizer_run:
                self.localizer_run=snapshot.get('run_id');self.last_pose=None;self.last_seq=-1
                self.window.clear();self.good_since=None
        reason=self.check(now,snapshot,feedback,env)
        if reason:
            soft=reason in SOFT_REASONS or reason.startswith('LOCALIZATION_LOST:')
            if self.armed and soft:
                self._hold(now,reason);self.renew_deadman(now)
                return self._result(zero('HOLD:'+reason,target=self.target,armed=True,ready=False))
            self.stop(reason)
            return self._result(zero(reason,target=self.target,armed=False,ready=False))
        generation=snapshot['generation']
        changed=self.observed_generation is not None and generation!=self.observed_generation
        map_changed=changed and generation[0]!=self.observed_generation[0]
        if changed and generation[1]!=self.observed_generation[1]:self.reassociation_notices+=1
        self.observed_generation=list(generation)
        if self.armed and map_changed:self._hold(now,'MAP_REACQUISITION')
        elif map_changed:self.window.clear();self.window_ready=False
        # A continuous-correction notice is diagnostic, not proof of a global
        # reset. Adopt it only AFTER freshness, real pose-jump, corridor, speed,
        # safety and authority checks above. Large corrections stay stopped and
        # cannot be accepted automatically outside the held pose envelope.
        self.generation=list(generation)
        self.last_pose=(snapshot['mono'],np.asarray(snapshot['pose'],float).copy());self.last_seq=snapshot['seq']
        if self.good_since is None:self.good_since=now
        if self.holding or not self.armed:
            self.window_ready=self.window.observe(now,snapshot,self.stationary(feedback),
                                safety_policy(now,snapshot['safety_records'])['mode']=='FRESH')
        if self.holding:
            self.renew_deadman(now)
            if self.window_ready:
                self.holding=False;self.hold_reason=None;self.hold_anchor=None;self.hold_started=None;self.resume_count+=1
                self.recover_after=now+1.;self.last_command=zero('RESUME_READY');self.last_tick=now
                return self._result(zero('RESUME_READY',target=self.target,armed=True,ready=False))
            return self._result(zero('HOLD:'+self.hold_reason,target=self.target,armed=True,ready=False))
        if self.armed:self.renew_deadman(now)
        command=Guard.tick(self,now,snapshot,feedback,env)
        command['ready']=bool(not self.armed and not self.complete and self.window_ready)
        return self._result(command)

    def arm(self,now):
        if not self.window_ready or not self.stationary(self.latest.get('feedback')):
            return False,'WAIT_FRESH_STATIONARY_MAP_WINDOW'
        return Guard.arm(self,now)
