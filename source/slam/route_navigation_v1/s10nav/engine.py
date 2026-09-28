import collections
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .util import read,inv,apply,delta,voxel,cap
from .timing import DeviceClock
from .motion import Motion
from .navigation import RouteFollower

class Engine:
    def __init__(self,assets,cfg,matcher,initial=None):
        self.cfg=cfg; self.matcher=matcher; self.ext=read(assets/'extrinsics.json')
        self.clocks={s:DeviceClock(cfg['clock']) for s in ['front','rear','camera']}
        self.motion=Motion(cfg['motion'],{s:np.array(T) for s,T in self.ext['imu'].items()})
        self.route=RouteFollower(read(assets/'route.json'),cfg['route'])
        self.T=None;self.last_t=None;self.last_good=None;self.velocity=np.zeros(3);self.initial=initial
        self.state='WAIT_INITIALIZATION';self.reason='waiting_for_healthy_scan';self.confirmations=0;self.last_confirm=-1e9
        self.last_global=-1e9;self.last_processed={s:-1e9 for s in ['front','rear']};self.last_native={}
        self.sensor_good={s:-1e9 for s in ['front','rear']};self.perception={};self.counts=collections.Counter()
        self.last_metrics={};self.generation=0;self.reset_pending=False;self.last_event=None
        self.pose_history=collections.deque(maxlen=12)

    def event(self,e,now=None):
        now=e['received'] if now is None else now; self.last_event=now;kind=e['kind'];sensor=e.get('sensor')
        self.counts['received_'+kind]+=1
        expected={('imu','front'):'wym_front_imu',('imu','rear'):'wym_rear_imu',('imu','camera'):'camera_camera_imu_optical',
                  ('cloud','front'):'wym_front_lidar',('cloud','rear'):'wym_rear_lidar',('vio','camera'):'wym_insight9_vio_world',('depth','camera'):'camera_camera_depth'}
        if 'frame' in e and e['frame']!=expected.get((kind,sensor)):
            self.counts['unexpected_frame_'+str(sensor)]+=1;return None
        if now-e['received']>self.cfg['clock']['max_queue_age_s']:
            self.counts['queue_stale_'+kind]+=1;return None
        if kind=='imu':
            if not np.isfinite(np.r_[e['gyro'],e['accel']]).all() or np.linalg.norm(e['gyro'])>20 or np.linalg.norm(e['accel'])>100:
                self.counts['invalid_imu_'+sensor]+=1;return None
            clock=self.clocks[sensor];epoch=clock.epoch;t=clock.observe_imu(e['stamp'],e['received'])
            if clock.epoch!=epoch:self.motion.clear(sensor);self.counts['clock_epoch_change_'+sensor]+=1
            if t is not None:self.motion.add_imu(sensor,t,np.array(e['gyro']),np.array(e['accel']))
            return None
        if kind=='vio':
            if e.get('child_frame','wym_insight9_vio_sensor')!='wym_insight9_vio_sensor':
                self.counts['unexpected_vio_child_frame']+=1;return None
            t=self.clocks['camera'].map_stamp(e['stamp'],now)
            if t is not None:
                from .util import mat
                try:self.motion.add_vio(t,mat(e['pose'][:3],e['pose'][3:]))
                except ValueError:self.counts['invalid_vio_pose']+=1
            return None
        if kind=='depth':
            t=self.clocks['camera'].map_stamp(e['stamp'],now)
            if t is not None and -.02<=now-t<.35 and len(e['points']):
                self.perception['depth']=(t,apply(np.array(self.ext['depth']),e['points']))
            return None
        if kind!='cloud':return None
        if now-self.last_processed[sensor]<self.cfg['processing_period_per_lidar_s']:
            self.counts['rate_limited_'+sensor]+=1;return None
        self.last_processed[sensor]=now
        t=self.clocks[sensor].map_stamp(e['stamp'],now)
        epoch=self.clocks[sensor].epoch;last=self.last_native.get(sensor)
        if last is not None and last[0]==epoch and e['stamp']<=last[1]:return self.reject('duplicate_or_reordered_cloud',now)
        self.last_native[sensor]=(epoch,e['stamp'])
        if t is None:return self.reject('clock_not_ready_'+sensor,now)
        rel=np.asarray(e['rel']);pts=np.asarray(e['points'])
        if not len(rel) or len(rel)!=len(pts) or not np.isfinite(rel).all() or rel.min()<-.001 or not .03<float(rel.max())<.16:
            return self.reject('invalid_point_times',now)
        end=t+float(rel.max()); age=now-end
        if age<-.025 or age>self.cfg['clock']['max_cloud_age_s']:return self.reject('cloud_source_age',now)
        if self.last_t is not None and end<=self.last_t+.001:
            self.counts['out_of_order_scan_end']+=1;return None
        X=np.array(self.ext['lidar'][sensor]); body,deskew=self.motion.deskew(pts,rel,t,X,self.velocity)
        if body is None:return self.reject(deskew,now)
        self.perception[sensor]=(end,body)
        body=cap(voxel(body,self.cfg['scan_voxel_m']),self.cfg['scan_cap'])
        norm=np.linalg.norm(body,axis=1);body=body[(norm>.65)&(norm<32)]
        if len(body)<self.cfg['registration']['min_points']*2:return self.reject('insufficient_cloud',now)
        match_body=body;used_sensors=[sensor];other='rear' if sensor=='front' else 'front'
        if other in self.perception:
            ot,op=self.perception[other]
            if 0<=end-ot<=.12:
                motion,_=self.motion.relative(ot,end,self.velocity)
                if motion is not None:
                    op=apply(inv(motion),op);r=np.linalg.norm(op,axis=1);op=op[(r>.65)&(r<32)]
                    match_body=voxel(np.vstack([cap(body,2500),cap(op,2500)]),self.cfg['scan_voxel_m'])
                    used_sensors.append(other)
        self.poll(now)
        gravity=self.motion.gravity(end)
        initialized=False;prediction='none';D=None
        if self.T is not None and self.last_t is not None and end-self.last_t<self.cfg['motion']['max_predict_s']:
            D,prediction=self.motion.relative(self.last_t,end,self.velocity)
        if self.T is not None and D is not None and self.state!='LOST':
            guess=self.T@D;result=self.matcher.register(match_body,guess)
            if len(used_sensors)>1:
                # Joint fit cannot hide a bad sensor behind the other sensor's
                # point count. Validate each stream, then fall back to current.
                checks=[self.matcher.quality(p[1::2],result['T']) for p in (body,op)] if result['accepted'] else []
                if not result['accepted'] or any(q['overlap']<.5 or q['median_nn_m']>.2 or q['plane_rmse_m']>.15 for q in checks):
                    result=self.matcher.register(body,guess);used_sensors=[sensor]
        elif self.initial is not None:
            guess=self.initial.copy()
            if gravity is not None:
                y=np.arctan2(guess[1,0],guess[0,0]);guess[:3,:3]=Rotation.from_euler('z',y).as_matrix()@gravity
            result=self.matcher.register(match_body,guess,wide=True);initialized=True;prediction='explicit_initial_pose'
        elif self.T is not None and self.last_good is not None and now-self.last_good<2.0:
            # Bounded local reacquisition from the last observed pose. This is
            # not an integration over missing IMU and cannot complete a goal.
            guess=self.T.copy()
            if gravity is not None:
                yaw=np.arctan2(guess[1,0],guess[0,0]);guess[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()@gravity
            result=self.matcher.register(match_body,guess,wide=True);initialized=True;prediction='bounded_local_reacquisition'
            if result['accepted']:
                change=delta(self.T,result['T'])
                if change[0]>2.0 or change[1]>60:result.update(accepted=False,reasons=['reacquisition_outside_local_bound'])
        else:
            if now-self.last_global<self.cfg['relocalization']['retry_s']:return self.reject('waiting_relocalization_retry',now)
            self.last_global=now
            if gravity is None:return self.reject('relocalization_needs_reliable_gravity',now)
            self.state='RELOCALIZING';result=self.matcher.relocalize(body,sensor,gravity,match_body);initialized=True;prediction='blind_global_retrieval'
            self.counts['global_attempts']+=1
        self.last_metrics={k:v for k,v in result.items() if k!='T'}
        self.last_metrics.update(sensor=sensor,used_sensors=used_sensors,source_age_s=age,deskew=deskew,prediction=prediction)
        if not result['accepted']:return self.reject(';'.join(result['reasons']),now)
        T=result['T']
        if initialized and len(used_sensors)>1 and result['accepted']:
            checks=[self.matcher.quality(p[1::2],T) for p in (body,op)]
            if any(q['overlap']<.5 or q['median_nn_m']>.2 or q['plane_rmse_m']>.15 for q in checks):return self.reject('reacquisition_sensor_disagreement',now)
        if self.T is not None and self.last_t is not None and not initialized:
            # Never divide registration noise by a 2-ms front/rear offset.
            # A bounded displacement check still rejects true jumps; velocity
            # is estimated over >=200 ms when that history is available.
            dt=end-self.last_t;displacement=np.linalg.norm(T[:3,3]-self.T[:3,3])
            if displacement>self.cfg['motion']['max_speed_mps']*dt+self.cfg['motion'].get('velocity_noise_allowance_m',.15):return self.reject('unphysical_accepted_velocity',now)
            old=[p for p in self.pose_history if .2<=end-p[0]<=.6]
            tv,pv=old[-1] if old else (self.last_t,self.T[:3,3])
            dv=(T[:3,3]-pv)/max(end-tv,.2)
            speed=np.linalg.norm(dv)
            if speed>self.cfg['motion']['max_speed_mps']:dv*=self.cfg['motion']['max_speed_mps']/speed
            old_world=self.T[:3,:3]@self.velocity
            self.velocity=T[:3,:3].T@(.5*old_world+.5*dv)
        else:self.velocity[:]=0;self.pose_history.clear()
        if initialized:
            self.generation+=1;self.reset_pending=True;self.confirmations=0;self.last_confirm=-1e9
            self.counts['initialization_candidates']+=1
        if self.last_good is not None and now-self.last_good>self.cfg['relocalization']['max_confirmation_gap_s']:self.confirmations=0
        if end-self.last_confirm>=.12:self.confirmations+=1;self.last_confirm=end
        self.T=T;self.last_t=end;self.last_good=now;self.initial=None
        self.pose_history.append((end,T[:3,3].copy()));self.motion.anchor_gravity(end,T[:3,:3])
        for used in used_sensors:self.sensor_good[used]=now
        self.state='TRACKING' if self.confirmations>=self.cfg['relocalization']['confirmations'] else 'INITIALIZING'
        self.reason='';self.counts['accepted_'+sensor]+=1
        return dict(accepted=True,t=end,received=now,pose=T.tolist(),health=self.poll(now),metrics=self.last_metrics)

    def reject(self,reason,now):
        self.reason=reason;self.counts['reject_'+reason.split(';')[0]]+=1
        # A failed redundant sensor must not invalidate a still fresh accepted
        # solution from the other sensor. poll() expires it by measurement age.
        return dict(accepted=False,received=now,reason=reason,health=self.poll(now),metrics=self.last_metrics)

    def poll(self,now):
        age=None if self.last_good is None else max(now-self.last_good,now-self.last_t)
        if age is not None and age>self.cfg['health']['lost_age_s']:
            if self.state in ('TRACKING','INITIALIZING','DEGRADED'):self.counts['loss_transitions']+=1
            self.state='LOST';self.confirmations=0
        elif age is not None and age>self.cfg['health']['valid_age_s'] and self.state=='TRACKING':self.state='DEGRADED'
        recent=[s for s,t in self.sensor_good.items() if now-t<self.cfg['health']['valid_age_s']]
        valid=self.state=='TRACKING' and age is not None and age<self.cfg['health']['valid_age_s']
        return dict(state=self.state,valid=valid,age_s=age,single_lidar=len(recent)<2,healthy_lidars=recent,
                    reason=self.reason,confirmed_observations=self.confirmations,generation=self.generation,
                    clocks={s:c.status(now) for s,c in self.clocks.items()},covariance_calibrated=False)

    def proposal(self,now):
        health=self.poll(now);c=self.cfg['route'];fresh=False;obstacle=False
        for sensor,(t,p) in self.perception.items():
            if now-t>.3:continue
            if sensor=='front':fresh=True
            # Positive obstacles only. No inference that missing returns mean
            # traversable stairs/ground; terrain warning edges remain held.
            hit=(p[:,0]>.45)&(p[:,0]<c['stop_obstacle_x_m'])&(abs(p[:,1])<c['stop_obstacle_half_width_m'])&(p[:,2]>-.15)&(p[:,2]<1.2)
            if np.count_nonzero(hit)>=8:obstacle=True
        changed=self.reset_pending and health['valid']
        result=self.route.update(self.T,health,now,obstacle,fresh,changed)
        if changed:self.reset_pending=False
        return {**result,'localization':health}
