"""Independent LiDAR/gyro frontend; never uses the frozen map or commands motion."""
import json
import queue
import time
import uuid
from pathlib import Path
from collections import deque
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from paths import BASE, ASSETS
from indoor.pipeline import install_ingest, unpack_cloud, latest_put
from indoor.messages import convert
from indoor.continuity import use_redundancy
from obstacle_evidence import clearance
from native_nav.robust_timing import use_slewed_clocks
from s10nav.engine import Engine
from s10nav.util import inv, apply, voxel, cap
from s10nav.matching import small_gicp
from vio_continuity import bridge_local
from registration_log import RegistrationLog
from registration_motion import use_registration_motion
from localization_quality import classify
from lidar_continuity import ContinuousLidarFrame
from registration_prior import RegistrationPrior


def rigid(T):
    """Keep SE(3) products on SO(3); GICP preserves scale in its initial guess.

    Using transpose as the inverse of a slightly non-orthogonal keyframe and
    composing that frame back into the fit amplifies roundoff at every update.
    Rotation normalization prevents artificial scale/shear, not robot motion.
    """
    result=np.array(T,dtype=float,copy=True)
    u,_,vh=np.linalg.svd(result[:3,:3])
    if np.linalg.det(u@vh)<0:u[:,-1]*=-1
    result[:3,:3]=u@vh
    result[3]=[0.,0.,0.,1.]
    return result


def verify_geometry(ref,body,candidate,last_T):
    """Check an uncertain motion prior against a separate cloud subset.

    Start from the last actual pose, independently of the IMU/velocity guess.
    Motion-prior disagreement alone must not discard consistent geometry.
    """
    source,_=small_gicp.preprocess_points(body[1::2],downsampling_resolution=.20,
                                        num_neighbors=20,num_threads=2)
    fit=small_gicp.align(ref['cloud'],source,ref['tree'],init_T_target_source=rigid(inv(ref['T'])@last_T),
                        registration_type='GICP',max_correspondence_distance=.65,
                        max_iterations=40,num_threads=2,rotation_epsilon=.001,translation_epsilon=.001)
    T=rigid(np.asarray(fit.T_target_source));other=rigid(ref['T']@T)
    difference=inv(candidate)@other
    distances,_=ref['points'].query(apply(T,body[::2]),workers=1)
    eig=np.linalg.eigvalsh(np.asarray(fit.H))
    result=dict(converged=bool(fit.converged),overlap=float(np.mean(distances<.25)),
                median_nn_m=float(np.median(distances)),condition=float(max(0.,eig[0])/max(eig[-1],1e-12)),
                difference_m=float(np.linalg.norm(difference[:3,3])),
                difference_rad=float(Rotation.from_matrix(difference[:3,:3]).magnitude()))
    result['accepted']=bool(result['converged'] and result['overlap']>=.65 and result['median_nn_m']<=.16
                            and result['condition']>=1e-6 and result['difference_m']<=.08 and result['difference_rad']<=.035)
    return result


class LocalOdometry:
    """Separate keyframe per sensor, shared SE(3) odom frame. No flat-floor prior."""
    def __init__(self, engine, registration_log=None, scan_log=None, obstacle_log=None):
        self.engine = engine
        self.registration_log=registration_log
        self.scan_log=scan_log
        self.obstacle_log=obstacle_log;self.obstacle_logged={}
        self.T = np.eye(4); self.t = None; self.velocity = np.zeros(3)
        self.epoch = str(uuid.uuid4()); self.frames = {}; self.keys = {}
        self.accepted = 0; self.rejected = 0; self.reason = 'WAIT_SCAN'
        self.records = {}; self.quality = {}; self.path = 0.
        self.sensor_poses = {}; self.velocity_samples = deque(maxlen=8)
        self.last_sensor=None;self.keyframe_refreshes=0
        self.continuity=ContinuousLidarFrame();self.output_quality={};self.output_continuity={}
        self.prior=RegistrationPrior();self.recovery_reference={}

    def keyframe(self, sensor, body, T, end):
        cloud, tree = small_gicp.preprocess_points(body, downsampling_resolution=.20,
                                                  num_neighbors=20, num_threads=2)
        self.frames[sensor] = dict(cloud=cloud, tree=tree,points=cKDTree(body),
                                  T=rigid(T), t=end)
        self.keyframe_refreshes+=1

    def event(self, e, now, perception_only=False):
        eng = self.engine; s = e['sensor']; clock = eng.clocks[s]
        t = clock.map_stamp(e['stamp'], now)
        key = (clock.epoch, e['stamp'])
        if key <= self.keys.get(s, (-1, -1e30)): return None
        # A device clock restart changes timestamps, not the physical scene
        # or sensor extrinsics. Retain geometric references and their offsets.
        self.keys[s] = key
        if t is None: raise ValueError('CLOCK_NOT_READY')
        rel = np.asarray(e['rel']); pts = np.asarray(e['points'])
        if (e['frame'] != f'wym_{s}_lidar' or len(pts) < 600 or len(pts) != len(rel)
                or not np.isfinite(rel).all() or rel.min() < -.001 or not .03 < rel.max() < .16):
            raise ValueError('INVALID_SCAN')
        end = t + float(rel.max())
        if not -.025 <= now-end <= .25: raise ValueError('SCAN_AGE')
        # Independent obstacle evidence includes people; no map filtering.
        clearance_body=apply(np.array(eng.ext['lidar'][s]),pts)
        self.records[s] = clearance(clearance_body, end, clock.epoch)
        if self.obstacle_log is not None and self.records[s]['blocked'] and end-self.obstacle_logged.get(s,-1e9)>=.25:
            # Keep every near-field point, including below the current height
            # threshold. The JSON's 24 samples cannot explain terrain mistakes.
            near=(clearance_body[:,0]>0)&(clearance_body[:,0]<2)&(abs(clearance_body[:,1])<1)&(abs(clearance_body[:,2])<2)
            self.obstacle_log.emit(dict(kind='obstacle_observation',sensor=s,measurement_mono=end,
                epoch=self.epoch,points_body=clearance_body[near].astype(np.float32),
                assessment=self.records[s],continuous_T=self.T.copy()))
            self.obstacle_logged[s]=end
        if perception_only:
            self.reason='PERCEPTION_ONLY_LIGHTNING_ODOMETRY'
            return None
        if self.t is not None and end <= self.t+.02: return None
        deskew_velocity=self.velocity if self.t is not None and end-self.t<=.6 else np.zeros(3)
        body, why = eng.motion.deskew(pts, rel, t, np.array(eng.ext['lidar'][s]), deskew_velocity)
        if body is None: raise ValueError('DESKEW:'+str(why))
        r = np.linalg.norm(body, axis=1)
        body = cap(voxel(body[(r>.7)&(r<25)], .16), 4500).astype(float)
        if len(body) < 600: raise ValueError('SPARSE_SCAN')
        measured_bridge = bridge_local(self.sample(), end, eng.motion) if self.t is not None and end-self.t>.6 else None
        recovering=self.t is not None and end-self.t>.6 and measured_bridge is None
        D = None;prediction_method='initial_reference'
        if self.t is not None:
            if measured_bridge is not None:
                tail,_=eng.motion.relative(measured_bridge['estimate_mono'],end,self.velocity)
                if tail is not None and end-measured_bridge['estimate_mono']<=.1:D=measured_bridge['D']@tail
            else:D, prediction_method = eng.motion.relative(self.t, end, self.velocity)
            if D is None:
                # Preserve the geometric reference through an IMU/scan gap.
                # The last measured pose is only a search seed, never an update.
                D=np.eye(4);prediction_method='last_lidar_pose_recovery_seed';recovering=True
        attitude_prediction=self.prior.predict(self.T,self.t,end,eng.motion,deskew_velocity)
        predicted=rigid(attitude_prediction)
        if D is not None and not recovering:predicted[:3,3]=(self.T@D)[:3,3]
        prediction_method=self.prior.details.get('increment_method',prediction_method)
        if self.scan_log is not None:
            self.scan_log.emit(dict(kind='scan_registration_input',sensor=s,measurement_mono=end,
                epoch=self.epoch,body=body.astype(np.float32),deskew=why,prediction_method=prediction_method,
                predicted=predicted.copy(),previous=self.sample(),prior=dict(self.prior.details),
                imu={name:np.asarray([[row[0],*row[1],*row[2]] for row in q if end-.4<=row[0]<=end+.01],dtype=np.float64)
                     for name,q in getattr(eng.motion,'imu',{}).items()}))
        if s not in self.frames:
            # Bootstrapping another sensor is NOT an accepted odometry update.
            self.keyframe(s, body, predicted, end)
            self.continuity.seed(s,predicted,end)
            if self.t is not None: return None
            self.T = predicted; self.t = end; self.reason = 'INITIALIZED_NOT_MOVEMENT'
            return self.sample()
        raw_prediction=self.continuity.raw_prediction(s,predicted)
        ref = self.frames[s]; guess = rigid(inv(ref['T']) @ raw_prediction)
        source, _ = small_gicp.preprocess_points(body, downsampling_resolution=.20,
                                                num_neighbors=20, num_threads=2)
        fit = small_gicp.align(ref['cloud'], source, ref['tree'], init_T_target_source=guess,
                              registration_type='GICP', max_correspondence_distance=.65,
                              max_iterations=24, num_threads=2, rotation_epsilon=.001,
                              translation_epsilon=.001)
        relative = np.asarray(fit.T_target_source)
        if not np.isfinite(relative).all(): raise ValueError('NONFINITE_FIT')
        fit_rotation_error=float(np.linalg.norm(relative[:3,:3].T@relative[:3,:3]-np.eye(3)))
        relative=rigid(relative)
        candidate = rigid(ref['T'] @ relative)
        offset=self.continuity.offsets.get(s,np.eye(4))
        corrected=self.prior.correct_tilt(offset@candidate,min(.2,max(0.,end-(self.t or end))))
        tilt_changed=np.linalg.norm(corrected[:3,:3]-(offset@candidate)[:3,:3])>1e-9
        candidate=rigid(inv(offset)@corrected)
        relative=rigid(inv(ref['T'])@candidate)
        # Refit translation with the fused orientation held fixed. Tilting an
        # already fitted cloud without this step would bias its position.
        check_body=body[1::2]
        for _ in range(2 if tilt_changed else 0):
            rotated=check_body@relative[:3,:3].T
            d,ids=ref['points'].query(rotated+relative[:3,3],workers=1)
            if not hasattr(ref['points'],'data'):break
            weights=1./(1.+(d/.25)**2)
            relative[:3,3]=np.average(ref['points'].data[ids]-rotated,axis=0,weights=weights)
        candidate=rigid(ref['T']@relative)
        distances, _ = ref['points'].query(apply(relative, body[1::2]), workers=1)
        overlap = float(np.mean(distances < .25)); median = float(np.median(distances))
        eig = np.linalg.eigvalsh(np.asarray(fit.H))
        condition = float(max(0., eig[0])/max(eig[-1], 1e-12))
        correction = inv(raw_prediction) @ candidate
        angle = float(Rotation.from_matrix(correction[:3,:3]).magnitude())
        translation=float(np.linalg.norm(correction[:3,3]))
        checks={'not_converged':not fit.converged,'overlap':overlap<.65,'median_nn':median>.16,
                'degenerate':condition<1e-6,'translation_innovation':translation>.20,'rotation_innovation':angle>.10}
        self.quality = dict(overlap=overlap, median_nn_m=median, condition=condition, sensor=s,
            measurement_mono=end,converged=bool(fit.converged),iterations=getattr(fit,'iterations',None),
            correction_m=translation,correction_rad=angle,rejection_checks=[k for k,v in checks.items() if v],
            keyframe_age_s=end-ref['t'],keyframe_refreshes=self.keyframe_refreshes)
        self.quality['fit_rotation_orthogonality_error']=fit_rotation_error
        self.quality.update(deskew=why,prediction=prediction_method)
        self.quality['attitude_prior']=dict(self.prior.details)
        level=classify(overlap,median,condition,bool(fit.converged))
        self.quality['level']=level
        self.quality['quality_warnings']=[k for k,v in checks.items() if v]
        self.quality['motion_prior_disagreed']=checks['translation_innovation'] or checks['rotation_innovation']
        self.quality['rejection_checks']=self.quality['quality_warnings'] if level=='unusable' else []
        if level=='unusable':
            if self.registration_log is not None:
                self.registration_log.emit(dict(sensor=s,measurement_mono=end,epoch=self.epoch,
                    source=body.copy(),target=ref['points'].data.copy(),guess=guess.copy(),fit=relative.copy(),
                    keyframe_T=ref['T'].copy(),keyframe_mono=ref['t'],predicted=predicted.copy(),
                    last_T=self.T.copy(),last_measurement_mono=self.t,recovering=recovering,
                    quality=dict(self.quality),velocity=self.velocity.copy()))
            # A sensor may have lost overlap with its own old reference while
            # the other LiDAR still measures the shared odom frame. Re-seed its
            # reference at that measured frame (with the existing IMU delta),
            # without accepting this rejected fit as a movement observation.
            if self.last_sensor not in (None,s) and self.t is not None and 0<=end-self.t<=.15:
                self.keyframe(s,body,predicted,end)
                self.continuity.seed(s,predicted,end);self.sensor_poses.pop(s,None)
                self.quality['keyframe_reseeded_from']=self.last_sensor
            elif (self.t is not None and end-self.t>1. and
                    self.prior.gravity is not None and end-self.recovery_reference.get(s,-1e9)>1.):
                # The scene can disappear during a fall/lift. Establish a new
                # local cloud at the continuously propagated attitude, never
                # at the stale overturned attitude. This frame alone is not
                # a pose measurement; a subsequent scan must register to it.
                self.keyframe(s,body,predicted,end);self.continuity.seed(s,predicted,end)
                self.sensor_poses.pop(s,None);self.velocity_samples.clear();self.velocity[:]=0.
                self.recovery_reference[s]=end
                self.quality['recovery']=dict(method='attitude_aligned_reference',
                    unobserved_position_gap_s=end-self.t,translation_source='last_measured_position',
                    prior=dict(self.prior.details))
            raise ValueError('LOCAL_GEOMETRY_REJECTED')
        # Keep each keyframe in its own registration coordinates, including
        # standby frames. Search seeds are mapped into those coordinates above.
        if (np.linalg.norm(relative[:3,3])>.5 or
                Rotation.from_matrix(relative[:3,:3]).magnitude()>.22 or overlap<.8):
            self.keyframe(s,body,candidate,end)
        previous_sensor=self.continuity.active
        continuous=self.continuity.update(s,candidate,end,predicted)
        self.quality['continuity']=dict(self.continuity.diagnostics)
        if continuous is None:return None
        candidate=rigid(continuous)
        travel=float(np.linalg.norm(candidate[:3,3]-self.T[:3,3]))
        if previous_sensor!=s:
            # Never differentiate across a re-anchored source reference.
            self.sensor_poses.clear();self.velocity_samples.clear()
        history=self.sensor_poses.setdefault(s,deque(maxlen=16))
        old=[v for v in history if .3<=end-v[0]<=.7]
        if old:
            stamp,position=old[-1]
            v=(candidate[:3,3]-position)/(end-stamp)
            self.velocity_samples.append((end,v))
            recent=[v for t,v in self.velocity_samples if end-t<=.5]
            world=np.median(recent,axis=0)
            self.velocity=candidate[:3,:3].T@world
        history.append((end,candidate[:3,3].copy()))
        self.T = candidate; self.t = end; self.path += travel
        if hasattr(eng.motion,'observe_lidar'):eng.motion.observe_lidar(end,candidate)
        self.last_sensor=s
        self.accepted += 1; self.reason = 'LIDAR_GYRO_MEASURED'
        self.output_quality=dict(self.quality)
        self.output_continuity=dict(self.continuity.diagnostics)
        return self.sample()

    def sample(self):
        return dict(T=self.T.tolist(), t=self.t, epoch=self.epoch, path=self.path,
                    accepted=self.accepted, quality=self.output_quality, velocity=self.velocity.tolist(),
                    continuity=self.output_continuity)


def reset_reference(local,request_id,reason='OPERATOR_RELOCALIZATION_SUCCEEDED'):
    """Operator map relocalization also discards the old posture's reference."""
    old_epoch=local.epoch;engine=local.engine;engine.motion.reset_lidar()
    new=LocalOdometry(engine,local.registration_log,local.scan_log,local.obstacle_log)
    if local.scan_log is not None:
        local.scan_log.emit(dict(kind='local_reference_reset',mono=time.monotonic(),reason=reason,
            request_id=request_id,old_epoch=old_epoch,new_epoch=new.epoch))
    return new


def frontend_main(inboxes, outbox, stop, heartbeat, commands=None, perception_only=False):
    cfg = json.loads((BASE/'config.json').read_text())
    engine = use_registration_motion(use_slewed_clocks(Engine(ASSETS, cfg, None)))
    registration_log=RegistrationLog(Path(__file__).parent/'logs/rejected-registration.bin')
    scan_log=RegistrationLog(Path(__file__).parent/'logs/local-scans.bin',max_bytes=64_000_000,backups=7)
    obstacle_log=RegistrationLog(Path(__file__).parent/'logs/obstacle-observations.bin',max_bytes=16_000_000,backups=3)
    local = LocalOdometry(engine,registration_log,scan_log,obstacle_log); side = 0; last_emit = 0.; last = None
    reset_request_id=None
    while not stop.is_set():
        heartbeat.value = time.monotonic(); item = None
        if commands is not None:
            try:command=commands.get_nowait()
            except queue.Empty:command=None
            if command and command['request_id']!=reset_request_id:
                local=reset_reference(local,command['request_id'],command.get('reason','OPERATOR_RELOCALIZATION_SUCCEEDED'))
                last=None;last_emit=0.;reset_request_id=command['request_id']
        for _ in range(2):
            try: item = inboxes[side].get_nowait()
            except queue.Empty: pass
            side = 1-side
            if item is not None: break
        if item is None: stop.wait(.003); continue
        now = time.monotonic()
        if now-item['received'] > .25: continue
        try:
            install_ingest(engine, item['ingest'])
            e = convert(item['topic'], unpack_cloud(item['cloud']), item['received'])
            measured = local.event(e, now, perception_only=perception_only)
            if measured: last = measured
        except (ValueError, RuntimeError, KeyError, TypeError, IndexError, np.linalg.LinAlgError) as exc:
            local.rejected += 1; local.reason = str(exc)[:180]
        now = time.monotonic()
        if now-last_emit >= .04:
            latest_put(outbox, dict(odom=last, records=local.records.copy(), reason=local.reason,reset_request_id=reset_request_id,
                                   fit_diagnostics=dict(local.quality),registration_recording=registration_log.status(),
                                   scan_recording=scan_log.status(),
                                   obstacle_recording=obstacle_log.status(),
                                   rejected=local.rejected, worker_mono=now))
            last_emit = now
    registration_log.close()
    scan_log.close()
    obstacle_log.close()
