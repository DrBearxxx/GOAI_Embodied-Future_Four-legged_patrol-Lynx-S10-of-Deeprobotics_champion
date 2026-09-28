"""Short map-fit dropout bridge using received VIO AND measured gyro.

Not a timeout extension for constant-velocity prediction. Bounds are engineering
trial envelopes, NOT calibrated covariance or a guarantee of position accuracy.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation

MAX_MAP_AGE=1.2
MAX_TRAVEL=.15


def budgets(age,travel,turn):
    return .04+.03*age+.10*travel, .035+.035*age+.05*turn


def measured_bridge(now,anchor,motion,diagnostic=None):
    def reject(reason):
        if diagnostic is not None:diagnostic['reason']=reason
        return None
    start=anchor['measurement_mono'];age=now-start
    if anchor.get('confirmed') is not True or not .55<age<=MAX_MAP_AGE:return reject('ANCHOR_NOT_CONFIRMED_OR_EXPIRED')
    q=[row for row in motion.vio if row[0]<=now]
    if len(q)<4 or q[0][0]>start:return reject('VIO_NO_ANCHOR_COVERAGE')
    if now-q[-1][0]>.08:return reject('VIO_ENDPOINT_STALE')
    epoch=q[-1][2]
    q=[row for row in q if row[2]==epoch]
    if not q or q[0][0]>start or any(b[0]<=a[0] for a,b in zip(q,q[1:])):return reject('VIO_EPOCH_OR_ORDER')
    imu_ends=[]
    for rows in motion.imu.values():
        for row in reversed(rows):
            if row[0]<=now:
                if now-row[0]<=.08:imu_ends.append(row[0])
                break
    if not imu_ends:return reject('NO_FRESH_IMU_ENDPOINT')
    # VIO and IMU callbacks arrive independently. Use their COMMON received
    # endpoint, not a VIO pose a few ms beyond all received gyro samples.
    end=min(q[-1][0],max(imu_ends))
    if end<=start:return reject('VIO_NO_INTERVAL')
    stamps=[start]+[row[0] for row in q if start<row[0]<end]+[end]
    if len(stamps)<4 or max(np.diff(stamps))>min(.12,motion.cfg['max_vio_gap_s']):return reject('VIO_GAP')
    # Both endpoints must be bracketed by samples from this epoch. Never call
    # the motion helper's VIO extrapolation path or integrate across a reset.
    poses={row[0]:row[1] for row in q}
    a=poses[start] if start in poses else motion.vio_at(start)
    if a is None:return reject('VIO_ANCHOR_NOT_BRACKETED')
    Rgyro=np.eye(3);travel=0.;turn=0.;max_gap=0.;last=a
    # Integrate a few contiguous chunks, not a full history scan for each VIO
    # sample. Each chunk keeps the original gap checks and prediction horizon.
    edges=np.linspace(start,end,int(np.ceil((end-start)/.55))+1)
    for left,right in zip(edges,edges[1:]):
        gyro=motion.gyro_path(float(left),float(right))
        if gyro is None:
            if diagnostic is not None:
                diagnostic['interval']=[float(left),float(right)]
                diagnostic['imu_ranges']={sensor:[rows[0][0],rows[-1][0],len(rows)] for sensor,rows in motion.imu.items() if rows}
            return reject('GYRO_COVERAGE')
        Rgyro=Rgyro@gyro['rotations'][-1];max_gap=max(max_gap,gyro['max_gap'])
    for left,right in zip(stamps,stamps[1:]):
        b=poses[right] if right in poses else motion.vio_at(right)
        if b is None:return reject('VIO_ENDPOINT_NOT_BRACKETED')
        D=np.linalg.inv(last)@b;dt=right-left
        distance=float(np.linalg.norm(D[:3,3]));angle=float(Rotation.from_matrix(D[:3,:3]).magnitude())
        if distance>.35*dt+.008 or angle>.7*dt+.025:return reject('VIO_INCREMENT_JUMP')
        travel+=distance;turn+=angle;last=b
    Dvio=np.linalg.inv(a)@last
    disagreement=float(Rotation.from_matrix(Rgyro.T@Dvio[:3,:3]).magnitude())
    v=np.asarray(anchor['velocity'],float)
    if not np.isfinite(v).all() or np.linalg.norm(v)>.5:return reject('ANCHOR_VELOCITY')
    if disagreement>.10:return reject('VIO_GYRO_DISAGREE')
    if travel>MAX_TRAVEL or turn>.30:return reject('TRAVEL_BUDGET')
    if np.linalg.norm(Dvio[:3,3]-v*(end-start))>.06+.10*(end-start):return reject('VIO_VELOCITY_DISAGREE')
    D=np.eye(4);D[:3,:3]=Rgyro;D[:3,3]=Dvio[:3,3]
    xy,yaw=budgets(age,travel,turn)
    if not np.isfinite(D).all() or xy>.14 or yaw>.18:return reject('UNCERTAINTY_BUDGET')
    return dict(D=D,estimate_mono=end,xy_budget_m=xy,yaw_budget_rad=yaw,
                evidence=dict(schema='s10.measured_odometry_bridge.v1',vio_epoch=int(epoch),
                    samples=len(stamps),start_mono=start,end_mono=end,path_length_m=travel,
                    turn_rad=turn,rotation_disagreement_rad=disagreement,max_imu_gap_s=max_gap,
                    extrapolated=False))
