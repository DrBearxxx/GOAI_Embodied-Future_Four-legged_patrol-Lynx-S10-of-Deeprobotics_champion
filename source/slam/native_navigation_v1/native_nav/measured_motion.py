"""Actual same-epoch VIO increments cross-checked against contiguous gyro."""
import numpy as np
from scipy.spatial.transform import Rotation


def increment(motion,start,end):
    if not .01<=end-start<=.60:return None
    q=list(motion.vio)
    if len(q)<2 or q[0][0]>start or q[-1][0]<end:return None
    used=[row for row in q if start-.16<=row[0]<=end+.16]
    if len(used)<2 or len({row[2] for row in used})!=1:return None
    relevant=[row for row in used if start<=row[0]<=end]
    times=[start,*[row[0] for row in relevant if start<row[0]<end],end]
    if any(b<=a or b-a>min(.12,motion.cfg['max_vio_gap_s']) for a,b in zip(times,times[1:])):return None
    a,b=motion.vio_at(start),motion.vio_at(end);g=motion.gyro_path(start,end)
    if a is None or b is None or g is None:return None
    D=np.linalg.inv(a)@b;R=g['rotations'][-1]
    if Rotation.from_matrix(R.T@D[:3,:3]).magnitude()>.08:return None
    if np.linalg.norm(D[:3,3])>.5*(end-start):return None
    D[:3,:3]=R
    return D
