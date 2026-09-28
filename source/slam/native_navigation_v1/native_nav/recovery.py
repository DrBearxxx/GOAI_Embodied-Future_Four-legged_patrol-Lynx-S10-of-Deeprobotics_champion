"""Fresh measured-map recovery evidence, not a timer-only permission to move."""
from collections import deque
import math
import numpy as np


def angle_distance(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))


class RecoveryWindow:
    def __init__(self):self.clear()

    def clear(self):
        self.since=None;self.samples=deque(maxlen=100);self.last_stamp=None

    def observe(self,now,snapshot,stationary,fresh_clear):
        s=snapshot['solution'];age=now-s['measurement_mono']
        if not stationary or not fresh_clear:
            self.clear();return False
        # DEGRADED intervals between actual map measurements need not erase
        # the window. Expired/lost evidence is rejected by the caller first.
        if self.since is None:self.since=now
        stamp=s['measurement_mono']
        if 0<=age<=.25 and stamp!=self.last_stamp:
            self.samples.append((stamp,np.asarray(s['measured_pose'],float)))
            self.last_stamp=stamp
        while self.samples and now-self.samples[0][0]>2.4:self.samples.popleft()
        if now-self.since<2. or age>.25 or len(self.samples)<6:return False
        times=np.array([t for t,p in self.samples]);poses=np.array([p for t,p in self.samples])
        if times[-1]-times[0]<1.5 or np.any(np.diff(times)<=0) or np.max(np.diff(times))>.55:return False
        center=np.median(poses[:,:3],axis=0)
        # No large correction is made acceptable just by waiting. All measured
        # samples in the window must remain inside this stationary envelope.
        if np.max(np.linalg.norm(poses[:,:2]-center[:2],axis=1))>.10:return False
        if np.max(abs(poses[:,2]-center[2]))>.10:return False
        return all(angle_distance(p[3],poses[-1,3])<=.12 for p in poses)
