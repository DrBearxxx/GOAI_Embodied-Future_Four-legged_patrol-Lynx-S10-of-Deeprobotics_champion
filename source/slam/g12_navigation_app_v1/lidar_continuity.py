"""A continuous SE(3) output with independent, redundant LiDAR references.

Use one healthy registration frame at a time instead of interleaving two
different absolute fits. The standby still registers and renews keyframes.
On failover, its measured relative motion transfers the output frame; its
absolute registration bias is never injected as robot displacement.
"""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation


def inverse(T):
    result=np.eye(4);result[:3,:3]=T[:3,:3].T
    result[:3,3]=-result[:3,:3]@T[:3,3]
    return result


def interpolate(a,b,u):
    result=np.eye(4)
    rot=Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec()
    result[:3,:3]=a[:3,:3]@Rotation.from_rotvec(u*rot).as_matrix()
    result[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
    return result


class ContinuousLidarFrame:
    def __init__(self,source_timeout_s=.25):
        self.source_timeout_s=source_timeout_s
        self.active=None;self.T=None;self.t=None;self.offsets={};self.history={}
        self.switches=0;self.diagnostics={}

    def reset_sensor(self,sensor):
        self.offsets.pop(sensor,None);self.history.pop(sensor,None)
        if self.active==sensor:self.active=None

    def seed(self,sensor,T,stamp):
        self.reset_sensor(sensor)
        self.offsets[sensor]=np.eye(4)
        self.history[sensor]=deque([(stamp,T.copy())],maxlen=16)
        if self.T is None:self.T=T.copy();self.t=stamp

    def raw_prediction(self,sensor,predicted):
        return inverse(self.offsets.get(sensor,np.eye(4)))@predicted

    def update(self,sensor,raw,stamp,predicted):
        history=self.history.setdefault(sensor,deque(maxlen=16))
        if history and stamp<=history[-1][0]:return None
        history.append((stamp,raw.copy()))
        standby=(self.active is not None and sensor!=self.active and
                 self.t is not None and stamp-self.t<=self.source_timeout_s)
        self.diagnostics=dict(method='continuous_source_frame',sensor=sensor,active_sensor=self.active,
            measurement_mono=stamp,standby=standby,source_switches=self.switches,
            raw_T=raw.tolist(),handover=None)
        if standby:return None
        if self.t is not None and stamp<=self.t:return None
        if sensor!=self.active:
            previous=self.active
            # A reference with an established output-frame transform remains
            # a geometric observation after a gap. Copying the motion prior
            # here used to erase a recovered rotation (including being righted).
            known=self.offsets.get(sensor)
            output=(known@raw if known is not None else raw.copy()) if self.T is None or known is not None else predicted.copy()
            method='registered_reference' if known is not None else ('initial' if self.T is None else 'motion_prediction')
            if self.T is not None:
                for (ta,a),(tb,b) in zip(history,list(history)[1:]):
                    if ta<=self.t<tb and tb-ta<=.6:
                        at_previous=interpolate(a,b,(self.t-ta)/(tb-ta))
                        output=self.T@inverse(at_previous)@raw
                        method='same_sensor_relative_motion'
                        break
            self.offsets[sensor]=output@inverse(raw)
            self.active=sensor;self.switches+=int(previous is not None)
            self.diagnostics['handover']=dict(previous_sensor=previous,method=method,
                removed_offset_m=float(np.linalg.norm(output[:3,3]-raw[:3,3])))
        else:output=self.offsets.get(sensor,np.eye(4))@raw
        self.T=output.copy();self.t=stamp
        self.diagnostics.update(active_sensor=self.active,source_switches=self.switches,output_T=output.tolist())
        return output
