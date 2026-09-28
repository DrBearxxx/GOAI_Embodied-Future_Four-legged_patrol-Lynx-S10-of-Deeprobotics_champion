from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .util import inv

class Motion:
    def __init__(self, cfg, extrinsics):
        self.cfg=cfg; self.X=extrinsics
        self.imu={s:deque(maxlen=1400) for s in ('front','rear','camera')}
        self.vio=deque(maxlen=200); self.vio_epoch=0; self.vio_rejects=0
        self.gravity_anchor=None

    def clear(self, sensor):
        self.imu[sensor].clear()
        self.gravity_anchor=None
        if sensor=='camera': self.vio.clear(); self.vio_epoch+=1

    def add_imu(self, sensor, t, gyro, accel):
        q=self.imu[sensor]
        if q and t<=q[-1][0]: return
        R=self.X[sensor][:3,:3]
        q.append((t,R@gyro,R@accel))

    def add_vio(self, t, T_camera):
        T=T_camera @ inv(self.X['camera'])
        if self.vio:
            dt=t-self.vio[-1][0]
            D=inv(self.vio[-1][1])@T
            if dt<=0 or dt>self.cfg['max_vio_gap_s'] or np.linalg.norm(D[:3,3])>self.cfg['max_speed_mps']*dt+.03 or Rotation.from_matrix(D[:3,:3]).magnitude()>self.cfg['max_vio_rate_radps']*dt+.05:
                self.vio.clear(); self.vio_epoch+=1; self.vio_rejects+=1
                # A jump establishes a new origin only; never integrates across it.
        self.vio.append((t,T,self.vio_epoch))

    def gyro_path(self, start, end, preferred=None):
        if end<start or end-start>self.cfg['max_predict_s']+.01: return None
        order=list(dict.fromkeys(([preferred] if preferred else [])+['front','rear','camera']))
        for sensor in order:
            q=list(self.imu[sensor])
            if len(q)<2: continue
            ts=np.array([v[0] for v in q]); i=max(0,np.searchsorted(ts,start)-1); j=np.searchsorted(ts,end)+1
            q=q[i:j]; ts=ts[i:j]
            if len(ts)<2 or ts[0]>start+.002 or ts[-1]<end-.002: continue
            if np.max(np.diff(ts))>self.cfg['max_imu_gap_s']: continue
            times=np.unique(np.r_[start,ts[(ts>start)&(ts<end)],end])
            gyro=np.array([v[1] for v in q]); rates=np.column_stack([np.interp(times,ts,gyro[:,k]) for k in range(3)])
            R=np.eye(3); rotations=[R.copy()]
            for k,dt in enumerate(np.diff(times)):
                R=R@Rotation.from_rotvec((rates[k]+rates[k+1])*.5*dt).as_matrix(); rotations.append(R.copy())
            if len(times)<2: return None
            return dict(sensor=sensor,times=times,rotations=np.array(rotations),max_gap=float(np.max(np.diff(ts))))
        return None

    def vio_at(self,t):
        q=list(self.vio)
        if len(q)<2: return None
        ts=np.array([v[0] for v in q]); j=int(np.searchsorted(ts,t))
        if j==0:
            return q[0][1] if abs(ts[0]-t)<.002 else None
        if j==len(ts):
            if abs(ts[-1]-t)<.002:return q[-1][1]
            # Camera VIO normally arrives later than LiDAR. Only extrapolate
            # the most recent observed increment for <=100 ms, never wait for
            # future VIO or use bag-wide interpolation.
            a,b=q[-2:];dt=b[0]-a[0];extra=t-b[0]
            if extra>self.cfg.get('max_vio_extrapolation_s',.1) or dt>self.cfg['max_vio_gap_s'] or a[2]!=b[2]:return None
            T=b[1].copy();T[:3,3]+=(b[1][:3,3]-a[1][:3,3])*(extra/dt)
            dr=Rotation.from_matrix(a[1][:3,:3].T@b[1][:3,:3]).as_rotvec()
            T[:3,:3]=b[1][:3,:3]@Rotation.from_rotvec(dr*(extra/dt)).as_matrix()
            return T
        a,b=q[j-1],q[j]; dt=b[0]-a[0]
        if dt>self.cfg['max_vio_gap_s'] or a[2]!=b[2]: return None
        u=(t-a[0])/dt; T=np.eye(4)
        T[:3,3]=(1-u)*a[1][:3,3]+u*b[1][:3,3]
        T[:3,:3]=Slerp([0,1],Rotation.from_matrix([a[1][:3,:3],b[1][:3,:3]]))([u]).as_matrix()[0]
        return T

    def relative(self,start,end,velocity_body=None):
        if abs(end-start)<.001: return np.eye(4),'same_time'
        g=self.gyro_path(start,end)
        if g is None: return None,'no_contiguous_gyro_prediction'
        T=np.eye(4); T[:3,:3]=g['rotations'][-1]
        a,b=self.vio_at(start),self.vio_at(end)
        v=np.zeros(3) if velocity_body is None else np.asarray(velocity_body)
        predicted=v*(end-start)
        if a is not None and b is not None:
            D=inv(a)@b
            angle=Rotation.from_matrix(T[:3,:3].T@D[:3,:3]).magnitude()
            if angle<.2 and np.linalg.norm(D[:3,3]-predicted)<.35:
                w=self.cfg['vio_translation_weight']; T[:3,3]=(1-w)*predicted+w*D[:3,3]
                return T,'gyro_plus_gated_vio_translation'
        T[:3,3]=predicted
        return T,'gyro_plus_bounded_constant_velocity'

    def deskew(self, points, rel, start, X, velocity_body):
        rel=np.asarray(rel,dtype=np.float64)
        end=start+float(np.max(rel)); g=self.gyro_path(start,end)
        if g is None: return None,'no_scan_imu_coverage'
        bins=np.unique(np.round(rel/.005)*.005)
        tq=np.clip(start+bins,start,end)
        rotations=Slerp(g['times'],Rotation.from_matrix(g['rotations']))(tq).as_matrix()
        R_end=g['rotations'][-1]; p=points@X[:3,:3].T+X[:3,3]; out=np.empty_like(p)
        v=np.asarray(velocity_body)
        for k,b in enumerate(bins):
            mask=np.abs(np.round(rel/.005)*.005-b)<1e-6
            out[mask]=p[mask]@(R_end.T@rotations[k]).T + R_end.T@(v*(tq[k]-end))
        return out,dict(gyro_sensor=g['sensor'],max_imu_gap_s=g['max_gap'],translation='past_velocity_approximation',quantization_s=.005)

    def anchor_gravity(self,t,R):
        self.gravity_anchor=(t,np.asarray(R).copy(),t)

    def gravity(self,now):
        for sensor in ('front','rear','camera'):
            q=[v for v in self.imu[sensor] if now-.5<=v[0]<=now]
            if len(q)<30 or now-q[-1][0]>.05: continue
            acc=np.array([v[2] for v in q]); gyro=np.array([v[1] for v in q]); a=np.median(acc,axis=0)
            if abs(np.linalg.norm(a)-9.81)>1.0 or np.max(np.std(acc,axis=0))>1.0 or np.median(np.linalg.norm(gyro,axis=1))>.5: continue
            # Only a gravity-level initialization for place retrieval; no forced flat motion.
            unit=a/np.linalg.norm(a); axis=np.cross(unit,[0,0,1]); length=np.linalg.norm(axis); dot=np.clip(unit[2],-1,1)
            R=Rotation.from_rotvec(axis/max(length,1e-12)*np.arctan2(length,dot)).as_matrix() if length>1e-8 else (np.eye(3) if dot>0 else Rotation.from_euler('x',np.pi).as_matrix())
            self.gravity_anchor=(now,R,now)
            return R
        # A prior geometric tilt can be propagated through CONTIGUOUS gyro
        # during walking. It is only a retrieval hypothesis, not a valid pose.
        if self.gravity_anchor is not None:
            t,R,origin=self.gravity_anchor
            if now<t or now-origin>self.cfg.get('gravity_propagation_limit_s',20):return None
            g=self.gyro_path(t,now)
            if g is not None:
                R=R@g['rotations'][-1];self.gravity_anchor=(now,R,origin)
                yaw=np.arctan2(R[1,0],R[0,0])
                return Rotation.from_euler('z',-yaw).as_matrix()@R
        return None
