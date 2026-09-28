"""Lightning-to-navigation boundary; ROS-independent and replayable.

Native sensor stamps are mapped by the navigation engine's SAME clock used by
the map matcher. Neither DDS receipt nor publication time is a measurement.
The sidecar publishes T_world_imu; this module applies the calibrated lever arm
exactly once and expresses velocity in the current robot body frame.
"""
from collections import deque
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

SCHEMA = 'goai.lightning.odometry.v1'
TOPIC = '/wym/navigation/lightning/state'


def rigid(value):
    T = np.asarray(value, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-3)
            or abs(np.linalg.det(T[:3, :3])-1) > 1e-3):
        raise ValueError('INVALID_RIGID_TRANSFORM')
    return T.copy()


def load_config(root):
    path = Path(root)/'odometry.json'
    config = json.loads(path.read_text()) if path.exists() else {'backend': 'gicp'}
    if config.get('backend') not in ('gicp', 'lightning'):
        raise ValueError('UNKNOWN_ODOMETRY_BACKEND')
    if config['backend'] == 'lightning':
        if config.get('sensor') not in ('front', 'rear'):
            raise ValueError('INVALID_LIGHTNING_SENSOR')
        rigid(config['T_body_imu'])
    return config


class LightningOdometry:
    def __init__(self, config):
        self.sensor = config['sensor']
        self.T_imu_body = np.linalg.inv(rigid(config['T_body_imu']))
        self.epoch = None
        self.retired = deque(maxlen=64)
        self.origin = None
        self.history = deque(maxlen=8)
        self.latest = None
        self.accepted = 0
        self.rejected = 0
        self.path = 0.
        self.reason = 'WAIT_LIGHTNING'

    def accept(self, payload, measurement_mono, now):
        """Reject malformed/reordered data, never confidence, speed or IMU magnitude."""
        try:
            return self._accept(payload, measurement_mono, now)
        except (ValueError, TypeError, KeyError, OverflowError, np.linalg.LinAlgError) as exc:
            self.rejected += 1
            self.reason = str(exc)[:160]
            return None

    def _accept(self, p, t, now):
        if p['schema'] != SCHEMA or p['sensor'] != self.sensor:
            raise ValueError('LIGHTNING_SOURCE_MISMATCH')
        epoch = str(p['epoch'])
        if not epoch or len(epoch) > 160 or epoch in self.retired:
            raise ValueError('RETIRED_LIGHTNING_EPOCH')
        stamp = int(p['stamp_ns'])
        if stamp <= 0:
            raise ValueError('INVALID_NATIVE_STAMP')
        if t is None:
            raise ValueError('WAIT_SENSOR_CLOCK')
        t = float(t)
        if not math.isfinite(t) or t > now+.025:
            raise ValueError('INVALID_MEASUREMENT_TIME')
        # A queued sample is retained with its true old time, never made fresh.
        T = rigid(p['T_world_imu']) @ self.T_imu_body
        if epoch == self.epoch and self.latest:
            if stamp <= self.latest['native_stamp_ns'] or t <= self.latest['t']:
                raise ValueError('OUT_OF_ORDER_LIGHTNING')
        if epoch != self.epoch:
            if self.epoch is not None:
                self.retired.append(self.epoch)
            self.epoch = epoch
            self.origin = np.linalg.inv(T)
            self.history.clear()
            self.accepted = 0
            self.path = 0.
            self.latest = None
        T = self.origin @ T
        if self.latest:
            self.path += float(np.linalg.norm(T[:3, 3]-np.asarray(self.latest['T'])[:3, 3]))
        velocity = np.zeros(3)
        omega = np.zeros(3)
        # Estimate measured motion over <= 0.3 s, independent of control commands.
        recent = [(ts, pose) for ts, pose in self.history if 0 < t-ts <= .3]
        if recent:
            ts, before = recent[0]
            velocity = T[:3, :3].T @ (T[:3, 3]-before[:3, 3])/(t-ts)
            omega = Rotation.from_matrix(before[:3, :3].T @ T[:3, :3]).as_rotvec()/(t-ts)
        self.accepted += 1
        self.history.append((t, T))
        self.latest = dict(T=T.tolist(), t=t, epoch=epoch, path=self.path,
            accepted=self.accepted, velocity=velocity.tolist(), angular_velocity=omega.tolist(),
            native_stamp_ns=stamp, sensor=self.sensor, source='lightning',
            receipt_mono=now, measurement_age_s=now-t,
            quality={'level': 'tracking', 'method': 'lightning_lio'},
            continuity={'method': 'single_lio_frame', 'sensor': self.sensor, 'epoch': epoch})
        self.reason = ''
        return self.latest

    def status(self, now):
        return dict(backend='lightning', sensor=self.sensor, epoch=self.epoch,
            accepted=self.accepted, rejected=self.rejected, reason=self.reason,
            age_s=None if self.latest is None else now-self.latest['t'],
            odom=self.latest)
