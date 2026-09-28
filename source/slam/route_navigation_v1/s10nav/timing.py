"""Causal device clocks. Never rewrite a past measurement or consult bag-wide fits."""
from collections import deque
import numpy as np

class DeviceClock:
    def __init__(self, config):
        self.cfg=config; self.native=None; self.received=None; self.offset=None
        self.samples=deque(maxlen=400); self.stable_since=None; self.epoch=0; self.faults=0; self.reason='warming_up'

    def observe_imu(self, native, received):
        if not np.isfinite([native,received]).all():
            self.reason='nonfinite_clock'; return None
        if self.native is not None:
            dn, dr=native-self.native, received-self.received
            if dn == 0:
                self.reason='duplicate_stamp'; self.faults+=1; return None
            if dr < 0 or dn < 0 or (dr < .3 and abs(dn-dr)>self.cfg['jump_s']):
                self.epoch+=1; self.faults+=1; self.samples.clear(); self.stable_since=received
                self.offset=received-native; self.reason='clock_discontinuity'
                self.native,self.received=native,received
                return None
        self.native,self.received=native,received
        self.samples.append(received-native)
        if self.stable_since is None: self.stable_since=received
        target=float(np.quantile(self.samples,.05))
        if self.offset is None: self.offset=target
        # Slow tracking while warm; freeze after warmup rather than treating
        # arrival jitter as sensor motion or retroactively changing the past.
        if received-self.stable_since < self.cfg['warmup_s']:
            self.offset=target; self.reason='warming_up'; return None
        if abs(target-self.offset)>.06:
            self.epoch+=1; self.faults+=1; self.samples.clear(); self.offset=target
            self.stable_since=received; self.reason='clock_offset_drift'; return None
        t=native+self.offset
        if received-t > self.cfg['max_queue_age_s'] or t-received > .05:
            self.reason='imu_source_age'; self.faults+=1; return None
        self.reason='ok'; return t

    def map_stamp(self, native, now):
        if self.offset is None or self.stable_since is None or now-self.stable_since < self.cfg['warmup_s'] or self.reason != 'ok':
            return None
        return native+self.offset

    def status(self, now):
        age=None if self.received is None else max(0.,now-self.received)
        return dict(epoch=self.epoch,reason=self.reason,age_s=age,fault_count=self.faults,
                    ready=self.reason=='ok' and age is not None and age<.1)
