"""Same clock gates, amortized drift quantile for high-rate live IMU streams."""
import math
from . import bootstrap
from s10nav.timing import DeviceClock

class LiveClock(DeviceClock):
    def __init__(self,config):super().__init__(config);self.last_quantile=-1e30
    def observe_imu(self,native,received):
        # Discontinuities and sample age are always checked per packet. Only
        # the slow offset-drift statistic runs at 20 Hz after full warmup.
        stable=self.stable_since is not None and received-self.stable_since>=self.cfg['warmup_s']+.01
        fast=stable and received-self.last_quantile<.05 and self.native is not None and self.offset is not None
        if not fast:
            self.last_quantile=received;return super().observe_imu(native,received)
        if not math.isfinite(native) or not math.isfinite(received):self.reason='nonfinite_clock';return None
        dn,dr=native-self.native,received-self.received
        if dn==0:self.reason='duplicate_stamp';self.faults+=1;return None
        if dr<0 or dn<0 or dr<.3 and abs(dn-dr)>self.cfg['jump_s']:
            self.last_quantile=received;return super().observe_imu(native,received)
        self.native,self.received=native,received;self.samples.append(received-native);t=native+self.offset
        if received-t>self.cfg['max_queue_age_s'] or t-received>.05:self.reason='imu_source_age';self.faults+=1;return None
        self.reason='ok';return t

def use_live_clocks(engine):
    engine.clocks={s:LiveClock(engine.cfg['clock']) for s in ['front','rear','camera']}
    return engine
