import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.clock import LiveClock
from s10nav.timing import DeviceClock
C=dict(warmup_s=.6,max_queue_age_s=.35,jump_s=.15)
class ClockTests(unittest.TestCase):
    def test_same_clock_mapping_for_steady_stream(self):
        a,b=LiveClock(C),DeviceClock(C)
        for i in range(3000):
            native=100+i*.005;received=i*.005+.003+(i%7)*.0001
            self.assertEqual(a.observe_imu(native,received),b.observe_imu(native,received))
    def test_jump_and_duplicate_checked_between_quantiles(self):
        c=LiveClock(C)
        for i in range(200):c.observe_imu(100+i*.005,i*.005)
        old=c.epoch;self.assertIsNone(c.observe_imu(102.,1.));self.assertGreater(c.epoch,old)
        self.assertIsNone(c.observe_imu(102.,1.005));self.assertEqual(c.reason,'duplicate_stamp')
    def test_stale_packet_never_freshened(self):
        c=LiveClock(C)
        for i in range(200):c.observe_imu(100+i*.005,i*.005)
        self.assertIsNone(c.observe_imu(101.,1.5))
        self.assertIn(c.reason,['clock_offset_drift','imu_source_age'])
if __name__=='__main__':unittest.main(verbosity=2)
