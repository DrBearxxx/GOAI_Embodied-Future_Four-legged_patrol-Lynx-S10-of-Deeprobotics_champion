import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from native_nav.bootstrap import INDOOR
from native_nav.robust_timing import SlewedClock, RobustContinuity
from native_nav.bounded_log import BoundedLog
from native_nav.watchdog import HeartbeatWatchdog
from indoor.continuity import Continuity,RedundantMotion
import numpy as np


CFG=json.loads((INDOOR/'config.json').read_text())


def make_motion():
    return RedundantMotion(CFG['motion'],{s:np.eye(4) for s in ('front','rear','camera')})


def anchor(t=1):
    return dict(T=np.eye(4).tolist(),measurement_mono=t,velocity=[0,0,0],confirmed=True,generation=[1],healthy_lidars=['front','rear'])


class RobustTimingTests(unittest.TestCase):
    def test_future_head_uses_past_measured_endpoint_not_now(self):
        m=make_motion()
        for s in m.imu:
            for t in np.arange(.9,1.154,.005):m.add_imu(s,float(t),np.zeros(3),np.array([0,0,9.81]))
        old=Continuity();new=RobustContinuity()
        old.update(anchor(),m);new.update(anchor(),m)
        self.assertEqual(old.estimate(1.148,m)['reason'],'NO_CONTIGUOUS_GYRO')
        result=new.estimate(1.148,m)
        self.assertEqual(result['mode'],'TRACKING')
        self.assertAlmostEqual(result['estimate_mono'],1.145)
        self.assertGreater(result['imu_age_s'],0)

    def test_common_gap_still_stops(self):
        m=make_motion()
        for s in m.imu:
            for t in np.r_[np.arange(.9,1.041,.005),np.arange(1.1,1.156,.005)]:
                m.add_imu(s,float(t),np.zeros(3),np.array([0,0,9.81]))
        c=RobustContinuity();c.update(anchor(),m)
        self.assertEqual(c.estimate(1.16,m)['reason'],'IMU_PATH_HAS_GAP_OR_NO_COVERAGE')

    def test_short_outage_no_turn_long_outage_stops(self):
        m=make_motion()
        for t in np.arange(.9,1.201,.005):m.add_imu('rear',float(t),np.zeros(3),np.array([0,0,9.81]))
        c=RobustContinuity();c.update(anchor(),m)
        result=c.estimate(1.26,m)
        self.assertEqual((result['mode'],result['max_wz']),('PREDICT_ONLY',0))
        self.assertEqual(c.estimate(1.29,m)['reason'],'NO_FRESH_PAST_IMU')
        self.assertEqual(c.estimate(1.56,m)['reason'],'MAP_OBSERVATION_EXPIRED')

    def test_clock_slew_never_rewrites_or_reorders(self):
        c=SlewedClock(CFG['clock']);out=[];prior_offset=None;prior_received=None
        for i in range(12000):
            native=100+i*.005
            received=i*.005+max(.001,.008-max(0,i-150)*.000001)
            previous=c.offset
            t=c.observe_imu(native,received)
            if t is not None:out.append(t)
            if i>150 and previous is not None:
                self.assertLessEqual(abs(c.offset-previous),.00002000001)
        self.assertTrue(all(a<b for a,b in zip(out,out[1:])))
        self.assertEqual(c.epoch,0)
        self.assertLess(abs(c.offset-(received-native)),.001)
        self.assertEqual(c.reason,'ok')

    def test_duplicates_jumps_and_stale_are_not_hidden(self):
        c=SlewedClock(CFG['clock'])
        for i in range(200):c.observe_imu(100+i*.005,i*.005+.003)
        self.assertIsNone(c.observe_imu(100.995,.999))
        self.assertEqual(c.reason,'duplicate_stamp')
        self.assertIsNone(c.observe_imu(10,1.))
        self.assertGreater(c.epoch,0)
        d=SlewedClock(CFG['clock'])
        for i in range(200):d.observe_imu(100+i*.005,i*.005+.003)
        self.assertIsNone(d.observe_imu(101,1.5))


class BoundedLogTests(unittest.TestCase):
    def test_rotation_is_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'diagnostic.jsonl';sink=BoundedLog(p,100,2)
            for _ in range(50):sink.write('x'*29+'\n')
            sink.close()
            self.assertLessEqual(sum(f.stat().st_size for f in Path(temp).iterdir()),300)
            self.assertEqual(len(list(Path(temp).iterdir())),3)

    def test_disk_failure_does_not_raise(self):
        with tempfile.TemporaryDirectory() as temp:
            sink=BoundedLog(Path(temp)/'diagnostic.jsonl')
            with patch.object(Path,'open',side_effect=OSError('simulated disk full')):
                self.assertEqual(sink.write('x\n'),0)
            self.assertEqual(sink.errors,1)
            sink.close()


class WatchdogTests(unittest.TestCase):
    def test_startup_and_loop_timeout_are_separate(self):
        w=HeartbeatWatchdog(100)
        self.assertFalse(w.expired(144.9))
        self.assertTrue(w.expired(145.1))
        w.beat(146)
        self.assertFalse(w.expired(150.9))
        self.assertTrue(w.expired(151.1))

    def test_no_sensor_input_does_not_trigger_restart(self):
        w=HeartbeatWatchdog(0)
        for i in range(3600):
            w.beat(float(i))
            self.assertFalse(w.expired(i+.9))
