import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from lidar_continuity import ContinuousLidarFrame,inverse


def pose(t,speed=2.):
    T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.08*t,-.03*t,.2*t]).as_matrix()
    T[:3,3]=[speed*t,.1*t,.15*t];return T


class LidarContinuityTests(unittest.TestCase):
    def test_interleaved_biased_sensors_do_not_replace_continuous_pose(self):
        f=ContinuousLidarFrame();bias=pose(1.);bias[:3,3]=[1.8,-.5,.2]
        f.seed('front',pose(0),0.)
        for i in range(1,50):
            t=i*.1;truth=pose(t)
            if i%2:
                out=f.update('front',truth,t,truth)
                np.testing.assert_allclose(out,truth,atol=1e-10)
            else:self.assertIsNone(f.update('rear',bias@truth,t,truth))
        self.assertEqual(f.switches,0);self.assertEqual(f.active,'front')

    def test_failover_transfers_relative_motion_without_frame_bias(self):
        f=ContinuousLidarFrame();f.seed('front',pose(0),0.)
        bias=pose(3.);bias[:3,3]=[8,-3,1]
        f.update('front',pose(.1),.1,pose(.1))
        f.update('rear',bias@pose(.2),.2,pose(.2))
        f.update('front',pose(.3),.3,pose(.3))
        f.update('rear',bias@pose(.4),.4,pose(.4))
        out=f.update('rear',bias@pose(.6),.6,pose(.6))
        np.testing.assert_allclose(out,pose(.6),atol=1e-9)
        self.assertEqual(f.active,'rear');self.assertEqual(f.switches,1)
        np.testing.assert_allclose(f.update('rear',bias@pose(.7),.7,pose(.7)),pose(.7),atol=1e-9)
        self.assertIsNone(f.update('front',pose(.8),.8,pose(.8)))
        np.testing.assert_allclose(f.raw_prediction('rear',pose(.9)),bias@pose(.9),atol=1e-9)

    def test_failover_uses_measured_increment_instead_of_stale_motion_prior(self):
        f=ContinuousLidarFrame();f.seed('front',pose(0),0.)
        f.update('front',pose(.1),.1,pose(.1))
        bias=pose(2.)
        f.update('rear',bias@pose(.2),.2,pose(.2))
        f.update('front',pose(.3),.3,pose(.3))
        # Delayed processing: standby has an actual measurement bracketing
        # the last output, so even a frozen IMU seed does not discard travel.
        out=f.update('rear',bias@pose(.6),.6,pose(.3))
        np.testing.assert_allclose(out,pose(.6),atol=1e-9)
        self.assertEqual(f.diagnostics['handover']['method'],'same_sensor_relative_motion')

    def test_no_speed_or_tilt_clamp(self):
        f=ContinuousLidarFrame();f.seed('front',pose(0),0.)
        for t in (.1,.2,.4):
            truth=pose(t,speed=8.)
            np.testing.assert_allclose(f.update('front',truth,t,truth),truth,atol=1e-10)

    def test_clock_reset_reseeds_only_affected_sensor(self):
        f=ContinuousLidarFrame();f.seed('front',pose(0),0.)
        f.update('front',pose(.1),.1,pose(.1))
        f.seed('rear',pose(.2),.2)
        self.assertEqual(f.active,'front');np.testing.assert_allclose(f.T,pose(.1))
        f.seed('front',pose(.3),.3)
        out=f.update('front',pose(.4),.4,pose(.4))
        np.testing.assert_allclose(out,pose(.4),atol=1e-10)

    def test_same_source_recovers_large_real_movement_without_speed_gate(self):
        f=ContinuousLidarFrame();f.seed('front',pose(0),0.)
        f.update('front',pose(.1),.1,pose(.1))
        out=f.update('front',pose(1.5),1.5,pose(.1))
        np.testing.assert_allclose(out,pose(1.5),atol=1e-10)

    def test_duplicate_does_not_refresh_output_time(self):
        f=ContinuousLidarFrame();f.seed('front',pose(0),0.)
        f.update('front',pose(.1),.1,pose(.1))
        self.assertIsNone(f.update('front',pose(.1),.1,pose(.1)));self.assertEqual(f.t,.1)


if __name__=='__main__':unittest.main()
