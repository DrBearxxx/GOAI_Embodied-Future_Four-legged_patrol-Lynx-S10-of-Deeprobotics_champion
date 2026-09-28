"""Regressions for the 21 Sep stairs / fall-and-righting field log."""
import math,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from registration_prior import RegistrationPrior
from registration_motion import RegistrationMotion
import test_registration_motion
import test_keyframes
from lidar_continuity import ContinuousLidarFrame
from seed_matcher import SeedMatcher


class AttitudeRecoveryTests(unittest.TestCase):
    def test_gyro_is_accumulated_through_long_geometry_outage(self):
        motion=test_registration_motion.RegistrationMotionTests().motion();prior=RegistrationPrior();T=np.eye(4)
        prior.predict(T,10.,10.,motion,np.zeros(3))
        for i in range(1,301):
            end=10.+i*.1
            # A complete roll, with a deliberately large old translation.
            for t in np.arange(end-.105,end+.001,.005):
                motion.add_imu('front',t,np.array([2*math.pi/30,0.,0.]),np.zeros(3))
            result=prior.predict(T,10.,end,motion,np.array([2.,0.,0.]))
        np.testing.assert_allclose(result[:3,:3],np.eye(3),atol=.003)
        np.testing.assert_allclose(result[:3,3],T[:3,3],atol=1e-10)
        self.assertEqual(prior.observed_t,10.)  # No invented measurement.

    def test_interleaved_lidars_do_not_integrate_the_same_imu_interval_twice(self):
        motion=test_registration_motion.RegistrationMotionTests().motion();prior=RegistrationPrior();T=np.eye(4)
        for t in np.arange(9.9,10.501,.005):motion.add_imu('front',t,np.array([0.,0.,1.]),np.zeros(3))
        for end in (10.1,10.2,10.15,10.3):result=prior.predict(T,10.,end,motion,np.zeros(3))
        self.assertAlmostEqual(Rotation.from_matrix(result[:3,:3]).as_rotvec()[2],.3,places=8)

    def test_missing_imu_does_not_block_lidar_and_gravity_does_not_flatten_stairs(self):
        motion=test_registration_motion.RegistrationMotionTests().motion();prior=RegistrationPrior();T=np.eye(4)
        T[:3,:3]=Rotation.from_euler('xyz',[.1,-.3,.7]).as_matrix()
        result=prior.predict(T,10.,10.1,motion,np.zeros(3))
        np.testing.assert_allclose(result,T,atol=1e-10)
        for t in np.arange(10.,10.201,.005):
            motion.add_imu('front',t,np.zeros(3),T[:3,:3].T@np.array([0.,0.,9.81]))
        prior.predict(T,10.1,10.2,motion,np.zeros(3))
        np.testing.assert_allclose(prior.correct_tilt(T,.1),T,atol=1e-10)
        tilted=T.copy();tilted[:3,:3]=Rotation.from_euler('x',.2).as_matrix()@T[:3,:3]
        corrected=prior.correct_tilt(tilted,.1)
        before=np.linalg.norm(tilted[:3,:3]@prior.gravity['unit']-prior.up)
        after=np.linalg.norm(corrected[:3,:3]@prior.gravity['unit']-prior.up)
        self.assertLess(after,before)

    def test_clock_epoch_does_not_discard_spatial_reference(self):
        local,cloud=test_keyframes.KeyframeTests().setup_local();local.keys['front']=(0,10.)
        local.engine.clocks['front'].epoch=1
        measured=local.event(test_keyframes.KeyframeTests().event(cloud,10.2),10.23)
        self.assertIsNotNone(measured);self.assertEqual(measured['accepted'],6)
        self.assertEqual(local.frames['front']['t'],9.8)

    def test_new_reference_handover_keeps_registered_rotation(self):
        f=ContinuousLidarFrame();fallen=np.eye(4);fallen[:3,:3]=Rotation.from_euler('x',2.4).as_matrix()
        f.seed('front',fallen,10.);f.update('front',fallen,10.1,fallen)
        upright=np.eye(4);upright[:3,3]=fallen[:3,3]
        f.seed('rear',upright,11.)
        observed=upright.copy();observed[:3,:3]=Rotation.from_euler('z',.12).as_matrix()
        result=f.update('rear',observed,11.1,fallen)
        np.testing.assert_allclose(result,observed,atol=1e-10)
        self.assertEqual(f.diagnostics['handover']['method'],'registered_reference')

    def test_recovery_seed_is_not_a_pose_observation(self):
        local,cloud=test_keyframes.KeyframeTests().setup_local();local.last_sensor='front'
        motion=test_registration_motion.RegistrationMotionTests().motion();local.engine.motion=motion
        for t in np.arange(11.,11.301,.005):motion.add_imu('front',t,np.zeros(3),np.array([0.,0.,9.81]))
        with patch('local_odometry.small_gicp.align',return_value=test_keyframes.KeyframeTests().rejected()):
            with self.assertRaises(ValueError):local.event(test_keyframes.KeyframeTests().event(cloud,11.2),11.23)
        self.assertEqual(local.t,10.);self.assertEqual(local.accepted,5)
        self.assertEqual(local.quality['recovery']['method'],'attitude_aligned_reference')


class HeadingCandidateTests(unittest.TestCase):
    def matcher(self):
        m=SeedMatcher.__new__(SeedMatcher);m.scope=dict(xyz=[0,0,0],radius=3.,height_tolerance=.8,heading_prior=0.)
        return m

    def test_heading_prior_ranks_near_equal_fits_but_does_not_veto_geometry(self):
        m=self.matcher()
        def result(yaw,score):
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
            return dict(accepted=True,T=T,score=score,reasons=[],quality=dict(overlap=.8,median_nn_m=.1,condition=.01))
        with patch('seed_matcher.IndoorMatcher.register',return_value=result(math.pi,.52)):
            opposite=m.register(np.zeros((10,3)),np.eye(4),True)
        with patch('seed_matcher.IndoorMatcher.register',return_value=result(0.,.50)):
            continuous=m.register(np.zeros((10,3)),np.eye(4),True)
        self.assertTrue(opposite['accepted']);self.assertGreater(continuous['score'],opposite['score'])
        with patch('seed_matcher.IndoorMatcher.register',return_value=result(math.pi,.9)):
            strong=m.register(np.zeros((10,3)),np.eye(4),True)
        self.assertGreater(strong['score'],continuous['score'])

    def test_prior_is_an_additional_search_candidate_not_an_assigned_pose(self):
        m=self.matcher();yaw_calls=[]
        def register(points,T,wide):
            yaw=math.atan2(T[1,0],T[0,0]);yaw_calls.append(yaw)
            return dict(accepted=True,T=T,score=1.-abs(yaw),pose=[0,0,0,yaw])
        m.register=register
        with patch('seed_matcher.IndoorMatcher.relocalize',return_value=dict(accepted=False,reasons=['ambiguous_global_candidates'])):
            r=m.relocalize(np.zeros((10,3)),'front',np.eye(3))
        self.assertEqual(len(yaw_calls),3);self.assertTrue(r['accepted'])
        self.assertAlmostEqual(r['pose'][3],0.);self.assertTrue(r['heading_prior_used'])


if __name__=='__main__':unittest.main()
