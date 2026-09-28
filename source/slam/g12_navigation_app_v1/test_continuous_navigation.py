"""Continuous measured navigation through map/IMU noise and recovery."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
import paths
from continuous_pose import ContinuousPose
from pose_source import PoseSources
from navigation import Navigator
from mission import Mission
from pose_pid import PosePID
import test_continuity_v2 as fixture
import test_keyframes
import test_unified
import test_registration_motion


class ContinuousNavigationTests(unittest.TestCase):
    def fusion(self):
        c=ContinuousPose()
        for i in range(5):
            t=1.+i*.1;s=fixture.sample(t,i*.2);s['velocity']=[2.,0.,0.]
            c.odometry(s)
        c.map_update(fixture.anchor(1.4,.8));c.estimate(1.4)
        return c

    def test_long_map_dropout_keeps_measured_pose_and_native_speed(self):
        c=self.fusion()
        for i in range(1,401):
            t=1.4+i*.1;x=.8+i*.2;c.odometry(fixture.sample(t,x))
            s=c.estimate(t)
            self.assertIn(s['mode'],('TRACKING','ODOM_BRIDGE'))
            self.assertTrue(s['arrival_valid']);self.assertIsNone(s['max_vx'])
            self.assertAlmostEqual(s['pose'][0],x);self.assertAlmostEqual(s['measurement_mono'],t)
        self.assertGreater(s['xy_budget_m'],.2);self.assertGreater(s['map_age_s'],39.)

    def test_repeated_map_outliers_cannot_stop_or_move_measured_lidar(self):
        c=self.fusion();p=PoseSources()
        for i in range(1,8):
            t=1.4+i*.1;x=.8+i*.2;c.odometry(fixture.sample(t,x))
            a=fixture.anchor(t,x+8.,(1,i+1))
            self.assertFalse(c.map_update(a))
            s=p.choose(c.estimate(t),a,t)
            self.assertIn(s['mode'],('TRACKING','ODOM_BRIDGE'))
            self.assertAlmostEqual(s['pose'][0],x)
        self.assertEqual(c.map_t,1.4)

    def test_map_reacquisition_filters_correction_without_changing_frame(self):
        c=self.fusion();p=PoseSources();a=fixture.anchor(1.4,.8)
        first=p.choose(c.estimate(1.4),a,1.4);rev=c.revision
        c.odometry(fixture.sample(1.5,1.));a=fixture.anchor(1.5,1.1,(1,99))
        self.assertFalse(c.map_update(a));s=c.estimate(1.5)
        self.assertEqual(c.revision,rev)
        self.assertLess(s['pose'][0]-1.,.013)

    def test_rejected_map_does_not_reenter_via_fallback(self):
        c=self.fusion();p=PoseSources();p.choose(c.estimate(1.4),fixture.anchor(1.4,.8),1.4)
        c.odometry(fixture.sample(1.5,1.));a=fixture.anchor(1.5,8.)
        c.map_update(a);s=c.estimate(2.)
        self.assertEqual(s['mode'],'LOST');self.assertEqual(p.map_estimate['measurement_mono'],1.4)

    def test_fast_lidar_prediction_without_imu_is_marked_and_bounded(self):
        c=self.fusion();s=c.estimate(1.7)
        self.assertEqual(s['mode'],'PREDICT_ONLY');self.assertFalse(s['arrival_valid'])
        self.assertEqual(s['measurement_mono'],1.4);self.assertAlmostEqual(s['pose'][0],1.4)
        self.assertIsNone(s['max_vx']);self.assertEqual(s['prediction']['method'],'accepted_lidar_constant_twist')
        self.assertEqual(c.estimate(1.86)['mode'],'LOST')
        self.assertAlmostEqual(s['measurement_pose'][0],.8)

    def test_pid_prediction_does_not_become_a_new_velocity_measurement(self):
        p=PosePID();p.step([0,0,0,0],[1,0],1.,measurement_mono=1.,measurement_pose=[0,0,0,0])
        p.step([.3,0,0,0],[1,0],1.3,measurement_mono=1.,measurement_pose=[0,0,0,0])
        self.assertEqual(p.rate,[0.,0.,0.])
        p.step([.4,0,0,0],[1,0],1.4,measurement_mono=1.1,measurement_pose=[.1,0,0,0])
        self.assertGreater(p.rate[0],.5);self.assertLess(p.rate[0],1.)

    def test_navigation_passes_actual_measurement_to_pid(self):
        n=Navigator({'indoor':fixture.route()});n.start(fixture.solution(1.),1.)
        n.step(fixture.solution(1.),fixture.clear(1.),1.)
        s=fixture.solution(1.4,.3,'PREDICT_ONLY');s.update(measurement_mono=1.1,measurement_pose=[.1,0,0,0])
        out=n.step(s,fixture.clear(1.4),1.4)
        self.assertEqual(out['tracking']['rate_measurement_mono'],1.1)
        self.assertEqual(n.tracker.measured_previous[1],(.1,0,0))

    def test_twist_prediction_couples_translation_and_turn(self):
        c=ContinuousPose()
        for i in range(5):
            t=1.+i*.1;s=fixture.sample(t)
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',i*.1).as_matrix()
            T[:3,3]=[np.sin(i*.1),1-np.cos(i*.1),0.]
            s.update(T=T.tolist(),velocity=[1.,0.,0.]);c.odometry(s)
        a=dict(T=T.tolist(),measurement_mono=1.4,generation=[1,1],confirmed=True)
        c.map_update(a);s=c.estimate(1.7)
        np.testing.assert_allclose(s['pose'],[np.sin(.7),1-np.cos(.7),0.,.7],atol=1e-9)

    def test_no_velocity_or_history_cannot_invent_prediction(self):
        for sample in (fixture.sample(1.4),dict(fixture.sample(1.4),velocity=[1.,0.,0.])):
            c=ContinuousPose();c.odometry(sample);c.map_update(fixture.anchor(1.4))
            self.assertEqual(c.estimate(1.7)['mode'],'LOST')

    def test_map_dropout_does_not_stop_intermediate_or_final_waypoints(self):
        c=ContinuousPose();p=PoseSources();n=Navigator({'indoor':fixture.route()})
        x=0.;t=1.;a=fixture.anchor(t);c.odometry(fixture.sample(t,x));c.map_update(a)
        n.start(p.choose(c.estimate(t),a,t),t);states=[]
        for _ in range(600):
            t+=.05;c.odometry(fixture.sample(t,x));s=p.choose(c.estimate(t),a,t)
            n.keepalive(n.run_id,t);out=n.step(s,fixture.clear(t),t);states.append(out['state'])
            x+=out['vx']*.05
            if out['state']=='COMPLETE':break
        self.assertEqual(out['state'],'COMPLETE');self.assertEqual(n.reached,[0,1,2,3,4])
        self.assertNotIn('HOLD_LOCALIZATION',states)

    def test_manual_policy_does_not_override_terrain_on_resume(self):
        m=Mission({'indoor':test_unified.route()},test_unified.FakeBridge())
        m.plans.set_policy('indoor',[0,1,2,3],'stairs_normal')
        for policy in ('basic','stairs','platform','low','high'):
            m.command(dict(action='manual_shadow',policy=policy),None,10.)
            self.assertIsNone(m.override)
        m.pause();m.command(dict(action='shadow'),test_unified.solution(x=0.),10.)
        self.assertEqual(m.desired_policy(),'stairs_normal')

    def test_stand_to_manual_preserves_segment_presets(self):
        m=Mission({'indoor':test_unified.route()},test_unified.FakeBridge())
        m.command(dict(action='stand',manual_after_stand=True,policy='basic'),None,10.)
        self.assertEqual(m.owner,'manual');self.assertIsNone(m.override)


class GeometricRecoveryTests(unittest.TestCase):
    def setup_local(self):
        local,cloud=test_keyframes.KeyframeTests().setup_local()
        local.engine.motion=test_registration_motion.RegistrationMotionTests().motion()
        return local,cloud

    def test_gap_does_not_erase_valid_geometric_reference(self):
        local,cloud=self.setup_local();epoch=local.epoch
        sample=local.event(test_keyframes.KeyframeTests().event(cloud,10.8),10.83)
        self.assertEqual(sample['epoch'],epoch);self.assertEqual(sample['accepted'],6)
        self.assertEqual(local.quality['prediction'],'last_lidar_pose_recovery_seed')
        self.assertEqual(local.quality['level'],'tracking')
        np.testing.assert_allclose(sample['T'],np.eye(4),atol=.01)

    def test_wrong_motion_prior_cannot_veto_independently_consistent_geometry(self):
        local,cloud=self.setup_local();D=np.eye(4);D[0,3]=.3
        with patch.object(local.engine.motion,'relative',return_value=(D,'noisy_imu')):
            sample=local.event(test_keyframes.KeyframeTests().event(cloud,10.2),10.23)
        self.assertEqual(sample['accepted'],6);self.assertTrue(local.quality['motion_prior_disagreed'])
        np.testing.assert_allclose(sample['T'],np.eye(4),atol=.01)

    def test_bad_geometry_after_gap_cannot_renew_pose_or_destroy_reference(self):
        local,cloud=self.setup_local();epoch=local.epoch
        with patch('local_odometry.small_gicp.align',return_value=test_keyframes.KeyframeTests().rejected()):
            with self.assertRaisesRegex(ValueError,'LOCAL_GEOMETRY_REJECTED'):
                local.event(test_keyframes.KeyframeTests().event(cloud,10.8),10.83)
        self.assertEqual(local.epoch,epoch);self.assertEqual(local.t,10.);self.assertEqual(local.accepted,5)
        self.assertEqual(local.frames['front']['t'],9.8)

    def test_scan_replay_log_includes_input_before_geometry_rejection(self):
        local,cloud=self.setup_local();records=[];local.scan_log=SimpleNamespace(emit=records.append)
        with patch('local_odometry.small_gicp.align',return_value=test_keyframes.KeyframeTests().rejected()):
            with self.assertRaises(ValueError):local.event(test_keyframes.KeyframeTests().event(cloud,10.2),10.23)
        self.assertEqual(len(records),1);self.assertEqual(records[0]['measurement_mono'],10.2)
        self.assertEqual(records[0]['previous']['t'],10.);self.assertGreater(len(records[0]['body']),600)
        self.assertEqual(records[0]['body'].dtype,np.float32)


if __name__=='__main__':unittest.main()
