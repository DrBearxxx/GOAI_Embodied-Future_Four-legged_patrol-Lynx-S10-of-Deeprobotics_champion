import copy
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from automatic_localization import load_config
from continuous_pose import ContinuousPose
from pose_pid import PosePID
from test_continuity_v2 import sample,anchor


def transform(x=0.,y=0.,yaw=0.):
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();T[:2,3]=[x,y];return T


class AutomaticCorrectionTests(unittest.TestCase):
    def ready(self):
        self.options=load_config(Path(__file__).parent)
        c=ContinuousPose(self.options)
        for t in np.arange(1.,5.01,.1):c.odometry(sample(float(t),epoch='e'))
        c.map_update(anchor(1.));c.estimate(5.);return c

    def observation(self,c,t,x=.2,seq=1,yaw=0.,accepted=True):
        return dict(T=transform(x,yaw=yaw).tolist(),measurement_mono=t,seq=seq,accepted=accepted,
            alignment_generation=[c.epoch,c.revision],odometry_epoch=c.epoch,reason='LOCAL_GEOMETRY')

    def test_two_agreeing_fits_correct_gradually_without_generation_change(self):
        c=self.ready();generation=c.estimate(5.)['generation']
        self.assertFalse(c.automatic_update(self.observation(c,4.7),5.))
        self.assertTrue(c.automatic_update(self.observation(c,4.9,seq=2),5.))
        c.odometry(sample(5.1,epoch='e'));value=c.estimate(5.1)
        self.assertGreater(value['pose'][0],0);self.assertLess(value['pose'][0],.02)
        self.assertEqual(value['generation'],generation);self.assertTrue(value['arrival_valid'])
        for t in np.arange(5.2,16.,.1):c.odometry(sample(float(t),epoch='e'));value=c.estimate(float(t))
        self.assertAlmostEqual(value['pose'][0],.2,delta=.002)
        self.assertNotEqual(value['mode'],'LOST');self.assertIsNone(value['max_vx'])

    def test_an_isolated_bad_fit_does_not_move_stationary_pose(self):
        c=self.ready();c.automatic_update(self.observation(c,4.8,x=2.),5.)
        c.odometry(sample(5.1,epoch='e'));self.assertAlmostEqual(c.estimate(5.1)['pose'][0],0.)
        c.automatic_update(self.observation(c,4.9,x=0.,seq=2),5.1)
        c.automatic_update(self.observation(c,5.0,x=0.,seq=3),5.1)
        c.odometry(sample(5.2,epoch='e'));self.assertAlmostEqual(c.estimate(5.2)['pose'][0],0.)

    def test_weak_or_out_of_scope_geometry_does_not_hold_odometry(self):
        c=self.ready();before=c.offset.copy()
        for i in range(12):
            t=5.1+i*.1;c.odometry(sample(t,epoch='e'))
            c.automatic_update(self.observation(c,t,x=5.,seq=i,accepted=i%2==0),t+.05)
            s=c.estimate(t+.05);self.assertTrue(s['arrival_valid']);self.assertNotEqual(s['mode'],'LOST')
        np.testing.assert_allclose(c.offset,before)

    def test_delayed_fit_uses_its_measurement_time(self):
        c=ContinuousPose(load_config(Path(__file__).parent))
        for t in np.arange(1.,3.01,.1):c.odometry(sample(float(t),x=2*float(t),epoch='e'))
        c.map_update(anchor(1.,x=2.));c.estimate(3.)
        c.automatic_update(self.observation(c,2.4,x=5.0),3.)
        self.assertTrue(c.automatic_update(self.observation(c,2.6,x=5.4,seq=2),3.))
        self.assertAlmostEqual(c.target[0,3],.2,places=8)
        self.assertAlmostEqual(c.map_t,2.6)

    def test_duplicates_and_reordered_fits_cannot_renew_timestamp(self):
        c=self.ready();a=self.observation(c,4.7);b=self.observation(c,4.9,seq=2)
        c.automatic_update(a,5.);c.automatic_update(b,5.)
        self.assertFalse(c.automatic_update(b,5.1));a['seq']=3
        self.assertFalse(c.automatic_update(a,5.1));self.assertEqual(c.map_t,4.9)

    def test_old_result_cannot_override_manual_relocalization(self):
        c=self.ready();a=self.observation(c,4.9)
        c.map_update(anchor(5.,x=8.,generation=(2,1)))
        self.assertFalse(c.automatic_update(a,5.1));self.assertEqual(len(c.auto_candidates),0)
        self.assertAlmostEqual(c.estimate(5.1)['pose'][0],8.)

    def test_old_epoch_cannot_restore_a_reset_odometry_frame(self):
        c=self.ready();a=self.observation(c,4.9)
        c.odometry(sample(5.2,epoch='new'))
        self.assertFalse(c.automatic_update(a,5.2));self.assertIsNone(c.estimate(5.2))

    def test_smoothing_rotation_occurs_at_body_far_from_map_origin(self):
        c=ContinuousPose(load_config(Path(__file__).parent));odom=transform(100.,40.)
        for t in np.arange(1.,5.01,.1):c.odometry(dict(T=odom.tolist(),t=float(t),epoch='e',path=0.,accepted=20))
        a=anchor(1.);a['T']=odom.tolist();c.map_update(a);c.estimate(5.)
        for seq,t in enumerate((4.7,4.9)):
            obs=self.observation(c,t,seq=seq);T=transform(100.,40.,.05);obs['T']=T.tolist();c.automatic_update(obs,5.)
        c.odometry(dict(T=odom.tolist(),t=5.1,epoch='e',path=0.,accepted=21))
        pose=c.estimate(5.1)['pose'];np.testing.assert_allclose(pose[:2],[100.,40.],atol=1e-10)
        self.assertGreater(pose[3],0.);self.assertLess(pose[3],.01)


class CorrectionDerivativeTests(unittest.TestCase):
    def test_stationary_robot_has_zero_derivative_during_map_correction(self):
        pid=PosePID();odom=transform(40.,20.)
        for i in range(15):
            alignment=transform(.02*i,.01*i,.001*i);T=alignment@odom
            yaw=np.arctan2(T[1,0],T[0,0]);pose=[*T[:3,3],yaw]
            result=pid.step(pose,pose[:2],1.+i*.1,yaw,measurement_mono=1.+i*.1,measurement_source='lightning',
                measurement_pose=pose,measurement_frame=dict(map_to_odom=alignment.tolist(),odom_T=odom.tolist()))
            np.testing.assert_allclose(result['tracking']['measured_rate_map'],[0.,0.,0.],atol=1e-10)

    def test_actual_motion_remains_in_derivative_while_alignment_changes(self):
        pid=PosePID()
        for i in range(30):
            alignment=transform(.03*i);odom=transform(.1*i);T=alignment@odom;pose=[*T[:3,3],0.]
            result=pid.step(pose,pose[:2],1.+i*.1,0.,measurement_mono=1.+i*.1,measurement_source='lightning',
                measurement_pose=pose,measurement_frame=dict(map_to_odom=alignment.tolist(),odom_T=odom.tolist()))
        self.assertAlmostEqual(result['tracking']['measured_rate_map'][0],1.,places=6)


if __name__=='__main__':unittest.main()
