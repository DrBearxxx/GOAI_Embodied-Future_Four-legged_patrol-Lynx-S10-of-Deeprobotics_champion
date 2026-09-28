import copy,json,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from registration_motion import RegistrationMotion,use_registration_motion
from local_odometry import LocalOdometry
from seed_matcher import AppEngine
import test_keyframes
from paths import BASE


class RegistrationMotionTests(unittest.TestCase):
    def motion(self):
        cfg=json.loads((BASE/'config.json').read_text())['motion']
        return RegistrationMotion(cfg,{s:np.eye(4) for s in ('front','rear','camera')})

    def test_complete_imu_retains_measured_deskew(self):
        m=self.motion()
        for t in np.arange(9.9,10.205,.005):m.add_imu('front',t,np.zeros(3),np.array([0.,0.,9.81]))
        pts=np.array([[1.,0,0],[2.,0,0],[3.,0,0]])
        out,why=m.deskew(pts,np.array([0.,.05,.1]),10.,np.eye(4),np.zeros(3))
        np.testing.assert_allclose(out,pts);self.assertEqual(why['method'],'measured_gyro')

    def test_scan_with_no_imu_is_a_rigid_registration_input_not_dropped(self):
        m=self.motion();pts=np.array([[1.,0,0],[2.,0,0]])
        out,why=m.deskew(pts,np.array([0.,.1]),10.,np.eye(4),np.zeros(3))
        np.testing.assert_allclose(out,pts)
        self.assertEqual(why['angular_source'],'rigid_scan_initial_guess')
        self.assertTrue(why['requires_geometric_registration'])
        self.assertIsNone(m.gyro_path(10.,10.1));self.assertFalse(any(m.imu.values()))

    def test_partial_imu_deskew_estimate_is_explicit_and_does_not_fill_imu(self):
        m=self.motion()
        for t in (10.,10.005,10.010):m.add_imu('front',t,np.array([0.,0.,1.]),np.array([0.,0.,9.81]))
        out,why=m.deskew(np.array([[1.,0,0],[1.,0,0]]),np.array([0.,.1]),10.,np.eye(4),np.zeros(3))
        np.testing.assert_allclose(out[0],[np.cos(.1),-np.sin(.1),0.],atol=1e-10)
        np.testing.assert_allclose(out[1],[1.,0.,0.])
        self.assertEqual(why['angular_source'],'partial_imu:front');self.assertEqual(len(m.imu['front']),3)
        self.assertIsNone(m.gyro_path(10.,10.1))

    def test_accepted_lidar_rotation_can_supply_scan_prediction(self):
        m=self.motion();a=np.eye(4);b=np.eye(4);b[:3,:3]=Rotation.from_euler('z',.1).as_matrix()
        m.observe_lidar(10.,a);m.observe_lidar(10.2,b)
        D,method=m.relative(10.2,10.3,np.array([1.,0.,0.]))
        self.assertEqual(method,'registration_prediction:accepted_lidar_twist')
        np.testing.assert_allclose(D[:3,3],[.1,0,0],atol=1e-10)
        self.assertAlmostEqual(Rotation.from_matrix(D[:3,:3]).as_rotvec()[2],.05)

    def test_stale_lidar_twist_is_not_reused(self):
        m=self.motion();m.observe_lidar(10.,np.eye(4));m.observe_lidar(10.2,np.eye(4))
        _,source=m.angular_estimate(12.,12.1)
        self.assertEqual(source,'rigid_scan_initial_guess')
        self.assertIsNone(m.relative(10.,12.,np.zeros(3))[0])

    def test_actual_lidar_geometry_advances_without_imu(self):
        local,cloud=test_keyframes.KeyframeTests().setup_local();local.engine.motion=self.motion()
        event=test_keyframes.KeyframeTests().event(cloud,10.2)
        sample=local.event(event,10.23)
        self.assertEqual(sample['accepted'],6)
        self.assertEqual(local.quality['deskew']['method'],'estimated_motion_deskew')
        self.assertEqual(len(local.engine.motion.lidar_history),1)
        np.testing.assert_allclose(sample['T'],np.eye(4),atol=.01)

    def test_rejected_registration_does_not_update_pose_or_lidar_twist(self):
        local,cloud=test_keyframes.KeyframeTests().setup_local();local.engine.motion=self.motion()
        with patch('local_odometry.small_gicp.align',return_value=test_keyframes.KeyframeTests().rejected()):
            with self.assertRaisesRegex(ValueError,'LOCAL_GEOMETRY_REJECTED'):
                local.event(test_keyframes.KeyframeTests().event(cloud,10.1),10.13)
        self.assertEqual(local.t,10.);self.assertEqual(local.accepted,5)
        self.assertEqual(len(local.engine.motion.lidar_history),0)


if __name__=='__main__':unittest.main()
