"""Rejected geometry cannot become odometry; fresh redundant scans can re-seed."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from local_odometry import LocalOdometry,rigid
from scipy.spatial.transform import Rotation


class KeyframeTests(unittest.TestCase):
    def setup_local(self):
        motion=SimpleNamespace(deskew=lambda pts,*a:(pts,None),relative=lambda *a:(np.eye(4),None))
        clock=SimpleNamespace(epoch=0,map_stamp=lambda stamp,now:stamp)
        engine=SimpleNamespace(motion=motion,clocks={'front':clock,'rear':clock},ext={'lidar':{'front':np.eye(4),'rear':np.eye(4)}})
        local=LocalOdometry(engine)
        rng=np.random.default_rng(25);cloud=rng.uniform([-8,-8,-2],[8,8,2],(2500,3))
        local.keyframe('front',cloud,np.eye(4),9.8)
        local.t=10.;local.accepted=5;local.last_sensor='rear';local.T=np.eye(4)
        return local,cloud

    def event(self,cloud,end):
        return dict(sensor='front',frame='wym_front_lidar',stamp=end-.1,points=cloud,rel=np.linspace(0.,.1,len(cloud)))

    def rejected(self):
        T=np.eye(4);T[0,3]=.4
        return SimpleNamespace(T_target_source=T,H=np.eye(6),converged=True,iterations=3)

    def test_reseed_from_other_lidar_does_not_publish_rejected_movement(self):
        local,cloud=self.setup_local()
        with patch('local_odometry.small_gicp.align',return_value=self.rejected()):
            with self.assertRaisesRegex(ValueError,'LOCAL_GEOMETRY_REJECTED'):local.event(self.event(cloud,10.1),10.15)
        self.assertEqual(local.accepted,5);self.assertEqual(local.t,10.)
        np.testing.assert_array_equal(local.T,np.eye(4))
        self.assertAlmostEqual(local.frames['front']['t'],10.1)
        self.assertEqual(local.quality['keyframe_reseeded_from'],'rear')

    def test_rejection_without_fresh_other_lidar_cannot_reanchor_frame(self):
        for sensor,end in (('front',10.1),('rear',10.3)):
            local,cloud=self.setup_local();local.last_sensor=sensor
            with patch('local_odometry.small_gicp.align',return_value=self.rejected()):
                with self.assertRaisesRegex(ValueError,'LOCAL_GEOMETRY_REJECTED'):local.event(self.event(cloud,end),end+.05)
            self.assertEqual(local.frames['front']['t'],9.8);self.assertEqual(local.t,10.)

    def test_stationary_geometry_keeps_its_reference_instead_of_integrating_noise(self):
        local,cloud=self.setup_local()
        # Exact duplicate scene has a valid identity registration. Elapsed time
        # alone must not replace its stable reference and integrate fit noise.
        fit=SimpleNamespace(T_target_source=np.eye(4),H=np.eye(6),converged=True,iterations=1)
        with patch('local_odometry.small_gicp.align',return_value=fit):
            measured=local.event(self.event(cloud,10.2),10.25)
        self.assertEqual(measured['accepted'],6);self.assertEqual(local.last_sensor,'front')
        self.assertAlmostEqual(local.frames['front']['t'],9.8)
        np.testing.assert_array_equal(local.T,np.eye(4))

    def test_accepted_but_declining_scene_overlap_renews_reference(self):
        local,cloud=self.setup_local()
        distances=np.r_[np.full(700,.05),np.full(300,.4)]
        local.frames['front']['points']=SimpleNamespace(query=lambda *a,**kw:(distances,None))
        fit=SimpleNamespace(T_target_source=np.eye(4),H=np.eye(6),converged=True,iterations=1)
        with patch('local_odometry.small_gicp.align',return_value=fit):
            measured=local.event(self.event(cloud,10.2),10.25)
        self.assertEqual(measured['accepted'],6)
        self.assertAlmostEqual(local.frames['front']['t'],10.2)

    def test_repeated_keyframe_inverse_and_composition_cannot_amplify_scale(self):
        # Seed a roundoff-size error then exercise the actual ref->guess->fit
        # composition. Without normalization this cubically amplifies the
        # rotation scale at each keyframe renewal.
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.013,-.017,.027]).as_matrix()*(1.+1e-14)
        T[:3,3]=[1.,2.,.3]
        for _ in range(1000):
            ref=rigid(T);predicted=rigid(T)
            inverse=np.eye(4);inverse[:3,:3]=ref[:3,:3].T;inverse[:3,3]=-ref[:3,:3].T@ref[:3,3]
            relative=rigid(inverse@predicted)
            T=rigid(ref@relative)
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1.,places=12)
        np.testing.assert_allclose(T[:3,3],[1.,2.,.3],atol=1e-10)

    def test_rotation_projection_preserves_translation_and_tilt(self):
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.23,-.17,1.1]).as_matrix()
        T[:3,3]=[4.,-3.,.7];perturbed=T.copy();perturbed[:3,:3]*=1.+1e-5
        np.testing.assert_allclose(rigid(perturbed),T,atol=1e-12)


if __name__=='__main__':unittest.main()
