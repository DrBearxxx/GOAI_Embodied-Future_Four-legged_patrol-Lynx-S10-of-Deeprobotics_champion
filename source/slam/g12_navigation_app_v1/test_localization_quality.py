import copy,queue,unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from localization_quality import classify,navigation_config
from reference_recovery import ReferenceRecovery
from pose_source import PoseSources
from continuous_pose import ContinuousPose
from test_pose_feedback import anchor,local
import test_keyframes
from test_forward_entry import pose,clear
from test_unified import FakeBridge,route
from mission import Mission
from seed_matcher import SeedMatcher

class LocalizationQualityTests(unittest.TestCase):
    def test_quality_is_not_an_all_or_nothing_confidence_gate(self):
        self.assertEqual(classify(.5,.24,1e-7,False),'degraded')
        self.assertEqual(classify(.9,.05,.002,False),'degraded')
        self.assertEqual(classify(.9,.05,.002,True),'tracking')
        self.assertEqual(classify(.15,.6,.002,True),'unusable')
        self.assertEqual(classify(.9,.05,float('nan')),'unusable')
    def test_medium_local_geometry_updates_instead_of_freezing_reference(self):
        k=test_keyframes.KeyframeTests();o,cloud=k.setup_local();distances=np.r_[np.full(500,.12),np.full(500,.31)]
        o.frames['front']['points']=NS(query=lambda *a,**kw:(distances,None))
        fit=NS(T_target_source=np.eye(4),H=np.diag([1e-7,1,1,1,1,1]),converged=False,iterations=24)
        with patch('local_odometry.small_gicp.align',return_value=fit):s=o.event(k.event(cloud,10.2),10.25)
        self.assertEqual(s['accepted'],6);self.assertEqual(s['quality']['level'],'degraded')
        self.assertEqual(s['quality']['rejection_checks'],[])
    def test_medium_map_fit_is_usable_without_additional_confirmation(self):
        matcher=SeedMatcher.__new__(SeedMatcher);matcher.scope=None
        result=dict(accepted=False,T=np.eye(4),reasons=['not_converged','large_plane_residual'],
            quality=dict(overlap=.5,median_nn_m=.24,condition=1e-7))
        with patch('seed_matcher.IndoorMatcher.register',return_value=result):r=matcher.register(None,None)
        self.assertTrue(r['accepted']);self.assertEqual(r['quality_level'],'degraded')
    def test_global_map_hint_does_not_influence_local_odometry(self):
        k=test_keyframes.KeyframeTests();o,cloud=k.setup_local()
        o.map_reference=((1,),10.,np.eye(4),np.eye(4));o.map_hint=anchor(10.9,1.)
        expected=np.eye(4);expected[0,3]=1.
        fit=NS(T_target_source=expected,H=np.eye(6),converged=True,iterations=4)
        with patch('local_odometry.small_gicp.align',return_value=fit) as align,patch('local_odometry.bridge_local',return_value=None),patch.object(o.engine.motion,'relative',return_value=(None,'gap')):
            s=o.event(k.event(cloud-np.array([1.,0.,0.]),11.),11.03)
        np.testing.assert_allclose(align.call_args.kwargs['init_T_target_source'],np.eye(4))
        self.assertEqual(s['accepted'],6);self.assertEqual(o.quality['prediction'],'last_lidar_pose_recovery_seed')
        np.testing.assert_allclose(o.T,expected)
    def test_clicked_region_remains_the_requested_search_region(self):
        matcher=SeedMatcher.__new__(SeedMatcher);matcher.scope=dict(xyz=[0,0,0],radius=1,height_tolerance=.8)
        T=np.eye(4);T[0,3]=4
        result=dict(accepted=False,T=T,reasons=['not_converged'],quality=dict(overlap=.5,median_nn_m=.24,condition=1e-7))
        with patch('seed_matcher.IndoorMatcher.register',return_value=result):r=matcher.register(None,None)
        self.assertFalse(r['accepted']);self.assertEqual(r['reasons'],['FIT_OUTSIDE_SELECTED_REGION'])
    def test_degraded_geometry_can_navigate_without_a_quality_speed_cap(self):
        m=Mission({'indoor':route()},FakeBridge());s=pose(x=.5);s['mode']='DEGRADED'
        m.command(dict(action='start'),s,10.);out=m.step(s,clear(10.),10.)
        self.assertTrue(out['motion_authorized']);self.assertGreater(out['vx'],0.)
        self.assertNotEqual(out['state'],'HOLD_LOCALIZATION');self.assertIsNone(out['speed_limit_mps'])
    def test_one_real_local_fit_is_usable_and_degraded_quality_is_preserved(self):
        c=ContinuousPose();sample=dict(T=np.eye(4).tolist(),t=1.,epoch='local',accepted=1,path=0.,quality={'level':'degraded'})
        c.odometry(sample);c.map_update(anchor(1.));s=c.estimate(1.1)
        self.assertEqual(s['mode'],'DEGRADED');self.assertTrue(s['arrival_valid']);self.assertIsNone(s['max_vx'])
        self.assertEqual(PoseSources().choose(s,anchor(1.),1.1)['mode'],'DEGRADED')
    def test_source_expiry_holds_last_published_pose_and_true_timestamp(self):
        p=PoseSources();s=p.choose(local(1.,'LOST'),anchor(1.,10.),1.1)
        lost=p.choose(local(1.,'LOST'),anchor(1.,10.),1.5)
        self.assertEqual(lost['mode'],'LOST');self.assertEqual(lost['pose'],s['pose'])
        self.assertEqual(lost['measurement_mono'],1.);self.assertFalse(lost['arrival_valid'])
    def test_map_processing_gap_bridges_with_measured_velocity_then_expires(self):
        p=PoseSources()
        for i in range(4):s=p.choose(local(1.,'LOST'),anchor(1.+i*.1,i*.1),1.+i*.1)
        pred=p.choose(local(1.,'LOST'),anchor(1.3,.3),1.6)
        self.assertEqual(pred['mode'],'PREDICT_ONLY');self.assertFalse(pred['arrival_valid'])
        self.assertAlmostEqual(pred['pose'][0],.6);self.assertEqual(pred['measurement_mono'],1.3)
        self.assertEqual(p.choose(local(1.,'LOST'),anchor(1.3,.3),1.8)['mode'],'LOST')
    def test_config_requires_one_geometry_match_without_changing_base_project(self):
        old=dict(registration={},relocalization={'confirmations':3});cfg=navigation_config(old)
        self.assertEqual(cfg['relocalization']['confirmations'],1);self.assertEqual(old['relocalization']['confirmations'],3)

class ReferenceRecoveryTests(unittest.TestCase):
    def frontend(self):return dict(odom=dict(t=1.,epoch='old'),reason='LOCAL_GEOMETRY_REJECTED')
    def test_one_usable_map_fit_can_replace_failed_reference_only_once(self):
        r=ReferenceRecovery();e=r.observe(self.frontend(),anchor(2.),2.1)
        self.assertIsNotNone(e);self.assertIsNone(r.observe(self.frontend(),anchor(2.1),2.2))
    def test_fresh_local_or_unusable_map_cannot_reset_reference(self):
        r=ReferenceRecovery();self.assertIsNone(r.observe(self.frontend(),anchor(1.2),1.3))
        self.assertIsNone(r.observe(self.frontend(),anchor(1.2),2.))
        self.assertIsNone(r.observe(self.frontend(),anchor(2.,confirmed=False),2.1))
        self.assertIsNone(r.observe(self.frontend(),anchor(2.),2.1,rejected_map_time=2.))

if __name__=='__main__':unittest.main()
