import unittest,queue
from types import SimpleNamespace
import numpy as np
import paths
from runtime import MapService
from continuous_pose import ContinuousPose
from local_odometry import reset_reference
import test_keyframes,test_registration_motion,test_continuity_v2


class RelocalizationSyncTests(unittest.TestCase):
    def service(self):
        return SimpleNamespace(applied_relocalization=None,local_reset_id=None,local_commands=queue.Queue(1),
            frontend_latest={'old':True},fusion=ContinuousPose(),continuity=object(),pose_sources=object(),
            trace=SimpleNamespace(emit=lambda *a,**k:None))

    def test_success_keeps_odometry_and_commits_alignment_once(self):
        s=self.service();old=s.fusion;job=dict(state='SUCCEEDED',request_id='operator-1')
        self.assertTrue(MapService.sync_relocalization(s,job));self.assertIs(s.fusion,old)
        self.assertEqual(s.frontend_latest,{'old':True});self.assertTrue(s.local_commands.empty())
        new=s.fusion
        for _ in range(4):self.assertFalse(MapService.sync_relocalization(s,job))
        self.assertIs(s.fusion,new);self.assertTrue(s.local_commands.empty())

    def test_failed_search_and_internal_retrieval_do_not_erase_reference(self):
        for state in ('QUEUED','SEARCHING','FAILED','CANCELLED','IDLE'):
            s=self.service();old=s.fusion
            self.assertFalse(MapService.sync_relocalization(s,dict(state=state,request_id='test')))
            self.assertIs(s.fusion,old);self.assertEqual(s.frontend_latest,{'old':True})

    def test_operator_alignment_preserves_delayed_odometry_history(self):
        s=self.service();before=dict(reset_request_id=None,odom=test_continuity_v2.sample(1.,epoch='local'))
        self.assertTrue(MapService.install_frontend(s,before))
        MapService.sync_relocalization(s,dict(state='SUCCEEDED',request_id='new'))
        after=dict(reset_request_id=None,odom=test_continuity_v2.sample(1.2,epoch='local'))
        self.assertTrue(MapService.install_frontend(s,after));self.assertEqual(len(s.fusion.history),2)
        self.assertIsNotNone(s.fusion.at(1.1));self.assertTrue(s.local_commands.empty())

    def test_each_operator_commit_keeps_local_frame(self):
        s=self.service()
        for rid in ('one','two'):
            self.assertTrue(MapService.sync_relocalization(s,dict(state='SUCCEEDED',request_id=rid)))
            self.assertIsNone(s.local_reset_id);self.assertTrue(s.local_commands.empty())

    def test_frontend_reset_clears_pose_reference_twist_but_keeps_received_imu_and_clocks(self):
        local,cloud=test_keyframes.KeyframeTests().setup_local()
        motion=test_registration_motion.RegistrationMotionTests().motion();local.engine.motion=motion
        motion.add_imu('front',10.,np.zeros(3),np.zeros(3))
        motion.observe_lidar(9.8,np.eye(4));motion.observe_lidar(10.,np.eye(4));clocks=local.engine.clocks
        new=reset_reference(local,'operator-reset')
        self.assertNotEqual(new.epoch,local.epoch);self.assertIsNone(new.t);self.assertEqual(new.frames,{})
        self.assertIsNone(motion.twist_time);self.assertEqual(len(motion.lidar_history),0)
        self.assertEqual(len(motion.imu['front']),1);self.assertIs(new.engine.clocks,clocks)


if __name__=='__main__':unittest.main()
