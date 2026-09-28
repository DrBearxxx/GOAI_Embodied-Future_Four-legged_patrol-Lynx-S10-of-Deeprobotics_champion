import math,tempfile,unittest
from pathlib import Path
import numpy as np
from pose_source import PoseSources
from pose_pid import PosePID,HeadingReference
from registration_log import RegistrationLog,read_records


def anchor(t,x=0.,generation=(1,1),confirmed=True):
    T=np.eye(4);T[0,3]=x
    return dict(T=T.tolist(),measurement_mono=t,generation=list(generation),confirmed=confirmed)


def local(t,mode='TRACKING',epoch='local-a'):
    return dict(pose=[0,0,0,0],mode=mode,estimate_mono=t,measurement_mono=t,generation=[epoch,1],arrival_valid=mode=='TRACKING')


class PoseSourceTests(unittest.TestCase):
    def test_confirmed_map_remains_available_when_local_registration_fails(self):
        p=PoseSources();s=p.choose(local(1.,'LOST'),anchor(1.,.2),1.1)
        self.assertEqual(s['mode'],'TRACKING');self.assertEqual(s['measurement_source'],'confirmed_map_fallback')
        self.assertEqual(s['pose'][0],.2);self.assertEqual(s['estimate_mono'],1.)
    def test_stale_or_unconfirmed_map_does_not_restore_localization(self):
        for a,now in ((anchor(1.),1.3),(anchor(1.,confirmed=False),1.1)):
            p=PoseSources();s=p.choose(local(1.,'LOST'),a,now)
            self.assertEqual(s['mode'],'LOST')
    def test_duplicate_map_does_not_refresh_timestamp(self):
        p=PoseSources();a=anchor(1.);p.choose(local(1.,'LOST'),a,1.1)
        self.assertEqual(p.choose(local(1.,'LOST'),a,1.3)['mode'],'LOST')
    def test_source_and_local_epoch_switch_do_not_change_navigation_frame(self):
        p=PoseSources();a=anchor(1.)
        one=p.choose(local(1.),a,1.1)
        two=p.choose(local(1.,'LOST'),a,1.15)
        three=p.choose(local(1.2,epoch='local-b'),anchor(1.2),1.25)
        self.assertEqual(one['generation'],two['generation']);self.assertEqual(two['generation'],three['generation'])
        self.assertEqual(three['pose_source_switches'],2)
    def test_actual_map_revision_changes_navigation_frame(self):
        p=PoseSources();a=p.choose(local(1.),anchor(1.),1.1)
        b=p.choose(local(1.2),anchor(1.2,generation=(2,1)),1.25)
        self.assertNotEqual(a['generation'],b['generation'])
    def test_brief_lidar_prediction_keeps_the_same_smoothed_map_alignment(self):
        p=PoseSources();s=p.choose(local(1.1,'PREDICT_ONLY'),anchor(1.),1.15)
        self.assertEqual(s['measurement_source'],'local_odometry');self.assertFalse(s['arrival_valid'])
    def test_causal_map_fit_preserves_measured_constant_velocity(self):
        p=PoseSources()
        for i in range(20):
            t=1.+i*.1;s=p.choose(local(t,'LOST'),anchor(t,i*.08),t+.1)
            self.assertAlmostEqual(s['pose'][0],i*.08,places=6)
        self.assertAlmostEqual(s['velocity_body'][0],.8,places=6)
    def test_map_revision_discards_old_position_filter(self):
        p=PoseSources()
        for t in (1.,1.1,1.2):p.choose(local(t,'LOST'),anchor(t),t)
        s=p.choose(local(1.3,'LOST'),anchor(1.3,5.,(2,1)),1.4)
        self.assertEqual(s['pose'][0],5.)


class PIDMeasurementTests(unittest.TestCase):
    def test_held_pose_does_not_make_derivative_decay_between_measurements(self):
        p=PosePID();p.step([0,0,0,0],[1,0],10.,measurement_mono=9.9)
        p.step([.1,0,0,0],[1,0],10.1,measurement_mono=10.)
        rate=list(p.rate)
        p.step([.1,0,0,0],[1,0],10.15,measurement_mono=10.)
        self.assertEqual(p.rate,rate)
    def test_differentiation_uses_sensor_time_not_fast_control_loop(self):
        p=PosePID();p.step([0,0,0,0],[1,0],10.,measurement_mono=9.9)
        p.step([.1,0,0,0],[1,0],10.05,measurement_mono=10.)
        self.assertLess(p.rate[0],1.);self.assertGreater(p.rate[0],.5)
    def test_source_offset_does_not_create_derivative_impulse(self):
        p=PosePID();p.step([0,0,0,0],[1,0],10.,measurement_mono=9.9,measurement_source='local')
        out=p.step([.2,0,0,.1],[1,0],10.05,measurement_mono=10.,measurement_source='map')
        self.assertEqual(out['tracking']['d'],[0.,0.,0.])
    def test_spatial_reference_does_not_double_count_cruise_feedforward(self):
        p=PosePID();out=p.step([0,0,0,0],[.12,0],10.,0.,[.8,0,0],follow_path=True)
        self.assertEqual(out['tracking']['d'],[0.,0.,0.])
        self.assertAlmostEqual(out['vx'],.8+.9*.12)
    def test_path_derivative_damps_lateral_motion(self):
        p=PosePID();p.step([0,0,0,0],[.12,0],10.,0.,[.8,0,0],follow_path=True)
        out=p.step([.08,.03,0,0],[.20,0],10.1,0.,[.8,0,0],follow_path=True)
        self.assertAlmostEqual(out['tracking']['d'][0],0.)
        self.assertLess(out['tracking']['d'][1],0.)
    def test_heading_step_generates_continuous_angle_and_its_own_feedforward(self):
        h=HeadingReference();samples=[h.step(math.pi/2,0,10.+i*.05) for i in range(100)]
        self.assertLess(samples[0][0],.02);self.assertGreater(samples[0][1],0.)
        self.assertTrue(all(b[0]>=a[0] for a,b in zip(samples,samples[1:])))
        self.assertLessEqual(max(x[0] for x in samples),math.pi/2)
        self.assertLess(abs(samples[-1][0]-math.pi/2),.001)
    def test_heading_wrap_follows_short_rotation(self):
        h=HeadingReference();yaw,rate=h.step(-math.pi+.05,math.pi-.05,10.)
        self.assertLess(abs(rate),.1)
    def test_no_rate_clipping_is_added_to_reference_model(self):
        slow=HeadingReference(2.5);fast=HeadingReference(10.)
        a=slow.step(3.,0,10.);b=fast.step(3.,0,10.)
        self.assertGreater(b[1],a[1]);self.assertGreater(b[1],2.)


class RejectedRegistrationLogTests(unittest.TestCase):
    def test_replay_preserves_cloud_and_alignment_guess(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'rejected.bin';log=RegistrationLog(path)
            log.emit(dict(source=np.array([[1.,2.,3.]]),guess=np.eye(4),quality={'rejection_checks':['not_converged']}));log.close()
            rows=list(read_records(path));self.assertEqual(len(rows),1)
            np.testing.assert_equal(rows[0]['source'],[[1,2,3]])
            self.assertEqual(log.written,1);self.assertEqual(log.failed,0)
    def test_rotation_is_bounded_and_truncated_last_record_is_ignored(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'rejected.bin';log=RegistrationLog(path,max_bytes=1,backups=2)
            for i in range(6):log.emit({'index':i});log.queue.join()
            log.close();self.assertEqual(len(list(Path(folder).iterdir())),3)
            with path.open('ab') as f:f.write(b'\x00\x01')
            self.assertEqual(list(read_records(path)),[{'index':5}])


if __name__=='__main__':unittest.main()
