"""Body axes, timing, epochs, alignment and manual-resume integration."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from lightning_odometry import LightningOdometry, SCHEMA, load_config
from continuous_pose import ContinuousPose
from mission import Mission
from test_unified import FakeBridge, route
from test_forward_entry import clear

ROOT = Path(__file__).parent


def transform(x=0., y=0., z=0., angles=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def fixture():
    config = load_config(ROOT)
    return LightningOdometry(config), np.array(config['T_body_imu'])


def payload(T_body, extrinsics, t, epoch='run-1'):
    return dict(schema=SCHEMA, sensor='front', epoch=epoch,
                stamp_ns=round((1_800_000_000+t)*1e9), T_world_imu=(T_body@extrinsics).tolist())


def accept(lio, ext, T, t, delay=.06, epoch='run-1'):
    return lio.accept(payload(T, ext, t, epoch), t, t+delay)


def align(fusion, t=10., x=0., request='alignment-1', epoch='run-1'):
    return fusion.map_update(dict(T=transform(x).tolist(), measurement_mono=t,
        generation=[request, 1], confirmed=True, odometry_epoch=epoch))


class AdapterTests(unittest.TestCase):
    def test_real_mounting_forward_left_up_no_axis_swap(self):
        for target, velocity in [(transform(x=.2), [2,0,0]), (transform(y=.2), [0,2,0]), (transform(z=.2), [0,0,2])]:
            lio, ext = fixture()
            accept(lio, ext, np.eye(4), 10.)
            s = accept(lio, ext, target, 10.1)
            np.testing.assert_allclose(s['T'], target, atol=1e-12)
            np.testing.assert_allclose(s['velocity'], velocity, atol=1e-10)

    def test_lever_arm_rotation_does_not_invent_translation(self):
        lio, ext = fixture()
        accept(lio, ext, np.eye(4), 10.)
        target = transform(angles=(.8, -.5, 1.7))
        s = accept(lio, ext, target, 10.1)
        np.testing.assert_allclose(s['T'], target, atol=1e-12)
        np.testing.assert_allclose(s['velocity'], np.zeros(3), atol=1e-10)

    def test_turned_robot_forward_velocity_remains_positive_x(self):
        lio, ext = fixture()
        heading = transform(angles=(0,0,np.pi/2))
        accept(lio, ext, heading, 10.)
        s = accept(lio, ext, heading@transform(x=.3), 10.1)
        np.testing.assert_allclose(s['velocity'], [3,0,0], atol=1e-10)

    def test_large_motion_and_rotation_are_not_confidence_gated(self):
        lio, ext = fixture()
        accept(lio, ext, np.eye(4), 10.)
        s = accept(lio, ext, transform(x=2, angles=(1.7,.8,2.8)), 10.1)
        self.assertIsNotNone(s)
        self.assertAlmostEqual(np.linalg.norm(s['velocity']), 20.)

    def test_late_receipt_never_renews_measurement_time(self):
        lio, ext = fixture()
        s = accept(lio, ext, np.eye(4), 10., delay=1.2)
        self.assertEqual(s['t'], 10.)
        self.assertAlmostEqual(s['measurement_age_s'], 1.2)

    def test_duplicate_and_reordered_do_not_refresh(self):
        lio, ext = fixture()
        initial = accept(lio, ext, np.eye(4), 10.)
        self.assertIsNone(accept(lio, ext, transform(x=5), 10., delay=1))
        self.assertIsNone(accept(lio, ext, transform(x=5), 9.))
        self.assertIs(lio.latest, initial)

    def test_restart_is_explicit_epoch_and_retires_old_publisher(self):
        lio, ext = fixture()
        accept(lio, ext, np.eye(4), 10.)
        accept(lio, ext, transform(x=10), 11., epoch='run-2')
        self.assertEqual(lio.latest['epoch'], 'run-2')
        np.testing.assert_allclose(lio.latest['T'], np.eye(4), atol=1e-12)
        self.assertIsNone(accept(lio, ext, np.eye(4), 12., epoch='run-1'))

    def test_malformed_or_wrong_sensor_keeps_valid_state(self):
        lio, ext = fixture()
        initial = accept(lio, ext, np.eye(4), 10.)
        for change in ({'sensor':'rear'}, {'T_world_imu':np.full((4,4),np.nan).tolist()}, {'stamp_ns':-1}):
            p = payload(np.eye(4), ext, 11.)
            p.update(change)
            self.assertIsNone(lio.accept(p, 11., 11.1))
            self.assertIs(lio.latest, initial)

    def test_velocity_expires_through_gap_and_recovers_without_rearming(self):
        lio, ext = fixture()
        accept(lio, ext, np.eye(4), 10.)
        accept(lio, ext, transform(x=.3), 10.1)
        s = accept(lio, ext, transform(x=1), 12.)
        np.testing.assert_equal(s['velocity'], [0,0,0])
        s = accept(lio, ext, transform(x=1.3), 12.1)
        np.testing.assert_allclose(s['velocity'], [3,0,0], atol=1e-10)


class NavigationTests(unittest.TestCase):
    def runtime_method(self,name,scope=None):
        tree=ast.parse((ROOT/'runtime.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MapService')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
        module=ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[]))
        scope=dict(scope or {})
        exec(compile(module,'runtime.py','exec'),scope)
        return scope[name]

    def setup_pose(self):
        lio, ext = fixture()
        f = ContinuousPose()
        for t, x in [(10.,0), (10.1,.1), (10.2,.2)]:
            f.odometry(accept(lio, ext, transform(x=x), t))
        self.assertTrue(align(f))
        return lio, ext, f

    def test_relocalization_changes_alignment_only_once(self):
        lio, ext, f = self.setup_pose()
        self.assertFalse(align(f, x=99))
        self.assertTrue(align(f, x=2, request='operator-2'))
        self.assertAlmostEqual(f.estimate(10.22)['pose'][0], 2.2)
        self.assertEqual(len(f.history),3)
        self.assertEqual(lio.accepted,3)

    def test_new_lio_epoch_cannot_reuse_old_map_match(self):
        lio, ext, f = self.setup_pose()
        f.odometry(accept(lio, ext, transform(x=12), 11., epoch='run-2'))
        self.assertIsNone(f.estimate(11.05))
        self.assertFalse(align(f, t=11., request='late-old-match'))
        self.assertTrue(align(f, t=11., request='new-request', epoch='run-2'))

    def test_prediction_and_lost_do_not_forge_arrival_observations(self):
        lio, ext, f = self.setup_pose()
        f.odometry(accept(lio, ext, transform(x=.4), 10.4))
        s = f.estimate(10.71)
        self.assertEqual(s['mode'],'PREDICT_ONLY')
        self.assertFalse(s['arrival_valid'])
        self.assertEqual(s['measurement_mono'],10.4)
        self.assertEqual(s['measurement_source'],'lightning')
        self.assertEqual(f.estimate(11.)['mode'],'LOST')

    def test_old_map_does_not_stop_new_lightning_odometry(self):
        lio, ext, f = self.setup_pose()
        f.odometry(accept(lio, ext, transform(x=10), 100.))
        s = f.estimate(100.05)
        self.assertEqual(s['mode'],'ODOM_BRIDGE')
        self.assertIsNone(s['max_vx'])
        self.assertFalse(s['map_association_enabled'])

    def test_manual_then_resume_uses_latest_lightning_and_route_ahead(self):
        lio, ext, f = self.setup_pose()
        b = FakeBridge()
        m = Mission({'indoor':route()}, b)
        m.command({'action':'manual_shadow','policy':'stairs_normal'}, f.estimate(10.21), 10.21)
        for t, x in [(10.3,.5), (10.4,1.), (10.5,1.85)]:
            f.odometry(accept(lio, ext, transform(x=x,y=.2), t))
            m.operator(dict(session='manual',sample_seq=round(t*100),sample_age_ms=0,fresh=True,axes=[.8,0,0]),t)
            out = m.step(f.estimate(t+.01),clear(t),t+.01)
            self.assertEqual(out['owner'],'manual')
        m.operator(dict(session='manual',sample_seq=1051,sample_age_ms=0,fresh=True,axes=[0,0,0]),10.505)
        m.command({'action':'shadow'}, f.estimate(10.51), 10.51)
        out = m.step(f.estimate(10.52),clear(10.52),10.52)
        self.assertEqual(m.navigator.target,2)
        np.testing.assert_allclose(m.navigator.entry['path'][0], [1.85,.2,0.], atol=1e-12)
        self.assertEqual(out['policy_mode'],'route_preset')
        self.assertGreater(out['vx'],0.)
        self.assertEqual(len(m.plans.sources['indoor']['waypoints']),5)

    def test_runtime_frontend_results_cannot_override_lightning(self):
        # Compile the actual method for a unit test, without substituting ROS.
        tree = ast.parse((ROOT/'runtime.py').read_text())
        cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MapService')
        method = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='install_frontend')
        module = ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[]))
        scope = {}
        exec(compile(module,'runtime.py','exec'),scope)
        _, _, f = self.setup_pose()
        s = SimpleNamespace(local_reset_id=None,odom_backend='lightning',fusion=f)
        before = list(f.history)
        scope['install_frontend'](s,dict(reset_request_id=None,odom={'bad':'gicp'},records={'front':{}}))
        self.assertEqual(list(f.history),before)
        self.assertEqual(s.frontend_latest['records'],{'front':{}})

    def test_perception_restart_keeps_lightning_history_and_alignment(self):
        _, _, f=self.setup_pose()
        calls=[]
        service=SimpleNamespace(odom_backend='lightning',fusion=f,
            stop_frontend=lambda:calls.append('stop'),start_frontend=lambda:calls.append('start'),
            mission=SimpleNamespace(pause=lambda *_:calls.append('pause')),
            bridge=SimpleNamespace(stop=lambda:calls.append('motion_stop')))
        self.runtime_method('restart_frontend',{'ContinuousPose':ContinuousPose})(service)
        self.assertIs(service.fusion,f)
        self.assertIsNotNone(f.estimate(10.22))
        self.assertEqual(calls,['stop','start'])

    def test_runtime_uses_shared_clock_not_publication_time(self):
        lio,ext,f=self.setup_pose()
        mapped=[]
        def map_stamp(stamp,now):
            mapped.append((stamp,now));return 10.3
        service=SimpleNamespace(lightning=lio,fusion=f,rejects={},trace=SimpleNamespace(emit=lambda *a,**k:None),
            ingest=SimpleNamespace(clocks={'front':SimpleNamespace(map_stamp=map_stamp)}))
        p=payload(transform(x=.3),ext,10.3);p['published_mono']=8888888.
        callback=self.runtime_method('lightning_input',{'json':json,'time':SimpleNamespace(monotonic=lambda:10.4)})
        callback(service,SimpleNamespace(data=json.dumps(p)))
        self.assertEqual(mapped,[(p['stamp_ns']*1e-9,10.4)])
        self.assertEqual(f.history[-1]['t'],10.3)
        callback(service,SimpleNamespace(data='[]'))
        self.assertEqual(service.rejects['lightning'],'INVALID_LIGHTNING_MESSAGE')


if __name__ == '__main__':
    unittest.main()
