"""Pure synthetic 50 cm controller tests; no robot connection or acceptance."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav import bootstrap
from native_nav.short_trial import ShortTrial, make_short_route
from native_nav.acceptance import code_hash
from native_nav.first_trial import PROFILE
from native_nav.client import Client
from test_resilient_route import sample as base_sample
from test_route import simple_route


def sample(*args, **kwargs):
    s, f, e = base_sample(*args, **kwargs)
    f['code_hash'] = code_hash()
    return s, f, e


class ShortTrialTests(unittest.TestCase):
    def ready(self, route=None, p=(0., 0., 0., 0.)):
        g = ShortTrial(route or simple_route(), 'map1', 'boot1')
        for i in range(61):
            c = g.tick(i*.04, *sample(i*.04, p))
            self.assertEqual(c['vx'], 0.)
        self.assertTrue(c['ready'], c)
        self.assertFalse(g.armed)
        return g

    def active(self):
        g = self.ready()
        self.assertTrue(g.arm(2.4)[0])
        g.tick(2.44, *sample(2.44))
        g.tick(2.48, *sample(2.48))
        return g

    def assert_terminal(self, g, t, p=(0., 0., 0., 0.)):
        self.assertIsNotNone(g.terminal_reason)
        self.assertFalse(g.armed)
        for i in range(70):
            c = g.tick(t+i*.04, *sample(t+i*.04, p))
            self.assertEqual((c['vx'], c['vy'], c['wz']), (0., 0., 0.))
            self.assertFalse(c['ready'])
        self.assertFalse(g.arm(t+3.)[0])

    def test_clip_real_frozen_route_and_preserve_source(self):
        source = json.loads((bootstrap.INDOOR/'assets/route.json').read_text())
        original = copy.deepcopy(source)
        short = make_short_route(source)
        self.assertEqual(source, original)
        self.assertEqual(len(short['waypoints']), 3)
        xyz = np.array([p['xyz'] for p in short['waypoints']])
        self.assertAlmostEqual(np.linalg.norm(np.diff(xyz[:,:2], axis=0), axis=1).sum(), .50)
        self.assertEqual(short['trial']['max_vx_mps'], .2)
        self.assertFalse(short['automatic_return'])
        self.assertTrue(short['stop_at_end'])
        self.assertNotEqual(short['waypoints'][-1]['xyz'], source['waypoints'][2]['xyz'])

    def test_invalid_short_or_warned_route_rejected(self):
        for kind in ('short', 'nan', 'warnings', 'duplicate', 'edges'):
            r = simple_route()
            if kind == 'short': r['waypoints'][1]['xyz'][0] = .1
            if kind == 'nan': r['waypoints'][1]['xyz'][0] = float('nan')
            if kind == 'warnings': r['edges'][0]['warnings'] = ['gap']
            if kind == 'duplicate': r['waypoints'][1]['xyz'] = [0, 0, 1]
            if kind == 'edges': r['edges'] = []
            with self.assertRaises(ValueError): make_short_route(r)

    def test_start_position_heading_and_body_feedback(self):
        for p in ((.09, 0, 0, 0), (0, 0, 0, .36)):
            g = ShortTrial(simple_route(), 'map1', 'boot1')
            for i in range(80): c = g.tick(i*.04, *sample(i*.04, p))
            self.assertFalse(c['ready'])
            self.assertFalse(g.arm(3.16)[0])
        g = self.active()
        s, f, e = sample(2.52)
        f['feedback']['speed'] = .31
        g.tick(2.52, s, f, e)
        self.assert_terminal(g, 2.56)

    def test_commissioning_backend_is_explicit_and_never_accepts_simulation(self):
        g = ShortTrial(simple_route(), 'map1', 'boot1', first_trial=True)
        for i in range(61):
            t = i*.04
            s, f, e = sample(t)
            f.update(backend='first_trial', first_trial=dict(schema=PROFILE, acceptance='UNVERIFIED',
                max_distance_m=.50, max_vx_mps=.20, max_seconds=60., one_shot=True, guardian_ready=True))
            c = g.tick(t, s, f, e)
        self.assertTrue(c['ready'], c)
        self.assertTrue(g.arm(2.4)[0])
        f.update(mono=2.44, backend='simulation')
        c = g.tick(2.44, s, f, e)
        self.assertFalse(g.armed)
        self.assertEqual(c['vx'], 0.)

    def test_full_first_trial_through_loopback_gateway(self):
        server = subprocess.Popen([sys.executable, '-m', 'native_nav.gateway', '--simulate-first-trial', '--port', '0'],
                                  cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        client = None
        try:
            port = json.loads(server.stdout.readline())['port']
            client = Client(port)
            route = json.loads((bootstrap.INDOOR/'assets/route.json').read_text())
            short = make_short_route(route)
            p = np.r_[short['waypoints'][0]['xyz'], short['waypoints'][0]['yaw']]
            g = ShortTrial(route, 'map1', 'boot1', first_trial=True)
            start = previous = time.monotonic()
            pending = active = False
            arm_count = sent = 0
            while time.monotonic()-start < 20.:
                now = time.monotonic()
                t, dt = now-start, now-previous
                previous = now
                client.poll(now)
                status = client.get(now)
                s, _, e = sample(t, p)
                # This synthetic test ONLY translates simulator identity. There
                # is no such translation/override in the production ROS runner.
                f = dict(status, mono=client.received-start, backend='first_trial') if status else None
                was = g.armed
                c = g.tick(t, s, f, e)
                if not active and not pending and c.get('ready') and client.action('ARM_TRIAL50', now):
                    pending = True
                    arm_count += 1
                if pending and status and status['armed']:
                    self.assertTrue(g.arm(t)[0], c)
                    pending, active = False, True
                if g.armed:
                    sent += int(client.action('VELOCITY', now, [c['vx'], 0., c['wz']]))
                elif was:
                    client.stop()
                if active:
                    self.assertTrue(g.armed or g.complete, c)
                if g.complete and status and not status['armed']:
                    break
                # Ideal actuator uses the simulated gateway's applied command,
                # not the route's requested command (includes transport lag).
                if status:
                    fbody = status['feedback']
                    v, w = fbody['speed'], fbody['wz']
                    p[0] += v*math.cos(p[3])*dt
                    p[1] += v*math.sin(p[3])*dt
                    p[3] += w*dt
                a, b = g.xyz[max(1, g.target)-1:max(1, g.target)+1]
                v = b-a
                p[2] = (a+np.clip((p[:3]-a)@v/max(v@v, 1e-9), 0, 1)*v)[2]
                time.sleep(.04)
            self.assertTrue(g.complete, c)
            self.assertFalse(status['armed'])
            self.assertEqual(arm_count, 1)
            self.assertGreater(sent, 10)
            self.assertEqual(status['first_trial']['acceptance'], 'UNVERIFIED')
        finally:
            if client: client.close()
            server.terminate()
            server.communicate(timeout=3)

    def test_full_ideal_straight_and_real_frozen_trial(self):
        for route in (simple_route(), json.loads((bootstrap.INDOOR/'assets/route.json').read_text())):
            short = make_short_route(route)
            p = np.r_[short['waypoints'][0]['xyz'], short['waypoints'][0]['yaw']]
            g = self.ready(route, p)
            self.assertTrue(g.arm(2.4)[0])
            highest = 0.
            for i in range(1500):
                t = 2.44+i*.04
                c = g.tick(t, *sample(t, p))
                self.assertTrue(g.armed or g.complete, c)
                self.assertLessEqual(c['vx'], .2)
                self.assertLessEqual(abs(c['wz']), .2)
                highest = max(highest, c['vx'])
                if g.complete: break
                p[0] += c['vx']*math.cos(p[3])*.04
                p[1] += c['vx']*math.sin(p[3])*.04
                p[3] += c['wz']*.04
                a, b = g.xyz[max(1, g.target)-1:max(1, g.target)+1]
                v = b-a
                u = np.clip((p[:3]-a)@v/max(v@v, 1e-9), 0, 1)
                p[2] = (a+u*v)[2]
            self.assertTrue(g.complete, c)
            self.assertLessEqual(g.final_error, .05)
            self.assertEqual(g.reached, list(range(len(g.xyz))))
            self.assertGreater(highest, .03)
            self.assert_terminal(g, t+.04, p)
            print(json.dumps(dict(test='trial50_IDEAL_NOT_PHYSICAL', seconds=t-2.4, max_vx=highest,
                                  final_error=g.final_error, commanded_distance=g.command_distance)))

    def test_degraded_limits_preserved(self):
        for kwargs, cap in ((dict(age=.30, mode='DEGRADED'), .04),
                            (dict(age=.40, mode='PREDICT_ONLY'), .02), (dict(safety_age=.3), .02)):
            g = self.active()
            c = g.tick(2.52, *sample(2.52, **kwargs))
            self.assertTrue(g.armed, c)
            self.assertLessEqual(c['vx'], cap)

    def test_hold_authority_faults_and_stop_end_trial(self):
        faults = [lambda s,f,e: s['solution'].update(mode='LOST'),
                  lambda s,f,e: s['safety_records']['rear'].update(blocked=True),
                  lambda s,f,e: f.update(armed=False),
                  lambda s,f,e: f.update(backend='simulation'),
                  lambda s,f,e: f.update(code_hash='old-gateway'),
                  lambda s,f,e: e.update(other_nav_publishers=1),
                  lambda s,f,e: s.update(run_id='restarted'),
                  lambda s,f,e: s.update(pose=[.4,0,0,0]),
                  lambda s,f,e: s.update(mono=-1.)]
        for mutate in faults:
            g = self.active()
            args = sample(2.52)
            mutate(*args)
            g.tick(2.52, *args)
            self.assert_terminal(g, 2.56)
        g = self.active()
        g.stop('OPERATOR_STOP')
        self.assert_terminal(g, 2.52)

    def test_fresh_ceiling_and_post_degradation_recovery_cap(self):
        g = self.active()
        self.assertEqual(g.velocity_limits(g.latest['snapshot']), (.2, .2))
        g.tick(2.52, *sample(2.52, age=.3, mode='DEGRADED'))
        g.tick(2.56, *sample(2.56))
        self.assertLessEqual(g.velocity_limits(g.latest['snapshot'])[0], .04)

    def test_loop_stall_time_and_command_budgets(self):
        for kind in ('stall', 'timeout', 'distance', 'turn'):
            g = self.active()
            t = 2.52
            if kind == 'stall': t = 3.
            if kind == 'timeout': g.started = -58.
            if kind == 'distance': g.command_distance = .5
            if kind == 'turn': g.command_turn = 1.2
            g.tick(t, *sample(t))
            self.assert_terminal(g, t+.04)

    def test_prediction_or_single_observation_cannot_finish(self):
        # Arrange near-goal state without a teleport; repeated timestamp becomes
        # stale and never counts as several independent map observations.
        for predicted in (False, True):
            g = self.active()
            g.last_pose = None
            for i in range(16):
                t = 2.52+i*.04
                age = .4 if predicted else t-2.50
                c = g.tick(t, *sample(t, (.46,0,0,0), age=age,
                                      mode='PREDICT_ONLY' if predicted else 'TRACKING'))
                self.assertEqual(c['vx'], 0.)
                self.assertFalse(g.complete)

    def test_arrival_requires_stationary_raw_feedback(self):
        g = self.active()
        g.last_pose = None
        for i in range(35):
            t = 2.52+i*.04
            s, f, e = sample(t, (.46,0,0,0))
            f['feedback']['speed'] = .02
            c = g.tick(t, s, f, e)
            self.assertEqual(c['vx'], 0.)
            self.assertFalse(g.complete)
        for i in range(20):
            t = 4.+i*.04
            c = g.tick(t, *sample(t, (.46,0,0,0)))
        self.assertTrue(g.complete, c)

    @unittest.skipIf(sys.platform == 'win32', 'Production runner uses Linux fcntl/ROS')
    def test_preview_is_non_ros_and_invalid_args_refused(self):
        cmd = [sys.executable, str(ROOT/'scripts/route_ros.py'), '--trial-50cm']
        r = subprocess.run(cmd+['--describe'], capture_output=True, text=True, timeout=10)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertFalse(json.loads(r.stdout)['execute'])
        for args in (['--resume-checkpoint'], ['--seconds', 'nan'], ['--describe', '--execute']):
            r = subprocess.run(cmd+args, capture_output=True, text=True, timeout=10)
            self.assertNotEqual(r.returncode, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
