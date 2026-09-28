import json
from pathlib import Path
import sys
import subprocess
import time
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav import bootstrap
from native_nav.route import NativeRoute
from native_nav.client import Client
sys.path.insert(0, str(bootstrap.INDOOR/'tests'))
from test_continuity import robust_data
from test_control import simple_route


def data(t, p=(0., 0., 0., 0.), **kwargs):
    s, _, _ = robust_data(t, p, **kwargs)
    f = dict(mono=t, backend='execute', ready=True, session='gateway1', armed=True)
    e = dict(mono=t, localization_publishers=1, other_nav_publishers=0,
             other_named_joint_publishers=[], mode_publishers=0, rl_service_active=False)
    return s, f, e


class RouteTests(unittest.TestCase):
    def ready(self, route=None, p=(0., 0., 0., 0.)):
        g = NativeRoute(route or simple_route(), 'map1', 'boot1')
        for i in range(56):
            c = g.tick(i*.04, *data(i*.04, p))
        self.assertTrue(c['ready'], c)
        return g

    def active(self):
        g = self.ready()
        self.assertTrue(g.arm(2.2)[0])
        g.tick(2.24, *data(2.24))
        self.assertGreater(g.tick(2.28, *data(2.28))['vx'], 0.)
        return g

    def test_never_automatically_arms(self):
        g = self.ready()
        self.assertFalse(g.armed)
        self.assertEqual(g.tick(2.24, *data(2.24))['vx'], 0.)

    def test_v2_short_faults_preserved(self):
        for kwargs, vx, wz in [(dict(age=.32), .04, .10), (dict(age=.50), .02, .06),
                               (dict(safety_age=.3), .02, 0.), (dict(imu_age=.06, age=.10), .02, 0.)]:
            g = self.active()
            c = g.tick(2.32, *data(2.32, **kwargs))
            self.assertTrue(g.armed, c)
            self.assertGreater(c['vx'], 0.)
            self.assertLessEqual(c['vx'], vx)
            self.assertLessEqual(abs(c['wz']), wz)

    def test_v2_long_faults_latch(self):
        for kwargs in [dict(age=.56), dict(imu_age=.09), dict(safety_age=.41), dict(mode='LOST')]:
            g = self.active()
            self.assertEqual(g.tick(2.32, *data(2.32, **kwargs))['vx'], 0.)
            self.assertFalse(g.armed)
            for i in range(70):
                self.assertEqual(g.tick(2.36+i*.04, *data(2.36+i*.04))['vx'], 0.)

    def test_backend_and_identity_faults(self):
        changes = [lambda s,f,e: f.update(backend='simulation'), lambda s,f,e: f.update(armed=False),
                   lambda s,f,e: f.update(ready=False), lambda s,f,e: f.update(session='new'),
                   lambda s,f,e: f.update(mono=0), lambda s,f,e: e.update(other_nav_publishers=1),
                   lambda s,f,e: e.update(mode_publishers=1), lambda s,f,e: e.update(rl_service_active=True),
                   lambda s,f,e: e.update(other_named_joint_publishers=['goai']),
                   lambda s,f,e: e.update(localization_publishers=2),
                   lambda s,f,e: s.update(generation=[99]), lambda s,f,e: s.update(run_id='new'),
                   lambda s,f,e: s.update(asset_id='wrong'), lambda s,f,e: s.update(boot_id='wrong'),
                   lambda s,f,e: s['safety_records']['rear'].update(blocked=True)]
        for change in changes:
            g = self.active()
            x = data(2.32)
            change(*x)
            self.assertEqual(g.tick(2.32, *x)['vx'], 0.)
            self.assertFalse(g.armed)

    def test_stall_and_prediction_no_waypoint_completion(self):
        g = self.active()
        self.assertEqual(g.tick(2.60, *data(2.60))['vx'], 0.)
        self.assertFalse(g.armed)
        g.solution_mode = 'PREDICT_ONLY'
        g.measurement_age = .5
        self.assertFalse(g.goal_reached(data(3)[0], 0.))

    def test_frozen_dependencies(self):
        self.assertEqual(len(bootstrap.verify()), 64)

    def test_full_indoor_route_in_order(self):
        route = json.loads((bootstrap.INDOOR/'assets/route.json').read_text())
        p = np.r_[route['waypoints'][0]['xyz'], 0.]
        g = self.ready(route, p)
        self.assertTrue(g.arm(2.2)[0])
        for i in range(6500):
            t = 2.24+i*.04
            c = g.tick(t, *data(t, p))
            self.assertTrue(g.armed or g.complete, c)
            self.assertLessEqual(c['vx'], .10)
            self.assertLessEqual(abs(c['wz']), .20)
            if g.complete:
                break
            p[0] += c['vx']*np.cos(p[3])*.04
            p[1] += c['vx']*np.sin(p[3])*.04
            p[3] += c['wz']*.04
            k = max(1, g.target)
            a, b = g.xyz[k-1:k+1]
            v = b-a
            u = np.clip((p[:3]-a)@v/max(v@v, 1e-9), 0, 1)
            p[2] = (a+u*v)[2]
        self.assertTrue(g.complete)
        self.assertEqual(g.reached, list(range(len(route['waypoints']))))
        print(json.dumps(dict(test='ideal_kinematics_not_physical_accuracy', points=len(g.reached),
                              seconds=t, endpoint_distance_m=float(np.linalg.norm(p[:3]-g.xyz[-1])))))

    def test_route_and_tcp_gateway_25hz_handshake_and_fault(self):
        # Only this synthetic test maps simulated feedback to the route contract.
        # There is intentionally no such override in the production ROS runner.
        server = subprocess.Popen([sys.executable, '-m', 'native_nav.gateway', '--simulation', '--port', '0'],
                                  cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        client = None
        try:
            port = json.loads(server.stdout.readline())['port']
            client = Client(port)
            g = NativeRoute(simple_route(), 'map1', 'boot1')
            start = time.monotonic()
            pending = None
            active_at = None
            stopped = False
            samples = 0
            sent = 0
            last_status = None
            p = np.zeros(4)
            while time.monotonic()-start < 4.:
                now = time.monotonic()
                t = now-start
                client.poll(now)
                st = client.get(now)
                s, f, e = data(t, p)
                if st:
                    last_status = st
                    f = dict(st, mono=client.received-start, backend='execute')
                else:
                    f = None
                if active_at is not None and t-active_at > .6:
                    s['solution']['measurement_mono'] = t-.56
                was_armed = g.armed
                c = g.tick(t, s, f, e)
                if pending is None and active_at is None and c.get('ready'):
                    if client.action('ARM', now):
                        pending = t
                if pending is not None and st and st['armed']:
                    self.assertTrue(g.arm(t)[0])
                    pending = None
                    active_at = t
                if g.armed:
                    sent += int(client.action('VELOCITY', now, [c['vx'], c['vy'], c['wz']]))
                    samples += 1
                    p[0] += c['vx']*.04
                elif was_armed:
                    client.stop()
                    stopped = True
                if stopped and st and not st['armed']:
                    break
                time.sleep(.04)
            self.assertIsNotNone(active_at)
            self.assertGreater(samples, 10)
            self.assertGreater(sent, 5)
            self.assertTrue(stopped)
            self.assertFalse(last_status['armed'])
            self.assertGreater(last_status['sent_nonzero'], 0)
            self.assertFalse(client.closed, client.error)
        finally:
            if client:
                client.close()
            server.terminate()
            try:
                server.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                server.kill()
                server.communicate(timeout=2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
