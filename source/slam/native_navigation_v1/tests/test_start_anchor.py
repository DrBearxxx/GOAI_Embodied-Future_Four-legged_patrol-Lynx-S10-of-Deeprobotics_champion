"""Temporary test-route placement; frozen map and localization stay untouched."""
import copy
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native_nav.short_trial import ShortTrial
from native_nav.first_trial import PROFILE
from test_short_trial import sample
from test_route import simple_route


def observed(t, pose=(.8, .6, .04, 0.), **kwargs):
    s, f, e = sample(t, pose, **kwargs)
    f.update(backend='first_trial', first_trial=dict(schema=PROFILE, acceptance='UNVERIFIED',
             max_distance_m=.50, max_vx_mps=.20, max_seconds=60., one_shot=True, guardian_ready=True))
    return s, f, e


class StartAnchorTests(unittest.TestCase):
    def make(self):
        return ShortTrial(simple_route(), 'map1', 'boot1', first_trial=True, allow_start_offset=True)

    def prepared(self):
        g = self.make()
        for i in range(140):
            now = i*.04
            c = g.tick(now, *observed(now))
            self.assertEqual(c['vx'], 0.)
        self.assertTrue(c['ready'], c)
        return g, now

    def test_translation_preserves_shape_length_heading_and_source(self):
        source = simple_route()
        original = copy.deepcopy(source)
        g = ShortTrial(source, 'map1', 'boot1', first_trial=True, allow_start_offset=True)
        for i in range(140):
            now = i*.04
            s, f, e = observed(now)
            saved = copy.deepcopy(s)
            c = g.tick(now, s, f, e)
            self.assertEqual(s, saved)
        self.assertEqual(source, original)
        self.assertTrue(c['ready'], c)
        np.testing.assert_allclose(g.xyz[0], [.8, .6, .04])
        self.assertAlmostEqual(np.linalg.norm(np.diff(g.xyz[:, :2], axis=0), axis=1).sum(), .5)
        self.assertEqual(g.route['waypoints'][0]['yaw'], 0.)
        self.assertFalse(g.armed)

    def test_no_anchor_or_arm_from_stale_bad_identity_or_moving_input(self):
        for kind in ('stale', 'identity', 'motion', 'backend', 'uncertain', 'obstacle'):
            g = self.make()
            for i in range(90):
                t = i*.04
                s, f, e = observed(t)
                if kind == 'stale': s['mono'] = t-1.
                if kind == 'identity': s['asset_id'] = 'wrong-map'
                if kind == 'motion': f['feedback']['speed'] = .10
                if kind == 'backend': f['ready'] = False
                if kind == 'uncertain': s['solution']['mode'] = 'LOST'
                if kind == 'obstacle': s['safety_records']['front']['blocked'] = True
                c = g.tick(t, s, f, e)
            self.assertIsNone(g.start_anchor, kind)
            self.assertFalse(c['ready'], kind)
            self.assertFalse(g.arm(t)[0], kind)

    def test_anchor_does_not_follow_drift_or_stop(self):
        g, t = self.prepared()
        anchor, xyz = copy.deepcopy(g.start_anchor), g.xyz.copy()
        g.stop('OPERATOR_STOP')
        for i in range(1, 80):
            c = g.tick(t+i*.04, *observed(t+i*.04, (1.1, .6, .04, 0.)))
        self.assertFalse(c['ready'])
        self.assertEqual(g.start_anchor, anchor)
        np.testing.assert_array_equal(g.xyz, xyz)

    def test_map_generation_change_during_collection_requires_new_window(self):
        g = self.make()
        for i in range(80):
            t = i*.04
            s, f, e = observed(t)
            s['generation'] = s['solution']['generation'] = [i//25, 0]
            g.tick(t, s, f, e)
        self.assertIsNone(g.start_anchor)

    def test_offset_bounds_reject_wrong_floor_and_nonlocal_start(self):
        for pose in ((2.1, 0., 0., 0.), (.8, .6, .21, 0.)):
            g = self.make()
            for i in range(90):
                t = i*.04
                c = g.tick(t, *observed(t, pose))
            self.assertIsNone(g.start_anchor)
            self.assertEqual(c['state'], 'START_OFFSET_OUTSIDE_LOCAL_TRIAL_ENVELOPE')

    def test_not_available_in_formal_navigation(self):
        with self.assertRaisesRegex(ValueError, 'ONLY_FOR_FIRST_TRIAL'):
            ShortTrial(simple_route(), 'map1', 'boot1', allow_start_offset=True)

    def test_translated_trial_ideal_closed_loop_stays_fifty_cm(self):
        g, t = self.prepared()
        self.assertTrue(g.arm(t)[0])
        pose = np.array([.8, .6, .04, 0.])
        initial = pose.copy()
        for i in range(1, 1400):
            now = t+i*.04
            c = g.tick(now, *observed(now, pose.tolist()))
            pose[0] += c['vx']*np.cos(pose[3])*.04
            pose[1] += c['vx']*np.sin(pose[3])*.04
            pose[3] += c['wz']*.04
            self.assertLessEqual(c['vx'], .2)
            if g.complete or g.terminal_reason:
                break
        self.assertTrue(g.complete, c)
        self.assertLessEqual(np.linalg.norm(pose[:2]-initial[:2]), .5)
        self.assertLessEqual(g.final_error, .05)
        self.assertFalse(g.armed)


if __name__ == '__main__': unittest.main(verbosity=2)
