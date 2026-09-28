import unittest
from native_nav.gate import Gate
from native_nav.posture import posture_packet, posture_reason


class PostureTests(unittest.TestCase):
    def gate(self):
        g = Gate()
        g.feedback = dict(state=0, basic_state=0, gait=0, basic_gait=0, mode=0,
                          speed=0., wz=0., hard_stop=0, charge=0, sleep=False,
                          height=.068, roll=0., pitch=0.)
        g.basic_mono = g.motion_mono = g.env_mono = 1.
        g.environment = True
        return g

    def test_low_idle_stand_does_not_arm_or_enable_axes(self):
        g = self.gate()
        self.assertEqual(posture_packet(g, 1., 'STAND')['PatrolDevice']['Items'], {'MotionParam': 1})
        self.assertFalse(g.armed)
        self.assertFalse(g.owned)
        self.assertIsNotNone(g.check(1.))
        for action in ('ARM', 'VELOCITY', 'MODE_NAV', 'SDK_OFF', 'FLAT'):
            with self.assertRaises(ValueError):
                posture_packet(g, 1., action)

    def test_unknown_pose_or_non_regular_mode_is_rejected(self):
        for key, value in [('height', None), ('height', .4), ('roll', .4), ('pitch', float('nan')),
                           ('mode', 1), ('hard_stop', 1), ('charge', 1), ('sleep', True),
                           ('speed', .04), ('wz', .06)]:
            g = self.gate()
            g.feedback[key] = value
            self.assertIsNotNone(posture_reason(g, 1., 'STAND'), key)
        self.assertIsNotNone(posture_reason(self.gate(), 2., 'STAND'))

    def test_lie_requires_live_stationary_standing_feedback(self):
        g = self.gate()
        self.assertIsNotNone(posture_reason(g, 1., 'LIE'))
        g.feedback.update(state=17, basic_state=17, height=.32)
        self.assertEqual(posture_packet(g, 1., 'LIE')['PatrolDevice']['Items'], {'MotionParam': 4})
        g.environment = False
        self.assertIsNotNone(posture_reason(g, 1., 'LIE'))
