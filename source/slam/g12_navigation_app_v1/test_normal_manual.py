import unittest
from mission import Mission
from planning import Plans, OFFICIAL, MANUAL
from test_unified import FakeBridge, route, solution


class NormalManualTests(unittest.TestCase):
    def test_catalog_and_presets_have_separate_scopes(self):
        self.assertEqual(len(MANUAL),10)
        self.assertEqual(set(OFFICIAL),{'basic_normal','stairs_normal','platform','step_move'})
        plans=Plans({'indoor':route()})
        for policy in ('basic','stairs'):
            with self.assertRaisesRegex(ValueError,'PRESET_OFFICIAL_ONLY'):
                plans.set_policy('indoor',[0],policy)

    def test_normal_manual_passes_full_axes_without_navigation_caps(self):
        for policy in ('basic_normal','stairs_normal'):
            with self.subTest(policy=policy):
                bridge=FakeBridge(); mission=Mission({'indoor':route()},bridge)
                mission.command(dict(action='manual_shadow',policy=policy),None,10.)
                mission.operator(dict(session='test',sample_seq=1,sample_age_ms=0,fresh=True,axes=[1,-1,.9]),10.)
                state=mission.step(None,{},10.)
                self.assertEqual(state['owner'],'manual')
                self.assertEqual(state['policy'],policy)
                self.assertEqual(state['input_kind'],'native_axes')
                self.assertEqual(state['axes'],[1,-1,.9])

    def test_normal_selection_overrides_while_remaining_in_navigation(self):
        mission=Mission({'indoor':route()},FakeBridge())
        mission.command(dict(action='shadow'),solution(),10.)
        mission.command(dict(action='override',policy='basic_normal'),solution(),10.1)
        self.assertEqual(mission.owner,'auto')
        self.assertEqual(mission.override,'basic_normal')
        self.assertTrue(mission.navigator.active)
        mission.command(dict(action='shadow'),solution(t=11.),11.)
        self.assertEqual(mission.policy_mode(),'route_preset')
        self.assertEqual(mission.desired_policy(),'basic_normal')
