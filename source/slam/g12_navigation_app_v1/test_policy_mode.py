"""Route preset/override behavior, including the recorded B03 regression."""
import json
from pathlib import Path
import unittest
from mission import Mission
from test_unified import FakeBridge,route,solution
from test_forward_entry import clear


class PolicyModeTests(unittest.TestCase):
    def make(self):
        b=FakeBridge();m=Mission({'indoor':route()},b)
        m.plans.set_policy('indoor',[1],'stairs_normal');m.plans.set_policy('indoor',[2],'platform')
        return m,b

    def test_start_and_shadow_reset_override_using_selected_future_segment(self):
        for action in ('start','shadow'):
            m,b=self.make();m.override='basic'
            m.command(dict(action=action),solution(x=1.2),10.)
            self.assertEqual(m.navigator.target,2);self.assertIsNone(m.override)
            self.assertEqual(m.state()['policy_mode'],'route_preset')
            self.assertEqual(m.desired_policy(),'stairs_normal')

    def test_failed_start_preserves_override(self):
        m,b=self.make();m.override='platform'
        with self.assertRaisesRegex(ValueError,'FRESH_ODOMETRY_ALIGNMENT_REQUIRED'):
            m.command(dict(action='start'),None,10.)
        self.assertEqual(m.override,'platform');self.assertEqual(m.state()['policy_mode'],'override')

    def test_override_survives_waypoints_until_explicit_preset_without_stopping(self):
        m,b=self.make();m.command(dict(action='start'),solution(x=.7),10.)
        m.command(dict(action='override',policy='basic'),None,10.)
        for t,x,preset in [(10.,.7,'basic_normal'),(10.1,.85,'stairs_normal'),(10.2,1.8,'platform')]:
            m.keepalive(m.navigator.run_id,t);out=m.step(solution(t=t,x=x),clear(t),t)
            self.assertEqual(out['policy'],'basic');self.assertEqual(out['policy_mode'],'override')
            self.assertEqual(out['preset_policy'],preset);self.assertTrue(out['motion_authorized'])
            self.assertGreater(out['vx'],0.)
        m.command(dict(action='override',policy=None),None,10.25)
        out=m.step(solution(t=10.25,x=1.85),clear(10.25),10.25)
        self.assertEqual(out['policy'],'platform');self.assertEqual(out['policy_mode'],'route_preset')
        self.assertGreater(out['vx'],0.);self.assertEqual(b.stops,0)

    def test_resume_clears_previous_override(self):
        m,b=self.make();m.command(dict(action='shadow'),solution(x=1.2),10.)
        m.command(dict(action='override',policy='basic'),None,10.1)
        m.pause();self.assertEqual(m.override,'basic')
        m.command(dict(action='shadow'),solution(t=11.,x=1.25),11.)
        self.assertEqual(m.desired_policy(),'stairs_normal');self.assertIsNone(m.override)

    def test_explicit_repeat_selection_resends_once_without_automatic_retries(self):
        m,b=self.make();m.command(dict(action='start'),solution(x=.7),10.)
        m.command(dict(action='override',policy='stairs'),None,10.)
        m.step(solution(x=.7),clear(10.),10.);b.selected='stairs'
        m.command(dict(action='override',policy='stairs'),None,10.1)
        m.step(solution(t=10.1,x=.72),clear(10.1),10.1)
        m.step(solution(t=10.2,x=.74),clear(10.2),10.2)
        self.assertEqual(b.actions,['select:stairs','select:stairs']);self.assertEqual(b.stops,0)

    def test_recorded_b04_join_drops_stale_basic_and_selects_stairs(self):
        r=json.loads((Path(__file__).parent/'routes/full.json').read_text(encoding='utf-8'))
        b=FakeBridge();m=Mission({'full':r,'indoor':route()},b);m.navigator.select('full')
        m.navigator.target=3;m.navigator.reached=[0,1];m.navigator.skipped=[2];m.override='basic'
        s=solution();s['pose']=[-4.63847,-39.19361,-.25163,0.]
        m.command(dict(action='start'),s,10.)
        out=m.step(s,clear(10.),10.)
        self.assertEqual(out['target_index'],3);self.assertEqual(out['policy'],'stairs_normal')
        self.assertEqual(out['policy_mode'],'route_preset');self.assertEqual(b.actions,['select:stairs_normal'])
        self.assertEqual(m.plans.presets['full'][1:3],['stairs_normal','stairs_normal'])


if __name__=='__main__':unittest.main()
