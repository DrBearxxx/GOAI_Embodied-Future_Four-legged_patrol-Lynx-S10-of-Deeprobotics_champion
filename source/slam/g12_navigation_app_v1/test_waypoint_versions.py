import unittest
from mission import Mission
from planning import OFFICIAL,MANUAL,WAYPOINT_POLICIES
from test_unified import FakeBridge,route,solution
from backend.core import MODES,WAYPOINT

class VersionTests(unittest.TestCase):
    def test_new_versions_are_additional_manual_choices(self):
        self.assertEqual(set(WAYPOINT_POLICIES),{'low','high','low_v5','high_v2'})
        self.assertEqual(set(WAYPOINT),set(WAYPOINT_POLICIES))
        self.assertEqual(len(MANUAL),10)
        self.assertEqual(set(OFFICIAL),{'basic_normal','stairs_normal','platform','step_move'})
        for policy in ('low_v5','high_v2'):
            m=Mission({'indoor':route()},FakeBridge())
            m.command(dict(action='manual_shadow',policy=policy),None,10.)
            m.operator(dict(session='test',sample_seq=1,sample_age_ms=0,fresh=True,axes=[1.,-.6,.8]),10.)
            out=m.step(None,{},10.)
            self.assertEqual(out['policy'],policy);self.assertEqual(out['input_kind'],'waypoint_axes')
            self.assertEqual(out['axes'],[1.,-.6,.8]);self.assertEqual(out['owner'],'manual')
            with self.assertRaisesRegex(ValueError,'PRESET_OFFICIAL_ONLY'):m.plans.set_policy('indoor',[0],policy)

    def test_override_does_not_send_waypoint_as_velocity(self):
        for policy in ('low_v5','high_v2'):
            b=FakeBridge();m=Mission({'indoor':route()},b)
            m.command(dict(action='shadow'),solution(),10.)
            m.command(dict(action='override',policy=policy),None,10.1)
            self.assertEqual(m.owner,'paused');self.assertFalse(m.navigator.active)
            self.assertEqual(b.actions,['select:'+policy]);self.assertIsNone(m.override)

    def test_restart_reopens_all_waypoints_and_preserves_route_geometry(self):
        m=Mission({'indoor':route()},FakeBridge());n=m.navigator
        n.set_progress(4,'first',10.);n.reached.append(4);n.pause('COMPLETE')
        original=n.route
        m.command(dict(action='set_progress',route='indoor',waypoint_id='0',target_index=0),None,11.)
        self.assertIs(n.route,original);self.assertEqual(len(n.route['waypoints']),5)
        self.assertEqual(n.reached,[]);self.assertEqual(n.skipped,[]);self.assertEqual(n.manual_target,0)
        m.command(dict(action='shadow'),solution(t=11.,x=3.8),11.)
        self.assertEqual(n.target,0);self.assertEqual(n.entry['path'][-1],[0.,0.,0.])

    def test_full_route_progress_and_restart_after_replan(self):
        for index in (0,1):
            m=Mission({'indoor':route()},FakeBridge());n=m.navigator
            planned=m.plans.propose('indoor',4,[],solution(x=2.5),10.)
            m.command(dict(action='apply_plan',plan_id=planned['id']),solution(x=2.5),10.)
            self.assertEqual(n.route['waypoints'][0]['id'],'reentry')
            self.assertLess(len(n.route['waypoints']),5)
            m.command(dict(action='set_progress',route='indoor',route_scope='preset',
                waypoint_id=str(index),target_index=index),None,11.)
            self.assertEqual(len(n.route['waypoints']),5)
            self.assertEqual(n.route['waypoints'][0]['id'],'0')
            self.assertEqual(n.reached,list(range(index)));self.assertEqual(n.skipped,[])
            self.assertEqual(m.owner,'paused');self.assertEqual(n.manual_target,index)
            m.command(dict(action='shadow'),solution(t=11.,x=3.8),11.)
            self.assertEqual(n.target,index);self.assertEqual(n.entry['selection'],'manual_target')

    def test_invalid_original_progress_does_not_replace_replan(self):
        m=Mission({'indoor':route()},FakeBridge());n=m.navigator
        planned=m.plans.propose('indoor',4,[],solution(x=2.5),10.)
        m.command(dict(action='apply_plan',plan_id=planned['id']),solution(x=2.5),10.)
        original=n.route
        with self.assertRaisesRegex(ValueError,'PROGRESS_ROUTE_CHANGED'):
            m.command(dict(action='set_progress',route='indoor',route_scope='preset',
                waypoint_id='wrong',target_index=0),None,11.)
        self.assertIs(n.route,original)

if __name__=='__main__':unittest.main()
