"""Manual completion closes preceding points; the selected point remains next."""
import unittest
from mission import Mission
from test_unified import FakeBridge,route
from test_forward_entry import pose,clear

class ManualProgressTests(unittest.TestCase):
    def make(self):
        b=FakeBridge();m=Mission({'indoor':route()},b)
        m.plans.set_policy('indoor',[1],'stairs_normal');return m,b
    def select(self,m,index):
        return m.command(dict(action='set_progress',route='indoor',target_index=index,
            waypoint_id=m.navigator.route['waypoints'][index]['id']),None,10.)
    def test_jump_marks_preceding_points_completed(self):
        m,b=self.make();m.navigator.reached=[0];self.select(m,3)
        self.assertEqual(m.navigator.reached,[0,1,2]);self.assertEqual(m.navigator.skipped,[])
        self.assertEqual(m.owner,'paused');self.assertEqual(b.stops,1);self.assertEqual(b.actions,[])
    def test_rewind_reopens_completed_route_and_does_not_skip_exact_target(self):
        m,b=self.make();n=m.navigator;n.reached=list(range(5));n.target=4;n.reason='COMPLETE'
        self.select(m,1);m.command(dict(action='shadow'),pose(x=3.6),10.)
        out=m.step(pose(x=3.6),clear(10.),10.)
        self.assertEqual(out['target_index'],1);self.assertEqual(out['reached_indices'],[0])
        self.assertEqual(n.entry['selection'],'manual_target');self.assertEqual(n.entry['path'][-1],[1.,0.,0.])
    def test_pause_and_relocalization_keep_selected_target(self):
        m,b=self.make();self.select(m,1);m.command(dict(action='shadow'),pose(x=3.6),10.)
        m.pause();m.command(dict(action='shadow'),pose(t=11.,x=3.5,epoch=2),11.)
        out=m.step(pose(t=11.,x=3.5,epoch=3),clear(11.),11.)
        self.assertEqual(out['target_index'],1);self.assertEqual(m.navigator.manual_target,1)
    def test_arrival_releases_target_and_continues_route_presets(self):
        m,b=self.make();self.select(m,1);m.command(dict(action='shadow'),pose(x=.4),10.)
        out=m.step(pose(x=.9),clear(10.),10.)
        self.assertEqual(out['target_index'],2);self.assertIsNone(out['manual_target'])
        self.assertEqual(out['policy'],'stairs_normal');self.assertEqual(out['reached_indices'],[0,1]);self.assertEqual(out['skipped_indices'],[])
    def test_invalid_selection_does_not_interrupt_navigation(self):
        m,b=self.make();m.command(dict(action='shadow'),pose(x=.5),10.)
        for index,rid,pid in [(True,'indoor','1'),(-1,'indoor','0'),(99,'indoor','0'),(1,'full','1'),(1,'indoor','old')]:
            with self.assertRaises(ValueError):
                m.command(dict(action='set_progress',target_index=index,route=rid,waypoint_id=pid),None,10.)
        self.assertEqual(m.owner,'auto');self.assertEqual(b.stops,0)
    def test_start_clears_override_and_old_preview(self):
        m,b=self.make();m.plans.preview={'id':'old'};m.override='basic_normal';self.select(m,2)
        self.assertIsNone(m.plans.preview);self.assertEqual(m.override,'basic_normal')
        m.command(dict(action='shadow'),pose(x=3.6),10.)
        self.assertEqual(m.desired_policy(),'stairs_normal');self.assertEqual(m.policy_mode(),'route_preset')
    def test_checkpoint_preserves_selection_but_new_route_clears_it(self):
        m,b=self.make();self.select(m,2);new,bridge=self.make()
        new.restore_paused(dict(route_id='indoor',route=m.navigator.route,target_index=2,reached_indices=[],
            skipped_indices=[0,1],manual_target=2,manual_progress=m.navigator.manual_progress,override=None))
        self.assertFalse(bridge.actions);self.assertEqual(bridge.stops,0)
        new.command(dict(action='shadow'),pose(x=3.6),10.);self.assertEqual(new.navigator.target,2)
        new.command(dict(action='select',route='indoor'),None,10.)
        self.assertIsNone(new.navigator.manual_target);self.assertIsNone(new.navigator.manual_progress)

if __name__=='__main__':unittest.main()
