"""Route-entry behavior with simulated motion; no robot transport."""
import copy,math,unittest
from navigation import Navigator
from planning import forward_entry
from mission import Mission
from test_unified import FakeBridge,route,solution


def pose(t=10.,x=0.,y=0.,yaw=0.,epoch=1):
    s=solution(t=t,x=x,epoch=epoch);s['pose']=[x,y,0.,yaw];return s

def clear(t):return {'front':dict(valid=True,blocked=False,mono=t)}


class ForwardEntryTests(unittest.TestCase):
    def test_starts_42cm_away_without_manual_return_to_origin(self):
        n=Navigator({'indoor':route()});s=pose(x=.1,y=.41)
        n.start(s,10.,execute=True);out=n.step(s,clear(10.),10.)
        self.assertEqual(n.target,1);self.assertEqual(n.skipped,[0]);self.assertEqual(n.reached,[])
        self.assertEqual(out['state'],'JOINING_ROUTE');self.assertTrue(out['motion_authorized'])
        self.assertGreater(abs(out['vx'])+abs(out['wz']),0.)
    def test_nearest_point_behind_is_not_selected(self):
        p=forward_entry(route(),0,[1.2,.1,0.])
        self.assertEqual(p['target_index'],2)
    def test_reached_and_skipped_points_are_distinct_and_never_reselected(self):
        n=Navigator({'indoor':route()});n.reached=[0];n.target=1
        n.start(pose(x=2.2,y=.4),10.)
        self.assertEqual(n.target,3);self.assertEqual(n.reached,[0]);self.assertEqual(n.skipped,[1,2])
        n.pause();n.start(pose(t=11.,x=.1,y=.4),11.)
        self.assertGreaterEqual(n.target,3);self.assertEqual(n.reached,[0])
    def test_entry_preserves_original_route_and_joins_incoming_segment(self):
        r=route();before=copy.deepcopy(r);p=forward_entry(r,0,[1.4,.8,0.])
        self.assertEqual(r,before);self.assertEqual(p['target_index'],2)
        self.assertEqual(p['path'],[[1.4,.8,0.],[2.,0.,0.]])  # merge ahead, never return to the perpendicular projection
    def test_three_dimensional_distance_does_not_pick_same_xy_upstairs(self):
        r=route();r['waypoints'][3]['xyz']=[1.4,0.,3.]
        self.assertEqual(forward_entry(r,1,[1.4,0.,0.])['target_index'],2)
    def test_relocalization_resume_replans_with_new_pose_and_keeps_progress(self):
        n=Navigator({'indoor':route()});n.reached=[0];n.target=1;n.generation=(1,0)
        n.pause('RELOCALIZATION_REQUESTED');n.start(pose(x=2.4,y=.4,epoch=2),10.)
        self.assertEqual(n.target,3);self.assertEqual(n.generation,(2,0));self.assertEqual(n.reached,[0])
    def test_resume_changes_entry_origin_and_run_id(self):
        n=Navigator({'indoor':route()});n.start(pose(x=.4,y=.8),10.)
        previous=n.run_id;n.pause();n.start(pose(t=11.,x=1.4,y=.5),11.)
        self.assertNotEqual(n.run_id,previous);self.assertEqual(n.entry['path'][0],[1.4,.5,0.])
    def test_connector_still_uses_obstacle_feedback(self):
        n=Navigator({'indoor':route()});s=pose(x=.1,y=.4);n.start(s,10.,execute=True)
        p=clear(10.);p['front']['blocked']=True;out=n.step(s,p,10.)
        self.assertEqual(out['state'],'HOLD_OBSTACLE');self.assertEqual(out['vx'],0.)
    def test_selected_future_segment_inherits_policy_and_warning(self):
        r=route();r['edges'][1]['warnings']=['stairs_unverified']
        m=Mission({'indoor':r},FakeBridge());m.plans.set_policy('indoor',[1],'stairs_normal')
        m.command(dict(action='shadow'),pose(x=1.4,y=.4),10.)
        out=m.step(pose(x=1.4,y=.4),clear(10.),10.)
        self.assertEqual(out['policy'],'stairs_normal');self.assertEqual(out['state'],'HOLD_ROUTE_VALIDATION')
    def test_new_route_selection_resets_entry_and_progress(self):
        n=Navigator({'indoor':route()});n.start(pose(x=1.2),10.)
        n.select('indoor');self.assertIsNone(n.entry);self.assertEqual(n.skipped,[]);self.assertEqual(n.target,0)
    def test_single_remaining_endpoint_can_be_reached_from_beyond_it(self):
        self.assertEqual(forward_entry(route(),4,[5.,.2,0.])['target_index'],4)
    def test_completed_route_cannot_reselect_reached_endpoint_after_pause(self):
        n=Navigator({'indoor':route()});n.reached=list(range(5));n.target=4;n.pause()
        with self.assertRaisesRegex(ValueError,'SELECT_ROUTE'):n.start(pose(x=4.),10.)
    def test_closed_loop_connector_then_remaining_route_completes(self):
        n=Navigator({'indoor':route()});x,y,yaw,t=1.4,.8,-math.pi/2,10.
        n.start(pose(t,x,y,yaw),t)
        for _ in range(3000):
            n.keepalive(n.run_id,t);out=n.step(pose(t,x,y,yaw),clear(t),t)
            self.assertNotIn(out['state'],('WAIT_ROUTE_ENTRY','HOLD_OFF_ROUTE'))
            x+=(out['vx']*math.cos(yaw)-out['vy']*math.sin(yaw))*.05
            y+=(out['vx']*math.sin(yaw)+out['vy']*math.cos(yaw))*.05;yaw+=out['wz']*.05;t+=.05
            if out['state']=='COMPLETE':break
        self.assertEqual(out['state'],'COMPLETE');self.assertEqual(n.reached,[2,3,4]);self.assertEqual(n.skipped,[0,1])


if __name__=='__main__':unittest.main()
