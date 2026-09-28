"""Manual-to-navigation handover and causal trajectory progress regressions."""
import math
import unittest
from mission import Mission
from navigation import Navigator
from planning import forward_entry
from trajectory import RouteTrajectory
from test_unified import FakeBridge,route
from test_forward_entry import pose,clear


class ResumeTests(unittest.TestCase):
    def test_manual_authorization_is_not_reused_as_navigation_link_failure(self):
        b=FakeBridge();b.input_kind='native_axes';b.enabled=False
        m=Mission({'indoor':route()},b);m.owner='manual';m.execution=True
        m.last=dict(motion_authorized=True,state='MANUAL');m.pending_stand='old-manual'
        m.command({'action':'start'},pose(x=.5),10.)
        out=m.step(pose(x=.5),clear(10.),10.)
        self.assertEqual(out['owner'],'auto');self.assertEqual(out['state'],'WAIT_POLICY_FEEDBACK')
        self.assertIsNone(m.pending_stand);self.assertTrue(m.resume_pending)

    def test_waiting_handover_does_not_pass_waypoint_and_connects_at_release_pose(self):
        b=FakeBridge();b.input_kind='native_axes';b.enabled=False
        m=Mission({'indoor':route()},b);m.command({'action':'start'},pose(x=.5),10.)
        initial_target=m.navigator.target
        for t,x in ((10.,.5),(10.2,1.2),(10.4,1.8)):
            out=m.step(pose(t=t,x=x),clear(t),t)
            self.assertEqual(m.navigator.target,initial_target)
            self.assertEqual(m.navigator.reached,[]);self.assertIsNone(m.navigator.tracker.previous)
            self.assertEqual(out['vx'],0.)
        b.selected='basic_normal';b.input_kind='velocity';b.enabled=True
        out=m.step(pose(t=10.5,x=1.85,y=.4),clear(10.5),10.5)
        self.assertEqual(m.navigator.target,2);self.assertFalse(m.resume_pending)
        self.assertEqual(m.navigator.entry['path'][0],[1.85,.4,0.])
        self.assertTrue(out['motion_authorized']);self.assertEqual(out['policy_mode'],'route_preset')

    def test_alignment_change_during_handover_waits_for_release_before_reentry(self):
        b=FakeBridge();b.input_kind='native_axes';b.enabled=False
        m=Mission({'indoor':route()},b);m.command({'action':'start'},pose(x=.5),10.)
        out=m.step(pose(t=10.1,x=2.2,epoch=2),clear(10.1),10.1)
        self.assertEqual(m.navigator.target,1);self.assertEqual(out['vx'],0.)
        b.input_kind='velocity';b.selected='basic_normal';b.enabled=True
        out=m.step(pose(t=10.2,x=2.2,epoch=2),clear(10.2),10.2)
        self.assertEqual(m.navigator.target,3);self.assertEqual(m.navigator.generation,(2,0))
        self.assertTrue(out['motion_authorized'])

    def test_resume_does_not_add_a_second_arrival_confirmation_for_short_scan_gap(self):
        b=FakeBridge();b.enabled=False;b.input_kind='native_axes'
        m=Mission({'indoor':route()},b);m.command({'action':'start'},pose(x=.5),10.)
        m.step(pose(x=.5),clear(10.),10.)
        b.enabled=True;b.input_kind='velocity';b.selected='basic_normal'
        s=pose(t=10.3,x=.6);s.update(mode='PREDICT_ONLY',arrival_valid=False,measurement_mono=10.,measurement_pose=[.5,0,0,0])
        out=m.step(s,clear(10.3),10.3)
        self.assertFalse(m.resume_pending);self.assertTrue(out['motion_authorized'])
        self.assertEqual(out['passed_indices'],[])

    def test_stick_takeover_clears_previous_authority_too(self):
        b=FakeBridge();m=Mission({'indoor':route()},b)
        m.command({'action':'start'},pose(x=.5),10.)
        m.last=dict(motion_authorized=True);b.enabled=False
        m.operator(dict(session='test',sample_seq=1,sample_age_ms=0,fresh=True,axes=[.6,0,0]),10.)
        out=m.step(pose(x=.5),clear(10.),10.)
        self.assertEqual(out['owner'],'manual');self.assertNotEqual(out['state'],'CONTROL_LINK_LOST')

    def test_manual_progress_is_exact_after_handover_and_original_route_is_preserved(self):
        b=FakeBridge();m=Mission({'indoor':route()},b)
        m.command(dict(action='set_progress',target_index=3,route='indoor',waypoint_id='3'),None,10.)
        m.command({'action':'shadow'},pose(t=11.,x=1.),11.)
        out=m.step(pose(t=11.,x=1.),clear(11.),11.)
        self.assertEqual(m.navigator.target,3);self.assertEqual(m.navigator.reached,[0,1,2])
        self.assertEqual(m.navigator.entry['path'],[[1.,0.,0.],[3.,0.,0.]])
        self.assertEqual(len(m.plans.route('indoor')['waypoints']),5)
        self.assertTrue(out['active'])

    def test_connection_has_no_small_backwards_projection_leg(self):
        r=route();r['waypoints'][1]['xyz']=[0.,0.,0.];r['waypoints'][2]['xyz']=[0.,5.,0.]
        entry=forward_entry(r,2,[.26,-.1,0.,math.pi/2])
        self.assertGreater(entry['path'][1][1],entry['path'][0][1])

    def test_flythrough_join_can_be_passed_outside_25cm_sphere(self):
        r=route();r['waypoints']=[dict(id=str(i),name=str(i),xyz=[i*5.,0.,0.]) for i in range(5)]
        n=Navigator({'indoor':r});n.start(pose(x=1.,y=1.),10.)
        self.assertEqual(len(n.entry['path']),3)
        join=n.entry['path'][1]
        out=n.step(pose(t=10.1,x=join[0]+.5,y=.35),clear(10.1),10.1)
        self.assertEqual(n.entry['leg'],2);self.assertGreater(out['vx'],0.)

    def test_single_forward_pose_spike_does_not_latch_reference_arc(self):
        curve=RouteTrajectory([[0,0,0],[10,0,0]],2.)
        false=curve.reference([7,0,0,0],0)
        recovered=curve.reference([1,0,0,0],0,false['progress_m'])
        self.assertAlmostEqual(recovered['progress_m'],1.)
        self.assertLess(recovered['xyz'][0],1.4)

    def test_lateral_error_steers_heading_even_if_sideways_motion_is_weak(self):
        r=route();n=Navigator({'indoor':r});n.start(pose(x=.5),10.)
        out=n.step(pose(t=10.1,x=.6,y=.5),clear(10.1),10.1)
        self.assertLess(out['tracking']['heading_trajectory']['target_yaw'],0.)
        self.assertGreater(out['vx'],0.);self.assertLess(out['wz'],0.)

    def test_resume_with_weak_lateral_response_and_delayed_velocity_finishes_remaining_route(self):
        r=route()
        for point,xy in zip(r['waypoints'],((0,0),(5,0),(10,0),(10,5),(10,10))):point['xyz']=[*xy,0.]
        for heading in (0.,math.pi/2,math.pi):
            n=Navigator({'indoor':r});n.reached=[0];n.target=1
            x,y,yaw,t=6.,1.2,heading,10.;actual=[0.,0.,0.];n.start(pose(t,x,y,yaw),t)
            for _ in range(1600):
                n.keepalive(n.run_id,t);out=n.step(pose(t,x,y,yaw),clear(t),t)
                demand=[.23*out['vx'],.04*out['vy'],.6*out['wz']]
                actual=[a+(1-math.exp(-.05/.18))*(b-a) for a,b in zip(actual,demand)]
                x+=(math.cos(yaw)*actual[0]-math.sin(yaw)*actual[1])*.05
                y+=(math.sin(yaw)*actual[0]+math.cos(yaw)*actual[1])*.05
                yaw+=actual[2]*.05;t+=.05
                if out['state']=='COMPLETE':break
            self.assertEqual(out['state'],'COMPLETE',heading)
            self.assertEqual(n.reached,[0,2,3,4]);self.assertEqual(n.skipped,[1])

    def test_stop_during_resume_cancels_pending_navigation(self):
        b=FakeBridge();b.enabled=False;b.input_kind='native_axes'
        m=Mission({'indoor':route()},b);m.command({'action':'start'},pose(x=.5),10.)
        m.step(pose(x=.5),clear(10.),10.);m.command({'action':'pause'},None,10.1)
        b.enabled=True;b.input_kind='velocity';b.selected='basic_normal'
        out=m.step(pose(t=10.2,x=.7),clear(10.2),10.2)
        self.assertFalse(m.resume_pending);self.assertEqual(out['owner'],'paused');self.assertFalse(out['motion_authorized'])


if __name__=='__main__':unittest.main()
