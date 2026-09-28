"""Online map-point edits reuse the trajectory controller and saved route progress."""
import copy
import math
import unittest
from mission import Mission
from navigation import Navigator
from temporary_waypoints import TemporaryWaypoints
from test_unified import FakeBridge,route
from test_forward_entry import pose,clear


class QueueTests(unittest.TestCase):
    def test_order_update_remove_replace_and_retry(self):
        q=TemporaryWaypoints()
        for i in range(3):q.edit('append',dict(request_id=str(i),xyz=[i,1,0]))
        q.edit('update',dict(point_id='1',xyz=[4,2,0]))
        q.edit('remove',dict(point_id='0'))
        self.assertEqual([p['id'] for p in q.pending],['1','2'])
        self.assertEqual(q.pending[0]['xyz'],[4,2,0])
        q.passed(['1']);q.edit('append',dict(request_id='1',xyz=[0,0,0]))
        self.assertEqual([p['id'] for p in q.pending],['2'])
        q.edit('replace',dict(request_id='new',xyz=[8,9,0]))
        self.assertEqual([p['id'] for p in q.pending],['new'])
        with self.assertRaisesRegex(ValueError,'ALREADY_PASSED'):
            q.edit('update',dict(point_id='1',xyz=[1,1,0]))

    def test_malformed_coordinates_do_not_mutate_queue(self):
        q=TemporaryWaypoints();q.edit('append',dict(request_id='one',xyz=[1,2,3]));before=q.state()
        for xyz in ([float('nan'),0,0],[float('inf'),0,0],[True,0,0],[0,0],None):
            with self.assertRaises(ValueError):q.edit('replace',dict(request_id='bad',xyz=xyz))
            self.assertEqual(q.state(),before)

    def test_snapshot_does_not_allow_external_mutation(self):
        q=TemporaryWaypoints();q.edit('append',dict(request_id='one',xyz=[1,2,3]))
        state=q.state();state['pending'][0]['xyz'][0]=900
        self.assertEqual(q.pending[0]['xyz'][0],1)


class OnlineTests(unittest.TestCase):
    def setup_nav(self):
        b=FakeBridge();b.selected='basic_normal'
        m=Mission({'indoor':route()},b)
        m.command({'action':'shadow'},pose(t=10.,x=.5),10.)
        m.step(pose(t=10.,x=.5),clear(10.),10.)
        return m,b

    def edit(self,m,operation,t=10.05,xyz=None,point_id=None,request='one',solution=None):
        body=dict(action='temporary_'+operation,route='indoor',request_id=request)
        if xyz is not None:body['xyz']=xyz
        if point_id is not None:body['point_id']=point_id
        return m.command(body,solution or pose(t=t,x=.5),t)

    def test_edit_running_route_does_not_pause_restart_or_move_formal_progress(self):
        m,b=self.setup_nav();n=m.navigator
        original=copy.deepcopy(n.route);run=n.run_id;progress=(n.target,list(n.reached));stops=b.stops
        self.edit(m,'append',xyz=[.8,1.,0])
        self.edit(m,'append',xyz=[1.1,1.,0],request='two')
        self.assertEqual(m.owner,'auto');self.assertTrue(n.active);self.assertEqual(n.run_id,run)
        self.assertEqual((n.target,n.reached),progress);self.assertEqual(n.route,original);self.assertEqual(b.stops,stops)
        out=m.step(pose(t=10.1,x=.5),clear(10.1),10.1)
        self.assertNotIn(out['state'],['WAIT_POLICY_FEEDBACK','PAUSED','CONFIRM_WAYPOINT'])
        self.assertGreater(math.hypot(out['vx'],out['vy']),0)
        self.assertEqual(out['next_temporary']['id'],'one')
        self.assertEqual(n.entry['path'][1:3],[[.8,1.,0],[1.1,1.,0]])

    def test_temporary_points_pass_without_formal_progress_or_dwell(self):
        m,_=self.setup_nav();n=m.navigator;target=n.target
        self.edit(m,'append',xyz=[.6,.8,0]);self.edit(m,'append',xyz=[.9,.8,0],request='two')
        out=m.step(pose(t=10.2,x=.6,y=.8),clear(10.2),10.2)
        self.assertEqual([p['id'] for p in n.temporary.completed],['one'])
        self.assertEqual(n.target,target);self.assertNotEqual(out['state'],'CONFIRM_WAYPOINT')
        out=m.step(pose(t=10.4,x=.9,y=.8),clear(10.4),10.4)
        self.assertEqual(n.temporary.pending,[]);self.assertEqual(n.target,target)
        self.assertGreater(math.hypot(out['vx'],out['vy']),0)
        out=m.step(pose(t=10.6,x=1.,y=0),clear(10.6),10.6)
        self.assertEqual(n.target,2);self.assertIn(1,n.reached)

    def test_prediction_cannot_consume_temporary_point(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.7,.8,0])
        s=pose(t=10.2,x=.7,y=.8);s.update(mode='PREDICT_ONLY',arrival_valid=False,measurement_mono=10.)
        m.step(s,clear(10.2),10.2)
        self.assertEqual(len(m.navigator.temporary.pending),1);self.assertEqual(m.navigator.entry['leg'],1)

    def test_tracking_extrapolation_cannot_mark_an_unmeasured_point_passed(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.7,.8,0])
        s=pose(t=10.2,x=.7,y=.8);s['measurement_pose']=[.5,0.,0.,0.]
        m.step(s,clear(10.2),10.2)
        self.assertEqual(len(m.navigator.temporary.pending),1)
        self.assertEqual(m.navigator.entry['leg'],1)

    def test_upgrade_restore_retains_queue_without_starting_motion(self):
        m,b=self.setup_nav();self.edit(m,'append',xyz=[.7,.8,0])
        self.edit(m,'append',xyz=[1.2,.8,0],request='two')
        m.navigator.temporary.passed(['one']);n=m.navigator
        saved=dict(route_id=n.route_id,route=copy.deepcopy(n.route),target_index=n.target,
                   reached_indices=list(n.reached),skipped_indices=list(n.skipped),
                   override=m.override,manual_policy=m.manual_policy,
                   temporary_waypoints=n.temporary.state())
        restored=Mission({'indoor':route()},FakeBridge());restored.restore_paused(saved)
        self.assertEqual(restored.navigator.temporary.state(),saved['temporary_waypoints'])
        self.assertEqual(restored.owner,'paused');self.assertFalse(restored.navigator.active)
        self.assertEqual(restored.navigator.temporary.pending[0]['id'],'two')

    def test_manual_edits_keep_authority_and_resume_keeps_all_pending_points(self):
        m,b=self.setup_nav();m.command({'action':'manual_shadow'},pose(),10.1)
        owner=m.owner;stops=b.stops
        self.edit(m,'append',xyz=[.8,1.,0]);self.edit(m,'append',xyz=[1.2,1.,0],request='two')
        self.assertEqual(m.owner,owner);self.assertEqual(b.stops,stops)
        m.command({'action':'shadow'},pose(t=11.,x=2.,y=.2),11.)
        m.step(pose(t=11.,x=2.,y=.2),clear(11.),11.)
        self.assertEqual(m.navigator.entry['path'][:3],[[2.,.2,0],[.8,1.,0],[1.2,1.,0]])
        self.assertEqual(m.navigator.target,1)

    def test_update_remove_clear_never_restore_visited_temporary_points(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.7,.7,0]);self.edit(m,'append',xyz=[1.,.7,0],request='two')
        m.step(pose(t=10.2,x=.7,y=.7),clear(10.2),10.2)
        self.edit(m,'update',xyz=[1.5,.5,0],point_id='two')
        self.assertEqual(m.navigator.entry['path'][1],[1.5,.5,0])
        self.edit(m,'clear')
        self.assertEqual(m.navigator.entry['path'][-1],[1.,0,0]);self.assertEqual(m.navigator.temporary.pending,[])
        self.assertEqual(m.owner,'auto');self.assertTrue(m.navigator.active)

    def test_relocalization_rebuilds_from_new_pose_without_replaying_visited_points(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.7,.8,0]);self.edit(m,'append',xyz=[1.4,.8,0],request='two')
        m.step(pose(t=10.2,x=.7,y=.8),clear(10.2),10.2)
        s=pose(t=10.4,x=1.,y=.5,epoch=2)
        m.step(s,clear(10.4),10.4)
        self.assertEqual(m.navigator.entry['path'][0],[1.,.5,0])
        self.assertEqual(m.navigator.entry['temporary_ids'],['two'])

    def test_relocalization_restarts_temporary_heading_at_current_yaw(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.5,2.,0])
        n=m.navigator;run=n.run_id;target=n.target
        n.heading_reference.yaw=0.;n.heading_reference.rate=0.;n.heading_reference.stamp=10.05
        s=pose(t=10.1,x=.5,yaw=math.pi/2,epoch=2)
        out=m.step(s,clear(10.1),10.1)
        heading=out['tracking']['heading_trajectory']
        self.assertTrue(out['reconnected_after_localization'])
        self.assertAlmostEqual(heading['yaw'],math.pi/2,delta=.01)
        self.assertLess(abs(out['wz']),.1)
        self.assertGreater(out['vx'],0.)
        self.assertEqual(n.run_id,run);self.assertEqual(n.target,target)
        self.assertEqual([p['id'] for p in n.temporary.pending],['one'])
        self.assertTrue(n.active)

    def test_cancelling_last_temporary_point_joins_forward_and_preserves_route(self):
        for operation in ('clear','remove'):
            with self.subTest(operation=operation):
                m,b=self.setup_nav();n=m.navigator;original=copy.deepcopy(n.route)
                self.edit(m,'append',xyz=[3.,1.,0])
                run=n.run_id;stops=b.stops;reached=list(n.reached)
                self.edit(m,operation,point_id='one',solution=pose(t=10.3,x=2.4,y=.6))
                self.assertEqual(n.target,3);self.assertEqual(n.skipped,[0,1,2])
                self.assertEqual(n.reached,reached);self.assertEqual(n.route,original)
                self.assertEqual(n.entry['path'][0],[2.4,.6,0.])
                self.assertTrue(all(p[0]>=2.4 for p in n.entry['path']))
                self.assertEqual(n.run_id,run);self.assertEqual(b.stops,stops)
                out=m.step(pose(t=10.35,x=2.4,y=.6),clear(10.35),10.35)
                self.assertEqual(out['state'],'JOINING_ROUTE');self.assertGreater(out['vx'],0.)
                self.assertTrue(n.active);self.assertEqual(m.owner,'auto')

    def test_cancelling_detour_respects_explicit_manual_target(self):
        m,_=self.setup_nav();n=m.navigator
        m.command(dict(action='set_progress',route='indoor',waypoint_id='1',target_index=1),None,10.1)
        m.command({'action':'shadow'},pose(t=10.2,x=.5),10.2)
        self.edit(m,'append',xyz=[3.,1.,0])
        self.edit(m,'clear',solution=pose(t=10.3,x=2.4,y=.6))
        self.assertEqual(n.target,1);self.assertEqual(n.manual_target,1)
        self.assertEqual(n.entry['path'][-1],[1.,0.,0.])
        self.assertEqual(n.reached,[0]);self.assertEqual(n.skipped,[])

    def test_manual_clear_rejoins_ahead_without_taking_control(self):
        m,b=self.setup_nav();n=m.navigator
        self.edit(m,'append',xyz=[3.,1.,0])
        m.command({'action':'manual_shadow'},pose(t=10.1,x=.5),10.1)
        stops=b.stops
        self.edit(m,'clear',solution=pose(t=10.3,x=2.4,y=.6))
        self.assertEqual(m.owner,'manual');self.assertFalse(n.active)
        self.assertEqual(b.stops,stops);self.assertEqual(n.target,3)
        m.command({'action':'shadow'},pose(t=10.4,x=2.5,y=.6),10.4)
        self.assertEqual(n.target,3);self.assertEqual(n.entry['path'][0],[2.5,.6,0.])

    def test_clear_without_pose_rejoins_forward_when_measurement_returns(self):
        m,_=self.setup_nav();n=m.navigator;self.edit(m,'append',xyz=[3.,1.,0])
        m.command(dict(action='temporary_clear',route='indoor',request_id='clear-offline'),None,10.2)
        self.assertTrue(n.temporary.dirty)
        out=m.step(pose(t=10.3,x=2.4,y=.6),clear(10.3),10.3)
        self.assertEqual(n.target,3);self.assertFalse(n.temporary.dirty)
        self.assertGreater(out['vx'],0.)

    def test_policies_override_and_original_route_are_preserved(self):
        m,_=self.setup_nav();m.override='stairs_normal'
        presets=copy.deepcopy(m.plans.presets);source=copy.deepcopy(m.plans.sources)
        self.edit(m,'append',xyz=[.8,1.,0]);out=m.step(pose(t=10.2,x=.5),clear(10.2),10.2)
        self.assertEqual(out['policy'],'stairs_normal');self.assertEqual(m.plans.presets,presets)
        self.assertEqual(m.plans.sources,source)
        m.command(dict(action='set_progress',route='indoor',waypoint_id='3',target_index=3),None,10.3)
        self.assertEqual(m.navigator.temporary.pending,[]);self.assertEqual(m.navigator.reached,[0,1,2])

    def test_long_detour_rejoins_future_route_without_returning_to_bypassed_point(self):
        m,_=self.setup_nav();n=m.navigator;self.edit(m,'append',xyz=[2.4,.7,0])
        self.assertEqual(n.target,1)  # Preview cannot commit progress.
        self.assertEqual(n.entry['temporary_return_index'],3)
        m.step(pose(t=10.3,x=2.4,y=.7),clear(10.3),10.3)
        self.assertEqual(n.target,3);self.assertEqual(n.skipped,[0,1,2])
        self.assertEqual(n.route['waypoints'][1]['xyz'],[1.,0.,0.])

    def test_manual_visit_is_not_replayed_after_resume(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.8,.8,0]);self.edit(m,'append',xyz=[1.4,.8,0],request='two')
        m.command({'action':'manual_shadow'},pose(),10.1)
        m.step(pose(t=10.2,x=.8,y=.8),clear(10.2),10.2)
        self.assertEqual([p['id'] for p in m.navigator.temporary.pending],['two'])
        m.command({'action':'shadow'},pose(t=10.4,x=1.,y=.8),10.4)
        self.assertEqual(m.navigator.entry['temporary_ids'],['two'])

    def test_manual_prediction_does_not_consume_queue(self):
        m,_=self.setup_nav();self.edit(m,'append',xyz=[.8,.8,0]);m.command({'action':'manual_shadow'},pose(),10.1)
        predicted=pose(t=10.2,x=.8,y=.8);predicted['arrival_valid']=False
        m.step(predicted,clear(10.2),10.2)
        self.assertEqual(len(m.navigator.temporary.pending),1)

    def test_wrong_route_is_rejected_without_mutating_current_queue(self):
        m,_=self.setup_nav()
        with self.assertRaisesRegex(ValueError,'ROUTE_CHANGED'):
            m.command(dict(action='temporary_append',route='full',request_id='wrong',xyz=[1,1,0]),pose(),10.)
        self.assertEqual(m.navigator.temporary.pending,[])

    def test_points_can_be_queued_without_localization_while_paused(self):
        m=Mission({'indoor':route()},FakeBridge())
        m.command(dict(action='temporary_append',route='indoor',request_id='one',xyz=[1,1,0]),None,10.)
        self.assertEqual(m.owner,'paused');self.assertFalse(m.navigator.active)
        self.assertEqual(m.navigator.temporary.pending[0]['xyz'],[1,1,0])

    def test_completed_route_can_execute_only_new_temporary_destination(self):
        m,b=self.setup_nav();n=m.navigator;n.target=4;n.reached=list(range(5));m.pause('COMPLETE')
        self.edit(m,'append',xyz=[5.,1.,0])
        m.command({'action':'shadow'},pose(t=11.,x=4.),11.)
        self.assertTrue(n.temporary.terminal_only);self.assertEqual(n.entry['path'],[[4.,0,0],[5.,1.,0]])
        for t in (11.1,11.25,11.4):m.step(pose(t=t,x=5.,y=1.),clear(t),t)
        self.assertEqual(m.owner,'paused');self.assertEqual(n.reason,'COMPLETE')
        self.assertEqual(n.reached,list(range(5)));self.assertEqual(n.temporary.pending,[])

    def test_closed_loop_detour_returns_forward_and_finishes_formal_route(self):
        r=route()
        for point in r['waypoints']:point['xyz'][0]*=2
        m=Mission({'indoor':r},FakeBridge());x,y,yaw=.3,0.,0.;t=10.
        m.command({'action':'shadow'},pose(t=t,x=x,y=y,yaw=yaw),t)
        added=False;nonterminal_zero=0
        for _ in range(1200):
            if not added and t>=10.5:
                for identity,point in [('one',[1.2,.8,0]),('two',[2.8,.8,0])]:
                    self.edit(m,'append',t=t,xyz=point,request=identity,solution=pose(t=t,x=x,y=y,yaw=yaw))
                added=True
            m.keepalive(m.navigator.run_id,t)
            out=m.step(pose(t=t,x=x,y=y,yaw=yaw),clear(t),t)
            if out['state']=='COMPLETE':break
            if added and m.navigator.temporary.pending and math.hypot(out['vx'],out['vy'])<1e-8:nonterminal_zero+=1
            c,s=math.cos(yaw),math.sin(yaw)
            x+=.05*.75*(c*out['vx']-s*out['vy']);y+=.05*.75*(s*out['vx']+c*out['vy']);yaw+=.05*.7*out['wz'];t+=.05
        self.assertEqual(out['state'],'COMPLETE')
        self.assertEqual([p['id'] for p in m.navigator.temporary.completed],['one','two'])
        self.assertEqual(nonterminal_zero,0)
        self.assertLess(math.hypot(x-8,y),.25)


class ApiTests(unittest.TestCase):
    def test_actual_navigation_dispatch_is_idempotent_and_does_not_start_motion(self):
        import ast,json,logging,threading,uuid
        from pathlib import Path
        from types import SimpleNamespace
        from protocol import Tickets
        tree=ast.parse((Path(__file__).parent/'runtime.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MapService')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='request')
        scope=dict(uuid=uuid,json=json,logging=logging,time=SimpleNamespace(monotonic=lambda:10.))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'runtime.py','exec'),scope)
        m=Mission({'indoor':route()},FakeBridge())
        service=SimpleNamespace(lock=threading.RLock(),mission=m,navigator=m.navigator,bridge=m.bridge,
            navigation_requests={},navigation_tickets=Tickets(.8),map_id='map',state={'solution':pose()},
            trace=SimpleNamespace(emit=lambda *a,**k:None))
        request=dict(request_id=str(uuid.uuid4()),map_id='map',ticket=service.navigation_tickets.issue(10.),
                     action='temporary_append',route='indoor',xyz=[1,1,0])
        first=scope['request'](service,'/navigation',request)
        again=scope['request'](service,'/navigation',request)
        self.assertEqual(first,again);self.assertEqual(len(m.navigator.temporary.pending),1)
        self.assertEqual(m.owner,'paused');self.assertFalse(m.execution);self.assertEqual(m.bridge.sent,[])


if __name__=='__main__':unittest.main()
