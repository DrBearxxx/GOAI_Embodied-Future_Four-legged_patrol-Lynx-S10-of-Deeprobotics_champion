import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from mission import Mission
from planning import Plans
sys.path.insert(0,str(Path(__file__).parent/'backend'))
from unified_control import Controller,Simulator,validate_velocity
from wire import decode
from zero_guardian import zero_packet


def route():
    return dict(waypoints=[dict(id=str(i),name='P'+str(i),xyz=[float(i),0.,0.]) for i in range(5)],
                edges=[dict(warnings=[],speed_limit_mps=.2,corridor_half_width_m=.5) for _ in range(4)])

def solution(t=10.,x=.5,z=0.,epoch=1):
    return dict(pose=[x,0.,z,0.],generation=[epoch,0],estimate_mono=t,measurement_mono=t,
        arrival_valid=True,mode='TRACKING',max_vx=.2,max_wz=.2)


class FakeBridge:
    client=True
    def __init__(self):self.selected='basic';self.enabled=True;self.sent=[];self.actions=[];self.stops=0;self.input_kind='velocity';self.last_action=None
    def stop(self):self.enabled=False;self.stops+=1
    def ready(self):return self.enabled
    def policy_ready(self,p,input_kind=None):return self.selected==p and (input_kind is None or input_kind==self.input_kind)
    def action(self,p,r,t,input_kind=None):self.actions.append(p);self.last_action=dict(kind=p,request_id=r,state='QUEUED')
    def send(self,p,t):self.sent.append(dict(p))
    def state(self):return dict(backend='simulation',confirmed=self.selected,requested=self.selected,enabled=self.enabled,ready=self.ready(),last_action=self.last_action,input_kind=self.input_kind)


class PlanningTests(unittest.TestCase):
    def setUp(self):self.p=Plans({'indoor':route()})
    def test_only_official_presets(self):
        with self.assertRaisesRegex(ValueError,'OFFICIAL'):self.p.set_policy('indoor',[0],'low')
        self.p.set_policy('indoor',[0,2],'stairs_normal')
        self.assertEqual(self.p.presets['indoor'],['stairs_normal','basic_normal','stairs_normal','basic_normal'])
    def test_persistent_atomic_revision(self):
        with tempfile.TemporaryDirectory() as d:
            p=Plans({'indoor':route()},Path(d)/'presets.json');p.set_policy('indoor',[1],'platform')
            q=Plans({'indoor':route()},Path(d)/'presets.json');self.assertEqual(q.presets,p.presets)
            changed=route();changed['waypoints'][0]['xyz'][2]=2
            with self.assertRaises(ValueError):Plans({'indoor':changed},Path(d)/'presets.json')
    def test_replan_preserves_sources_and_order(self):
        before=copy.deepcopy(self.p.sources)
        p=self.p.propose('indoor',4,[],solution(),10.)
        self.assertEqual(p['source_edges'],[0,1,2,3]);self.assertEqual(p['skipped_point_indices'],[0])
        r=self.p.apply(p['id'],solution(),10.)
        self.assertEqual([w['id'] for w in r['waypoints']],['reentry','1','2','3','4'])
        self.assertEqual(self.p.sources,before)
    def test_blocked_edge_no_invented_detour(self):
        with self.assertRaisesRegex(ValueError,'NO_ROUTE'):self.p.propose('indoor',4,[2],solution(),10.)
    def test_height_no_wrong_floor_association(self):
        with self.assertRaisesRegex(ValueError,'NO_NEARBY'):self.p.propose('indoor',4,[],solution(z=3),10.)
    def test_preview_relocalization_invalidates(self):
        p=self.p.propose('indoor',4,[],solution(),10.)
        with self.assertRaisesRegex(ValueError,'EPOCH'):self.p.apply(p['id'],solution(epoch=2),10.)
    def test_preview_motion_invalidates(self):
        p=self.p.propose('indoor',4,[],solution(),10.)
        with self.assertRaisesRegex(ValueError,'MOVED'):self.p.apply(p['id'],solution(x=1),10.)
    def test_warnings_not_removed_by_preset(self):
        self.p.sources['indoor']['edges'][1]['warnings']=['stairs_unverified']
        self.p.set_policy('indoor',[1],'stairs_normal')
        p=self.p.propose('indoor',4,[],solution(),10.)
        self.assertEqual(p['warnings'],['stairs_unverified'])


class ArbitrationTests(unittest.TestCase):
    def setUp(self):
        self.b=FakeBridge();self.m=Mission({'indoor':route()},self.b)
        self.perception={k:dict(valid=True,blocked=False,mono=10.) for k in ('front','rear')}
    def rc(self,seq=1,axes=None,t=10.):
        self.m.operator(dict(session='test',sample_seq=seq,sample_age_ms=0,fresh=True,axes=axes or [0.,0.,0.]),t)
    def test_manual_enable_before_first_rc_sample_is_accepted(self):
        self.m.command({'action':'manual_shadow'},None,10.)
        out=self.m.step(None,{},10.)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(out['state'],'MANUAL_WAIT_INPUT')
        self.assertEqual(out['axes'],[0,0,0])
        self.rc(axes=[.6,0,.2],t=11.)
        out=self.m.step(None,{},11.)
        self.assertEqual(out['state'],'MANUAL');self.assertEqual(out['axes'],[.6,0,.2])
    def test_manual_enable_does_not_require_centering(self):
        self.rc(axes=[1,0,0]);self.m.command({'action':'manual_shadow'},None,10.)
        self.assertEqual(self.m.step(None,{},10.)['axes'],[1,0,0])
    def test_stick_deflection_takes_navigation_authority(self):
        self.m.command({'action':'shadow'},solution(x=0),10.)
        self.m.override='stairs';self.rc(axes=[.5,0,0])
        self.assertEqual(self.m.owner,'manual');self.assertFalse(self.m.navigator.active)
        self.assertEqual(self.m.manual_policy,'stairs')
    def test_manual_shadow_preserves_full_normalized_axes(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        self.rc(2,[.5,1,-1]);o=self.m.step(None,self.perception,10.)
        self.assertEqual(o['axes'],[.5,1,-1]);self.assertEqual(o['input_kind'],'native_axes')
        self.assertIsNone(o['speed_limit_mps']);self.assertEqual((o['vx'],o['vy'],o['wz']),(0,0,0))
        self.assertFalse(o['execution_requested'])
    def test_manual_selection_survives_stop_and_resume(self):
        self.rc();self.m.command({'action':'override','policy':'basic'},None,10.)
        self.m.command({'action':'manual_shadow'},None,10.)
        self.m.command({'action':'override','policy':'stairs'},None,10.)
        self.m.pause();self.m.command({'action':'manual_shadow'},None,10.)
        self.assertEqual(self.m.manual_policy,'stairs')
    def test_wake_keeps_mission_paused(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        self.m.command({'action':'wake'},None,10.)
        self.assertEqual(self.m.owner,'paused');self.assertEqual(self.b.actions[-1],'wake')
    def test_manual_input_gap_zeros_output_without_canceling_enable(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        out=self.m.step(None,self.perception,10.4)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(out['state'],'MANUAL_WAIT_INPUT')
        self.assertEqual(out['axes'],[0,0,0])
        self.rc(2,[.8,0,0],t=10.5);out=self.m.step(None,{},10.5)
        self.assertEqual(out['axes'],[.8,0,0]);self.assertEqual(out['state'],'MANUAL')
    def test_same_rc_sample_does_not_stop_or_renew_freshness(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        self.rc(1,t=10.1)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(self.m.rc_at,10.)
        out=self.m.step(None,self.perception,10.4)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(out['state'],'MANUAL_WAIT_INPUT')
    def test_manual_does_not_depend_on_navigation_perception(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        self.rc(2,[-1,0,0]);o=self.m.step(None,{'front':self.perception['front']},10.)
        self.assertEqual(o['axes'],[-1,0,0]);self.assertEqual(o['state'],'MANUAL')
    def test_manual_remains_active_with_rc_without_map_keepalive(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        for seq in range(2,42):
            now=10.+seq*.1;self.rc(seq,t=now)
            self.m.step(None,{},now+.05)
            self.assertEqual(self.m.owner,'manual')
    def test_manual_start_selects_policy_atomically(self):
        self.rc();self.m.override='stairs';self.b.input_kind='native_axes'
        self.m.command({'action':'manual','policy':'basic'},None,10.)
        self.assertEqual(self.m.manual_policy,'basic');self.assertEqual(self.m.override,'stairs')
        self.m.step(None,{},10.)
        self.assertEqual(self.b.actions,['run'])
    def test_manual_start_rejects_unknown_policy_before_taking_control(self):
        self.rc()
        with self.assertRaisesRegex(ValueError,'UNKNOWN_POLICY'):
            self.m.command({'action':'manual','policy':'bad'},None,10.)
        self.assertEqual(self.m.owner,'paused')
    def test_stale_axes_are_never_replayed_when_waiting_for_input(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        self.rc(2,[1,1,1],t=10.)
        self.assertEqual(self.m.step(None,{},10.30)['axes'],[1,1,1])
        out=self.m.step(None,{},10.36)
        self.assertEqual(out['axes'],[0,0,0]);self.assertEqual(out['state'],'MANUAL_WAIT_INPUT')
    def test_invalid_rc_freshness_does_not_cancel_pending_manual_intent(self):
        self.m.command({'action':'manual_shadow'},None,10.)
        self.m.operator(dict(session='test',fresh=False,sample_seq=0,sample_age_ms=500,axes=[1,1,1]),10.)
        out=self.m.step(None,{},10.)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(out['axes'],[0,0,0])
    def test_explicit_manual_binds_new_app_session_before_first_sample(self):
        self.rc();self.m.command({'action':'manual_shadow','operator_session':'new-app'},None,11.)
        self.m.operator(dict(session='new-app',fresh=True,sample_seq=1,sample_age_ms=0,axes=[.5,0,0]),11.)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(self.m.step(None,{},11.)['axes'],[.5,0,0])
    def test_stop_is_not_undone_by_input_returning(self):
        self.m.command({'action':'manual_shadow'},None,10.);self.m.pause('OPERATOR_STOP')
        self.rc(t=11.)
        self.assertEqual(self.m.owner,'paused');self.assertFalse(self.m.step(None,{},11.)['active'])
    def test_stand_in_manual_page_automatically_runs_after_posture_receipt(self):
        self.b.input_kind='native_axes'
        self.m.command(dict(action='stand',manual_after_stand=True,policy='basic',request_id='stand-1'),None,10.)
        self.m.step(None,{},10.)
        self.assertEqual(self.b.actions,['stand']);self.assertEqual(self.m.owner,'manual')
        self.b.last_action['state']='RECEIVED';self.m.step(None,{},10.1)
        self.assertEqual(self.b.actions,['stand','run'])
        self.b.enabled=True;out=self.m.step(None,{},10.2)
        self.assertEqual(out['state'],'MANUAL_WAIT_INPUT');self.assertEqual(self.m.owner,'manual')
    def test_manual_selection_changes_policy_without_canceling_control(self):
        self.m.command(dict(action='manual',policy='basic'),None,10.);stops=self.b.stops
        self.m.command(dict(action='manual',policy='stairs'),None,10.1)
        self.assertEqual(self.m.manual_policy,'stairs');self.assertEqual(self.b.stops,stops)
    def test_manual_waiting_for_stand_does_not_have_navigation_timeout(self):
        self.m.command(dict(action='manual',policy='stairs'),None,10.)
        self.m.step(None,{},10.);self.m.step(None,{},40.)
        self.assertEqual(self.m.owner,'manual')
    def test_stop_cancels_automatic_control_after_stand(self):
        self.m.command(dict(action='stand',manual_after_stand=True,policy='basic',request_id='stand-2'),None,10.)
        self.m.pause('OPERATOR_STOP');self.b.last_action['state']='RECEIVED'
        self.m.step(None,{},10.1)
        self.assertEqual(self.m.owner,'paused');self.assertNotIn('run',self.b.actions)
    def test_manual_takeover_does_not_use_previous_navigation_authorization(self):
        self.m.owner='auto';self.m.execution=True;self.m.last={'motion_authorized':True}
        self.m.command(dict(action='manual',policy='basic'),None,10.)
        self.m.step(None,{},10.)
        self.assertEqual(self.m.owner,'manual');self.assertNotEqual(self.m.reason,'CONTROL_LINK_LOST')
    def test_relocalization_disarms_manual_and_auto(self):
        self.rc();self.m.command({'action':'manual_shadow'},None,10.)
        self.m.step(None,self.perception,10.,pending=True);self.assertEqual(self.m.owner,'paused')
    def test_low_high_never_enters_route_presets(self):
        self.m.command({'action':'override','policy':'high'},None,10.)
        self.assertEqual(self.m.manual_policy,'high');self.assertEqual(self.m.owner,'paused')
        self.assertIsNone(self.m.override);self.assertEqual(self.b.actions,['select:high'])
    def test_replan_apply_requires_explicit_resume(self):
        reply=self.m.command({'action':'replan','goal':4},solution(),10.)
        self.m.command({'action':'apply_plan','plan_id':reply['preview']['id']},solution(),10.)
        self.assertEqual(self.m.owner,'paused');self.assertFalse(self.m.navigator.active)
    def test_gateway_loss_latches(self):
        self.m.owner='auto';self.m.execution=True;self.m.last={'motion_authorized':True};self.b.enabled=False
        self.m.step(solution(),self.perception,10.)
        self.assertEqual(self.m.owner,'paused');self.assertIn('CONTROL_LINK_LOST',self.m.reason)
    def test_policy_waits_feedback_and_one_run(self):
        self.m.execution=True;self.b.enabled=False
        self.assertFalse(self.m._ensure_policy('stairs',10.))
        self.assertEqual(self.b.actions,['select:stairs'])
        self.m._ensure_policy('stairs',10.1);self.assertEqual(len(self.b.actions),1)
        self.b.selected='stairs';self.m._ensure_policy('stairs',10.2)
        self.assertEqual(self.b.actions[-1],'run')
        self.m._ensure_policy('stairs',10.3);self.assertEqual(self.b.actions.count('run'),1)

    def test_official_live_switch_queues_without_disarming_mission(self):
        self.m.owner='manual';self.m.execution=True;self.m.native_input_kind='native_axes';self.b.input_kind='native_axes'
        self.rc(axes=[.8,-.3,.5]);self.m.step(None,{},10.)
        for i,policy in enumerate(('stairs','platform','basic')):
            self.m.command(dict(action='override',policy=policy),None,10.+i*.05)
            out=self.m.step(None,{},10.+i*.05)
            self.assertEqual(out['axes'],[.8,-.3,.5]);self.assertEqual(out['state'],'MANUAL')
            self.assertTrue(out['motion_authorized']);self.assertIsNone(out['transition'])
        self.assertEqual(self.b.actions,['select:stairs','select:platform','select:basic']);self.assertEqual(self.b.stops,0)

    def test_navigation_live_switch_leaves_output_gating_to_gateway(self):
        self.m.owner='auto';self.m.execution=True
        self.b.state=lambda:dict(requested='stairs',confirmed='none',enabled=True,input_kind='velocity')
        self.assertTrue(self.m._ensure_policy('stairs',10.))
        self.assertTrue(self.m._ensure_policy('platform',10.1))
        self.assertEqual(self.b.actions,['select:platform']);self.assertIsNone(self.m.transition)


class GatewayTests(unittest.TestCase):
    def setUp(self):self.a=Simulator();self.c=Controller(self.a);self.c.new_session('test');self.seq=0
    def send(self,action='',axes=None,kind='velocity',t=10.):
        self.seq+=1;self.c.accept(dict(action=action,axes=axes or [0.,0.,0.],input_kind=kind,
            sample_seq=self.seq,sample_age_ms=0,fresh=True),t)
    def test_velocity_units_and_watchdog(self):
        self.send('select:basic');self.send('stand');self.send('run');self.send(axes=[.15,.02,-.1]);self.c.tick(10.)
        self.assertEqual(self.a.output,(.15,.02,-.1))
        self.c.tick(10.4);self.assertEqual(self.a.output,(0.,0.,0.));self.assertFalse(self.c.enabled)
    def test_wrong_units_cannot_drive_waypoint(self):
        self.send('select:low',kind='waypoint_axes');self.send('stand',kind='waypoint_axes');self.send('run',kind='waypoint_axes')
        self.send(axes=[.1,0,0]);self.assertFalse(self.c.enabled)
    def test_new_session_latches(self):
        self.send('select:basic');self.send('stand');self.send('run');self.c.new_session('new');self.assertFalse(self.c.enabled)
    def test_guardian_uses_real_velocity_zero(self):
        d=decode(zero_packet(1));self.assertEqual(d['command'],0x110002);self.assertFalse(any(d['items'].values()))
    def test_nonfinite_or_malformed_velocity(self):
        for v in ([math.nan,0,0],[0,0,math.inf],[True,0,0],[0,0],['1',0,0],None):
            with self.assertRaises(ValueError):validate_velocity(v)
    def test_native_policy_velocity_range_is_not_clipped_by_gateway(self):
        for policy in ('basic','stairs','platform'):
            self.send('select:'+policy);self.send('stand');self.send('run')
            self.send(axes=[1.2,-.65,1.5]);self.c.tick(10.)
            self.assertEqual(self.a.output,(1.2,-.65,1.5))
    def test_hardware_contract_uses_mode_one(self):
        self.send('select:basic');self.send('stand');self.send('run')
        self.assertEqual(self.c.status(10.)['input_kind'],'velocity')
        self.send('select:basic',kind='native_axes');self.send('run',kind='native_axes')
        self.send(axes=[1.,-1.,.8],kind='native_axes');self.c.tick(10.)
        self.assertEqual(self.a.output,(1.,-1.,.8))
        self.assertEqual(self.c.status(10.)['output_units'],'normalized')


if __name__=='__main__':unittest.main()
