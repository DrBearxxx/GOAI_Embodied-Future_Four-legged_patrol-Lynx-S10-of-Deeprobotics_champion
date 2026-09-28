"""End-to-end teaching/persistence tests, with no robot transport."""
import copy,json,math,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from mission import Mission
from test_unified import FakeBridge,route,solution
from route_recording import RouteRecorder
from planning import forward_entry


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'presets.json';self.base={'indoor':route()}
        self.m=Mission(self.base,FakeBridge(),self.path);self.addCleanup(lambda:self.m.recorder.close())
        self.m.owner='manual';self.m.manual_policy='basic_normal';self.now=10.

    def sample(self,x,y=0,z=0,policy=None,mode='TRACKING',generation=None):
        self.now+=.11
        if policy:self.m.manual_policy=policy
        s=solution(t=self.now,x=x,z=z);s['pose'][1]=y;s['mode']=mode
        if generation is not None:s['generation']=generation
        return s

    def action(self,action,s=None,**fields):
        return self.m.command(dict(action='recording_'+action,**fields),s,self.now)

    def tick(self,x,y=0,policy=None,**kw):
        s=self.sample(x,y,policy=policy,**kw)
        self.m.recorder.tick(s,self.now,self.m.manual_policy,self.m.bridge.state());return s

    def start(self,policy='basic_normal'):
        self.m.manual_policy=policy;self.action('start',self.sample(0),name='新楼梯路线')

    def save(self,x=6,y=0):return self.action('save',self.sample(x,y))['saved_route']

    def test_recording_and_marking_never_claim_control_or_modify_active_route(self):
        before=copy.deepcopy(self.m.navigator.route);self.start();self.action('mark',self.tick(2),name='入口')
        self.assertEqual(self.m.owner,'manual');self.assertEqual(self.m.bridge.stops,0);self.assertEqual(self.m.bridge.actions,[])
        self.assertEqual(before,self.m.navigator.route);self.assertEqual(self.m.recorder.marks[-1]['name'],'入口')

    def test_curved_trace_manual_marks_and_switch_positions_become_replayable_route(self):
        self.start()
        for x in range(1,6):self.tick(x*.5,math.sin(x*.3))
        self.action('mark',self.tick(3,1),name='楼梯入口')
        self.tick(3,1,policy='stairs_normal')
        self.tick(4,1);self.tick(5,.5)
        rid=self.save();r=self.m.plans.route(rid)
        self.assertIn('楼梯入口',[p['name'] for p in r['waypoints']]);self.assertEqual(len(self.m.recorder.marks),2)
        self.assertEqual(self.m.plans.presets[rid],['basic_normal','stairs_normal'])
        self.assertGreater(len(r['edges'][0]['control_points']),1)
        self.assertEqual(r['recording']['policy_switches'][0]['xyz'],[3,1,0.])
        self.assertEqual(self.m.navigator.route_id,'indoor');self.assertEqual(self.m.owner,'manual')
        self.m.command(dict(action='select',route=rid),None,self.now)
        self.m.command(dict(action='shadow'),self.sample(0),self.now)
        trajectory,_=self.m.navigator.path();self.assertGreater(trajectory.total,6.)

    def test_start_finish_and_restart_preserve_originals_and_custom_presets(self):
        self.m.plans.set_policy('indoor',[1],'platform');self.start();self.tick(3);rid=self.save()
        restarted=Mission(self.base,FakeBridge(),self.path);self.addCleanup(restarted.recorder.close)
        self.assertEqual(restarted.plans.sources['indoor'],self.base['indoor'])
        self.assertEqual(restarted.plans.presets['indoor'][1],'platform')
        self.assertIn(rid,restarted.navigator.routes);self.assertEqual(restarted.recorder.status,'saved')
        self.assertEqual(restarted.plans.route(rid),self.m.plans.route(rid))

    def test_each_new_session_has_distinct_route_and_keeps_prior_sessions(self):
        self.start();self.tick(3);first=self.save();self.start();self.tick(4);second=self.save(8)
        self.assertNotEqual(first,second);self.assertEqual(set(self.m.plans.sources),{'indoor',first,second})
        self.assertEqual(len(list((self.path.parent/'route-recordings').glob('*.jsonl'))),2)

    def test_service_restart_recovers_draft_paused_and_can_continue(self):
        self.start();self.tick(1);self.action('mark',self.tick(2),name='保留点');self.m.recorder.close()
        recovered=RouteRecorder(self.path.parent/'route-recordings');self.addCleanup(recovered.close)
        self.assertEqual(recovered.status,'paused');self.assertEqual(recovered.marks[-1]['name'],'保留点')
        self.assertEqual(len(recovered.samples),len(self.m.recorder.samples))
        self.m.recorder=recovered;self.action('resume',self.sample(2));self.tick(3);rid=self.save()
        self.assertIn(rid,self.m.plans.catalog())

    def test_restart_truncates_only_incomplete_last_journal_record(self):
        self.start();self.tick(2);r=self.m.recorder;r.close()
        journal=self.path.parent/'route-recordings'/(r.session['id']+'.jsonl')
        with journal.open('ab') as f:f.write(b'{"bad":"\xe4')
        recovered=RouteRecorder(journal.parent);self.addCleanup(recovered.close)
        self.assertEqual(len(recovered.samples),2);self.m.recorder=recovered
        self.action('resume',self.sample(3));recovered.close()
        for line in journal.read_text(encoding='utf-8').splitlines():json.loads(line)

    def test_pause_recording_does_not_pause_robot_and_resume_keeps_samples(self):
        self.start();self.tick(2);n=len(self.m.recorder.samples);self.action('pause');self.tick(3)
        self.assertEqual(len(self.m.recorder.samples),n);self.assertEqual(self.m.owner,'manual');self.assertEqual(self.m.bridge.stops,0)
        self.action('resume',self.sample(4));self.assertGreater(len(self.m.recorder.samples),n)

    def test_all_manual_policies_kept_exactly_and_feedback_is_separate(self):
        self.start(policy='basic');self.m.bridge.selected='platform';self.tick(2,policy='high_v2')
        sample=self.m.recorder.samples[-1]
        self.assertEqual(sample['policy'],'high_v2');self.assertEqual(sample['actual_policy'],'platform')
        rid=self.save();self.m.plans.set_policy('indoor',[0],'step_move')
        restart=Mission(self.base,FakeBridge(),self.path);self.addCleanup(restart.recorder.close)
        self.assertEqual(restart.plans.presets[rid],['basic','high_v2'])
        restart.command(dict(action='select',route=rid),None,self.now)
        with self.assertRaisesRegex(ValueError,'只接入人工摇杆'):restart.command(dict(action='shadow'),self.sample(0),self.now)
        restart.plans.set_policy(rid,[1],'step_move')
        restart.command(dict(action='shadow'),self.sample(0),self.now)
        self.assertEqual(restart.owner,'auto')

    def test_lost_localization_never_fabricates_points_degraded_measurement_allowed(self):
        self.start();self.tick(2);n=len(self.m.recorder.samples)
        with self.assertRaisesRegex(ValueError,'实测定位'):self.action('mark',self.sample(8,mode='LOST'))
        self.assertEqual(len(self.m.recorder.samples),n);self.assertEqual(self.m.recorder.status,'recording')
        s=self.sample(20,mode='PREDICT_ONLY');s['measurement_pose']=[3,.2,0.,0.]
        self.action('mark',s);self.assertEqual(self.m.recorder.marks[-1]['xyz'],[3,.2,0.])

    def test_generation_changes_are_logged_without_disabling_control(self):
        self.start();self.tick(2,generation=['new',1])
        self.assertEqual(self.m.recorder.gaps,1);self.assertEqual(self.m.recorder.status,'recording');self.assertEqual(self.m.owner,'manual')

    def test_repeated_stationary_switches_do_not_create_zero_length_legs(self):
        self.start();self.tick(0,policy='platform');self.tick(0,policy='stairs_normal');self.tick(3);rid=self.save()
        r=self.m.plans.route(rid);self.assertEqual(len(r['edges']),1);self.assertEqual(self.m.plans.presets[rid],['stairs_normal'])
        self.assertEqual(len(r['recording']['policy_switches']),2)

    def test_rejoin_curved_leg_uses_recorded_trace_instead_of_endpoint_chord(self):
        r=dict(waypoints=[dict(id=str(i),name=str(i),xyz=p) for i,p in enumerate([[0,0,0],[1,0,0],[8,0,0]])],
            edges=[dict(control_points=[dict(xyz=[4,4,0]),dict(xyz=[1,4,0])]),{}])
        entry=forward_entry(r,1,[3,4,0])
        self.assertEqual(entry['target_index'],1);self.assertIn([1,4,0],entry['path'])

    def test_undo_marks_keeps_raw_samples_and_policy_events(self):
        self.start();self.action('mark',self.tick(2));self.tick(3,policy='step_move');n=len(self.m.recorder.samples)
        self.action('undo');self.assertEqual(len(self.m.recorder.marks),1);self.assertEqual(len(self.m.recorder.samples),n)
        self.assertEqual(len(self.m.recorder.switches),1)

    def test_write_failure_retains_draft_and_never_stops_manual_control(self):
        self.start();self.tick(3)
        with patch('route_recording.atomic_json',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.save()
        self.assertEqual(self.m.recorder.status,'recording');self.assertEqual(self.m.owner,'manual')
        self.assertEqual(set(self.m.plans.sources),{'indoor'});self.assertGreater(len(self.m.recorder.samples),1)
        rid=self.save();self.assertIn(rid,self.m.plans.sources)

    def test_recording_can_be_calibrated_without_losing_unedited_trace_geometry(self):
        self.start();self.tick(1,.5);self.tick(2,1);self.action('mark',self.tick(3,0));self.tick(4,-.5);self.tick(5,0);rid=self.save()
        route_=self.m.plans.route(rid);cal=self.m.plans.calibration;controls=copy.deepcopy(route_['edges'][1]['control_points'])
        def edit(action,**kw):return self.m.command(dict(action='calibration_'+action,route=rid,revision=cal.data['revision'],**kw),None,self.now)
        edit('set',point_id=route_['waypoints'][0]['id'],xyz=[-.1,0.,0.]);edit('apply')
        self.assertEqual(self.m.plans.route(rid)['edges'][1]['control_points'],controls)
        cp=self.m.plans.route(rid)['edges'][0]['control_points'][0]
        edit('set',point_id=cp['id'],xyz=[1,.6,0]);edit('apply')
        self.assertEqual(self.m.plans.route(rid)['edges'][0]['control_points'][0]['xyz'],[1,.6,0])
        restart=Mission(self.base,FakeBridge(),self.path);self.addCleanup(restart.recorder.close)
        self.assertEqual(restart.plans.route(rid),self.m.plans.route(rid))


if __name__=='__main__':unittest.main()
