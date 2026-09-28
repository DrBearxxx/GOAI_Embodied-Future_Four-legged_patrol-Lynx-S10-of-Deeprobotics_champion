import copy,json,math,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from mission import Mission
from planning import Plans,forward_entry
from trajectory import RouteTrajectory
from test_unified import FakeBridge,route,solution
from test_forward_entry import pose,clear


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.path=Path(self.directory.name)/'presets.json'
        self.source=route()
        for p in self.source['waypoints']:p['xyz'][0]*=3
        self.m=Mission({'indoor':self.source},FakeBridge(),self.path)

    def action(self,action,point='1',solution_=None,**extra):
        return self.m.command(dict(action='calibration_'+action,route='indoor',revision=self.m.plans.calibration.data['revision'],point_id=point,**extra),solution_,10.)

    def test_capture_uses_real_measurement_and_preserves_manual_control(self):
        self.m.owner='manual';self.m.execution=True;before=copy.deepcopy(self.m.navigator.route)
        sample=solution(x=5);sample.update(measurement_pose=[3,.35,.4,.8],mode='PREDICT_ONLY',arrival_valid=False)
        out=self.action('record',solution_=sample)
        self.assertEqual(self.m.owner,'manual');self.assertEqual(self.m.bridge.stops,0)
        self.assertEqual(self.m.navigator.route,before)
        item=next(p for p in out['calibration']['items'] if p['id']=='1')
        self.assertEqual(item['xyz'],[3,.35,.4]);self.assertTrue(out['calibration']['dirty'])

    def test_draft_survives_restart_without_becoming_active(self):
        self.action('set',xyz=[3,.5,.3])
        restart=Plans({'indoor':self.source},self.path)
        self.assertEqual(restart.route('indoor')['waypoints'][1]['xyz'],[3.,0.,0.])
        self.assertEqual(restart.calibration.state('indoor')['items'][1]['xyz'],[3.,.5,.3])

    def test_apply_preserves_completed_progress_policy_override_and_original(self):
        self.m.command(dict(action='set_progress',route='indoor',target_index=2,waypoint_id='2'),None,9.)
        self.m.plans.set_policy('indoor',[1],'stairs_normal');self.m.override='platform'
        preset=copy.deepcopy(self.m.plans.presets);self.action('set',point='2',xyz=[6,.5,.2])
        self.action('apply')
        self.assertEqual(self.m.navigator.target,2);self.assertEqual(self.m.navigator.reached,[0,1])
        self.assertEqual(self.m.navigator.manual_target,2);self.assertEqual(self.m.navigator.route['waypoints'][2]['xyz'],[6.,.5,.2])
        self.assertEqual(self.m.override,'platform');self.assertEqual(self.m.plans.presets,preset)
        self.assertEqual(self.m.plans.sources['indoor'],self.source)
        self.assertFalse(self.m.navigator.active);self.assertEqual(self.m.owner,'paused')
        restart=Plans({'indoor':self.source},self.path)
        self.assertEqual(restart.route('indoor')['waypoints'][2]['xyz'],[6.,.5,.2])

    def test_inserted_controls_keep_waypoint_indices_and_inherit_the_segment_policy(self):
        self.m.plans.set_policy('indoor',[1],'stairs_normal')
        first=self.action('insert',solution_=dict(pose=[4,.3,0,0]))['selected_point']
        second=self.action('insert',point=first,solution_=dict(pose=[5,.4,0,0]))['selected_point']
        self.action('apply');r=self.m.plans.route('indoor')
        self.assertEqual([p['id'] for p in r['waypoints']],list('01234'))
        self.assertEqual([p['id'] for p in r['edges'][1]['control_points']],[first,second])
        self.assertEqual(self.m.plans.policy(r,2),'stairs_normal')
        self.assertEqual(len(self.m.plans.presets['indoor']),4)

    def test_offset_is_relative_to_route_not_robot_heading(self):
        self.action('offset',axis='left',distance=.2)
        item=self.m.plans.calibration.state('indoor')['items'][1]
        self.assertAlmostEqual(item['xyz'][0],3.);self.assertAlmostEqual(item['xyz'][1],.2)

    def test_undo_discard_and_restore_do_not_change_applied_route_until_apply(self):
        self.action('set',xyz=[3,.4,0]);self.action('apply')
        self.action('set',xyz=[3,.7,0]);self.action('undo')
        self.assertFalse(self.m.plans.calibration.state('indoor')['dirty'])
        self.action('previous');self.assertTrue(self.m.plans.calibration.state('indoor')['dirty'])
        self.assertEqual(self.m.plans.route('indoor')['waypoints'][1]['xyz'],[3.,.4,0.])
        self.action('apply');self.assertEqual(self.m.plans.route('indoor')['waypoints'][1]['xyz'],[3.,0.,0.])
        self.action('set',xyz=[3,.2,0]);self.action('discard')
        self.assertFalse(self.m.plans.calibration.state('indoor')['dirty'])

    def test_restore_original_restores_geometry_and_keeps_custom_presets(self):
        self.m.plans.set_policy('indoor',[1],'platform')
        self.action('set',xyz=[3,.6,0]);self.action('insert',solution_=dict(pose=[4,.7,0,0]));self.action('apply')
        self.action('reset_route');self.action('apply')
        self.assertEqual(self.m.plans.route('indoor')['waypoints'],self.source['waypoints'])
        self.assertNotIn('control_points',self.m.navigator.route['edges'][1])
        self.assertEqual(self.m.plans.presets['indoor'][1],'platform')

    def test_missing_pose_and_invalid_coordinates_do_not_modify_draft(self):
        before=copy.deepcopy(self.m.plans.calibration.data)
        for action,extra in (('record',{}),('set',{'xyz':[1,float('nan'),0]}),('set',{'xyz':[1,2]})):
            with self.assertRaises(ValueError):self.action(action,**extra)
        self.assertEqual(self.m.plans.calibration.data,before)

    def test_lost_pose_is_not_recorded_as_a_new_measurement_but_coordinate_edit_works(self):
        with self.assertRaisesRegex(ValueError,'实测定位'):
            self.action('record',solution_=dict(mode='LOST',pose=[3,0,0,0],measurement_pose=[3,0,0,0]))
        self.action('set',xyz=[3,.2,0])
        self.assertEqual(self.m.plans.calibration.state('indoor')['items'][1]['xyz'],[3.,.2,0.])

    def test_concurrent_revision_rejects_lost_update(self):
        self.action('set',xyz=[3,.4,0])
        with self.assertRaisesRegex(ValueError,'刷新'):
            self.m.command(dict(action='calibration_set',route='indoor',revision=0,point_id='1',xyz=[3,9,0]),None,10.)
        self.assertEqual(self.m.plans.calibration.state('indoor')['items'][1]['xyz'],[3.,.4,0.])

    def test_failed_write_retains_previous_disk_and_memory(self):
        self.action('set',xyz=[3,.4,0]);cal=self.m.plans.calibration
        before=copy.deepcopy(cal.data);raw=cal.path.read_bytes()
        with patch('route_calibration.os.replace',side_effect=OSError('disk error')):
            with self.assertRaises(OSError):self.action('set',xyz=[3,.8,0])
        self.assertEqual(cal.data,before);self.assertEqual(cal.path.read_bytes(),raw)

    def test_control_is_followed_on_resume_and_replan_without_returning_to_earlier_control(self):
        first=self.action('insert',solution_=dict(pose=[4,.5,0,0]))['selected_point']
        self.action('insert',point=first,solution_=dict(pose=[5,.5,0,0]));self.action('apply')
        r=self.m.plans.route('indoor');entry=forward_entry(r,2,[4.4,.55,0,0])
        self.assertEqual(entry['target_index'],2)
        self.assertTrue(all(p[0]>=4.4 for p in entry['path']))
        sample=solution(x=4.4);sample['pose'][1]=.5
        preview=self.m.plans.propose('indoor',4,[],sample,10.)
        controls=preview['route']['edges'][0]['control_points']
        self.assertEqual([p['xyz'][0] for p in controls],[5.])

    def test_control_points_shape_curve_but_are_not_completion_boundaries(self):
        path=RouteTrajectory([[0,0,0],[4,0,0],[8,0,0]],control_points=[[[1,.5,0],[2,.5,0]],[]])
        self.assertEqual(len(path.bounds),2)
        for point in ([1,.5,0],[2,.5,0]):self.assertLess(min(math.dist(s['xyz'],point) for s in path.samples),.025)
        boundary=path.samples[path.bounds[0][1]]['xyz'];self.assertEqual(boundary,[4.,0.,0.])

    def test_navigator_passes_inserted_points_without_stop_or_extra_progress_indices(self):
        self.action('insert',point='0',solution_=dict(pose=[1,.25,0,0]));self.action('apply')
        n=self.m.navigator;n.cruise_mps=.8;x=y=yaw=0.;t=10.;n.start(pose(t,x,y,yaw),t)
        zeros=[];closest=99.
        for _ in range(1500):
            n.keepalive(n.run_id,t);out=n.step(pose(t,x,y,yaw),clear(t),t)
            if out['target_index']<4 and not any(out[k] for k in ('vx','vy','wz')):zeros.append(out['state'])
            closest=min(closest,math.hypot(x-1,y-.25))
            c,s=math.cos(yaw),math.sin(yaw);x+=(c*out['vx']-s*out['vy'])*.05;y+=(s*out['vx']+c*out['vy'])*.05;yaw+=out['wz']*.05;t+=.05
            if out['state']=='COMPLETE':break
        self.assertEqual(out['state'],'COMPLETE');self.assertEqual(n.reached,[0,1,2,3,4]);self.assertEqual(zeros,[])
        self.assertLess(closest,.15)

    def test_frozen_outdoor_route_keeps_step_move_and_high_platform_indices(self):
        full=json.loads((Path(__file__).parent/'routes/full.json').read_text(encoding='utf-8'))
        self.m=Mission({'full':full,'indoor':route()},FakeBridge())
        cal=self.m.plans.calibration;pid=full['waypoints'][4]['id']
        self.m.command(dict(action='calibration_set',route='full',point_id=pid,revision=0,xyz=[17.8,-36.4,1.45]),None,10.)
        self.m.command(dict(action='calibration_apply',route='full',revision=1),None,10.)
        self.assertEqual([i for i,v in enumerate(self.m.plans.presets['full']) if v=='step_move'],list(range(30,41)))
        self.assertEqual(self.m.plans.presets['full'][28:30],['platform']*2)
        self.assertEqual(self.m.plans.sources['full'],full)


if __name__=='__main__':unittest.main()
