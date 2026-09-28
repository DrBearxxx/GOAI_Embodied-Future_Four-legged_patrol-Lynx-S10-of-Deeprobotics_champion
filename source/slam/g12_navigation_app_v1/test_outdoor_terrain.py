"""Terrain mapping, advance selection and continuous policy transitions."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from build_outdoor_presets import anticipate
from mission import Mission
from navigation import Navigator,blocking_warnings
from planning import Plans
from test_unified import FakeBridge,route,solution
from test_forward_entry import pose,clear

ROOT=Path(__file__).parent


class OutdoorTerrainTests(unittest.TestCase):
    def test_advance_entry_retains_last_terrain_edge(self):
        edges=[dict(policy=p,terrain=[dict(id=str(i))]) for i,p in enumerate(['basic_normal','basic_normal','stairs_normal','stairs_normal','basic_normal'])]
        anticipate(edges,'retain_current_terrain')
        self.assertEqual([e['policy'] for e in edges],['basic_normal','stairs_normal','stairs_normal','stairs_normal','basic_normal'])
        self.assertEqual([e['terrain_policy'] for e in edges],['basic_normal','basic_normal','stairs_normal','stairs_normal','basic_normal'])

    def test_neighboring_terrain_conflict_has_explicit_resolution(self):
        source=[dict(policy=p,terrain=[dict(id=str(i))]) for i,p in enumerate(['basic_normal','stairs_normal','platform','basic_normal'])]
        a=copy.deepcopy(source);b=copy.deepcopy(source)
        anticipate(a,'retain_current_terrain');anticipate(b,'upcoming_terrain')
        self.assertEqual([e['policy'] for e in a],['stairs_normal','stairs_normal','platform','basic_normal'])
        self.assertEqual([e['policy'] for e in b],['stairs_normal','platform','platform','basic_normal'])

    def test_exact_outdoor_defaults_and_unmodified_waypoints(self):
        original=json.loads((ROOT/'routes/full.json').read_text(encoding='utf-8'))
        profile=json.loads((ROOT/'routes/outdoor_terrain_presets.json').read_text(encoding='utf-8'))
        p=Plans({'full':original,'indoor':route()})
        self.assertEqual(p.presets['full'],[e['policy'] for e in profile['edges']])
        self.assertEqual(p.presets['indoor'],['basic_normal']*4)
        self.assertEqual(p.route('full')['waypoints'],original['waypoints'])
        self.assertEqual(p.route('full')['map_id'],original['map_id'])
        self.assertEqual(p.sources['full'],original)
        self.assertEqual(len(p.route('full')['edges']),65)
        self.assertEqual(p.presets['full'][1:9],['stairs_normal']*8)
        self.assertEqual(p.presets['full'][48:53],['platform']*5)
        self.assertEqual(p.presets['full'][9],'basic_normal')
        self.assertEqual(p.presets['full'][53],'basic_normal')

    def test_different_route_does_not_inherit_terrain(self):
        r=json.loads((ROOT/'routes/full.json').read_text(encoding='utf-8'))
        r['waypoints'][0]['xyz'][0]+=.01
        p=Plans({'full':r})
        self.assertFalse(p.terrain);self.assertEqual(p.presets['full'],['basic_normal']*65)

    def test_saved_operator_edits_survive_default_loading(self):
        routes={'full':json.loads((ROOT/'routes/full.json').read_text(encoding='utf-8')),'indoor':route()}
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'presets.json';p=Plans(routes,path)
            p.set_policy('full',[3],'basic_normal');p.set_policy('indoor',[1],'platform')
            q=Plans(routes,path)
            self.assertEqual(q.presets,p.presets)
            self.assertEqual(q.presets['full'][3],'basic_normal')
            self.assertEqual(q.presets['indoor'][1],'platform')

    def test_replan_keeps_original_advance_policy_and_labels(self):
        r=json.loads((ROOT/'routes/full.json').read_text(encoding='utf-8'))
        p=Plans({'full':r})
        a,b=r['waypoints'][1]['xyz'],r['waypoints'][2]['xyz']
        s=solution();s['pose']=[*[(x+y)*.5 for x,y in zip(a,b)],0.]
        preview=p.propose('full',10,[],s,10.)
        actual=p.apply(preview['id'],s,10.)
        self.assertEqual(actual['edges'][0]['source_index'],1)
        self.assertEqual(p.policy(actual,1),'stairs_normal')
        self.assertTrue(actual['edges'][0]['anticipated_terrain'])

    def test_height_terrain_note_does_not_stop_or_truncate_trajectory(self):
        r=route();r['edges'][1]['warnings']=['slope_or_stairs_requires_terrain_validation']
        n=Navigator({'indoor':r});n.start(pose(x=.7),10.)
        first=n.step(pose(x=.7),clear(10.),10.)
        trajectory,_=n.path()
        self.assertGreater(trajectory.arcs[-1],2.)
        out=n.step(pose(t=10.1,x=.85),clear(10.1),10.1)
        self.assertEqual(n.target,2);self.assertGreater(out['vx'],0.)
        self.assertNotEqual(out['state'],'HOLD_ROUTE_VALIDATION')
        self.assertEqual(r['edges'][1]['warnings'],['slope_or_stairs_requires_terrain_validation'])

    def test_other_route_warnings_still_reported(self):
        self.assertEqual(blocking_warnings({'warnings':['slope_or_stairs_requires_terrain_validation','unverified']}),['unverified'])

    def test_policy_switch_on_waypoint_queues_each_new_preset(self):
        # This fake acknowledges instantly. Real gateway pause/resume is covered
        # by test_switch_navigation and test_switch_pause.
        bridge=FakeBridge();bridge.selected='basic_normal';m=Mission({'indoor':route()},bridge)
        m.plans.set_policy('indoor',[1],'stairs_normal');m.plans.set_policy('indoor',[2],'platform')
        m.command({'action':'start'},solution(x=.7),10.)
        for t,x,policy in [(10.,.7,'basic_normal'),(10.1,.85,'stairs_normal'),(10.2,1.8,'platform'),(10.3,2.8,'basic_normal')]:
            m.keepalive(m.navigator.run_id,t)
            out=m.step(solution(t=t,x=x),clear(t),t)
            self.assertEqual(out['policy'],policy)
            self.assertNotEqual(out['state'],'WAIT_POLICY_FEEDBACK')
            self.assertTrue(out['motion_authorized']);self.assertGreater(out['vx'],0.)
            bridge.selected=policy
        self.assertEqual(bridge.actions,['select:stairs_normal','select:platform','select:basic_normal'])
        self.assertEqual(bridge.stops,0)

    def test_every_outdoor_edge_uses_preset_without_height_note_stop(self):
        r=json.loads((ROOT/'routes/full.json').read_text(encoding='utf-8'))
        m=Mission({'full':r,'indoor':route()},FakeBridge());m.navigator.select('full')
        n=m.navigator;expected=list(m.plans.presets['full'])
        for i in range(len(r['edges'])):
            n.target=i+1;n.active=True;n.execution=False;n.entry=None;n.generation=(1,0)
            n.operator_until=11.;n.previous_measured_pose=None;n.arrivals=[];n.trajectory_progress=0.
            trajectory,segment=n.path();a,b=trajectory.bounds[segment]
            ref=trajectory.at((trajectory.arcs[a]+trajectory.arcs[b])*.5)
            s=solution();s['pose']=[*ref['xyz'],ref['yaw']]
            m.owner='auto';m.execution=False
            out=m.step(s,clear(10.),10.)
            self.assertNotEqual(out['state'],'HOLD_ROUTE_VALIDATION',i)
            self.assertEqual(out['policy'],expected[out['target_index']-1],i)


if __name__=='__main__':unittest.main()
