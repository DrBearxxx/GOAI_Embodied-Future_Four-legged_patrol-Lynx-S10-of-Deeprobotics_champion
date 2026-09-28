import json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from mission import Mission
from planning import Plans,forward_entry
from obstacle_evidence import clearance,policy_obstacles
from test_unified import FakeBridge,route,solution
from test_forward_entry import clear
from test_continuity_v2 import sample,anchor
from continuous_pose import ContinuousPose


def shelf(z):
    return np.array([[x,y,z] for x in np.linspace(.48,1.08,18) for y in np.linspace(-.5,.5,26)])


class UpgradeTests(unittest.TestCase):
    def test_previous_progress_geometry_is_not_reused_after_manual_target(self):
        m=Mission({'indoor':route()},FakeBridge());n=m.navigator
        n.set_progress(2,'test',10.)
        n.start(solution(x=.1),10.)
        self.assertEqual(n.entry['path'],[[.1,0.,0.],[2.,0.,0.]])
        self.assertEqual(n.reached,[0,1]);self.assertEqual(n.skipped,[])
        n.step(solution(x=1.9),clear(10.),10.)
        path,segment=n.path()
        self.assertEqual(path.points,[[2.,0.,0.],[3.,0.,0.],[4.,0.,0.]])
        self.assertEqual(n.target,3);self.assertEqual(segment,0)

    def test_presets_migrate_but_agile_is_still_a_persistent_override(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'presets.json';plans=Plans({'indoor':route()})
            p.write_text(json.dumps(dict(digest=plans.digest,revision=9,presets={'indoor':['basic','stairs','platform','basic']})))
            m=Mission({'indoor':route()},FakeBridge(),p)
            self.assertEqual(m.plans.presets['indoor'],['basic_normal','stairs_normal','platform','basic_normal'])
            m.command(dict(action='shadow'),solution(),10.)
            m.command(dict(action='override',policy='basic'),None,10.)
            self.assertEqual(m.owner,'auto')
            for i in range(4):m.navigator.target=i;self.assertEqual(m.desired_policy(),'basic')
            m.command(dict(action='override',policy=None),None,10.)
            self.assertEqual(m.desired_policy(),'platform')

    def test_speed_is_persisted_and_not_used_as_a_command_cap(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'presets.json';m=Mission({'indoor':route()},FakeBridge(),p)
            m.command(dict(action='set_speed',cruise_mps=2.3),None,10.)
            m.command(dict(action='shadow'),solution(),10.)
            o=m.step(solution(),clear(10.),10.)
            self.assertGreater(o['vx'],2.3);self.assertIsNone(o['speed_limit_mps'])
            self.assertEqual(Mission({'indoor':route()},FakeBridge(),p).navigator.cruise_mps,2.3)
            for v in (-1,0,True,float('nan'),float('inf')):
                with self.assertRaises(ValueError):m.command(dict(action='set_speed',cruise_mps=v),None,10.)

    def test_supporting_platform_and_stair_returns_are_not_obstacles_in_their_modes(self):
        for policy,z in [('stairs_normal',0.),('stairs',.05),('platform',.28)]:
            rec=clearance(shelf(z),10.,0)
            self.assertTrue(rec['blocked']);self.assertTrue(policy_obstacles(rec,'basic_normal')['blocked'])
            self.assertFalse(policy_obstacles(rec,policy)['blocked'])
            self.assertGreater(policy_obstacles(rec,policy)['terrain_hits'],300)

    def test_vertical_wall_narrow_object_and_tall_object_still_block(self):
        wall=np.array([[.7,y,z] for y in np.linspace(-.4,.4,30) for z in np.linspace(-.1,1.1,30)])
        narrow=np.array([[x,y,.25] for x in np.linspace(.5,1.,30) for y in np.linspace(-.1,.1,30)])
        tall=np.vstack([shelf(.2),np.tile([.7,0,.8],(10,1))])
        for cloud in (wall,narrow,tall):
            self.assertTrue(policy_obstacles(clearance(cloud,10.,0),'platform')['blocked'])

    def test_new_target_policy_is_used_for_obstacles_on_same_tick(self):
        m=Mission({'indoor':route()},FakeBridge());m.plans.set_policy('indoor',[1],'platform')
        m.command(dict(action='shadow'),solution(x=.6),10.)
        o=m.step(solution(x=.9),{'front':clearance(shelf(.28),10.,0)},10.)
        self.assertEqual(o['policy'],'platform');self.assertEqual(o['obstacle_assessment']['policy'],'platform')
        self.assertGreater(o['vx'],0.)

    def test_map_matches_cannot_move_pause_or_renew_odometry_after_alignment(self):
        c=ContinuousPose();c.odometry(sample(1.,0.));c.map_update(anchor(1.))
        rev=c.revision
        for i in range(1,501):
            t=1.+i*.1;x=i*.2;c.odometry(sample(t,x))
            self.assertFalse(c.map_update(anchor(t,x+20*math.sin(i),(1,i))))
            s=c.estimate(t);self.assertTrue(s['arrival_valid']);self.assertAlmostEqual(s['pose'][0],x)
        self.assertEqual(c.revision,rev);self.assertEqual(c.map_t,1.)
        self.assertEqual(c.estimate(t+1)['mode'],'LOST')
        self.assertTrue(c.map_update(anchor(t,6.,(2,1))));self.assertAlmostEqual(c.estimate(t)['pose'][0],6.)

if __name__=='__main__':unittest.main()
