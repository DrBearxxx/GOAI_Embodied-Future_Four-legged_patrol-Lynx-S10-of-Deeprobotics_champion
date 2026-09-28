"""Step-move selection, frozen-route boundaries and one-time preset migration."""
import copy,hashlib,json,tempfile,unittest
from pathlib import Path
from planning import Plans,OFFICIAL,MANUAL,NATIVE_POLICIES
from mission import Mission
from build_outdoor_presets import apply_flat_step_move
from test_unified import FakeBridge,route,solution
from test_forward_entry import clear

ROOT=Path(__file__).parent


class StepMoveTests(unittest.TestCase):
    def routes(self):return {'full':json.loads((ROOT/'routes/full.json').read_text(encoding='utf-8')),'indoor':route()}

    def test_step_move_is_native_manual_override_and_preset_option(self):
        for catalog in (OFFICIAL,MANUAL,NATIVE_POLICIES):self.assertEqual(catalog['step_move'],'踏步移动')
        m=Mission({'indoor':route()},FakeBridge())
        m.plans.set_policy('indoor',[0,1,2,3],'step_move')
        m.command({'action':'shadow'},solution(x=.5),10.)
        out=m.step(solution(x=.5),clear(10.),10.)
        self.assertEqual(out['policy'],'step_move');self.assertEqual(out['input_kind'],'velocity')
        self.assertGreater(out['vx'],0.)
        m.command({'action':'override','policy':'stairs_normal'},solution(),10.1)
        self.assertEqual(m.desired_policy(),'stairs_normal')
        m.command({'action':'override','policy':None},solution(),10.2)
        self.assertEqual(m.desired_policy(),'step_move')

    def test_manual_full_axes_and_resume_to_step_move_preset(self):
        m=Mission({'indoor':route()},FakeBridge());m.plans.set_policy('indoor',[0,1,2,3],'step_move')
        m.command({'action':'manual_shadow','policy':'step_move'},None,10.)
        m.operator(dict(session='test',sample_seq=1,sample_age_ms=0,fresh=True,axes=[1,-1,.9]),10.)
        out=m.step(None,{},10.)
        self.assertEqual(out['axes'],[1,-1,.9]);self.assertEqual(out['input_kind'],'native_axes')
        m.command({'action':'shadow'},solution(t=11.,x=.5),11.)
        out=m.step(solution(t=11.,x=.5),clear(11.),11.)
        self.assertEqual(out['owner'],'auto');self.assertEqual(out['policy'],'step_move')
        self.assertEqual(out['policy_mode'],'route_preset')

    def test_exact_flat_run_and_both_high_platform_boundaries(self):
        routes=self.routes();original=copy.deepcopy(routes);p=Plans(routes)
        self.assertEqual([i for i,v in enumerate(p.presets['full']) if v=='step_move'],list(range(30,41)))
        self.assertEqual(p.presets['full'][28:30],['platform','platform'])
        self.assertEqual(p.presets['full'][48:53],['platform']*5)
        self.assertEqual(p.presets['full'][26:28],['stairs_normal']*2)
        self.assertEqual(p.presets['full'][53:],[ 'basic_normal']*12)
        self.assertEqual(p.sources,original);self.assertEqual(p.route('full')['waypoints'],routes['full']['waypoints'])
        self.assertEqual(p.route('full')['waypoints'][30]['name'],'WP 31')
        self.assertEqual(p.route('full')['waypoints'][41]['name'],'WP 42')
        self.assertEqual(p.presets['full'][41:48],['basic_normal']*7)
        # Active edge is target_index - 1; enter only after the terrace ends,
        # then return to normal at WP42; retain the platform advance at WP48.
        for target,policy in ((30,'platform'),(31,'step_move'),(41,'step_move'),(42,'basic_normal'),(48,'basic_normal'),(49,'platform')):
            self.assertEqual(p.policy(p.route('full'),target),policy)

    def test_existing_saved_presets_receive_requested_update_once(self):
        routes=self.routes();p=Plans(routes);old=copy.deepcopy(p.presets)
        old['full'][30:41]=['basic_normal']*11
        old['full'][3]='basic_normal';old['indoor'][1]='platform'
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'presets.json'
            path.write_text(json.dumps(dict(digest=p.digest,revision=4,presets=old)))
            updated=Plans(routes,path)
            self.assertEqual(updated.revision,5)
            self.assertEqual(updated.presets['full'][30:41],['step_move']*11)
            self.assertEqual(updated.presets['full'][3],'basic_normal')
            self.assertEqual(updated.presets['indoor'][1],'platform')
            again=Plans(routes,path);self.assertEqual(again.revision,5)
            again.set_policy('full',[35],'basic_normal')
            reloaded=Plans(routes,path)
            self.assertEqual(reloaded.presets['full'][35],'basic_normal');self.assertEqual(reloaded.revision,6)
            self.assertEqual(len(reloaded.applied_updates),1)

    def test_terrain_builder_reproduces_override_without_anticipating_into_terrace(self):
        expected=json.loads((ROOT/'routes/outdoor_terrain_presets.json').read_text(encoding='utf-8'))
        profile=copy.deepcopy(expected)
        for i in range(30,41):profile['edges'][i]['policy']='basic_normal'
        profile.pop('preset_updates')
        self.assertEqual(apply_flat_step_move(profile),expected)

    def test_different_route_does_not_receive_stored_update(self):
        routes=self.routes();routes['full']['waypoints'][30]['xyz'][0]+=.01
        p=Plans(routes);self.assertFalse(p.applied_updates)
        self.assertNotIn('step_move',p.presets['full'])


if __name__=='__main__':unittest.main()
