import copy,json,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.bootstrap import ROOT
from indoor.continuity import Continuity,RedundantMotion,safety_policy
from indoor.safety_stream import clearance,raw_points
from indoor.goai_control import GoaiRoute
from test_goai_control import data
from test_control import simple_route

def motion():
    cfg=json.loads((ROOT/'config.json').read_text())['motion']
    return RedundantMotion(cfg,{s:np.eye(4) for s in ('front','rear','camera')})

def feed(m,side,start,end,rate=.1):
    for t in np.arange(start,end+.0001,.005):m.add_imu(side,float(t),np.array([0,0,rate]),np.array([0,0,9.81]))

def anchor(t=1.,x=0.):
    T=np.eye(4);T[0,3]=x
    return dict(T=T.tolist(),measurement_mono=t,velocity=[.1,0,0],confirmed=True,generation=[1],healthy_lidars=['front','rear'])

def records(t):return {s:dict(mono=t,valid=True,blocked=False,epoch=0) for s in ('front','rear')}

def robust_data(t,p=(0,0,0,0),age=.02,safety_age=.02,mode='TRACKING',source='manual',nonce=0,imu_age=.005):
    s,f,e=data(t,p,source,nonce);s.update(schema='goai.localization.continuity.v2',generation=[1,0])
    s['pose']=list(p)
    s['solution']=dict(mode=mode,measurement_mono=t-age,estimate_mono=t-imu_age,
        measured_pose=list(p),pose=list(p),generation=s['generation'],reason='',velocity_body=[.1,0,0])
    s['safety_records']=records(t-safety_age)
    return s,f,e

class MotionTests(unittest.TestCase):
    def test_single_redundant_stream(self):
        m=motion();feed(m,'front',.95,1.05);feed(m,'rear',.95,1.55)
        self.assertEqual(m.gyro_path(1,1.5)['sensor'],'rear')
    def test_stitch_complementary_gaps(self):
        m=motion();feed(m,'front',.95,1.3);feed(m,'rear',1.25,1.55)
        p=m.gyro_path(1,1.5);self.assertEqual(p['sensor'],'stitched_front_rear')
        self.assertAlmostEqual(np.arctan2(p['rotations'][-1,1,0],p['rotations'][-1,0,0]),.05,places=5)
    def test_common_gap_rejected(self):
        m=motion();feed(m,'front',.95,1.2);feed(m,'rear',1.3,1.55)
        self.assertIsNone(m.gyro_path(1,1.5))
    def test_switch_without_overlap_or_agreement_rejected(self):
        for start,rate in [(1.30,.1),(1.25,.5)]:
            m=motion();feed(m,'front',.95,1.3);feed(m,'rear',start,1.55,rate)
            self.assertIsNone(m.gyro_path(1,1.5))
    def test_absent_camera_not_required(self):
        m=motion();feed(m,'front',.95,1.5);c=Continuity();c.update(anchor(),m)
        self.assertEqual(c.estimate(1.5,m)['mode'],'PREDICT_ONLY')

class ContinuityTests(unittest.TestCase):
    def test_expiry_and_no_restamping(self):
        m=motion();feed(m,'front',.95,1.7);c=Continuity();a=anchor();c.update(a,m)
        for t,mode in [(1.1,'TRACKING'),(1.3,'DEGRADED'),(1.5,'PREDICT_ONLY'),(1.56,'LOST')]:
            # Causal delivery only, not future IMU samples.
            m2=motion();feed(m2,'front',.95,t)
            c.update(a,m2);s=c.estimate(t,m2)
            self.assertEqual(s['mode'],mode,s);self.assertEqual(s['measurement_mono'],1.)
    def test_short_common_imu_outage_bounded(self):
        m=motion();feed(m,'front',.95,1.2);c=Continuity();c.update(anchor(),m)
        s=c.estimate(1.26,m);self.assertEqual(s['mode'],'PREDICT_ONLY');self.assertEqual(s['max_wz'],0.)
        self.assertAlmostEqual(s['estimate_mono'],1.2)
        self.assertEqual(c.estimate(1.29,m)['mode'],'LOST')
    def test_no_bridge_after_global_gap(self):
        m=motion();feed(m,'front',.95,1.1);feed(m,'front',1.3,1.4);c=Continuity();c.update(anchor(),m)
        self.assertEqual(c.estimate(1.4,m)['mode'],'LOST')
    def test_large_reassociation_changes_generation(self):
        m=motion();feed(m,'front',.95,1.5);c=Continuity();c.update(anchor(),m);c.update(anchor(1.4,.35),m)
        self.assertEqual(c.reassociation,1)
    def test_unconfirmed_future_and_excess_speed_rejected(self):
        for changes,t in [({'confirmed':False},1.1),({},.99),({'velocity':[2.1,0,0]},1.1)]:
            m=motion();feed(m,'front',.95,t);c=Continuity();a=anchor();a.update(changes);c.update(a,m)
            self.assertEqual(c.estimate(t,m)['mode'],'LOST')

class SafetyTests(unittest.TestCase):
    def test_stale_clear_is_bounded_and_no_turn(self):
        r=records(1.);self.assertEqual(safety_policy(1.2,r)['mode'],'FRESH')
        s=safety_policy(1.3,r);self.assertEqual((s['mode'],s['max_vx'],s['max_wz']),('CAUTIOUS',.02,0.))
        self.assertEqual(safety_policy(1.41,r)['mode'],'STOP')
    def test_bad_missing_future_blocked(self):
        cases=[{},None,{'front':{}},records(2.)]
        for field,value in [('mono',float('nan')),('blocked',True),('blocked',0),('valid',1),('mono',None)]:
            r=records(1);r['rear'][field]=value;cases.append(r)
        for r in cases:self.assertEqual(safety_policy(1.1,r)['mode'],'STOP',r)
    def test_raw_cloud_endianness_and_padding(self):
        for big in (True,False):
            dtype=np.dtype(('>' if big else '<')+'f4');p=np.array([[1,2,3,.1],[4,5,6,.05]],dtype=dtype)
            cloud=dict(data=p[0].tobytes()+bytes(4)+p[1].tobytes()+bytes(4),height=2,width=1,row_step=20,point_step=16,
                is_bigendian=big,fields=[(k,i*4,7,1) for i,k in enumerate(('x','y','z','time'))])
            np.testing.assert_allclose(raw_points(cloud),p)
    def test_positive_obstacle_is_not_filtered_away(self):
        p=np.tile([2.,2.,0.],(1000,1));p[:10]=[.8,0,.5]
        self.assertTrue(clearance(p,1,0)['blocked']);self.assertFalse(clearance(p[10:],1,0)['blocked'])
    def test_matching_worker_quarantines_bad_cloud(self):
        import queue,tempfile,time
        from unittest.mock import patch
        from indoor import pipeline
        class Stop:
            calls=0
            def is_set(self):self.calls+=1;return self.calls>1
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'live_logs').mkdir();q=queue.Queue();out=queue.Queue()
            q.put(dict(received=time.monotonic(),topic='/bad',cloud={}))
            with patch.object(pipeline,'ROOT',root),patch.object(pipeline,'read',return_value={}),\
                    patch.object(pipeline,'Engine'),patch.object(pipeline,'IndoorMatcher'),\
                    patch.object(pipeline,'use_live_clocks',side_effect=lambda e:e),\
                    patch.object(pipeline,'use_redundancy',side_effect=lambda e:e):
                pipeline.worker_main([q,queue.Queue()],out,Stop(),101,None)
            self.assertTrue(out.empty());entry=json.loads((root/'live_logs/localize-101.jsonl').read_text())
            self.assertFalse(entry['accepted']);self.assertIn('malformed_cloud',entry['reason'])

class RobustRouteTests(unittest.TestCase):
    def ready(self):
        g=GoaiRoute(simple_route(),'map1','boot1')
        for i in range(56):g.tick(i*.04,*robust_data(i*.04))
        return g
    def active(self):
        g=self.ready();self.assertTrue(g.prepare(2.2)[0]);g.tick(2.24,*robust_data(2.24,source='navigation',nonce=8))
        g.tick(2.28,*robust_data(2.28,source='navigation',nonce=8));self.assertTrue(g.armed);return g
    def test_short_faults_do_not_disarm(self):
        for kwargs,cap,wz in [(dict(age=.32),.04,.10),(dict(age=.50),.02,.06),
                (dict(safety_age=.3),.02,0.),(dict(imu_age=.06,age=.1),.02,0.)]:
            g=self.active();c=g.tick(2.32,*robust_data(2.32,source='navigation',nonce=8,**kwargs))
            self.assertTrue(g.armed,c);self.assertGreater(c['vx'],0);self.assertLessEqual(c['vx'],cap);self.assertLessEqual(abs(c['wz']),wz)
    def test_long_faults_latch(self):
        for kwargs in [dict(age=.56),dict(safety_age=.41),dict(imu_age=.09),dict(mode='LOST')]:
            g=self.active();c=g.tick(2.32,*robust_data(2.32,source='navigation',nonce=8,**kwargs))
            self.assertFalse(g.armed,c);self.assertEqual(c['vx'],0)
            for i in range(80):self.assertEqual(g.tick(2.36+i*.04,*robust_data(2.36+i*.04,source='navigation',nonce=8))['vx'],0)
    def test_recovery_hysteresis(self):
        g=self.active();g.tick(2.32,*robust_data(2.32,age=.32,source='navigation',nonce=8))
        for i in range(12):
            t=2.36+i*.04;g.tick(t,*robust_data(t,source='navigation',nonce=8));self.assertLessEqual(g.robust_limits[0],.04)
        g.tick(2.84,*robust_data(2.84,source='navigation',nonce=8));self.assertEqual(g.robust_limits[0],.1)
    def test_prediction_cannot_complete_waypoint(self):
        g=self.active();g.target=1;p=[*g.xyz[1],0.];s,f,e=robust_data(2.32,p,age=.5,source='navigation',nonce=8)
        # Evaluate arrival hook separately; moving there instantaneously would itself be rejected as a jump.
        g.solution_mode='PREDICT_ONLY';g.measurement_age=.5;self.assertFalse(g.goal_reached(s,0.))
        g.solution_mode='TRACKING';g.measurement_age=.02;s['solution']['measured_pose'][0]-=.20
        self.assertFalse(g.goal_reached(s,0.))
    def test_cannot_enter_in_degraded_quality(self):
        g=self.ready();self.assertTrue(g.prepare(2.2)[0]);g.tick(2.24,*robust_data(2.24,age=.32,source='navigation',nonce=8))
        self.assertFalse(g.armed);self.assertFalse(g.pending)
    def test_ready_display_matches_strong_window(self):
        g=self.ready();c=g.tick(2.24,*robust_data(2.24,age=.32));self.assertFalse(c['ready'])
        self.assertFalse(g.prepare(2.24)[0])
    def test_unknown_or_inconsistent_solution_rejected(self):
        for change in [lambda s:s['solution'].update(measured_pose=[]),lambda s:s['solution'].update(estimate_mono=5.),
                lambda s:s['solution'].update(generation=[99]),lambda s:s['solution'].update(pose=[9,0,0,0]),
                lambda s:s['solution'].update(velocity_body=[.6,0,0])]:
            g=self.active();s,f,e=robust_data(2.32,source='navigation',nonce=8);change(s)
            self.assertEqual(g.tick(2.32,s,f,e)['vx'],0);self.assertFalse(g.armed)

if __name__=='__main__':unittest.main(verbosity=2)
