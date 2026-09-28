import copy,math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.goai_control import GoaiRoute
from indoor.transport import JsonIngress
from test_control import simple_route,inputs


def data(t,p=(0,0,0,0),source='manual',nonce=0):
    s,_,_=inputs(t,p)
    s.update(asset_id='map1',boot_id='boot1',run_id='run1',perception_mono={'front':t-.02,'rear':t-.03})
    f=dict(mono=t,backend='goai_default',allow_actuation=True,healthy=True,fault=False,mode='policy',
        ownership=True,other_joint_publishers=0,foreign_commands=0,joint_receivers=1,source=source,
        manual_ready=True,nav_ready=True,nav_permit_active=source=='navigation',nav_permit_available=False,nav_permit_nonce=nonce)
    e=dict(mono=t,status_publishers=1,localization_publishers=1,other_nav_publishers=0,nav_subscribers=1,authority_publishers=1)
    return s,f,e


class GoaiTests(unittest.TestCase):
    def ready(self):
        g=GoaiRoute(simple_route(),'map1','boot1')
        for i in range(56):g.tick(i*.04,*data(i*.04))
        return g

    def active(self):
        g=self.ready();self.assertTrue(g.prepare(2.2)[0]);g.tick(2.24,*data(2.24,source='navigation',nonce=8))
        c=g.tick(2.28,*data(2.28,source='navigation',nonce=8));self.assertGreater(c['vx'],0)
        return g

    def test_zero_without_route_intent(self):
        g=self.ready()
        for i in range(20):self.assertEqual(g.tick(2.24+i*.04,*data(2.24+i*.04,source='navigation',nonce=8))['vx'],0)
        self.assertFalse(g.prepare(3)[0]);self.assertEqual(g.target,0)

    def test_prepare_only_manual_no_live_permit(self):
        for field,value in [('source','navigation'),('manual_ready',False),('nav_ready',False),('nav_permit_available',True),('nav_permit_active',True)]:
            g=self.ready();x=data(2.24);x[1][field]=value;g.tick(2.24,*x);self.assertFalse(g.prepare(2.24)[0])

    def test_no_movement_until_c(self):
        g=self.ready();self.assertTrue(g.prepare(2.2)[0])
        for i in range(20):self.assertEqual(g.tick(2.24+i*.04,*data(2.24+i*.04))['vx'],0)
        self.assertTrue(g.pending);self.assertFalse(g.armed)

    def test_takeover_and_recovery_no_auto_resume(self):
        for source in ('manual','stopped'):
            g=self.active();c=g.tick(2.32,*data(2.32,source=source));self.assertEqual(c['vx'],0);self.assertFalse(g.armed)
            for i in range(60):self.assertEqual(g.tick(2.36+i*.04,*data(2.36+i*.04,source='navigation',nonce=9))['vx'],0)
            self.assertFalse(g.prepare(4.72)[0]);self.assertEqual(g.target,1)

    def test_safety_faults_latch(self):
        changes=[lambda s,f,e:s.update(asset_id='wrong'),lambda s,f,e:s.update(boot_id='wrong'),
            lambda s,f,e:s.update(run_id='new'),lambda s,f,e:s.update(generation=[2,0,0,0]),
            lambda s,f,e:s.update(obstacle=True),lambda s,f,e:s.update(pose=[0,1,0,0]),
            lambda s,f,e:s.update(pose=[0,0,0,1]),lambda s,f,e:s['health'].update(valid=False),
            lambda s,f,e:s['health'].update(single_lidar=True),lambda s,f,e:s['perception_mono'].update(rear=1),
            lambda s,f,e:s.update(mono=1),lambda s,f,e:f.update(mono=1),lambda s,f,e:f.update(nav_permit_active=False),
            lambda s,f,e:f.update(nav_permit_nonce=9),lambda s,f,e:f.update(fault=True),lambda s,f,e:f.update(healthy=False),
            lambda s,f,e:f.update(ownership=False),lambda s,f,e:f.update(foreign_commands=1),lambda s,f,e:f.update(mode='hold'),
            lambda s,f,e:e.update(status_publishers=2),lambda s,f,e:e.update(localization_publishers=2),
            lambda s,f,e:e.update(other_nav_publishers=1),lambda s,f,e:e.update(authority_publishers=0),
            lambda s,f,e:e.update(nav_subscribers=0),lambda s,f,e:e.update(mono=1)]
        for i,change in enumerate(changes):
            with self.subTest(i=i):
                g=self.active();x=data(2.32,source='navigation',nonce=8);change(*x);c=g.tick(2.32,*x)
                self.assertEqual((c['vx'],c['wz']),(0,0));self.assertFalse(g.armed)
                c=g.tick(2.36,*data(2.36,source='navigation',nonce=8));self.assertEqual(c['vx'],0)

    def test_expired_intent_and_stalled_loop(self):
        g=self.ready();g.prepare(2.2)
        for i in range(760):g.tick(2.24+i*.04,*data(2.24+i*.04))
        self.assertFalse(g.pending)
        g=self.active();c=g.tick(3,*data(3,source='navigation',nonce=8))
        self.assertEqual(c['vx'],0);self.assertFalse(g.armed)

    def test_restart_during_pending(self):
        g=self.ready();g.prepare(2.2);x=data(2.24);x[0]['run_id']='new';g.tick(2.24,*x)
        self.assertFalse(g.pending)

    def test_old_nonce_cannot_rearm(self):
        g=self.active();g.stop()
        for i in range(60):g.tick(2.32+i*.04,*data(2.32+i*.04))
        self.assertTrue(g.prepare(4.68)[0]);c=g.tick(4.72,*data(4.72,source='navigation',nonce=8))
        self.assertFalse(g.armed);self.assertEqual(c['vx'],0)

    def test_restamped_snapshot(self):
        g=self.active();x=data(2.32,source='navigation',nonce=8);x[0]['seq']=g.last_seq
        self.assertEqual(g.tick(2.32,*x)['state'],'REPEATED_POSE_RESTAMPED')

    def test_complete_route_in_order(self):
        g=self.ready();g.prepare(2.2);p=[0.,0.,0.,0.]
        for i in range(2000):
            t=2.24+i*.04;c=g.tick(t,*data(t,p,source='navigation',nonce=8))
            self.assertLessEqual(c['vx'],.1);self.assertEqual(c['vy'],0);self.assertLessEqual(abs(c['wz']),.2)
            if g.complete:break
            p[0]+=c['vx']*math.cos(p[3])*.04;p[1]+=c['vx']*math.sin(p[3])*.04;p[3]+=c['wz']*.04
        self.assertTrue(g.complete);self.assertEqual(g.reached,[0,1]);self.assertFalse(g.prepare(t)[0])

    def test_full_indoor_21_points_goai(self):
        import json
        root=Path(__file__).resolve().parents[1]
        route=json.loads((root/'assets/route.json').read_text());g=GoaiRoute(route,'map1','boot1')
        p=np.r_[route['waypoints'][0]['xyz'],0.]
        for i in range(56):g.tick(i*.04,*data(i*.04,p))
        self.assertTrue(g.prepare(2.2)[0])
        for i in range(6000):
            t=2.24+i*.04;c=g.tick(t,*data(t,p,source='navigation',nonce=8))
            self.assertTrue(g.armed or g.complete,msg=c)
            if g.complete:break
            p[0]+=c['vx']*math.cos(p[3])*.04;p[1]+=c['vx']*math.sin(p[3])*.04;p[3]+=c['wz']*.04
            k=max(1,g.target);a,b=g.xyz[k-1:k+1];v=b-a;u=np.clip((p[:3]-a)@v/max(v@v,1e-9),0,1);p[2]=(a+u*v)[2]
        self.assertTrue(g.complete);self.assertEqual(g.reached,list(range(21)))
        out=root/'results/goai_route';out.mkdir(parents=True,exist_ok=True)
        (out/'kinematic_result.json').write_text(json.dumps(dict(complete=g.complete,reached=g.reached,seconds=t,
            final_distance_m=float(np.linalg.norm(p[:3]-g.xyz[-1])),physical_motion=False,test_type='ideal kinematics, not physical dynamics or true localization'))+'\n')


class TransportTests(unittest.TestCase):
    def sample(self):return '{"mono": 2.0}',dict(source_timestamp=2000000000,received_timestamp=2000000000,publisher_gid=bytes([1])*24),2000000000,2.
    def test_fresh_then_replay_invalidates(self):
        g=JsonIngress();x=self.sample();self.assertTrue(g.receive(*x));self.assertIsNotNone(g.get(2.2));self.assertIsNone(g.get(2.3))
        self.assertFalse(g.receive(*x));self.assertIsNone(g.get(2.0))
    def test_queue_future_bad_json(self):
        for field,value in [('source_timestamp',1),('received_timestamp',1),('source_timestamp',2500000000),('publisher_gid',bytes(24))]:
            g=JsonIngress();s,i,w,t=self.sample();i[field]=value;self.assertFalse(g.receive(s,i,w,t))
        g=JsonIngress();s,i,w,t=self.sample();self.assertFalse(g.receive('[]',i,w,t))
    def test_writer_replacement_invalidates_one_packet(self):
        g=JsonIngress();s,i,w,t=self.sample();g.receive(s,i,w,t);i['publisher_gid']=bytes([2])*24;i['source_timestamp']+=1
        self.assertFalse(g.receive(s,i,w,t));self.assertIsNone(g.get(t))

if __name__=='__main__':unittest.main(verbosity=2)
