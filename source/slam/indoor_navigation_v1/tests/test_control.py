import copy,math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.control import Guard
from indoor.bootstrap import ROOT
from s10nav.util import read,write

def inputs(t,xyz=(0,0,0,0),seq=None):
    return (dict(mono=t,seq=int(t*100) if seq is None else seq,pose=list(xyz),health=dict(age_s=.02,valid=True,single_lidar=False,healthy_lidars=['front','rear']),
        generation=[1,0,0,0],front_fresh=True,rear_fresh=True,obstacle=False,error=None),
        dict(mono=t,state=17,gait=0x3002,charge=0,charge_error=0,charge_mono=t,speed=0.),
        dict(mono=t,other_nav_publishers=0,rl_service_active=False,other_named_joint_publishers=[],nav_subscribers=1))

def simple_route():return dict(waypoints=[dict(xyz=[0,0,0]),dict(xyz=[1,0,0])],edges=[dict(warnings=[])])

class ControlTests(unittest.TestCase):
    def ready(self):
        g=Guard(simple_route())
        for t in np.arange(0,2.2,.1):g.tick(float(t),*inputs(float(t)))
        self.assertTrue(g.arm(2.1)[0]);return g
    def assertStopped(self,g,t,x):
        c=g.tick(t,*x);self.assertEqual((c['vx'],c['vy'],c['wz']),(0,0,0));self.assertFalse(g.armed)
    def test_default_never_moves(self):
        g=Guard(simple_route())
        for t in range(10):self.assertEqual(g.tick(t,*inputs(t))['vx'],0.)
        self.assertEqual(g.target,0)
    def test_start_requires_manual_entry(self):
        g=Guard(simple_route());g.tick(1,*inputs(1,(.5,0,0,0)));self.assertFalse(g.arm(1)[0])
    def test_stable_window(self):
        g=Guard(simple_route());g.tick(1,*inputs(1));self.assertFalse(g.arm(1)[0])
    def test_deadman_latched(self):
        g=self.ready();self.assertStopped(g,2.6,inputs(2.6));g.renew_deadman(2.7);self.assertStopped(g,2.7,inputs(2.7))
    def test_ownership_conflicts(self):
        for field,value in [('other_nav_publishers',1),('rl_service_active',True),('other_named_joint_publishers',['rl_deploy']),('nav_subscribers',0)]:
            with self.subTest(field=field):
                g=self.ready();x=inputs(2.2);x[2][field]=value;self.assertStopped(g,2.2,x)
    def test_input_faults(self):
        alterations=[lambda s:s.update(mono=1),lambda s:s.update(mono=10),lambda s:s.update(pose=[float('nan'),0,0,0]),
            lambda s:s.update(front_fresh=False),lambda s:s.update(rear_fresh=False),lambda s:s.update(obstacle=True),
            lambda s:s['health'].update(valid=False),lambda s:s['health'].update(single_lidar=True),
            lambda s:s.update(generation=[2,0,0,0]),lambda s:s.update(seq=1),lambda s:s.update(pose=[.01,0,0,1])]
        for i,alter in enumerate(alterations):
            with self.subTest(i=i):
                g=self.ready();x=inputs(2.2);alter(x[0]);self.assertStopped(g,2.2,x)
    def test_feedback_faults(self):
        for field,value in [('mono',0),('state',2),('gait',0x1001),('charge',1),('charge_error',1),('charge_mono',0),('speed',float('nan')),('speed',1.)]:
            with self.subTest(field=field,value=value):
                g=self.ready();x=inputs(2.2);x[1][field]=value;self.assertStopped(g,2.2,x)
    def test_off_route_and_height(self):
        for p in [(0,2,0,0),(0,0,1,0)]:
            g=self.ready();self.assertStopped(g,2.2,inputs(2.2,p))
    def test_stop_then_resume_preserves_order(self):
        g=self.ready();g.renew_deadman(2.2);g.tick(2.2,*inputs(2.2));self.assertEqual(g.target,1)
        g.stop();self.assertEqual(g.target,1)
        for t in np.arange(2.3,4.6,.1):g.tick(float(t),*inputs(float(t)))
        self.assertTrue(g.arm(4.5)[0]);self.assertEqual(g.target,1)
    def test_closed_loop_entire_indoor_route(self):
        route=read(ROOT/'assets/route.json');g=Guard(route);p=np.r_[route['waypoints'][0]['xyz'],0.];trajectory=[]
        for i in range(23):g.tick(i*.1,*inputs(i*.1,p))
        self.assertTrue(g.arm(2.2)[0]);t=2.2
        for _ in range(5000):
            t+=.1;g.renew_deadman(t);c=g.tick(t,*inputs(t,p));trajectory.append([t,*p,c['vx'],c['wz'],g.target])
            self.assertLessEqual(abs(c['vx']),.100001);self.assertEqual(c['vy'],0.);self.assertLessEqual(abs(c['wz']),.200001)
            self.assertTrue(g.armed or g.complete,msg=c)
            if g.complete:break
            p[0]+=c['vx']*math.cos(p[3])*.1;p[1]+=c['vx']*math.sin(p[3])*.1;p[3]+=c['wz']*.1
            # Kinematic simulation follows surveyed ground height; this does
            # not simulate legs, contact, stopping distance or localization.
            k=max(1,g.target);a,b=g.xyz[k-1:k+1];v=b-a;u=np.clip((p[:3]-a)@v/max(v@v,1e-9),0,1);p[2]=(a+u*v)[2]
        self.assertTrue(g.complete);self.assertEqual(g.reached,list(range(len(g.xyz))))
        self.assertFalse(g.arm(t)[0]);self.assertStopped(g,t+.1,inputs(t+.1,p))
        out=ROOT/'results/control_simulation';out.mkdir(parents=True,exist_ok=True)
        np.savetxt(out/'trajectory.csv',trajectory,delimiter=',',header='t,x,y,z,yaw,vx,wz,target',comments='')
        write(out/'summary.json',dict(completed=True,seconds=t,points_reached=g.reached,final_error_m=float(np.linalg.norm(p[:3]-g.xyz[-1])),
            test_type='kinematic only; not robot dynamics or ground truth',robot_commands_sent=False))

if __name__=='__main__':unittest.main(verbosity=2)
