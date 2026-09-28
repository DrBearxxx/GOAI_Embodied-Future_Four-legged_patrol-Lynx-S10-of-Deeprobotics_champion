import copy
import importlib.util
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,mat,wrap
from s10nav.timing import DeviceClock
from s10nav.motion import Motion
from s10nav.navigation import RouteFollower
from s10nav.engine import Engine
from s10nav.replay_io import inject

CFG=read(ROOT/'config.json')

class RouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.route=read(ROOT/'assets/route.json');cls.original=read(ROOT/'assets/original_waypoints.json')
    def test_all_coordinates_ids_and_order_preserved(self):
        self.assertEqual([(p['id'],p['xyz']) for p in self.route['waypoints']],[(p['id'],p['xyz']) for p in self.original['waypoints']])
        self.assertEqual(len(self.route['waypoints']),66)
    def test_headings_point_to_next(self):
        for i,p in enumerate(self.route['waypoints'][:-1]):
            q=self.route['waypoints'][i+1]; d=np.array(q['xyz'])-p['xyz']
            self.assertLess(abs(wrap(p['yaw']-np.arctan2(d[1],d[0]))),1e-10)
    def test_final_heading_continues_last_edge(self):
        self.assertEqual(self.route['waypoints'][-1]['yaw'],self.route['waypoints'][-2]['yaw'])
    def test_no_motion_authorization(self):self.assertFalse(self.route['autonomous_use_approved'])
    def test_dense_route_exact_endpoints(self):
        a=np.genfromtxt(ROOT/'assets/route_samples.csv',delimiter=',',names=True)
        for p in self.route['waypoints']:
            self.assertLess(np.linalg.norm(np.c_[a['x'],a['y'],a['z']]-p['xyz'],axis=1).min(),1e-8)
    def follower(self):return RouteFollower(self.route,CFG['route'])
    def pose(self,index):return mat(self.route['waypoints'][index]['xyz'],[0,0,0,1])
    def health(self,valid=True):return dict(valid=valid,single_lidar=False)
    def test_no_predicted_only_arrival(self):
        f=self.follower();r=f.update(self.pose(0),self.health(False),1,False,True)
        self.assertEqual(f.reached,[]);self.assertEqual(r['state'],'HOLD_LOCALIZATION')
    def test_no_automatic_skip_to_nearest_later_point(self):
        f=self.follower();r=f.update(self.pose(40),self.health(),1,False,True)
        self.assertEqual(f.target,0);self.assertEqual(r['state'],'WAIT_ROUTE_ENTRY')
    def test_arrival_requires_dwell_and_one_at_a_time(self):
        f=self.follower();T=self.pose(0)
        f.update(T,self.health(),0,False,True);self.assertEqual(f.target,0)
        f.update(T,self.health(),.3,False,True);self.assertEqual(f.target,1);self.assertEqual(f.reached,[0])
    def test_relocalization_requires_reassociation(self):
        f=self.follower();f.reached=[0];f.target=1
        r=f.update(self.pose(1),self.health(),2,False,True,True)
        self.assertEqual(r['state'],'HOLD_REASSOCIATION');self.assertEqual(r['vx'],0)
    def test_obstacle_stops_and_cannot_complete_goal(self):
        f=self.follower();r=f.update(self.pose(0),self.health(),1,True,True)
        self.assertEqual(r['state'],'HOLD_OBSTACLE');self.assertEqual(f.reached,[])
    def test_missing_forward_perception_stops(self):
        f=self.follower();r=f.update(self.pose(0),self.health(),1,False,False)
        self.assertEqual(r['state'],'HOLD_FORWARD_PERCEPTION')
    def test_wrong_height_stops(self):
        f=self.follower();f.target=1;T=self.pose(1);T[2,3]+=2
        self.assertEqual(f.update(T,self.health(),1,False,True)['state'],'HOLD_OFF_ROUTE')
    def test_positive_shadow_follow_preserves_target(self):
        f=self.follower();f.target=1
        a,b=np.array(self.route['waypoints'][0]['xyz']),np.array(self.route['waypoints'][1]['xyz'])
        heading=np.arctan2((b-a)[1],(b-a)[0]);T=mat((a+b)/2,Rotation.from_euler('z',heading).as_quat())
        r=f.update(T,self.health(),1,False,True)
        self.assertEqual(r['state'],'FOLLOW_SHADOW');self.assertGreater(r['vx'],0);self.assertFalse(r['motion_authorized']);self.assertEqual(f.target,1)

class ClockTests(unittest.TestCase):
    def clock(self):
        c=DeviceClock(CFG['clock'])
        for t in np.arange(0,1,.005):c.observe_imu(100+float(t),float(t))
        return c
    def test_clock_warmup(self):self.assertEqual(self.clock().reason,'ok')
    def test_duplicate_rejected(self):
        c=self.clock();self.assertIsNone(c.observe_imu(c.native,c.received+.001));self.assertEqual(c.reason,'duplicate_stamp')
    def test_clock_step_quarantined(self):
        c=self.clock();self.assertIsNone(c.observe_imu(102,1.));self.assertEqual(c.epoch,1);self.assertIsNone(c.map_stamp(102,1))
    def test_rollback_quarantined(self):
        c=self.clock();self.assertIsNone(c.observe_imu(99,1.));self.assertEqual(c.epoch,1)
    def test_delayed_old_samples_not_fresh(self):
        c=self.clock();self.assertIsNone(c.observe_imu(101,1.6))
    def test_recovery_requires_warmup(self):
        c=self.clock();c.observe_imu(102,1)
        for t in np.arange(1.005,1.8,.005):c.observe_imu(101+float(t),float(t))
        self.assertEqual(c.reason,'ok');self.assertEqual(c.epoch,1)

class MotionTests(unittest.TestCase):
    def motion(self):return Motion(CFG['motion'],{s:np.eye(4) for s in ['front','rear','camera']})
    def fill(self,m,start=0,end=.5):
        for t in np.arange(start,end+.001,.005):m.add_imu('front',float(t),np.array([0,0,1.]),np.array([0,0,9.81]))
    def test_gyro_integration(self):
        m=self.motion();self.fill(m);g=m.gyro_path(.1,.4)
        self.assertAlmostEqual(Rotation.from_matrix(g['rotations'][-1]).as_rotvec()[2],.3,places=6)
    def test_imu_gap_not_bridged(self):
        m=self.motion();self.fill(m,0,.1);self.fill(m,.2,.3);self.assertIsNone(m.gyro_path(.05,.25))
    def test_gyro_fallback(self):
        m=self.motion();self.fill(m);m.imu['rear']=m.imu['front'];m.imu['front'].clear()
        self.assertIsNone(m.gyro_path(.1,.2))
        self.fill(m);m.imu['rear']=copy.deepcopy(m.imu['front']);m.imu['front'].clear()
        self.assertEqual(m.gyro_path(.1,.2)['sensor'],'rear')
    def test_float32_scan_time_bounds(self):
        m=self.motion();self.fill(m);p=np.array([[1,0,0],[1,0,0.]])
        out,_=m.deskew(p,np.array([0,.09995198],np.float32),.1,np.eye(4),np.zeros(3))
        self.assertTrue(np.isfinite(out).all());np.testing.assert_allclose(out[-1],p[-1],atol=1e-7)
    def test_vio_jump_not_integrated(self):
        m=self.motion();m.add_vio(0,np.eye(4));T=np.eye(4);T[0,3]=15;m.add_vio(.05,T)
        self.assertEqual(m.vio_epoch,1);self.assertIsNone(m.vio_at(.025))
    def test_gravity_does_not_force_flat_when_moving(self):
        m=self.motion();self.fill(m);self.assertIsNone(m.gravity(.49))
    def test_gravity_antiparallel(self):
        m=self.motion()
        for t in np.arange(0,.5,.005):m.add_imu('front',float(t),np.zeros(3),np.array([0,0,-9.81]))
        np.testing.assert_allclose(m.gravity(.49)@np.array([0,0,-1]),[0,0,1],atol=1e-7)
    def test_gravity_propagation_is_bounded_and_gap_checked(self):
        m=self.motion();self.fill(m);m.anchor_gravity(.1,np.eye(3))
        self.assertIsNotNone(m.gravity(.4))
        self.assertIsNone(m.gravity(1.5))
        self.assertIsNone(m.gravity(30))
    def test_clock_reset_removes_gravity_prior(self):
        m=self.motion();self.fill(m);m.anchor_gravity(.1,np.eye(3));m.clear('front')
        self.assertIsNone(m.gravity(.4))
    def test_vio_extrapolation_uses_only_recent_increment(self):
        m=self.motion();m.add_vio(0,np.eye(4));T=np.eye(4);T[0,3]=.05;m.add_vio(.05,T)
        self.assertAlmostEqual(m.vio_at(.1)[0,3],.1)
        self.assertIsNone(m.vio_at(.2))

class RuntimeTests(unittest.TestCase):
    def test_unexpected_imu_frame_is_not_fused(self):
        e=Engine(ROOT/'assets',CFG,None);e.event(dict(kind='imu',sensor='front',received=0,stamp=100,frame='wrong',gyro=[0,0,0],accel=[0,0,9.81]))
        self.assertEqual(e.counts['unexpected_frame_front'],1)
    def test_nan_imu_is_not_fused(self):
        e=Engine(ROOT/'assets',CFG,None);e.event(dict(kind='imu',sensor='front',received=0,stamp=100,gyro=[float('nan'),0,0],accel=[0,0,9.81]))
        self.assertEqual(e.counts['invalid_imu_front'],1)
    def test_watchdog_expires_without_new_messages(self):
        e=Engine(ROOT/'assets',CFG,None);e.state='TRACKING';e.last_good=1;e.last_t=1;e.confirmations=3
        self.assertTrue(e.poll(1.2)['valid']);self.assertFalse(e.poll(1.5)['valid']);self.assertEqual(e.poll(2)['state'],'LOST')
    def test_proposal_never_authorizes_robot(self):
        e=Engine(ROOT/'assets',CFG,None);self.assertFalse(e.proposal(0)['motion_authorized'])
    def test_queue_old_input_rejected(self):
        e=Engine(ROOT/'assets',CFG,None);e.event(dict(kind='imu',sensor='front',received=0,stamp=100,gyro=[0,0,0],accel=[0,0,9.81]),1)
        self.assertEqual(e.counts['queue_stale_imu'],1);self.assertEqual(len(e.motion.imu['front']),0)
    def test_injected_delay_changes_arrival_not_native(self):
        events=[dict(kind='imu',sensor='front',received=t,stamp=100+t) for t in [184.,186.,187.,191.]]
        got=list(inject(events,'mixed_faults'));self.assertEqual(got[1]['received'],186.6);self.assertEqual(got[1]['stamp'],286.)
    def test_fault_drop_all(self):
        events=[dict(kind='imu',sensor='front',received=t,stamp=t) for t in [144.,146.,150.]]
        self.assertEqual([e['received'] for e in inject(events,'mixed_faults')],[144.,150.])

if __name__=='__main__':unittest.main(verbosity=2)
