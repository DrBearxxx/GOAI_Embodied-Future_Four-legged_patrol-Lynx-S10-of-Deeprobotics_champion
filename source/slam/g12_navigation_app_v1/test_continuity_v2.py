import unittest
from types import SimpleNamespace
import numpy as np
from continuous_pose import ContinuousPose
from navigation import Navigator
from protocol import Tickets


def sample(t,x=0.,epoch='e',path=None):
    T=np.eye(4);T[0,3]=x
    return dict(T=T.tolist(),t=t,epoch=epoch,path=x if path is None else path,accepted=10)
def anchor(t,x=0.,generation=(1,1)):
    T=np.eye(4);T[0,3]=x
    return dict(T=T.tolist(),measurement_mono=t,generation=generation,confirmed=True)
def route():
    return dict(waypoints=[dict(name=str(i),xyz=[i*.5,0.,0.]) for i in range(5)],
                edges=[dict(warnings=[],speed_limit_mps=.2) for _ in range(4)])
def solution(t,x=0.,mode='TRACKING',yaw=0.):
    return dict(pose=[x,0.,0.,yaw],generation=['e',1],estimate_mono=t,measurement_mono=t,
                mode=mode,arrival_valid=mode=='TRACKING',max_vx=.2 if mode=='TRACKING' else .06,max_wz=.2)
def clear(t):return dict(front=dict(mono=t,valid=True,blocked=False))


class ContinuityTests(unittest.TestCase):
    def ready(self):
        c=ContinuousPose()
        for t in (1.,1.1,1.2):c.odometry(sample(t,(t-1)*.1))
        self.assertTrue(c.map_update(anchor(1.1,.01)))
        return c
    def test_time_alignment(self):
        c=self.ready();s=c.estimate(1.2)
        self.assertAlmostEqual(s['pose'][0],.02,places=6)
    def test_small_map_correction_is_bounded(self):
        c=self.ready();c.estimate(1.2);c.map_update(anchor(1.2,.12))
        s=c.estimate(1.25)
        self.assertLessEqual(s['pose'][0]-.02,.00601)
        self.assertEqual(s['measurement_mono'],1.2)
    def test_large_jump_not_fresh(self):
        c=self.ready();self.assertFalse(c.map_update(anchor(1.2,3.)))
        self.assertEqual(c.map_t,1.1);self.assertEqual(c.estimate(1.2)['rejected_map_innovations'],0)
        self.assertEqual(c.estimate(4.)['mode'],'LOST')
    def test_odom_epoch_requires_new_alignment(self):
        c=self.ready();c.odometry(sample(1.3,epoch='reset'))
        self.assertIsNone(c.estimate(1.3))
    def test_duplicate_does_not_refresh(self):
        c=self.ready();c.odometry(sample(1.2,9.))
        self.assertEqual(c.estimate(2.)['mode'],'LOST')
    def test_delayed_map_waits_for_bracket(self):
        c=ContinuousPose();c.odometry(sample(1.))
        self.assertFalse(c.map_update(anchor(1.05)))
        c.odometry(sample(1.1));self.assertTrue(c.map_update(anchor(1.05)))
    def test_fresh_measured_odometry_continues_without_map_refresh(self):
        c=self.ready()
        for t in np.arange(1.3,2.01,.1):c.odometry(sample(round(float(t),6),float(t-1)*.1))
        s=c.estimate(2.);self.assertEqual(s['mode'],'ODOM_BRIDGE');self.assertTrue(s['arrival_valid'])
        for t in np.arange(2.1,4.01,.1):c.odometry(sample(round(float(t),6),float(t-1)*.1))
        self.assertEqual(c.estimate(4.)['mode'],'ODOM_BRIDGE')
    def test_relocalization_is_explicit_revision(self):
        c=self.ready();rev=c.revision
        self.assertTrue(c.map_update(anchor(1.2,8.,(2,1))))
        self.assertGreater(c.revision,rev)
    def test_single_map_fit_outlier_does_not_move_stationary_pose(self):
        c=ContinuousPose()
        for t in (1.,1.1,1.2,1.3):
            c.odometry(sample(t));c.map_update(anchor(t));c.estimate(t)
        c.odometry(sample(1.4));c.map_update(anchor(1.4,.2))
        self.assertAlmostEqual(c.estimate(1.4)['pose'][0],0.,places=6)
        self.assertEqual(c.map_t,1.)
    def test_map_correction_filter_does_not_delay_local_motion(self):
        c=ContinuousPose()
        for i in range(16):
            t=1.+i*.1;x=i*.02
            c.odometry(sample(t,x));c.map_update(anchor(t,x))
            self.assertAlmostEqual(c.estimate(t)['pose'][0],x,places=6)
    def test_new_map_alignment_clears_old_offset_window(self):
        c=self.ready();c.map_update(anchor(1.2,8.,(2,1)))
        self.assertAlmostEqual(c.estimate(1.2)['pose'][0],8.,places=6)
    def fake_motion(self):
        def pose(t):
            T=np.eye(4);T[0,3]=t*.1;return T
        stamps=[round(float(t),6) for t in np.arange(.95,1.81,.025)]
        return SimpleNamespace(vio=[(t,pose(t),0) for t in stamps],
            imu={'front':[(t,np.zeros(3),np.zeros(3)) for t in stamps]},
            cfg={'max_vio_gap_s':.16},vio_at=pose,
            gyro_path=lambda a,b:dict(rotations=[np.eye(3)],max_gap=.025),relative=lambda a,b,v:(None,'not_modeled'))
    def test_real_vio_bridge_not_constant_velocity(self):
        import paths
        c=self.ready();self.assertEqual(c.estimate(1.6)['mode'],'LOST')
        s=c.estimate(1.6,self.fake_motion())
        self.assertEqual(s['mode'],'ODOM_BRIDGE');self.assertIsNotNone(s['vio_bridge'])
        self.assertAlmostEqual(s['pose'][0],.06,places=5);self.assertFalse(s['arrival_valid'])
    def test_vio_epoch_break_cannot_bridge(self):
        import paths
        c=self.ready();m=self.fake_motion();m.vio=[(t,p,0 if t<1.4 else 1) for t,p,e in m.vio]
        self.assertEqual(c.estimate(1.6,m)['mode'],'LOST')
    def test_prediction_cannot_freshen_measurement_or_arrive(self):
        import paths
        c=self.ready();m=self.fake_motion();m.vio=[]
        m.relative=lambda a,b,v:(np.eye(4),'gyro_plus_bounded_constant_velocity')
        s=c.estimate(1.5,m)
        self.assertEqual(s['mode'],'PREDICT_ONLY');self.assertFalse(s['arrival_valid'])
        self.assertEqual(s['measurement_mono'],1.2);self.assertEqual(s['map_measurement_mono'],1.1);self.assertIsNone(s['max_vx'])
        self.assertEqual(c.estimate(1.7,m)['mode'],'LOST')


class NavigationTests(unittest.TestCase):
    def test_navigation_ticket_short_and_one_shot(self):
        q=Tickets(.8);t=q.issue(1.)
        self.assertTrue(q.consume(t,1.5));self.assertFalse(q.consume(t,1.6))
        t=q.issue(2.);self.assertFalse(q.consume(t,2.81))
    def test_keepalive_replay_cannot_extend_run(self):
        n=Navigator({'indoor':route()});n.start(solution(1.),1.)
        q=Tickets(1.);ticket=q.issue(1.)
        if q.consume(ticket,1.1):n.keepalive(n.run_id,1.1)
        deadline=n.operator_until
        if q.consume(ticket,1.8):n.keepalive(n.run_id,1.8)
        self.assertEqual(n.operator_until,deadline)
    def ready(self):
        n=Navigator({'indoor':route()});n.start(solution(1.),1.)
        for t in (1.,1.12,1.24):n.step(solution(t),clear(t),t)
        self.assertEqual(n.target,1)
        return n
    def test_order_and_native_policy_range(self):
        n=self.ready();s=n.step(solution(1.3),clear(1.3),1.3)
        self.assertEqual(s['state'],'FOLLOWING');self.assertGreater(s['vx'],.2)
        self.assertIsNone(s['speed_limit_mps'])
        self.assertEqual(n.reached,[0])
    def test_repeated_pose_cannot_advance_beyond_its_passed_waypoint(self):
        n=Navigator({'indoor':route()});n.start(solution(1.),1.)
        for t in np.arange(1.,1.5,.05):n.step(solution(1.),clear(t),float(t))
        self.assertEqual(n.target,1);self.assertEqual(n.reached,[0])
    def test_measured_bridge_can_pass_intermediate_waypoint(self):
        n=self.ready()
        for t in (1.3,1.42,1.54):n.step(solution(t,.5,'ODOM_BRIDGE'),clear(t),t)
        self.assertEqual(n.target,2);self.assertEqual(n.reached,[0,1])
    def test_sensor_recovery_retains_progress(self):
        n=self.ready();s=n.step(solution(1.3,mode='LOST'),clear(1.3),1.3)
        self.assertEqual(s['state'],'HOLD_LOCALIZATION');self.assertTrue(n.active)
        for t in (1.4,1.6,1.82):s=n.step(solution(t),clear(t),t)
        self.assertEqual(s['state'],'FOLLOWING');self.assertEqual(n.target,1)
    def test_relocalization_pauses(self):
        n=self.ready();s=solution(1.3)
        n.step(s,clear(1.3),1.3,pending=True);self.assertFalse(n.active)
        self.assertEqual(n.reason,'RELOCALIZATION_REQUESTED')
    def test_local_epoch_recovers_without_reselecting_route(self):
        n=self.ready();rid=n.run_id
        # The recorded failure switched briefly to a different fallback
        # estimator before the new local-odom map alignment was ready.
        s=solution(1.3,mode='DEGRADED');s['generation']=[1,1,40]
        out=n.step(s,clear(1.3),1.3)
        self.assertEqual(out['state'],'HOLD_LOCALIZATION');self.assertTrue(n.active)
        s=solution(1.4,mode='LOST');s['generation']=['new',32]
        n.step(s,clear(1.4),1.4);self.assertTrue(n.active)
        for t in (1.5,1.7,1.92):
            s=solution(t);s['generation']=['new',32]
            out=n.step(s,clear(t),t)
        self.assertEqual(out['state'],'JOINING_ROUTE');self.assertGreater(out['vx'],0)
        self.assertEqual(n.run_id,rid);self.assertEqual(n.reached,[0]);self.assertEqual(n.target,1)
    def test_new_generation_needs_measured_alignment(self):
        n=self.ready();s=solution(1.3,mode='ODOM_BRIDGE');s['generation']=['new',2]
        out=n.step(s,clear(1.3),1.3)
        self.assertEqual(out['state'],'HOLD_LOCALIZATION');self.assertTrue(n.active)
        self.assertEqual(n.generation,('e',1))
    def test_disconnect_no_auto_resume(self):
        n=self.ready();n.step(solution(4.),clear(4.),4.)
        n.keepalive(n.run_id,4.1);self.assertFalse(n.active)
    def test_obstacle_zero(self):
        n=self.ready();p=clear(1.3);p['front']['blocked']=True
        s=n.step(solution(1.3),p,1.3);self.assertEqual(s['vx'],0.)
        self.assertEqual(s['state'],'HOLD_OBSTACLE')
    def test_off_route_not_skipped(self):
        n=self.ready();s=solution(1.3);s['pose'][1]=2.
        out=n.step(s,clear(1.3),1.3);self.assertEqual(out['state'],'FOLLOWING');self.assertEqual(n.target,1);self.assertTrue(out['outside_route_corridor'])
    def test_shadow_feedback_loop_completes(self):
        n=Navigator({'indoor':route()});n.start(solution(1.),1.)
        x=0.;t=1.;maximum=0.
        for _ in range(2500):
            n.keepalive(n.run_id,t);out=n.step(solution(t,x),clear(t),t)
            x+=out['vx']*.05;maximum=max(maximum,out['vx']);t+=.05
            if out['state']=='COMPLETE':break
        self.assertEqual(out['state'],'COMPLETE');self.assertEqual(n.reached,[0,1,2,3,4])
        self.assertGreater(maximum,.2);self.assertLess(abs(x-2.),.26)

if __name__=='__main__':unittest.main()
