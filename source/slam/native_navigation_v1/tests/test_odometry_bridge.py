import copy
import unittest
import numpy as np
from native_nav import bootstrap
from native_nav.robust_timing import RobustContinuity
from native_nav.odometry_bridge import measured_bridge
from test_robust_timing import make_motion,anchor
import test_resilient_route as route_tests
from test_resilient_route import sample


def streams(end=2.0,velocity=.03,imu=True,vio=True):
    m=make_motion()
    if imu:
        for t in np.arange(.9,end+.0001,.005):m.add_imu('rear',float(t),np.zeros(3),np.array([0,0,9.81]))
    if vio:
        for t in np.arange(.9,end+.0001,.02):
            T=np.eye(4);T[0,3]=velocity*(t-1);m.add_vio(float(t),T)
    return m


class OdometryBridgeTests(unittest.TestCase):
    def test_missing_map_fit_with_measured_vio_imu_continues(self):
        m=streams();a=anchor();a['velocity']=[.03,0,0];c=RobustContinuity();c.update(a,m)
        s=c.estimate(2.,m)
        self.assertEqual(s['mode'],'ODOM_BRIDGE');self.assertLessEqual(s['estimate_mono'],2.)
        self.assertEqual(s['measurement_mono'],1.);self.assertFalse(s['bridge']['extrapolated'])
        self.assertAlmostEqual(s['pose'][0],.03,places=2)
        self.assertEqual(s['max_vx'],.03)

    def test_no_vio_no_imu_and_common_gap_cannot_extend(self):
        for m in (streams(vio=False),streams(imu=False)):
            self.assertIsNone(measured_bridge(2.,anchor(),m))
        m=streams();m.imu['rear']=type(m.imu['rear'])((x for x in m.imu['rear'] if not 1.3<x[0]<1.4),maxlen=1400)
        self.assertIsNone(measured_bridge(2.,anchor(),m))

    def test_asynchronous_vio_head_uses_common_received_endpoint(self):
        m=streams();m.imu['rear']=type(m.imu['rear'])((row for row in m.imu['rear'] if row[0]<=1.986),maxlen=1400)
        b=measured_bridge(2.,anchor(),m)
        self.assertIsNotNone(b);self.assertLessEqual(b['estimate_mono'],1.986)
        self.assertGreater(b['estimate_mono'],1.98)

    def test_cached_bridge_never_refreshes_measurement_or_endpoint(self):
        m=streams();c=RobustContinuity();c.update(anchor(),m)
        # Include the final real sample in both calls (arange's 2.0 can be
        # 2.000000000000001); otherwise the second call legitimately sees new
        # past evidence and is not testing cache reuse at all.
        first=c.estimate(2.01,m);second=c.estimate(2.03,m)
        self.assertEqual(first['mode'],'ODOM_BRIDGE');self.assertEqual(second['mode'],'ODOM_BRIDGE')
        self.assertEqual(first['estimate_mono'],second['estimate_mono'])
        self.assertEqual(second['measurement_mono'],1.)
        self.assertGreater(second['xy_budget_m'],first['xy_budget_m'])
        self.assertEqual(c.estimate(2.1,m)['mode'],'LOST')

    def test_vio_gap_reset_stale_and_reversal_rejected(self):
        for kind in ('gap','reset','stale','reversal'):
            m=streams();q=list(m.vio)
            if kind=='gap':q=[x for x in q if not 1.3<x[0]<1.5]
            if kind=='reset':q=[(t,T,1 if t>1.5 else e) for t,T,e in q]
            if kind=='stale':q=[x for x in q if x[0]<1.85]
            if kind=='reversal':q[20],q[21]=q[21],q[20]
            m.vio=type(m.vio)(q,maxlen=200)
            self.assertIsNone(measured_bridge(2.,anchor(),m),kind)

    def test_bridge_horizon_travel_and_rotation_disagreement_rejected(self):
        self.assertIsNone(measured_bridge(2.21,anchor(),streams(2.21)))
        self.assertIsNone(measured_bridge(2.,anchor(),streams(velocity=.25)))
        m=streams()
        m.imu['rear']=type(m.imu['rear'])(((t,np.array([0,0,.2]),a) for t,g,a in m.imu['rear']),maxlen=1400)
        self.assertIsNone(measured_bridge(2.,anchor(),m))

    def test_unconfirmed_same_map_does_not_erase_prior_anchor(self):
        m=streams(1.4);c=RobustContinuity();c.update(anchor(),m)
        candidate=anchor(1.3);candidate['confirmed']=False;c.update(candidate,m)
        self.assertEqual(c.anchor['measurement_mono'],1.)
        self.assertEqual(c.estimate(1.4,m)['mode'],'PREDICT_ONLY')
        candidate['generation']=[2];c.update(candidate,m)
        self.assertEqual(c.estimate(1.4,m)['mode'],'LOST')

    def test_route_uses_bridge_without_hold_but_cannot_claim_waypoint(self):
        g=route_tests.ResilientRouteTests().active()
        for i in range(29):
            t=2.54+i*.04
            m=streams(t-1.52+.005);a=anchor();a['velocity']=[.03,0,0]
            c=RobustContinuity();c.update(a,m);solution=c.estimate(t-1.52,m)
            # Shift the independent synthetic sensor epoch into the route clock.
            for k in ('measurement_mono','estimate_mono'):solution[k]+=1.52
            if 'bridge' in solution:
                for k in ('start_mono','end_mono'):solution['bridge'][k]+=1.52
            s,f,e=sample(t);s['solution']=solution;s['pose']=solution['pose'];s['generation']=solution['generation']
            command=g.tick(t,s,f,e)
            self.assertTrue(g.armed,command);self.assertFalse(g.holding,command)
            if solution['mode']=='ODOM_BRIDGE':
                self.assertGreater(command['vx'],0);self.assertLessEqual(command['vx'],.03)
                self.assertFalse(g.goal_reached(s,0.))
        self.assertEqual(g.target,1)

    def test_map_jitter_is_not_body_velocity_and_raw_fit_is_preserved(self):
        m=streams(4.);c=RobustContinuity();c.update(anchor(),m);last=None;max_step=0.
        for i in range(1,10):
            t=1+i*.24;a=anchor(t);a['velocity']=[.45,0,0]
            raw=.03*(t-1)+(.10 if i%2 else -.10);a['T'][0][3]=raw
            c.update(a,m);s=c.estimate(t+.02,m)
            self.assertEqual(s['velocity_source'],'measured_vio_gyro')
            self.assertAlmostEqual(s['velocity_body'][0],.03,places=6)
            self.assertAlmostEqual(s['measured_pose'][0],raw)
            self.assertEqual(s['fusion_method'],'measured_odom_bounded_map_correction')
            self.assertLessEqual(s['filter_residual_m'],.10)
            if last is not None:max_step=max(max_step,abs(s['pose'][0]-last))
            last=s['pose'][0]
        self.assertLess(max_step,.15)

    def test_outlier_does_not_refresh_map_time_or_hide_big_change(self):
        m=streams();c=RobustContinuity();c.update(anchor(),m)
        bad=anchor(1.4);bad['T'][0][3]=.8;c.update(bad,m)
        self.assertEqual(c.anchor['measurement_mono'],1.)
        self.assertEqual(c.rejected_map_innovations,1)

    def test_confirmation_can_change_at_same_measurement_time(self):
        m=streams();c=RobustContinuity();a=anchor();a['confirmed']=False;c.update(a,m)
        self.assertEqual(c.estimate(1.1,m)['mode'],'LOST')
        a['confirmed']=True;c.update(a,m)
        self.assertEqual(c.estimate(1.1,m)['mode'],'TRACKING')


if __name__=='__main__':unittest.main(verbosity=2)
