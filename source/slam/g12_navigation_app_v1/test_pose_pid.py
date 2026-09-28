"""Coupled controller regression; simulation only, no robot transport."""
import copy
import math
import unittest
from navigation import Navigator
from pose_pid import PosePID
from test_forward_entry import pose, clear


def recorded_route():
    # I02, I03 and the actual I02 exit pose from the failed run.
    xyz = [[0.,0.,0.],[.35,.32,0.],[.759980678,.372527579,-.029337568],
           [.876537540,.922399861,-.010854416],[.9,1.7,0.]]
    return dict(waypoints=[dict(name='I%02d'%i,xyz=p) for i,p in enumerate(xyz)],
                edges=[dict(warnings=[],speed_limit_mps=.1,corridor_half_width_m=.7) for _ in range(4)])


def after_i02():
    n=Navigator({'indoor':recorded_route()})
    n.target=3;n.reached=[2];n.skipped=[0,1];n.active=True;n.execution=True
    n.generation=(1,0);n.operator_until=100.
    return n


class CoupledPIDTests(unittest.TestCase):
    def test_i02_exit_translates_and_turns_in_same_tick(self):
        n=after_i02();s=pose(x=.638732,y=.373606,yaw=-.17426)
        n.generation=tuple(s['generation'])
        out=n.step(s,clear(10.),10.)
        self.assertEqual(out['target_name'],'I03')
        self.assertGreater(math.hypot(out['vx'],out['vy']),.4)
        self.assertGreater(out['vy'],.2);self.assertGreater(out['wz'],0.)
        self.assertGreater(out['vx'],0.)
        self.assertEqual(out['tracking']['controller'],'map_pose_pid')

    def test_no_heading_threshold_discontinuity(self):
        values=[]
        for yaw in (-math.pi/3-1e-6,-math.pi/3+1e-6):
            values.append(PosePID().step([0,0,0,yaw],[1,0],10.))
        for key in ('vx','vy','wz'):
            self.assertAlmostEqual(values[0][key],values[1][key],places=5)
        self.assertGreater(math.hypot(values[0]['vx'],values[0]['vy']),.8)

    def test_no_legacy_edge_localization_sensor_age_or_slew_cap(self):
        outputs=[]
        for mode,age in (('TRACKING',0.),('ODOM_BRIDGE',.3),('PREDICT_ONLY',.3)):
            n=after_i02();s=pose(x=.638732,y=.373606,yaw=-.17426)
            s.update(mode=mode,max_vx=.001,max_wz=.001)
            n.generation=tuple(s['generation']);out=n.step(s,clear(10.-age),10.)
            self.assertIsNone(out['speed_limit_mps']);outputs.append([out[k] for k in ('vx','vy','wz')])
        self.assertEqual(outputs[0],outputs[1]);self.assertEqual(outputs[0],outputs[2])

    def test_derivative_uses_measurement_not_reference_jump(self):
        pid=PosePID();pid.step([0,0,0,0],[1,0],10.)
        out=pid.step([0,0,0,0],[0,1],10.05)
        self.assertEqual(out['tracking']['d'],[0.,0.,0.])
        moved=pid.step([.05,.02,0,.01],[0,1],10.1)
        self.assertTrue(all(v<0 for v in moved['tracking']['d']))

    def test_integral_handles_bias_without_indefinite_lookahead_windup(self):
        pid=PosePID()
        for i in range(2000):out=pid.step([0,0,0,0],[1,0],10.+i*.05)
        self.assertAlmostEqual(out['tracking']['i'][0],.16,places=6)
        self.assertGreater(out['vx'],out['tracking']['p'][0])
        pid.reset();out=pid.step([0,0,0,0],[1,0],120.)
        self.assertEqual(out['tracking']['i'],[0.,0.,0.])

    def test_yaw_wrap_does_not_create_derivative_spike(self):
        pid=PosePID();pid.step([0,0,0,math.pi-.001],[-1,0],10.)
        out=pid.step([0,0,0,-math.pi+.001],[-1,0],10.05)
        self.assertLess(abs(out['wz']),.02)

    def test_world_command_is_independent_of_body_orientation(self):
        for yaw in (0.,.7,-1.5,math.pi):
            out=PosePID().step([0,0,0,yaw],[.4,.6],10.)
            c,s=math.cos(yaw),math.sin(yaw)
            self.assertAlmostEqual(c*out['vx']-s*out['vy'],.36)
            self.assertAlmostEqual(s*out['vx']+c*out['vy'],.54)

    def test_stop_clears_pid_memory_and_all_three_outputs(self):
        n=after_i02();s=pose(x=.638732,y=.373606,yaw=-.17426);n.generation=tuple(s['generation'])
        n.step(s,clear(10.),10.);n.pause()
        out=n.step(s,clear(10.05),10.05)
        self.assertEqual([out[k] for k in ('vx','vy','wz')],[0.,0.,0.])
        self.assertIsNone(n.tracker.previous)

    def test_recorded_corner_with_forward_bias_and_weak_yaw_reaches_i03_and_i04(self):
        for bias,yaw_gain in ((0.,1.),(.08,.45),(.12,.35)):
            n=after_i02();x,y,yaw,t=.638732,.373606,-.17426,10.
            actual=[0.,0.,0.];trace=[];i03_time=None
            for _ in range(800):
                s=pose(t,x,y,yaw);n.generation=tuple(s['generation']);n.keepalive(n.run_id,t)
                out=n.step(s,clear(t),t);trace.append(out)
                self.assertNotEqual(out['state'],'HOLD_OFF_ROUTE')
                desired=[out['vx']+(bias if abs(out['wz'])>.001 else 0.),out['vy']*.85,out['wz']*yaw_gain]
                alpha=1-math.exp(-.05/.15)
                actual=[v+alpha*(u-v) for v,u in zip(actual,desired)]
                c,ss=math.cos(yaw),math.sin(yaw)
                x+=(c*actual[0]-ss*actual[1])*.05;y+=(ss*actual[0]+c*actual[1])*.05
                yaw+=actual[2]*.05;t+=.05
                if 3 in n.reached and i03_time is None:i03_time=t-10.
                if out['state']=='COMPLETE':break
            self.assertEqual(n.reached,[2,3,4]);self.assertEqual(n.skipped,[0,1])
            self.assertLess(i03_time,5.);self.assertEqual(out['state'],'COMPLETE')
            self.assertLess(math.hypot(x-.9,y-1.7),.28)


if __name__=='__main__':unittest.main()
