"""Continuous-route traversal, feedforward, waypoint passage and end stop."""
import copy,math,unittest
from navigation import Navigator
from trajectory import RouteTrajectory
from test_forward_entry import pose,clear


def route(points):
    return dict(waypoints=[dict(name='P%d'%i,xyz=list(p)) for i,p in enumerate(points)],
        edges=[dict(warnings=[],speed_limit_mps=.1,corridor_half_width_m=.7) for _ in points[1:]])


def run_simulation(r,gain=.65,yaw_gain=.45,bias=.08):
    n=Navigator({'indoor':copy.deepcopy(r)});x,y,z=r['waypoints'][0]['xyz'];yaw=0.;t=10.;actual=[0.,0.,0.]
    initial=pose(t,x,y,yaw);initial['pose'][2]=z;n.start(initial,t)
    outputs=[];nonterminal_zero=[]
    for _ in range(2000):
        sol=pose(t,x,y,yaw);sol['pose'][2]=z;n.keepalive(n.run_id,t)
        out=n.step(sol,clear(t),t);outputs.append(out)
        if out['target_index']<len(r['waypoints'])-1 and not any(out[k] for k in ('vx','vy','wz')):
            nonterminal_zero.append(out)
        demand=[gain*out['vx']+(bias if abs(out['wz'])>.001 else 0.),gain*out['vy'],yaw_gain*out['wz']]
        alpha=1-math.exp(-.05/.15);actual=[v+alpha*(u-v) for v,u in zip(actual,demand)]
        c,s=math.cos(yaw),math.sin(yaw);x+=(c*actual[0]-s*actual[1])*.05;y+=(s*actual[0]+c*actual[1])*.05
        yaw+=actual[2]*.05;t+=.05
        if out['state']=='COMPLETE':break
    return n,outputs,nonterminal_zero,t-10.


class ContinuousTrajectoryTests(unittest.TestCase):
    def test_waypoint_passage_outputs_next_command_without_zero_or_pid_reset(self):
        r=route([[0,0,0],[1,0,0],[2,0,0],[3,0,0]])
        n=Navigator({'indoor':r});n.start(pose(x=.70),10.)
        a=n.step(pose(x=.70),clear(10.),10.);n.keepalive(n.run_id,10.1)
        b=n.step(pose(t=10.1,x=.78),clear(10.1),10.1)
        self.assertEqual(b['target_index'],2);self.assertEqual(b['passed_indices'],[1])
        self.assertGreater(b['vx'],.7);self.assertNotEqual(b['state'],'CONFIRM_WAYPOINT')
        self.assertTrue(n.tracker.history);self.assertGreater(b['tracking']['i'][0],0.)

    def test_dense_straight_waypoints_do_not_create_repeated_speed_dips(self):
        r=route([[i*.5,0,0] for i in range(15)])
        n,outs,zeros,_=run_simulation(r,gain=1.,yaw_gain=1.,bias=0.)
        self.assertEqual(n.reached,list(range(15)));self.assertEqual(zeros,[])
        middle=[o for o in outs if 1<o['target_index']<12]
        self.assertGreater(min(o['vx'] for o in middle),.7)
        self.assertLess(max(abs(b['vx']-a['vx']) for a,b in zip(middle,middle[1:])),.05)

    def test_path_reference_does_not_depend_on_collinear_waypoint_density(self):
        sparse=RouteTrajectory([[0,0,0],[5,0,0]])
        dense=RouteTrajectory([[i*.5,0,0] for i in range(11)])
        a=sparse.reference([1.3,.1,0,0],0);b=dense.reference([1.3,.1,0,0],2)
        for key in ('progress_m','reference_speed_mps','reference_arc_m'):
            self.assertAlmostEqual(a[key],b[key],places=5)
        for x,y in zip(a['velocity'],b['velocity']):self.assertAlmostEqual(x,y,places=5)

    def test_geometry_is_tangent_continuous_at_corner(self):
        p=RouteTrajectory([[0,0,0],[1,0,0],[1,1,0],[2,1,0]])
        corner=p.arcs[p.bounds[0][1]]
        left,right=p.at(corner-1e-5),p.at(corner+1e-5)
        self.assertLess(abs(left['yaw']-right['yaw']),.001)
        self.assertLess(math.dist(left['xyz'],right['xyz']),.0001)

    def test_right_angle_and_weak_execution_pass_all_points_without_dwell(self):
        r=route([[0,0,0],[.5,0,0],[1,0,0],[1,.5,0],[1,1,0],[1.5,1,0],[2,1,0]])
        for gain,yaw_gain,bias in ((1.,1.,0.),(.65,.45,.08)):
            n,outs,zeros,_=run_simulation(r,gain,yaw_gain,bias)
            self.assertEqual(n.reached,list(range(7)));self.assertEqual(zeros,[])
            self.assertNotIn('HOLD_OFF_ROUTE',[o['state'] for o in outs])
            self.assertTrue(any(abs(o['vy'])>.1 and abs(o['wz'])>.2 for o in outs))

    def test_valid_sensor_recovery_has_no_extra_dwell(self):
        n=Navigator({'indoor':route([[0,0,0],[1,0,0],[2,0,0]])});n.start(pose(x=.3),10.)
        p=clear(10.);p['front']['blocked']=True
        self.assertEqual(n.step(pose(x=.3),p,10.)['state'],'HOLD_OBSTACLE')
        out=n.step(pose(t=10.05,x=.3),clear(10.05),10.05)
        self.assertEqual(out['state'],'JOINING_ROUTE');self.assertGreater(out['vx'],0.)

    def test_prediction_alone_does_not_mark_passage(self):
        n=Navigator({'indoor':route([[0,0,0],[1,0,0],[2,0,0]])});n.start(pose(x=.5),10.)
        sol=pose(t=10.05,x=.85);sol.update(mode='PREDICT_ONLY',arrival_valid=False)
        out=n.step(sol,clear(10.05),10.05)
        self.assertEqual(n.target,1);self.assertEqual(out['passed_indices'],[])
        self.assertGreater(out['vx'],0.)

    def test_measurements_straddling_a_waypoint_do_not_make_robot_return_to_it(self):
        n=Navigator({'indoor':route([[0,0,0],[1,0,0],[2,0,0]])});n.start(pose(x=.7),10.)
        n.step(pose(x=.7),clear(10.),10.)
        out=n.step(pose(t=10.5,x=1.3),clear(10.5),10.5)
        self.assertEqual(n.target,2);self.assertEqual(out['passed_indices'],[1])
        self.assertGreater(out['vx'],0.)

    def test_final_waypoint_stops_and_requires_actual_map_arrival(self):
        n=Navigator({'indoor':route([[0,0,0],[1,0,0]])});n.start(pose(x=.5),10.)
        for t in (10.1,10.2,10.3):
            sol=pose(t=t,x=.9);sol.update(mode='ODOM_BRIDGE',arrival_valid=False)
            out=n.step(sol,clear(t),t)
            self.assertEqual([out[k] for k in ('vx','vy','wz')],[0.,0.,0.])
            self.assertTrue(n.active)
        for t in (10.4,10.52,10.64):out=n.step(pose(t=t,x=.9),clear(t),t)
        self.assertEqual(out['state'],'COMPLETE');self.assertFalse(n.active)

    def test_future_rejected_edge_is_not_entered_by_preview(self):
        r=route([[0,0,0],[1,0,0],[2,0,0]]);r['edges'][1]['warnings']=['unverified']
        n=Navigator({'indoor':r});n.start(pose(x=.70),10.)
        out=n.step(pose(x=.70),clear(10.),10.)
        self.assertLessEqual(out['tracking']['trajectory']['xyz'][0],1.)
        out=n.step(pose(t=10.05,x=.8),clear(10.05),10.05)
        self.assertEqual(out['state'],'HOLD_ROUTE_VALIDATION');self.assertEqual(out['vx'],0.)

    def test_current_warning_cannot_be_skipped_by_passage(self):
        r=route([[0,0,0],[1,0,0],[2,0,0]]);r['edges'][0]['warnings']=['unverified']
        n=Navigator({'indoor':r});n.start(pose(x=.8),10.)
        out=n.step(pose(x=.8),clear(10.),10.)
        self.assertEqual(n.target,1);self.assertEqual(out['state'],'HOLD_ROUTE_VALIDATION')

    def test_duplicate_waypoint_is_passed_once_and_commands_remain_finite(self):
        r=route([[0,0,0],[0,0,0],[1,0,0],[2,0,0]])
        n,outs,_,_=run_simulation(r,gain=1.,yaw_gain=1.,bias=0.)
        self.assertEqual(n.reached,[0,1,2,3])
        self.assertTrue(all(math.isfinite(o[k]) for o in outs for k in ('vx','vy','wz')))

    def test_corner_passage_uses_path_progress_when_robot_misses_arrival_sphere(self):
        r=route([[0,0,0],[1,0,0],[1,1,0],[1,2,0],[2,2,0]])
        n=Navigator({'indoor':r});n.start(pose(x=.5),10.)
        # Follow the bend with a 40-cm lateral tracking error. No measurement
        # (or chord between measurements) enters P1's 25-cm arrival sphere.
        for t,x,y in ((10.,.5,-.4),(10.1,.85,-.4),(10.2,1.4,-.15),(10.3,1.4,.3),(10.4,1.4,.8)):
            n.keepalive(n.run_id,t);out=n.step(pose(t=t,x=x,y=y),clear(t),t)
        self.assertIn(1,n.reached);self.assertGreaterEqual(n.target,2)
        self.assertEqual(out['state'],'FOLLOWING');self.assertGreater(out['vy'],0.)
        self.assertLess(out['vx'],0.)  # PID corrects back toward the route.

    def test_projection_cannot_skip_a_later_parallel_route_leg(self):
        r=route([[0,0,0],[2,0,0],[2,2,0],[0,2,0],[0,.1,0],[2,.1,0]])
        n=Navigator({'indoor':r});n.start(pose(x=.3),10.)
        out=n.step(pose(t=10.1,x=.4,y=.1),clear(10.1),10.1)
        self.assertEqual(n.target,1);self.assertEqual(out['passed_indices'],[])

    def test_height_is_diagnostic_and_lateral_error_does_not_stop_tracking(self):
        for y,z in ((.8,0.),(.1,.5)):
            n=Navigator({'indoor':route([[0,0,0],[1,0,0],[2,0,0],[3,0,0]])})
            n.start(pose(x=.5),10.)
            sol=pose(t=10.1,x=1.4,y=y);sol['pose'][2]=z
            out=n.step(sol,clear(10.1),10.1)
            self.assertEqual(n.target,1 if y>.7 else 2)
            self.assertNotEqual(out['state'],'HOLD_OFF_ROUTE');self.assertGreater(out['vx'],0.)


if __name__=='__main__':unittest.main()
