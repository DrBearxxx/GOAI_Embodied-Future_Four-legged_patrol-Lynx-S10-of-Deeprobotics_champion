"""Ordered 3D route / planar velocity closed loop, without robot I/O."""
import math
import uuid
from planning import forward_entry
from pose_pid import PosePID,HeadingReference
from trajectory import RouteTrajectory
from temporary_waypoints import TemporaryWaypoints
from obstacle_evidence import policy_obstacles


def clip(x,low,high):return max(low,min(high,x))


def blocking_warnings(edge):
    # A height-derived terrain classification is informational, not a stop or
    # an operator confirmation. Keep the original annotation in route data.
    return [w for w in edge.get('warnings',[]) if w!='slope_or_stairs_requires_terrain_validation']


class Navigator:
    def __init__(self,routes):
        self.routes=routes;self.route_id='indoor';self.route=routes[self.route_id]
        self.target=0;self.reached=[];self.active=False;self.execution=False
        self.generation=None;self.run_id=str(uuid.uuid4());self.last_tick=None
        self.speed=0.;self.turn=0.;self.arrivals=[];self.hold_since=None;self.good_since=None
        self.reason='IDLE';self.operator_until=0.;self.last={};self.seq=0
        self.entry=None;self.skipped=[]
        self.manual_target=None;self.manual_progress=None
        self.cruise_mps=2.0
        self.tracker=PosePID()
        self.heading_reference=HeadingReference()
        self.trajectory=None;self.trajectory_key=None;self.trajectory_progress=0.
        self.previous_measured_pose=None
        self.temporary=TemporaryWaypoints()

    def pause(self,reason='PAUSED'):
        self.active=False;self.speed=self.turn=0.;self.arrivals=[];self.reason=reason
        self.tracker.reset()
        self.heading_reference.reset()
        self.trajectory_key=None;self.trajectory_progress=0.
        self.previous_measured_pose=None

    def select(self,name):
        if name not in self.routes:raise ValueError('UNKNOWN_ROUTE')
        self.pause('ROUTE_SELECTED');self.route_id=name;self.route=self.routes[name]
        self.target=0;self.reached=[];self.generation=None;self.run_id=str(uuid.uuid4())
        self.entry=None;self.skipped=[]
        self.manual_target=None;self.manual_progress=None
        self.temporary=TemporaryWaypoints()

    def set_progress(self,index,request_id,now):
        if type(index) is not int or not 0<=index<len(self.route['waypoints']):raise ValueError('INVALID_PROGRESS_TARGET')
        self.temporary=TemporaryWaypoints()
        self.pause('PROGRESS_SET');self.target=index;self.entry=None
        self.reached=list(range(index));self.skipped=[]
        self.manual_target=index
        self.manual_progress=dict(target_index=index,target_name=self.route['waypoints'][index]['name'],
            manually_completed_indices=list(self.reached),request_id=request_id,mono=now)
        self.generation=None;self.previous_measured_pose=None
        self.run_id=str(uuid.uuid4())

    def start(self,solution,now,execute=False):
        if not solution or not solution.get('arrival_valid'):raise ValueError('FRESH_ODOMETRY_ALIGNMENT_REQUIRED')
        if self.reason=='COMPLETE' and not self.temporary.pending:raise ValueError('SELECT_ROUTE_TO_RESTART')
        minimum=max(self.target,max(self.reached,default=-1)+1)
        if minimum>=len(self.route['waypoints']) and not self.temporary.pending:raise ValueError('SELECT_ROUTE_TO_RESTART')
        self.reconnect(solution)
        self.run_id=str(uuid.uuid4())
        self.active=True;self.execution=execute
        self.operator_until=now+1.5;self.hold_since=None;self.good_since=None;self.reason='RUNNING'

    def edit_temporary(self,operation,body,solution):
        terminal=self.temporary.terminal_only or (self.target==len(self.route['waypoints'])-1 and self.target in self.reached)
        self.temporary.edit(operation,body,terminal_only=terminal)
        if self.temporary.dirty:
            if self.temporary.terminal_only and not self.temporary.pending:
                self.pause('COMPLETE');self.entry=None;self.temporary.dirty=False
            elif solution and solution.get('pose'):
                self.refresh_temporary_entry(solution)
        return self.temporary.state()

    def refresh_temporary_entry(self,solution):
        target=self.route['waypoints'][self.target]
        if self.temporary.pending:
            index=self.target;return_path=None
            if not self.temporary.terminal_only:
                returning=forward_entry(self.route,self.target,self.temporary.pending[-1]['xyz'],exact_target=self.manual_target)
                index=returning['target_index'];target=self.route['waypoints'][index]
                return_path=returning['path'][1:]
            self.entry=self.temporary.entry(solution['pose'],target,index,return_path)
        else:
            self.connect_remaining_route(solution['pose'])
        self.temporary.dirty=False
        self.trajectory_key=None;self.trajectory_progress=0.;self.previous_measured_pose=None
        self.tracker.reset();self.arrivals=[]

    def connect_remaining_route(self,pose):
        # Resume and cancellation use the same forward route entry. Only an
        # explicit manual progress target can require revisiting a point behind.
        minimum=max(self.target,max(self.reached,default=-1)+1)
        self.entry=forward_entry(self.route,minimum,pose,exact_target=self.manual_target)
        self.target=self.entry['target_index']
        self.skipped=sorted(set(self.skipped).union(range(minimum,self.target)))

    def reconnect(self,solution):
        if self.temporary.pending:
            self.refresh_temporary_entry(solution)
        else:
            self.connect_remaining_route(solution['pose'])
        # A changed map alignment invalidates heading history for both kinds
        # of route entry. The next reference starts at the new measured yaw.
        self.generation=tuple(solution['generation']);self.arrivals=[]
        self.speed=self.turn=0.;self.tracker.reset();self.heading_reference.reset()
        self.trajectory_key=None;self.trajectory_progress=0.;self.previous_measured_pose=None

    def keepalive(self,run_id,now):
        if self.active and run_id==self.run_id:self.operator_until=now+1.5

    def path(self):
        points=self.route['waypoints']
        future_target=self.target
        if self.entry:
            future_target=self.entry.get('temporary_return_index')
            if future_target is None:future_target=self.target
            track_points=[*self.entry['path'],*[point['xyz'] for point in points[future_target+1:]]]
            controls=[[] for _ in self.entry['path'][1:]]+[
                [p['xyz'] for p in e.get('control_points',[])] for e in self.route['edges'][future_target:]]
            segment=self.entry['leg']-1;first_future=len(self.entry['path'])-1
        else:
            start=(self.manual_progress or {}).get('target_index',0)
            track_points=[point['xyz'] for point in points[start:]];segment=max(0,self.target-1-start)
            controls=[[p['xyz'] for p in e.get('control_points',[])] for e in self.route['edges'][start:]]
            first_future=self.target-start
        if self.temporary.terminal_only:
            track_points=list(self.entry['path']);controls=[[] for _ in track_points[1:]]
        for index in ([] if self.temporary.terminal_only else range(future_target,len(self.route['edges']))):
            if blocking_warnings(self.route['edges'][index]):
                track_points=track_points[:first_future+(index-future_target)+1];break
        controls=controls[:max(0,len(track_points)-1)]
        key=(self.cruise_mps,tuple(tuple(point) for point in track_points),tuple(tuple(tuple(p) for p in c) for c in controls))
        if key!=self.trajectory_key:
            self.trajectory=RouteTrajectory(track_points,self.cruise_mps,control_points=controls);self.trajectory_key=key;self.trajectory_progress=0.
        return self.trajectory,segment

    def passed_on_path(self,pose):
        # Passage is measured progress along the same continuous path used by
        # PID. Missing a vertex's small arrival sphere must not pin the target
        # while feedforward already follows the outgoing curve.
        if self.target==0 or (self.entry and self.entry['leg']<len(self.entry['path'])-1):return False
        trajectory,segment=self.path()
        projection=trajectory.project(pose,segment,next_segments=1)
        boundary=trajectory.arcs[trajectory.bounds[segment][1]]
        edge=self.route['edges'][self.target-1]
        following=self.route['edges'][self.target] if self.target<len(self.route['edges']) else edge
        return (projection['arc_m']>boundary+1e-6 and
                projection['cross_track_m']<=min(.7,edge.get('corridor_half_width_m',.7),following.get('corridor_half_width_m',.7)))

    def step(self,solution,perception,now,pending=False,control_ready=True,control_hold=None,policy_for_target=None):
        self.last_tick=now
        if self.temporary.dirty and solution and solution.get('pose'):
            self.refresh_temporary_entry(solution)
        self.seq+=1;points=self.route['waypoints'];target=points[self.target]
        terminal_temporary=bool(self.temporary.terminal_only and self.temporary.pending)
        if terminal_temporary:target=self.temporary.pending[-1]
        out=dict(schema='s10.navigation.v2',run_id=self.run_id,seq=self.seq,mono=now,
            route_id=self.route_id,target_index=self.target,target_name=target['name'],
            target_xyz=target['xyz'],reached_indices=list(self.reached),skipped_indices=list(self.skipped),passed_indices=[],active=self.active,
            manual_target=self.manual_target,manual_progress=self.manual_progress,
            execution_requested=self.execution,vx=0.,vy=0.,wz=0.,state=self.reason,
            speed_limit_mps=None,cruise_mps=self.cruise_mps,control_range_source='native_policy',reason='',motion_authorized=False,
            entry=None if self.entry is None else dict(self.entry,path=[list(p) for p in self.entry['path']]))
        reason=None
        if not self.active:reason=self.reason
        elif now>self.operator_until:self.pause('OPERATOR_LINK_EXPIRED');reason=self.reason
        elif pending:self.pause('RELOCALIZATION_REQUESTED');reason=self.reason
        elif (not solution or solution.get('mode') not in ('TRACKING','DEGRADED','ODOM_BRIDGE','PREDICT_ONLY')
              or not 0<=now-solution.get('estimate_mono',-1e9)<=.25):reason='HOLD_LOCALIZATION'
        elif not control_hold and tuple(solution['generation'])!=self.generation:
            # A local odom reset is not a change to the frozen route/map.
            # Keep the mission alive while it obtains a fresh map alignment,
            # then reconnect to the remaining route in that map frame.
            if not solution.get('arrival_valid'):reason='HOLD_LOCALIZATION'
            else:
                self.reconnect(solution);target=self.temporary.pending[-1] if terminal_temporary else points[self.target]
                out.update(target_index=self.target,target_name=target['name'],target_xyz=target['xyz'],
                    skipped_indices=list(self.skipped),entry=dict(self.entry,path=[list(p) for p in self.entry['path']]),
                    reconnected_after_localization=True)
        if reason is None and self.execution and not control_ready:self.pause('CONTROL_LINK_LOST');reason=self.reason
        if reason is None and control_hold:reason=control_hold
        records=perception or {}
        fresh={k:v for k,v in records.items() if v.get('valid') and 0<=now-v['mono']<=.40}
        if reason is None:
            if 'front' not in fresh:reason='HOLD_FORWARD_PERCEPTION'
        p=solution['pose'] if solution else None
        passed=[]
        current_edge=self.route['edges'][max(0,self.target-1)] if self.route['edges'] else {}
        if reason is None and blocking_warnings(current_edge):
            reason='HOLD_ROUTE_VALIDATION';out['reason']=';'.join(blocking_warnings(current_edge))
        # Intermediate waypoints are passage markers. Measured continuous
        # odometry can establish passage without waiting for three map fits.
        # Prediction alone does not establish physical passage.
        if reason is None and solution.get('mode')!='PREDICT_ONLY':
            if self.entry and (not self.entry.get('temporary_ids') or solution.get('arrival_valid')):
                # A join is a trajectory marker, like every other waypoint.
                # Passing beside it must not pin the entry leg behind the robot.
                entry_pose=solution.get('measurement_pose',p) if self.entry.get('temporary_ids') else p
                while self.entry['leg']<len(self.entry['path'])-1:
                    trajectory,segment=self.path()
                    projection=trajectory.project(entry_pose,segment,next_segments=1)
                    boundary=trajectory.arcs[trajectory.bounds[segment][1]]
                    point=self.entry['path'][self.entry['leg']]
                    if (math.dist(entry_pose[:2],point[:2])>.25 and
                            not (projection['arc_m']>boundary and projection['cross_track_m']<=.7)):break
                    self.entry['leg']+=1
                out['entry']['leg']=self.entry['leg']
                self.temporary.passed(self.entry.get('temporary_ids',[])[:self.entry['leg']-1])
                returning=self.entry.get('temporary_return_index')
                if not self.temporary.pending and returning is not None and returning>self.target:
                    self.skipped=sorted(set(self.skipped).union(range(self.target,returning)))
                    self.target=returning;out['skipped_indices']=list(self.skipped)
            def passed_point(point):
                if math.dist(p[:2],point[:2])<=.25:return True
                if self.previous_measured_pose is None:return False
                a=self.previous_measured_pose;v=[p[i]-a[i] for i in range(2)]
                u=clip(sum((point[i]-a[i])*v[i] for i in range(2))/max(sum(x*x for x in v),1e-12),0.,1.)
                return math.dist([a[i]+u*v[i] for i in range(2)],point[:2])<=.25
            while not terminal_temporary and self.target<len(points)-1 and (passed_point(points[self.target]['xyz']) or
                    (self.manual_target!=self.target and self.passed_on_path(p))):
                if self.entry and self.entry['leg']<len(self.entry['path'])-1:break
                edge=self.route['edges'][max(0,self.target-1)] if self.route['edges'] else {}
                if blocking_warnings(edge):break
                self.reached.append(self.target);passed.append(self.target)
                if self.manual_target==self.target:self.manual_target=None
                self.target+=1;self.entry=None;self.arrivals=[]
            self.previous_measured_pose=list(p[:3])
            target=self.temporary.pending[-1] if terminal_temporary else points[self.target]
            out.update(target_index=self.target,target_name=target['name'],target_xyz=target['xyz'],
                reached_indices=list(self.reached),passed_indices=passed,manual_target=self.manual_target,
                entry=None if self.entry is None else out['entry'])
        dist=None;u=0.;a=b=None
        if p is not None:
            b=target['xyz'];dist=math.dist(p[:2],b[:2]);out['distance_to_target_m']=dist
        if reason is None:
            if self.entry:
                path=self.entry['path'];leg=self.entry['leg']
                # Intermediate join points are fly-through points, not stops.
                while not self.entry.get('temporary_ids') and leg<len(path)-1 and math.dist(p[:2],path[leg][:2])<=.25:leg+=1
                self.entry['leg']=leg;out['entry']['leg']=leg
                a=path[leg-1];b=path[leg]
            elif self.target>0:a=points[self.target-1]['xyz']
            if a is not None:
                trajectory,segment=self.path();projection=trajectory.project(p,segment)
                lateral=projection['cross_track_m'];height=projection['height_error_m']
                out.update(cross_track_m=lateral,height_error_m=height)
                edge=self.route['edges'][max(0,self.target-1)] if self.route['edges'] else {}
                out['outside_route_corridor']=lateral>edge.get('corridor_half_width_m',.7)
                if blocking_warnings(edge):reason='HOLD_ROUTE_VALIDATION';out['reason']=';'.join(blocking_warnings(edge))
        if reason is None:
            policy=policy_for_target(self.target) if policy_for_target else 'basic_normal'
            evidence={k:policy_obstacles(v,policy) for k,v in records.items() if 0<=now-v['mono']<=.6}
            out['obstacle_assessment']=dict(policy=policy,sensors=evidence)
            if any(v['blocked'] for v in evidence.values()):reason='HOLD_OBSTACLE'
        if reason is not None:
            self.speed=self.turn=0.;self.arrivals=[];self.good_since=None
            self.tracker.reset()
            self.heading_reference.reset()
            if self.active and self.hold_since is None:self.hold_since=now
            self.last={**out,'state':reason,'active':self.active};return self.last
        # Resume on valid evidence; no additional fixed recovery dwell.
        self.hold_since=self.good_since=None
        arrival_distance=math.dist(solution.get('measurement_pose',p)[:2],target['xyz'][:2]) if terminal_temporary else dist
        if self.target==len(points)-1 and arrival_distance<=.25 and (not self.entry or self.entry['leg']==len(self.entry['path'])-1):
            self.speed=self.turn=0.
            self.tracker.reset()
            self.heading_reference.reset()
            t=solution['measurement_mono']
            if solution.get('arrival_valid'):
                if not self.arrivals or t>self.arrivals[-1]:self.arrivals.append(t)
                self.arrivals=[v for v in self.arrivals if now-v<1.]
            else:self.arrivals=[]
            if len(self.arrivals)>=3 and self.arrivals[-1]-self.arrivals[0]>=.2:
                if terminal_temporary:self.temporary.passed([p['id'] for p in self.temporary.pending])
                else:self.reached.append(self.target)
                self.arrivals=[];self.entry=None
                self.manual_target=None;out['manual_target']=None
                if self.target==len(points)-1:self.pause('COMPLETE')
                else:self.target+=1
            self.last={**out,'state':'COMPLETE' if self.reason=='COMPLETE' else 'CONFIRM_WAYPOINT',
                       'active':self.active,'reached_indices':list(self.reached)};return self.last
        self.arrivals=[]
        trajectory,segment=self.path()
        reference=trajectory.reference(p,segment,self.trajectory_progress)
        self.trajectory_progress=reference['progress_m']
        # Cross-track correction also steers heading. Stair policies have weak
        # lateral response: a purely sideways PID correction cannot bring them
        # back to the route. This continuous vector-field reference couples
        # forward travel and yaw without a turn-only phase or velocity clamps.
        projection=trajectory.project(p,segment)
        tangent=[math.cos(reference['yaw']),math.sin(reference['yaw'])]
        error=[projection['xyz'][i]-p[i] for i in range(2)]
        along=sum(a*b for a,b in zip(error,tangent))
        cross=[error[i]-along*tangent[i] for i in range(2)]
        heading_vector=[reference['velocity'][i]+self.tracker.kp[i]*cross[i] for i in range(2)]
        heading_target=math.atan2(heading_vector[1],heading_vector[0]) if math.hypot(*heading_vector)>1e-9 else reference['yaw']
        yaw,yaw_rate=self.heading_reference.step(heading_target,p[3],now)
        velocity=[*reference['velocity'][:2],yaw_rate]
        command=self.tracker.step(p,reference['xyz'],now,yaw,velocity,
            solution.get('measurement_mono',solution['estimate_mono']),solution.get('measurement_source'),
            follow_path=True,measurement_pose=solution.get('measurement_pose'),
            measurement_frame=dict(map_to_odom=solution['map_to_odom'],odom_T=solution['odom_T'])
                if solution.get('map_to_odom') is not None and solution.get('odom_T') is not None else None)
        command['tracking']['heading_trajectory']=dict(target_yaw=heading_target,path_yaw=reference['yaw'],cross_track_vector=cross,yaw=yaw,rate=yaw_rate,
            bandwidth=self.heading_reference.bandwidth)
        command['tracking']['trajectory']=reference
        self.speed=command['vx'];self.turn=command['wz']
        self.last={**out,**command,'state':'JOINING_ROUTE' if self.entry else 'FOLLOWING',
                   'motion_authorized':self.execution}
        return self.last
