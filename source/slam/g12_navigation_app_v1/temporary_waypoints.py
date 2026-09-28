"""Ordered operator map clicks, separate from the saved route and its progress."""
import copy
import math


class TemporaryWaypoints:
    def __init__(self):
        self.pending=[];self.completed=[];self.revision=0;self.serial=0
        self.terminal_only=False;self.dirty=False
        self.manual_observation=None

    @staticmethod
    def coordinates(value):
        if (not isinstance(value,list) or len(value)!=3 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
            raise ValueError('INVALID_TEMPORARY_POINT')
        return [float(v) for v in value]

    def edit(self,operation,body,terminal_only=False):
        points=copy.deepcopy(self.pending)
        if operation in ('append','update','replace'):
            xyz=self.coordinates(body.get('xyz'))
        if operation in ('append','replace'):
            identity=body.get('request_id')
            if not isinstance(identity,str) or not identity:raise ValueError('INVALID_POINT_REQUEST')
            # Retry of an already visited point must not send the robot back.
            if any(p['id']==identity for p in self.pending+self.completed):return self.state()
            self.serial+=1
            point=dict(id=identity,name=f'临时{self.serial}',xyz=xyz)
            points=[point] if operation=='replace' else [*points,point]
        elif operation=='update':
            found=next((p for p in points if p['id']==body.get('point_id')),None)
            if found is None:raise ValueError('TEMPORARY_POINT_ALREADY_PASSED_OR_REMOVED')
            found['xyz']=xyz
        elif operation=='remove':
            points=[p for p in points if p['id']!=body.get('point_id')]
        elif operation=='clear':points=[]
        else:raise ValueError('UNKNOWN_TEMPORARY_OPERATION')
        if points==self.pending:return self.state()
        self.pending=points;self.revision+=1;self.dirty=True
        self.manual_observation=None
        self.terminal_only=bool(terminal_only)
        return self.state()

    def entry(self,pose,target,index,return_path=None):
        path=[list(pose[:3]),*[list(p['xyz']) for p in self.pending]]
        if not self.terminal_only:path.extend(return_path or [list(target['xyz'])])
        return dict(target_index=index,target_name=target['name'],path=path,leg=1,
            selection='temporary_waypoints',temporary_ids=[p['id'] for p in self.pending],
            temporary_return_index=None if self.terminal_only else index,
            length_m=sum(math.dist(a,b) for a,b in zip(path,path[1:])))

    def observe_manual(self,solution):
        """Keep only still-unvisited temporary points after manual intervention."""
        if not solution or not solution.get('arrival_valid') or not self.pending:return
        t=solution.get('measurement_mono');position=solution.get('measurement_pose',solution.get('pose'))
        if t is None or position is None:return
        previous=self.manual_observation
        generation=tuple(solution.get('generation',[]))
        if previous and (previous[2]!=generation or t<=previous[0]):previous=None
        passed=[]
        for point in self.pending:
            q=point['xyz'];near=math.dist(q[:2],position[:2])<=.25
            if not near and previous:
                a=previous[1];v=[position[i]-a[i] for i in range(2)]
                u=max(0.,min(1.,sum((q[i]-a[i])*v[i] for i in range(2))/max(sum(x*x for x in v),1e-12)))
                near=math.dist(q[:2],[a[i]+u*v[i] for i in range(2)])<=.25
            if not near:break
            passed.append(point['id'])
        if passed:self.passed(passed);self.dirty=True
        self.manual_observation=(t,list(position[:3]),generation)

    def passed(self,identities):
        passed=set(identities)
        arrived=[p for p in self.pending if p['id'] in passed]
        if arrived:
            self.completed.extend(arrived)
            self.pending=[p for p in self.pending if p['id'] not in passed]
            self.revision+=1

    def state(self):
        return copy.deepcopy(dict(pending=self.pending,completed=self.completed,revision=self.revision,
                                  terminal_only=self.terminal_only,serial=self.serial))

    def restore(self,state):
        pending=copy.deepcopy(state.get('pending',[]));completed=copy.deepcopy(state.get('completed',[]))
        seen=set()
        for p in pending+completed:
            p['xyz']=self.coordinates(p['xyz'])
            if not isinstance(p.get('id'),str) or not p['id'] or p['id'] in seen:raise ValueError('INVALID_TEMPORARY_POINT_ID')
            seen.add(p['id'])
        self.pending=pending;self.completed=completed;self.revision=int(state.get('revision',0))
        self.serial=max(len(seen),int(state.get('serial',len(seen))))
        self.terminal_only=bool(state.get('terminal_only'));self.dirty=bool(pending)
