"""Editable edge policies and explicit replans on the frozen route graph."""
import copy
import hashlib
import heapq
import json
import math
import os
import uuid
from pathlib import Path
from route_calibration import RouteCalibration
from route_geometry import edge_points,project_polyline,advance_polyline
from route_recording import RecordedRoutes

OFFICIAL={'basic_normal':'基础普通','stairs_normal':'楼梯普通','platform':'高台','step_move':'踏步移动'}
NATIVE_POLICIES={**OFFICIAL,'basic':'基础 · 敏捷','stairs':'楼梯 · 敏捷'}
LEGACY_PRESETS={'basic':'basic_normal','stairs':'stairs_normal'}
WAYPOINT_POLICIES={'low':'低速 waypoint · 40000','high':'高速 waypoint · 28600',
    'low_v5':'低速 v5 · 45200','high_v2':'高速 v2 · 35400'}
MANUAL={**NATIVE_POLICIES,**WAYPOINT_POLICIES}


def forward_entry(route,minimum,pose,exact_target=None):
    """Connect to the nearest remaining waypoint ahead along its route segment.

    The connector joins the incoming segment before following it to the target;
    this is a local route-entry polyline, not a free-space obstacle detour.
    """
    points=route['waypoints'];origin=list(pose[:3]);candidates=[]
    for i in ([exact_target] if exact_target is not None else range(minimum,len(points))):
        b=points[i]['xyz'];a=points[max(0,i-1)]['xyz']
        if i==0 and len(points)>1:a=b;b=points[1]['xyz']
        if i==0 and route['edges'] and route['edges'][0].get('control_points'):b=route['edges'][0]['control_points'][0]['xyz']
        if i>0 and route['edges'][i-1].get('control_points'):
            chain=edge_points(route,i-1);projection=project_polyline(origin,chain)
            if exact_target is None and projection['index']==len(chain)-2 and projection['raw_u']>1.+1e-6:continue
            candidates.append((math.dist(origin,points[i]['xyz']),i,projection['xyz']))
            continue
        v=[b[k]-a[k] for k in range(3)];length2=sum(x*x for x in v)
        u=sum((origin[k]-a[k])*v[k] for k in range(3))/length2 if length2>1e-12 else 0.
        if exact_target is None and ((i==0 and u>1e-6) or (i>0 and u>1.+1e-6)):continue
        q=list(points[i]['xyz']) if i==0 else [a[k]+max(0.,min(1.,u))*v[k] for k in range(3)]
        candidates.append((math.dist(origin,points[i]['xyz']),i,q))
    if not candidates:
        i=len(points)-1;candidates=[(math.dist(origin,points[i]['xyz']),i,list(points[i]['xyz']))]
    distance,index,join=min(candidates)
    if exact_target is None and index>0 and route['edges'][index-1].get('control_points'):
        chain=edge_points(route,index-1);projection=project_polyline(origin,chain)
        lateral=math.dist(origin[:2],projection['xyz'][:2])
        remainder=advance_polyline(chain,projection,max(.5,lateral))
        path=[origin,*remainder]
        return dict(target_index=index,target_name=points[index]['name'],path=path,leg=1,
            selection='nearest_future',source_edge=index-1,distance_m=distance,
            length_m=sum(math.dist(a,b) for a,b in zip(path,path[1:])))
    target=list(points[index]['xyz']);path=[origin]
    if exact_target is not None:join=target  # Previous route legs are already complete.
    elif index>0:
        # Join ahead of the perpendicular projection. Returning sideways to
        # that projection creates a short backward leg after manual takeover.
        # Stay on this incoming segment and never extend beyond its target.
        lateral=math.dist(origin[:2],join[:2]);remaining=math.dist(join[:2],target[:2])
        if lateral<=.25:join=target
        elif remaining>1e-9:
            fraction=min(1.,max(.5,lateral)/remaining)
            join=[a+fraction*(b-a) for a,b in zip(join,target)]
    if math.dist(origin,join)>.25 and math.dist(join,target)>.25:path.append(join)
    path.append(target)
    return dict(target_index=index,target_name=points[index]['name'],path=path,leg=1,
                selection='manual_target' if exact_target is not None else 'nearest_future',
                source_edge=max(0,index-1),distance_m=distance,
                length_m=sum(math.dist(a,b) for a,b in zip(path,path[1:])))


class Plans:
    def __init__(self,routes,path=None):
        self.sources=copy.deepcopy(routes); self.path=Path(path) if path else None
        self.digest=hashlib.sha256(json.dumps(routes,sort_keys=True).encode()).hexdigest()
        self.recorded=RecordedRoutes(self.path.with_name('recorded-routes.json') if self.path else None,self.digest)
        self.sources.update(copy.deepcopy(self.recorded.data['routes']))
        self.calibration=RouteCalibration(self.sources,self.path.with_name('route-calibration.json') if self.path else None,digest=self.digest)
        self.revision=0; self.presets={k:[e.get('recorded_policy','basic_normal') for e in v['edges']] for k,v in self.sources.items()}
        self.terrain={}
        terrain_path=Path(__file__).parent/'routes/outdoor_terrain_presets.json'
        if terrain_path.exists():
            profile=json.loads(terrain_path.read_text(encoding='utf-8'))
            name=profile['route_id'];source=routes.get(name)
            # Terrain belongs to this exact frozen route; unrelated routes keep
            # their own defaults. Never project labels onto different geometry.
            if source is not None and profile['route_digest']==hashlib.sha256(json.dumps(source,sort_keys=True).encode()).hexdigest():
                edges=profile['edges']
                if len(edges)!=len(source['edges']) or any(e['index']!=i or e['policy'] not in OFFICIAL for i,e in enumerate(edges)):
                    raise ValueError('INVALID_TERRAIN_PRESETS')
                self.terrain[name]=profile
                self.presets[name]=[e['policy'] for e in edges]
        self.preview=None;self.applied_updates=[]
        stored=None
        if self.path and self.path.exists():
            d=json.loads(self.path.read_text(encoding='utf-8'))
            if d.get('digest')!=self.digest: raise ValueError('PRESET_ROUTE_VERSION_MISMATCH')
            original=copy.deepcopy(d['presets'])
            d['presets']={k:[v if k in self.recorded.data['routes'] else LEGACY_PRESETS.get(v,v) for v in values] for k,values in original.items()}
            for name,values in d['presets'].items():
                allowed=MANUAL if name in self.recorded.data['routes'] else OFFICIAL
                if name not in self.presets or len(values)!=len(self.presets[name]) or any(v not in allowed for v in values):
                    raise ValueError('INVALID_STORED_PRESETS')
            self.presets.update(d['presets']);self.revision=d['revision']
            self.applied_updates=list(d.get('applied_updates',[]))
            stored=dict(presets=original,applied_updates=list(self.applied_updates))
        # Explicit route updates apply to existing installations once as well
        # as fresh defaults. Later operator edits are never re-applied over.
        for name,profile in self.terrain.items():
            for update in profile.get('preset_updates',[]):
                key=name+':'+update['id']
                if key in self.applied_updates:continue
                indices=update['segments'];policy=update['policy']
                if policy not in OFFICIAL or any(type(i) is not int or not 0<=i<len(self.presets[name]) for i in indices):
                    raise ValueError('INVALID_ROUTE_PRESET_UPDATE')
                for i in indices:self.presets[name][i]=policy
                self.applied_updates.append(key)
        if stored and (stored['presets']!=self.presets or stored['applied_updates']!=self.applied_updates):
            self.revision+=1;self.save(self.presets,self.revision)

    def save(self,values,revision):
        if not self.path:return
        self.path.parent.mkdir(parents=True,exist_ok=True)
        tmp=self.path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(digest=self.digest,revision=revision,presets=values,
            applied_updates=self.applied_updates),ensure_ascii=False),encoding='utf-8')
        os.replace(tmp,self.path)

    def load_recorded(self,name):
        source=copy.deepcopy(self.recorded.data['routes'][name]);self.sources[name]=source
        self.presets.setdefault(name,[e['recorded_policy'] for e in source['edges']])
        self.preview=None
        return self.route(name)

    def set_policy(self,route,indices,policy):
        if policy not in OFFICIAL: raise ValueError('PRESET_OFFICIAL_ONLY')
        if route not in self.presets or not isinstance(indices,list) or not indices: raise ValueError('INVALID_SEGMENTS')
        if any(type(i) is not int or not 0<=i<len(self.presets[route]) for i in indices): raise ValueError('INVALID_SEGMENTS')
        values=copy.deepcopy(self.presets)
        for i in indices: values[route][i]=policy
        self.save(values,self.revision+1)
        self.presets=values; self.revision+=1; self.preview=None

    def route(self,name):
        result=copy.deepcopy(self.sources[name])
        profile=self.terrain.get(name)
        for i,e in enumerate(result['edges']):
            e.update(source_route=name,source_index=i)
            if profile:
                entry=profile['edges'][i]
                e.update(default_policy=entry['policy'],terrain=copy.deepcopy(entry['terrain']),
                         anticipated_terrain=copy.deepcopy(entry['anticipated_terrain']),
                         scene_arc_m=list(entry['scene_arc_m']))
        if profile:result['display_name']=profile['display_name']
        return self.calibration.route(name,result)

    def policy(self,route,target):
        if not route['edges']: return 'basic_normal'
        e=route['edges'][max(0,min(len(route['edges'])-1,target-1))]
        return self.presets[e['source_route']][e['source_index']]

    def propose(self,name,goal,blocked,solution,now):
        if not solution or not solution.get('arrival_valid') or not 0<=now-solution.get('estimate_mono',-1e9)<=.3:
            raise ValueError('REPLAN_NEEDS_FRESH_LOCALIZATION')
        route=self.route(name); points=route['waypoints']; edges=route['edges']
        if type(goal) is not int or not 0<=goal<len(points): raise ValueError('INVALID_GOAL')
        if not isinstance(blocked,list) or any(type(i) is not int or not 0<=i<len(edges) for i in blocked): raise ValueError('INVALID_BLOCKED_EDGES')
        pose=solution['pose']; candidates=[]; adjacency={}
        # Directed original edges only. Never invent a shortcut across walls/stairs.
        for i,e in enumerate(edges):
            if i in blocked: continue
            chain=edge_points(route,i);projection=project_polyline(pose,chain)
            length=projection['length']
            if length<1e-8: continue
            q=projection['xyz']
            horizontal=math.hypot(pose[0]-q[0],pose[1]-q[1])
            if math.dist(pose[:3],q)<=.35 and horizontal<=min(.35,e.get('corridor_half_width_m',.35)) and abs(pose[2]-q[2])<=.25:
                candidates.append((horizontal+projection['remaining'],i+1,i,q))
            adjacency.setdefault(i,[]).append((i+1,length,i))
        if not candidates: raise ValueError('NO_NEARBY_ROUTE_CORRIDOR_MANUAL_REPOSITION_REQUIRED')
        best=None
        for cost,node,start_edge,q in candidates:
            queue=[(cost,node,[])]; seen=set()
            while queue:
                distance,n,path=heapq.heappop(queue)
                if n in seen: continue
                seen.add(n)
                if n==goal:
                    value=(distance,start_edge,q,[start_edge]+path)
                    if best is None or value[0]<best[0]:best=value
                    break
                for target,length,index in adjacency.get(n,[]):
                    heapq.heappush(queue,(distance+length,target,path+[index]))
        if best is None: raise ValueError('NO_ROUTE_AROUND_BLOCKED_EDGE_OR_REVERSE_GOAL')
        distance,start,q,chosen=best
        planned=copy.deepcopy(route)
        planned['waypoints']=[dict(id='reentry',name='重入路线',xyz=q),*[copy.deepcopy(points[i+1]) for i in chosen]]
        planned['edges']=[copy.deepcopy(edges[i]) for i in chosen]
        if planned['edges'][0].get('control_points'):
            projection=project_polyline(q,edge_points(route,start))
            planned['edges'][0]['control_points']=planned['edges'][0]['control_points'][projection['index']:]
        self.preview=dict(id=str(uuid.uuid4()),route_id=name,revision=self.revision,
            generation=list(solution['generation']),pose=list(pose),created=now,goal_index=goal,
            blocked_edges=list(blocked),source_edges=chosen,skipped_point_indices=list(range(start+1)),
            length_m=distance,route=planned,warnings=sorted({w for e in planned['edges'] for w in e.get('warnings',[])}))
        return copy.deepcopy(self.preview)

    def apply(self,plan_id,solution,now):
        p=self.preview
        if not p or p['id']!=plan_id or p['revision']!=self.revision: raise ValueError('PLAN_EXPIRED_OR_CHANGED')
        if not solution or not solution.get('arrival_valid') or not 0<=now-solution.get('estimate_mono',-1e9)<=.3:
            raise ValueError('REPLAN_NEEDS_FRESH_LOCALIZATION')
        if now-p['created']>60 or list(solution['generation'])!=p['generation']: raise ValueError('PLAN_POSE_EPOCH_CHANGED')
        if math.dist(solution['pose'][:3],p['pose'][:3])>.35: raise ValueError('MOVED_SINCE_PREVIEW_REPLAN')
        self.preview=None
        return copy.deepcopy(p['route'])

    def catalog(self):
        result={}
        for name in self.sources:
            route=self.route(name)
            result[name]=dict(display_name=route.get('display_name',{'indoor':'室内路线','full':'室外路线'}.get(name,name)),waypoints=route['waypoints'],
                edges=[dict(e,policy=self.presets[name][i]) for i,e in enumerate(route['edges'])])
        return result
