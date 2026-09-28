"""Versioned route calibration overlay. Source markers and presets stay intact."""
import copy,hashlib,json,math,os,tempfile,time,uuid
from pathlib import Path


def xyz(value):
    if not isinstance(value,list) or len(value)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in value):
        raise ValueError('坐标必须是三个有限数值 X/Y/Z')
    return [float(v) for v in value]


class RouteCalibration:
    def __init__(self,sources,path=None,digest=None):
        self.sources=sources;self.path=Path(path) if path else None
        self.digest=digest or hashlib.sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest()
        self.data=dict(schema='goai.route-calibration.v1',digest=self.digest,revision=0,active={},draft={},undo=[],versions=[])
        if self.path and self.path.exists():
            self.data=json.loads(self.path.read_text(encoding='utf-8'))
            if self.data['digest']!=self.digest:raise ValueError('CALIBRATION_ROUTE_VERSION_MISMATCH')
            self.validate(self.data['active']);self.validate(self.data['draft'])

    def point_id(self,point,index):return str(point.get('id',index))

    def validate(self,edits):
        for name,edit in edits.items():
            if name not in self.sources:raise ValueError('UNKNOWN_CALIBRATION_ROUTE')
            points=self.sources[name]['waypoints'];ids={self.point_id(p,i) for i,p in enumerate(points)}
            for pid,p in edit.get('points',{}).items():
                if pid not in ids:raise ValueError('UNKNOWN_CALIBRATION_POINT')
                xyz(p['xyz'])
            seen=set(ids)
            for edge,controls in edit.get('controls',{}).items():
                if not str(edge).isdigit() or not 0<=int(edge)<len(points)-1:raise ValueError('UNKNOWN_CALIBRATION_EDGE')
                for p in controls:
                    if not isinstance(p.get('id'),str) or p['id'] in seen:raise ValueError('DUPLICATE_CALIBRATION_POINT')
                    seen.add(p['id']);xyz(p['xyz'])

    def write(self,value):
        self.validate(value['draft']);self.validate(value['active'])
        if self.path:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            fd,temp=tempfile.mkstemp(prefix=self.path.name+'.',dir=self.path.parent)
            try:
                with os.fdopen(fd,'w',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False);f.flush();os.fsync(f.fileno())
                os.replace(temp,self.path)
            finally:
                if Path(temp).exists():Path(temp).unlink()
        self.data=value

    def route(self,name,route,draft=False):
        result=copy.deepcopy(route);edit=self.data['draft' if draft else 'active'].get(name,{})
        if not edit:return result
        points=result['waypoints']
        for i,p in enumerate(points):
            value=edit.get('points',{}).get(self.point_id(p,i))
            if value:p['original_xyz']=list(p['xyz']);p['xyz']=list(value['xyz']);p['calibrated']=True
        total=0.
        for i,e in enumerate(result['edges']):
            controls=copy.deepcopy(edit.get('controls',{}).get(str(i),e.get('control_points',[])))
            if controls or 'control_points' in e:e['control_points']=controls
            chain=[points[i]['xyz'],*[p['xyz'] for p in controls],points[i+1]['xyz']]
            length=sum(math.dist(a,b) for a,b in zip(chain,chain[1:]))
            e.update(length_m=length,start_s_m=total,end_s_m=total+length);total+=length
            a,b=chain[0],chain[1];points[i]['yaw']=math.atan2(b[1]-a[1],b[0]-a[0])
        if len(points)>1:points[-1]['yaw']=points[-2]['yaw']
        result['length_m']=total;result['calibration_revision']=self.data['revision']
        return result

    def state(self,name):
        if name not in self.sources:raise ValueError('UNKNOWN_ROUTE')
        route=self.route(name,self.sources[name],draft=True);items=[]
        for i,p in enumerate(route['waypoints']):
            item=dict(id=self.point_id(p,i),name=p['name'],xyz=p['xyz'],original_xyz=self.sources[name]['waypoints'][i]['xyz'],
                      kind='waypoint',index=i,modified=bool(p.get('calibrated')))
            items.append(item)
            if i<len(route['edges']):
                for j,c in enumerate(route['edges'][i].get('control_points',[])):
                    items.append(dict(c,kind='control',edge_index=i,control_index=j,modified=True))
        return dict(route=name,revision=self.data['revision'],dirty=self.data['draft']!=self.data['active'],items=items,
            can_undo=bool(self.data['undo']),can_restore=bool(self.data['versions']),
            modified_points=sum(bool(p.get('calibrated')) for p in route['waypoints']),
            control_points=sum(len(e.get('control_points',[])) for e in route['edges']))

    def prepare(self,body,solution,now):
        name=body.get('route')
        state=self.state(name)
        if body.get('revision')!=self.data['revision']:raise ValueError('校准草稿已更新，请刷新后重试')
        action=body['action'].removeprefix('calibration_');value=copy.deepcopy(self.data);selected=body.get('point_id')
        before=copy.deepcopy(value['draft'])
        edit=value['draft'].setdefault(name,dict(points={},controls={}))
        item=next((p for p in state['items'] if p['id']==selected),None)
        if item and item['kind']=='control':
            edge=item['edge_index']
            edit['controls'].setdefault(str(edge),copy.deepcopy(self.sources[name]['edges'][edge].get('control_points',[])))
        capture=None
        if action in ('record','insert'):
            pose=(solution or {}).get('measurement_pose') or (solution or {}).get('pose')
            if pose is None or (solution or {}).get('mode') in ('LOST','STALE','WAIT_SEED','RELOCALIZING'):
                raise ValueError('当前没有可用实测定位，请先重定位；也可直接编辑 XYZ')
            coords=xyz(list(pose[:3]));stamp=solution.get('measurement_mono',solution.get('estimate_mono'))
            capture=dict(method='measured_robot_pose',measurement_mono=stamp,sample_age_s=None if stamp is None else max(0.,now-stamp),
                         generation=solution.get('generation'),mode=solution.get('mode'),wall_time=time.time())
        if action in ('record','set','offset','insert','reset_point') and item is None:raise ValueError('请选择需要校准的点')
        if action=='set':coords=xyz(body.get('xyz'));capture=dict(method='coordinates',wall_time=time.time())
        if action=='offset':
            distance=body.get('distance');axis=body.get('axis')
            if type(distance) not in (float,int) or not math.isfinite(distance) or axis not in ('left','forward'):raise ValueError('INVALID_CALIBRATION_OFFSET')
            items=state['items'];i=items.index(item)
            a,b=(items[i],items[i+1]) if i+1<len(items) else (items[i-1],items[i])
            yaw=math.atan2(b['xyz'][1]-a['xyz'][1],b['xyz'][0]-a['xyz'][0])+(math.pi/2 if axis=='left' else 0)
            coords=[item['xyz'][0]+distance*math.cos(yaw),item['xyz'][1]+distance*math.sin(yaw),item['xyz'][2]]
            capture=dict(method='route_relative_offset',axis=axis,distance=distance,wall_time=time.time())
        if action in ('record','set','offset'):
            change=dict(xyz=coords,capture=capture)
            if item['kind']=='waypoint':edit['points'][selected]=change
            else:edit['controls'][str(item['edge_index'])][item['control_index']].update(change)
        elif action=='insert':
            edge=item['index'] if item['kind']=='waypoint' else item['edge_index']
            if edge>=len(self.sources[name]['edges']):raise ValueError('终点后没有路段，请选择前一个点')
            controls=edit['controls'].setdefault(str(edge),copy.deepcopy(self.sources[name]['edges'][edge].get('control_points',[])));at=0 if item['kind']=='waypoint' else item['control_index']+1
            selected='correction-'+uuid.uuid4().hex[:12]
            controls.insert(at,dict(id=selected,name='校正 '+selected[-4:],xyz=coords,capture=capture))
        elif action=='reset_point':
            if item['kind']=='waypoint':edit['points'].pop(selected,None)
            else:
                edit['controls'][str(item['edge_index'])].pop(item['control_index'])
                selected=self.point_id(self.sources[name]['waypoints'][item['edge_index']],item['edge_index'])
        elif action=='reset_route':value['draft'].pop(name,None)
        elif action=='undo':
            if value['undo']:value['draft']=value['undo'].pop()
        elif action=='discard':value['draft']=copy.deepcopy(value['active']);value['undo']=[]
        elif action=='previous':
            if not value['versions']:raise ValueError('还没有已应用的历史版本')
            value['draft']=copy.deepcopy(value['versions'][-1]['routes'])
        elif action not in ('record','set','offset'):raise ValueError('UNKNOWN_CALIBRATION_ACTION')
        # Avoid empty edits falsely appearing as a pending change.
        for rid,e in list(value['draft'].items()):
            e['controls']={k:v for k,v in e.get('controls',{}).items() if v or self.sources[rid]['edges'][int(k)].get('control_points')}
            if not e.get('points') and not e['controls']:value['draft'].pop(rid)
        if action not in ('undo','discard') and value['draft']!=before:
            value['undo']=(value['undo']+[before])[-40:]
        value['revision']+=1;self.write(value)
        return dict(calibration=self.state(name),selected_point=selected,capture=capture)

    def apply(self,body):
        if body.get('revision')!=self.data['revision']:raise ValueError('校准草稿已更新，请刷新后重试')
        self.state(body.get('route'));value=copy.deepcopy(self.data)
        if value['draft']!=value['active']:
            value['versions']=(value['versions']+[dict(revision=value['revision'],wall_time=time.time(),routes=value['active'])])[-20:]
            value['active']=copy.deepcopy(value['draft']);value['undo']=[];value['revision']+=1;self.write(value)
        return dict(calibration=self.state(body['route']),applied=True)
