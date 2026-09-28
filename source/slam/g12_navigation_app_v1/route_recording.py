"""Teach a route without taking control: durable samples, manual marks and policies."""
import copy,json,math,os,tempfile,time,uuid
from pathlib import Path


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix=path.name+'.',dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as f:
            json.dump(value,f,ensure_ascii=False,allow_nan=False);f.flush();os.fsync(f.fileno())
        os.replace(tmp,path)
    finally:
        if Path(tmp).exists():Path(tmp).unlink()


class RecordedRoutes:
    def __init__(self,path,digest):
        self.path=Path(path) if path else None
        self.data=dict(schema='goai.recorded-routes.v1',digest=digest,routes={})
        if self.path and self.path.exists():
            self.data=json.loads(self.path.read_text(encoding='utf-8'))
            if self.data['digest']!=digest:raise ValueError('RECORDED_ROUTE_MAP_MISMATCH')

    def add(self,name,route):
        if name in self.data['routes']:
            if self.data['routes'][name]!=route:raise ValueError('RECORDED_ROUTE_ID_COLLISION')
            return
        value=copy.deepcopy(self.data);value['routes'][name]=copy.deepcopy(route)
        if self.path:atomic_json(self.path,value)
        self.data=value


def simplify(points,tolerance=.04):
    """3D polyline simplification; raw samples are retained in the recording journal."""
    if len(points)<3:return points
    keep={0,len(points)-1};stack=[(0,len(points)-1)]
    while stack:
        lo,hi=stack.pop();a=points[lo];b=points[hi];v=[b[k]-a[k] for k in range(3)]
        length2=sum(x*x for x in v);best=tolerance;chosen=None
        for i in range(lo+1,hi):
            u=max(0.,min(1.,sum((points[i][k]-a[k])*v[k] for k in range(3))/length2)) if length2 else 0.
            d=math.dist(points[i],[a[k]+u*v[k] for k in range(3)])
            if d>best:best=d;chosen=i
        if chosen is not None:keep.add(chosen);stack.extend(((lo,chosen),(chosen,hi)))
    return [points[i] for i in sorted(keep)]


class RouteRecorder:
    def __init__(self,path=None):
        self.path=Path(path) if path else None;self.file=None;self.session=None
        self.samples=[];self.marks=[];self.switches=[];self.status='idle';self.error=''
        self.last_sample=-1e9;self.last_measurement=None;self.saved_route=None;self.gaps=0
        if self.path and (self.path/'active.json').exists():
            pointer=json.loads((self.path/'active.json').read_text(encoding='utf-8'))
            sid=str(uuid.UUID(pointer['session_id']))
            journal=self.path/(sid+'.jsonl')
            records=journal.read_bytes().splitlines(keepends=True)
            valid_bytes=0
            for i,line in enumerate(records):
                try:event=json.loads(line.decode('utf-8'))
                except (json.JSONDecodeError,UnicodeDecodeError):
                    if i!=len(records)-1:raise
                    break  # A power cut may leave only the final append incomplete.
                self.consume(event);valid_bytes+=len(line)
            with journal.open('r+b') as f:f.truncate(valid_bytes)
            if self.status in ('recording','paused'):
                self.status='paused';self.error='服务重启，录制草稿已恢复；点击继续录制'
                self.file=journal.open('a',encoding='utf-8',newline='\n')

    def consume(self,event):
        kind=event['kind']
        if kind=='start':self.session=event['session'];self.status='recording'
        elif kind=='sample':self.samples.append(event['sample'])
        elif kind=='mark':self.marks.append(event['mark'])
        elif kind=='unmark':self.marks.pop()
        elif kind=='switch':self.switches.append(event['switch'])
        elif kind=='gap':self.gaps+=1
        elif kind in ('pause','resume'):self.status='paused' if kind=='pause' else 'recording'
        elif kind=='saved':self.status='saved';self.saved_route=event['route_id']

    def append(self,event,durable=False):
        if self.file:
            self.file.write(json.dumps(event,ensure_ascii=False,allow_nan=False)+'\n');self.file.flush()
            if durable:os.fsync(self.file.fileno())
        self.consume(event)

    def close(self):
        if self.file:self.file.close();self.file=None

    def state(self,preview=False):
        out=dict(status=self.status,session_id=(self.session or {}).get('id'),name=(self.session or {}).get('name',''),
            sample_count=len(self.samples),mark_count=len(self.marks),switch_count=len(self.switches),
            marks=copy.deepcopy(self.marks if preview else self.marks[-8:]),switches=copy.deepcopy(self.switches if preview else self.switches[-8:]),gap_count=self.gaps,
            latest=copy.deepcopy(self.samples[-1]) if self.samples else None,error=self.error,saved_route=self.saved_route)
        if preview:
            stride=max(1,len(self.samples)//1200)
            out['trace']=[dict(xyz=s['xyz'],policy=s['policy']) for s in self.samples[::stride]]
        return out

    @staticmethod
    def measurement(solution,now,policy,feedback):
        pose=(solution or {}).get('measurement_pose') or (solution or {}).get('pose')
        if not pose or (solution or {}).get('mode') in ('LOST','STALE','WAIT_SEED','RELOCALIZING'):return None
        if len(pose)<3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pose):return None
        stamp=solution.get('measurement_mono',solution.get('estimate_mono'))
        return dict(xyz=list(pose[:3]),yaw=pose[3] if len(pose)>3 else 0.,mono=now,wall_time=time.time(),
            measurement_mono=stamp,generation=solution.get('generation'),mode=solution.get('mode'),
            policy=policy,requested_policy=feedback.get('requested'),actual_policy=feedback.get('actual_policy',feedback.get('confirmed')),
            input_kind=feedback.get('input_kind'),switch_paused=feedback.get('policy_switch_paused',False))

    def sample(self,solution,now,policy,feedback,force=False):
        if self.status!='recording':return None
        if not force and now-self.last_sample<.099:return None
        sample=self.measurement(solution,now,policy,feedback)
        if sample is None:
            self.error='当前无实测定位，轨迹等待恢复；已录制内容保留'
            return None
        self.error='';sample['index']=len(self.samples)
        previous=self.samples[-1] if self.samples else None
        key=(sample['measurement_mono'],sample['generation'],sample['policy'])
        if not force and key==self.last_measurement:return None
        if previous and (sample['generation']!=previous['generation'] or now-previous['mono']>1.5):
            self.append(dict(kind='gap',before=previous['index'],after=sample['index'],wall_time=time.time()))
        self.append(dict(kind='sample',sample=sample))
        if previous and previous['policy']!=policy:
            self.append(dict(kind='switch',switch=dict(sample_index=sample['index'],xyz=sample['xyz'],
                before=previous['policy'],policy=policy,actual_policy=sample['actual_policy'],wall_time=sample['wall_time'])),True)
        self.last_sample=now;self.last_measurement=key
        return sample

    def tick(self,solution,now,policy,feedback):
        try:self.sample(solution,now,policy,feedback)
        except (OSError,ValueError) as e:
            self.status='paused';self.error='录制写入失败，草稿保留：'+str(e)

    def command(self,body,solution,now,policy,feedback,library):
        action=body['action'].removeprefix('recording_')
        if action=='start':
            if self.status in ('recording','paused'):raise ValueError('已有录制草稿，请继续或保存后新建')
            name=str(body.get('name','')).strip()
            if not name or len(name)>80:raise ValueError('请输入 1–80 字的路线名称')
            sample=self.measurement(solution,now,policy,feedback)
            if sample is None:raise ValueError('当前没有实测定位，请先重定位后记录起点')
            self.close();sid=str(uuid.uuid4())
            session=dict(id=sid,name=name,started=time.time(),frame='joint_map')
            if self.path:
                self.path.mkdir(parents=True,exist_ok=True)
                self.file=(self.path/(sid+'.jsonl')).open('x',encoding='utf-8',newline='\n')
                # Journal header precedes its pointer, so recovery never sees an empty session.
                self.file.write(json.dumps(dict(kind='start',session=session),ensure_ascii=False)+'\n');self.file.flush();os.fsync(self.file.fileno())
                atomic_json(self.path/'active.json',dict(session_id=sid))
            self.session=session;self.samples=[];self.marks=[];self.switches=[];self.gaps=0
            self.status='recording';self.saved_route=None;self.error='';self.last_measurement=None
            sample=self.sample(solution,now,policy,feedback,True)
            self.append(dict(kind='mark',mark=dict(id='P001',name='起点',sample_index=sample['index'],xyz=sample['xyz'],policy=policy)),True)
        elif action=='pause':
            if self.status=='recording':self.append(dict(kind='pause'),True)
        elif action=='resume':
            if self.status!='paused':raise ValueError('当前没有暂停的录制')
            self.append(dict(kind='resume'),True);self.last_sample=-1e9;self.error=''
            self.sample(solution,now,policy,feedback,True)
        elif action=='mark':
            if self.status!='recording':raise ValueError('请先开始或继续录制')
            sample=self.sample(solution,now,policy,feedback,True)
            if sample is None:raise ValueError(self.error)
            pid='P%03d'%(len(self.marks)+1);name=str(body.get('name','')).strip() or pid
            if len(name)>80:raise ValueError('路点名称最多 80 字')
            self.append(dict(kind='mark',mark=dict(id=pid,name=name,sample_index=sample['index'],xyz=sample['xyz'],policy=policy)),True)
        elif action=='undo':
            if self.status not in ('recording','paused'):raise ValueError('当前没有录制草稿')
            if len(self.marks)>1:self.append(dict(kind='unmark'),True)
        elif action=='save':
            if self.status not in ('recording','paused'):raise ValueError('当前没有待保存录制')
            if self.status=='recording':self.sample(solution,now,policy,feedback,True)
            rid='recorded-'+self.session['id'];route=library.data['routes'].get(rid) or self.build_route()
            library.add(rid,route)
            self.append(dict(kind='saved',route_id=rid),True);self.close()
        else:raise ValueError('UNKNOWN_RECORDING_ACTION')
        return dict(route_recording=self.state(True))

    def build_route(self):
        if len(self.samples)<2:raise ValueError('尚未记录可用路线，请移动后记录路点')
        # Manual marks and policy boundaries are retained exactly. Trace controls
        # are geometry only and never become extra arrival/confirmation stops.
        events={0:dict(name='起点',kind='endpoint')}
        for s in self.switches:events[s['sample_index']]=dict(name='模式切换',kind='policy_change')
        for p in self.marks:events[p['sample_index']]=dict(name=p['name'],kind='manual',mark_id=p['id'])
        events.setdefault(len(self.samples)-1,dict(name='终点',kind='endpoint'))
        indices=sorted(events);points=[];edges=[];total=0.
        for index in indices:
            s=self.samples[index];event=events[index]
            # Mode changes at a stationary point update its outgoing policy,
            # rather than introducing a zero-length navigation leg.
            if points and math.dist(points[-1]['xyz'],s['xyz'])<.03:
                between=[p['xyz'] for p in self.samples[points[-1]['sample_index']:index+1]]
                if all(math.dist(points[-1]['xyz'],p)<.03 for p in between):
                    points[-1].update(sample_index=index,recorded_policy=s['policy'])
                    if event['kind']=='manual':points[-1].update(name=event['name'],recording_kind='manual')
                    continue
            points.append(dict(id='R%03d'%(len(points)+1),name=event['name'],xyz=list(s['xyz']),yaw=s['yaw'],
                sample_index=index,recording_kind=event['kind'],recorded_policy=s['policy']))
        if len(points)<2:raise ValueError('路线仍在起点，请移动后再保存')
        for i,(a,b) in enumerate(zip(points,points[1:])):
            lo=a['sample_index'];hi=b['sample_index'];raw=[s['xyz'] for s in self.samples[lo:hi+1]]
            chain=simplify(raw);length=sum(math.dist(x,y) for x,y in zip(chain,chain[1:]));policy=self.samples[lo]['policy']
            controls=[dict(id='trace-%d-%d'%(i,k),name='轨迹',xyz=p) for k,p in enumerate(chain[1:-1])]
            edges.append(dict(source=a['id'],target=b['id'],length_m=length,start_s_m=total,end_s_m=total+length,
                warnings=[],corridor_half_width_m=.6,control_points=controls,recorded_policy=policy))
            total+=length
        if total<.03:raise ValueError('路线尚无位移，请移动后再保存')
        return dict(schema='goai.recorded-route.v1',display_name=self.session['name'],frame='joint_map',waypoints=points,edges=edges,length_m=total,
            recording=dict(session=copy.deepcopy(self.session),sample_count=len(self.samples),marks=copy.deepcopy(self.marks),
                policy_switches=copy.deepcopy(self.switches),gap_count=self.gaps,journal=self.session['id']+'.jsonl'))
