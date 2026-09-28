"""ROS-free matching worker: separate process, bounded per-LiDAR inboxes."""
import copy,json,queue,time,cProfile
from collections import deque
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from .bootstrap import ROOT
from .clock import use_live_clocks
from .messages import convert
from s10nav.engine import Engine
from s10nav.matching import Matcher
from s10nav.util import read,apply
from .continuity import use_redundancy

class IndoorMatcher(Matcher):
    def __init__(self,assets,cfg):
        super().__init__(assets,cfg)
        # NpzFile.__getitem__ reads/decompresses on every access. Retrieval
        # repeatedly accesses poses inside its candidate loop; cache once.
        archive=self.gallery;self.gallery={k:archive[k] for k in archive.files};archive.close()

def latest_put(q,item):
    try:q.put_nowait(item);return 0
    except queue.Full:
        try:q.get_nowait()
        except queue.Empty:return 1
        try:q.put_nowait(item)
        except queue.Full:pass
        return 1

def snapshot_ingest(engine,now):
    names=('native','received','offset','stable_since','epoch','faults','reason')
    clocks={s:{k:getattr(c,k) for k in names} for s,c in engine.clocks.items()}
    imu={}
    for s,q in engine.motion.imu.items():
        selected=[v for v in q if now-v[0]<1.2]
        # Contiguous numeric arrays pickle cheaply; do not send thousands of
        # separate ndarray objects or the entire historical Engine per scan.
        if selected:imu[s]=np.c_[[v[0] for v in selected],np.array([v[1] for v in selected]),np.array([v[2] for v in selected])]
        else:imu[s]=np.empty((0,7))
    return dict(clocks=clocks,imu=imu,vio=[v for v in engine.motion.vio if now-v[0]<1.2],depth=engine.perception.get('depth'))

def install_ingest(engine,data):
    for s,fields in data['clocks'].items():
        if engine.clocks[s].epoch!=fields['epoch']:engine.motion.gravity_anchor=None
        for k,v in fields.items():setattr(engine.clocks[s],k,v)
    engine.motion.imu={s:deque(zip(a[:,0],a[:,1:4],a[:,4:7]),maxlen=1400) for s,a in data['imu'].items()}
    engine.motion.vio=deque(data['vio'],maxlen=200)
    if data['depth'] is not None:engine.perception['depth']=data['depth']

def pack_cloud(topic,msg,received,ingest):
    return dict(topic=topic,received=received,ingest=ingest,cloud=dict(
        frame=msg.header.frame_id,sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec,
        fields=[(f.name,f.offset,f.datatype,f.count) for f in msg.fields],height=msg.height,width=msg.width,
        is_bigendian=msg.is_bigendian,point_step=msg.point_step,row_step=msg.row_step,data=bytes(msg.data)))

def unpack_cloud(c):
    return NS(header=NS(frame_id=c['frame'],stamp=NS(sec=c['sec'],nanosec=c['nanosec'])),
        fields=[NS(name=f[0],offset=f[1],datatype=f[2],count=f[3]) for f in c['fields']],
        **{k:c[k] for k in ('height','width','is_bigendian','point_step','row_step','data')})

def diagnostic(engine,now,seq,timings,perception):
    if engine.T is None:return None
    h=engine.poll(now);T=engine.T;fresh={s:False for s in ['front','rear']};obstacle=False
    for s,(t,p) in perception.items():
        if not 0<=now-t<.25:continue
        if s in fresh:fresh[s]=True
        rxy=np.linalg.norm(p[:,:2],axis=1)
        hit=(rxy>.45)&(rxy<.85)&(p[:,2]>-.15)&(p[:,2]<1.2)
        front=(p[:,0]>.45)&(p[:,0]<1.)&(abs(p[:,1])<.45)&(p[:,2]>-.15)&(p[:,2]<1.2)
        obstacle |= np.count_nonzero(hit|front)>=8
    return dict(seq=seq,mono=now,pose=[*map(float,T[:3,3]),float(np.arctan2(T[1,0],T[0,0]))],health=h,
        generation=[engine.generation,*[engine.clocks[s].epoch for s in ['front','rear','camera']]],
        front_fresh=fresh['front'],rear_fresh=fresh['rear'],obstacle=bool(obstacle),error=None,pipeline=timings,
        perception_mono={s:perception.get(s,(-1.,None))[0] for s in ('front','rear')},
        anchor=dict(T=T.tolist(),measurement_mono=engine.last_t,velocity=engine.velocity.tolist(),
            confirmed=engine.confirmations>=engine.cfg['relocalization']['confirmations'],
            generation=[engine.generation],healthy_lidars=h['healthy_lidars']))

def worker_main(inboxes,outbox,stop,run_id,initial,profile=False):
    prof=cProfile.Profile() if profile else None
    if prof:prof.enable()
    log=None
    try:
        cfg=read(ROOT/'config.json');assets=ROOT/'assets';T=None
        if initial:
            T=np.eye(4);T[:3,3]=initial[:3];T[:3,:3]=Rotation.from_euler('z',np.radians(initial[3])).as_matrix()
        engine=use_redundancy(use_live_clocks(Engine(assets,cfg,IndoorMatcher(assets,cfg),T)));seq=0;side=0;perception={};perception_native={}
        log=(ROOT/'live_logs'/f'localize-{run_id}.jsonl').open('x',buffering=1)
        while not stop.is_set():
            item=None
            for _ in range(2):
                q=inboxes[side];side=1-side
                try:item=q.get_nowait();break
                except queue.Empty:pass
            if item is None:stop.wait(.001);continue
            before=time.monotonic();received=item['received']
            if before-received>.25:continue
            try:
                event=convert(item['topic'],unpack_cloud(item['cloud']),received)
                if event is None:raise ValueError('unrecognized_cloud')
            except (ValueError,KeyError,TypeError,IndexError,OverflowError) as exc:
                log.write(json.dumps(dict(accepted=False,received=received,reason='malformed_cloud:'+repr(exc)))+'\n')
                continue
            converted=time.monotonic()
            install_ingest(engine,item['ingest']);installed=time.monotonic()
            r=engine.event(event,installed);after=time.monotonic();seq+=1
            mapped=engine.clocks[event['sensor']].map_stamp(event['stamp'],after)
            # Obstacle freshness is independent of the slower map-fit rate.
            # Collision support uses current body-frame raw geometry, not the
            # human-filtered map. No extra motion or localization validity is
            # inferred from this stream. Stale/duplicate/bad frames cannot
            # keep perception alive.
            sensor=event['sensor'];native_key=(engine.clocks[sensor].epoch,event['stamp'])
            span=float(np.max(event['rel'])) if len(event['rel']) else 0.
            good=(mapped is not None and -.025<=after-mapped-span<.25 and
                event['frame']==f'wym_{sensor}_lidar' and len(event['points'])>=300 and
                .03<span<.16 and float(np.min(event['rel']))>=-.001 and
                native_key>perception_native.get(sensor,(-1,-1e30)))
            if good:
                perception[sensor]=(mapped+span,apply(np.array(engine.ext['lidar'][sensor]),event['points']))
                perception_native[sensor]=native_key
            if item['ingest']['depth'] is not None:perception['depth']=item['ingest']['depth']
            timings=dict(queue_ms=(before-received)*1000,convert_ms=(converted-before)*1000,copy_ms=(installed-converted)*1000,
                engine_ms=(after-installed)*1000,measurement_age_ms=None if mapped is None else (after-mapped-float(np.max(event['rel'])))*1000,
                raw_points=item['cloud']['width']*item['cloud']['height'],converted_points=len(event['points']))
            if r:r['pipeline']=timings;log.write(json.dumps(r)+'\n')
            latest_put(outbox,dict(snapshot=diagnostic(engine,after,seq,timings,perception),pipeline=timings))
    except Exception as exc:
        latest_put(outbox,dict(error=repr(exc)))
        if log:log.write(json.dumps(dict(error=repr(exc)))+'\n')
    finally:
        if log:log.close()
        if prof:prof.disable();prof.dump_stats(ROOT/'live_logs'/f'worker-{run_id}.prof')
