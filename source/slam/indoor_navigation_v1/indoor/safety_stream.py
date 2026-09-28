"""Fast independent positive-obstacle process. No map matching or ground removal."""
import json,queue,time
import numpy as np
from .bootstrap import ROOT
from .pipeline import latest_put


def raw_points(cloud):
    fields={f[0]:f for f in cloud['fields']};types={7:'f4',8:'f8'};order='>' if cloud['is_bigendian'] else '<'
    if len(cloud['data'])!=cloud['height']*cloud['row_step']:raise ValueError('bad_buffer')
    arrays=[]
    for key in ('x','y','z','time'):
        f=fields[key]
        if f[2] not in types or f[3]!=1:raise ValueError('bad_fields')
        arrays.append(np.ndarray((cloud['height'],cloud['width']),dtype=order+types[f[2]],buffer=cloud['data'],offset=f[1],
            strides=(cloud['row_step'],cloud['point_step'])).ravel())
    good=np.logical_and.reduce([np.isfinite(v) for v in arrays])
    return np.column_stack([v[good] for v in arrays])


def clearance(points,mono,epoch):
    # Inflate the original positive-obstacle zones by >=0.10 m. A <=0.40 s
    # stale-clear grace at low forward speed is not permission to ignore people.
    x,y,z=points.T;r=np.hypot(x,y)
    body_height=(z>-.15)&(z<1.2)
    hit=body_height&(((r>.45)&(r<.95))|((x>.45)&(x<1.1)&(abs(y)<.55)))
    return dict(mono=float(mono),epoch=int(epoch),valid=len(points)>=300,blocked=bool(np.count_nonzero(hit)>=8),hits=int(np.count_nonzero(hit)))


def safety_worker(inboxes,outbox,stop):
    ext=json.loads((ROOT/'assets/extrinsics.json').read_text());records={};keys={};side=0
    while not stop.is_set():
        item=None
        for _ in range(2):
            q=inboxes[side];side=1-side
            try:item=q.get_nowait();break
            except queue.Empty:pass
        if item is None:stop.wait(.001);continue
        sensor=item['sensor'];now=time.monotonic()
        try:
            if item['cloud']['frame']!=f'wym_{sensor}_lidar':raise ValueError('wrong_sensor_frame')
            key=(item['epoch'],item['native'])
            if key<=keys.get(sensor,(-1,-1e30)):continue
            keys[sensor]=key
            if not 0<=now-item['received']<=.25:continue
            p=raw_points(item['cloud']);span=float(np.max(p[:,3])) if len(p) else 0.
            if len(p)<300 or not .03<span<.16 or np.min(p[:,3])<-.001:raise ValueError('invalid_scan')
            end=item['mapped_start']+span
            if not 0<=now-end<=.25:continue
            X=np.array(ext['lidar'][sensor]);points=p[:,:3]@X[:3,:3].T+X[:3,3]
            records[sensor]=clearance(points,end,item['epoch'])
        except Exception as exc:
            records[sensor]=dict(mono=now,epoch=item['epoch'],valid=False,blocked=True,error=str(exc))
        latest_put(outbox,dict(records=dict(records),worker_mono=time.monotonic()))
