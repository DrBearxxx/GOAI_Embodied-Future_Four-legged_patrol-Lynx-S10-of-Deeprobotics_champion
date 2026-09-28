"""Bounded-cost live cloud conversion; full scan temporal bounds are preserved."""
import numpy as np
from . import bootstrap
from s10nav.messages import convert as original_convert,stamp
from s10nav.util import voxel,cap

def convert(topic,msg,received,K=None):
    if not topic.endswith('/points'):return original_convert(topic,msg,received,K)
    fields={f.name:f for f in msg.fields};fmt={7:'f4',8:'f8'};order='>' if msg.is_bigendian else '<'
    if not all(k in fields for k in ['x','y','z','time']):raise ValueError('XYZ and per-point time required')
    if len(msg.data)!=msg.row_step*msg.height:raise ValueError('Invalid cloud buffer length')
    arrays=[]
    for key in ['x','y','z','time']:
        f=fields[key]
        if f.datatype not in fmt or f.count!=1:raise ValueError('Unsupported cloud field')
        arrays.append(np.ndarray((msg.height,msg.width),dtype=order+fmt[f.datatype],buffer=msg.data,offset=f.offset,strides=(msg.row_step,msg.point_step)).ravel())
    valid=np.flatnonzero(np.logical_and.reduce([np.isfinite(x) for x in arrays]))
    if not len(valid):p=np.empty((0,4))
    else:
        # The old path voxel-sorted every raw point before capping to 5000,
        # which can make the entire scan stale on Orin. Spatially distributed
        # deterministic sampling bounds that sort to <=16002 points.
        temporal=arrays[3][valid];edges=valid[[np.argmin(temporal),np.argmax(temporal)]]
        ids=valid[np.linspace(0,len(valid)-1,min(len(valid),16000),dtype=int)]
        p=np.column_stack([a[ids] for a in arrays]);bounds=np.column_stack([a[edges] for a in arrays])
        p=np.vstack([bounds[:1],cap(voxel(p,.12),4998),bounds[1:]])
    return dict(received=float(received),stamp=stamp(msg),frame=msg.header.frame_id,kind='cloud',sensor=topic.split('/')[3],
        points=p[:,:3].astype(np.float32),rel=p[:,3].astype(np.float32))
