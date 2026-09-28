"""Compact raw arrival-ordered events; no corrected-clock ledger or query trajectory."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu,Image
from nav_msgs.msg import Odometry
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,write
from s10nav.messages import convert

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--session',required=True);a=p.parse_args()
    source=Path(a.source);out=ROOT/'replay_data'/a.session
    out.mkdir(parents=True,exist_ok=False)
    camera=read(source/'insight9.json')['calibration']; K=np.array(camera['left_info']['k']).reshape(3,3)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(source/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    known={'sensor_msgs/msg/PointCloud2':PointCloud2,'sensor_msgs/msg/Imu':Imu,'nav_msgs/msg/Odometry':Odometry,'sensor_msgs/msg/Image':Image}
    types={t.name:known[t.type] for t in reader.get_all_topics_and_types() if t.type in known}
    selected=[t for t in types if t.endswith('/points') or t.endswith('/imu') or t.endswith('/odometry') or '/depth/image_rect_raw' in t]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected)); index=[]; chunks=[]; cloud_count=0; t0=None;counts={};errors=[];last=time.monotonic();frames={}
    small=[];depth_last=-1e9
    def flush():
        if not chunks:return
        number=len(list(out.glob('clouds-*.npz')));lengths=[len(x[0]) for x in chunks]; offsets=np.r_[0,np.cumsum(lengths)]
        np.savez_compressed(out/f'clouds-{number:04d}.npz',points=np.vstack([x[0] for x in chunks]),rel=np.concatenate([x[1] for x in chunks]),offsets=offsets)
        chunks.clear()
    with (out/'events.jsonl').open('w') as f:
        while reader.has_next():
            topic,raw,ns=reader.read_next();received=ns/1e9
            if t0 is None:t0=received
            if '/depth/' in topic and received-depth_last<.5:continue
            try:
                m=deserialize_message(raw,types[topic]); e=convert(topic,m,received,K)
                if e is None:continue
                e['received']-=t0;frames[topic]=e['frame'];counts[e['kind']]=counts.get(e['kind'],0)+1
                if e['kind']=='cloud':
                    chunk=cloud_count//200;slot=cloud_count%200;cloud_count+=1
                    chunks.append((e.pop('points'),e.pop('rel')));e['chunk']=chunk;e['slot']=slot
                    f.write(json.dumps(e)+'\n')
                    if len(chunks)==200:flush()
                elif e['kind']=='depth':
                    depth_last=received;e['points']=e['points'].tolist();f.write(json.dumps(e)+'\n')
                else:f.write(json.dumps(e)+'\n')
            except (ValueError,RuntimeError) as exc:errors.append(dict(received=received-t0,topic=topic,reason=str(exc)))
            if time.monotonic()-last>15:print('EXTRACT',a.session,round(received-t0,1),counts,flush=True);last=time.monotonic()
        flush()
    write(out/'manifest.json',dict(session=a.session,source=str(source),t0_host=t0,duration_s=received-t0,counts=counts,errors=errors,frames=frames,
        processing='native stamps untouched; reception order; spatially downsampled LiDAR; depth diagnostic at 2Hz',
        optimized_query_poses_read=False,offline_clock_corrections_used=False))
    print('EXTRACT_DONE',a.session,counts,'errors',len(errors),flush=True)
if __name__=='__main__':main()
