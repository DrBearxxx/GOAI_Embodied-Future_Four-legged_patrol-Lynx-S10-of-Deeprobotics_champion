"""Map scene terrain intervals to the unchanged physical route's original edges."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def digest(route):
    return hashlib.sha256(json.dumps(route,sort_keys=True).encode()).hexdigest()


def anticipate(edges,conflict_policy):
    """Enter special gaits one original edge early, never exit terrain early."""
    original=[e['policy'] for e in edges]
    for e in edges:
        e['terrain_policy']=e['policy'];e['anticipated_terrain']=[]
    for start,policy in enumerate(original):
        if policy=='basic_normal' or (start>0 and original[start-1]==policy):continue
        if start==0:continue
        previous=edges[start-1]
        if original[start-1] not in ('basic_normal',policy) and conflict_policy=='retain_current_terrain':continue
        previous['policy']=policy
        end=start+1
        while end<len(edges) and original[end]==policy:end+=1
        seen=set()
        for e in edges[start:end]:
            for hit in e['terrain']:
                if hit['id'] not in seen:
                    previous['anticipated_terrain'].append(hit);seen.add(hit['id'])



def apply_flat_step_move(profile):
    # These indices name the user's unchanged, hashed 66-waypoint route.
    # Apply after terrain anticipation: do not switch out of the terrace early
    # or overwrite the advance selection for the next high-platform section.
    if profile['route_digest']!='9e3ead1ade1bbf7497ee838835737e647486d2c2047f9bcfd81a03e8c2cb31a2':return profile
    edges=profile['edges'];indices=list(range(30,41))
    if edges[30]['from_name']!='WP 31' or edges[40]['to_name']!='WP 42':raise ValueError('FLAT_ROUTE_BOUNDARY_CHANGED')
    if any(edges[i]['terrain_policy']!='basic_normal' or edges[i]['anticipated_terrain'] for i in indices):
        raise ValueError('FLAT_ROUTE_OVERLAPS_SPECIAL_TERRAIN')
    for i in indices:edges[i]['policy']='step_move'
    profile['preset_updates']=[dict(id='post-terrace-step-move-v1',segments=indices,policy='step_move',
        from_name='WP 31',to_name='WP 42',reason='用户指定：WP31 至 WP42 使用踏步移动 0xF002')]
    note='WP31 to WP42 uses step_move (0xF002) after leaving the terrace; preserve the next platform advance selection at WP48.'
    if note not in profile['notes']:profile['notes'].append(note)
    return profile

def build(route,annotation,scene_route,conflict_policy):
    labels=json.loads(annotation.read_text(encoding='utf-8'))
    assert hashlib.sha256(scene_route.read_bytes()).hexdigest()==labels['route_sha256']
    with np.load(scene_route) as data:
        indices=data['marker_indices'].astype(int)
        xy=np.asarray([p['xyz'][:2] for p in route['waypoints']])
        assert len(indices)==len(xy) and np.array_equal(xy,data['xy'][indices])
        arc=data['arc'][indices].copy()
    assert np.all(np.diff(arc)>0)
    policies={'stair':'stairs_normal','short':'stairs_normal','high':'platform','landing':'basic_normal'}
    priority={'basic_normal':0,'stairs_normal':1,'platform':2}
    edges=[]
    for i,(lo,hi) in enumerate(zip(arc,arc[1:])):
        hits=[r for r in labels['records'] if any(hi>=a and lo<=b for a,b in r['intervals_m'])]
        policy=max((policies[r['kind']] for r in hits),key=priority.get,default='basic_normal')
        edges.append(dict(index=i,from_name=route['waypoints'][i]['name'],
            to_name=route['waypoints'][i+1]['name'],scene_arc_m=[float(lo),float(hi)],
            policy=policy,terrain=[dict(id=r['id'],kind=r['kind'],label=r['label'],
                intervals_m=r['intervals_m']) for r in hits]))
    anticipate(edges,conflict_policy)
    covered={r['id'] for e in edges for r in e['terrain']}
    return apply_flat_step_move(dict(schema='goai.outdoor.terrain-presets.v1',route_id='full',
        route_digest=digest(route),display_name='室外路线（66点）',
        scene_route_sha256=labels['route_sha256'],scene_sha256=labels['scene_sha256'],
        annotation_sha256=hashlib.sha256(annotation.read_bytes()).hexdigest(),
        annotation_source='scene_correction_v16/acceptance/terrain_segments.json',
        entry_lookahead_waypoints=1,overlap_policy=conflict_policy,
        marker_xy_max_error_m=0.,scene_arc_end_m=float(arc[-1]),
        annotation_route_length_m=labels['route_length_m'],
        mapping='Exact original marker XY identity; intersect scene arc intervals with original incoming edges.',
        notes=['Enter each special terrain run one waypoint early; exit after its original last edge.',
               'Landing alone uses basic; high terrace uses platform; continuous/short stairs use stairs.',
               'Original physical waypoints, body heights and order are unchanged.',
               'Terrain labels describe reconstructed features, not measured traversal certification.'],
        excluded_annotations=[dict(id=r['id'],label=r['label'],intervals_m=r['intervals_m'])
                              for r in labels['records'] if r['id'] not in covered],edges=edges))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--annotations',type=Path,required=True)
    parser.add_argument('--scene-route',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--overlap-policy',choices=['retain_current_terrain','upcoming_terrain'],required=True)
    args=parser.parse_args()
    profile=build(json.loads(args.route.read_text(encoding='utf-8')),args.annotations,args.scene_route,args.overlap_policy)
    args.output.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({p:sum(e['policy']==p for e in profile['edges']) for p in ('basic_normal','stairs_normal','platform','step_move')}))
