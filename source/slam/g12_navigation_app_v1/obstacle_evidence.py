"""Obstacle evidence for the forward route controller, in the body frame."""
import numpy as np


def terrain_surface(p, ahead, ceiling):
    """Find a broad tread/top, then classify its lower riser as terrain.

    Work on the full cloud, not the 24 logging samples. A vertical wall or a
    narrow isolated return cannot establish a supporting horizontal surface.
    Heights are relative to the body, not to the global map.
    """
    q=p[ahead & (p[:,2]<=ceiling)]
    if len(q)<8:return None
    cells=np.unique(np.floor(q/.04).astype(np.int32),axis=0)
    levels=np.unique(cells[:,2]);tops=[]
    for level in levels:
        layer=cells[abs(cells[:,2]-level)<=1]
        if len(layer)<12:continue
        # Distributed support across the travel corridor and along the tread.
        if (np.ptp(layer[:,0])>=4 and np.ptp(layer[:,1])>=10
                and len(np.unique(layer[:,0]))>=4 and len(np.unique(layer[:,1]))>=8):
            tops.append(float(min(ceiling,(level+2)*.04)))
    return max(tops) if tops else None


def policy_obstacles(record,policy):
    kind='stairs' if policy in ('stairs','stairs_normal') else 'platform' if policy=='platform' else 'basic'
    profile=record.get('policy_clearance',{}).get(kind)
    return dict(profile or dict(blocked=bool(record.get('blocked')),hits=record.get('hits'),
        terrain_hits=0,method='raw_forward_corridor'))


def clearance(points,mono,epoch):
    p=np.asarray(points)
    p=p[np.isfinite(p).all(axis=1)]
    x,y,z=p.T;r=np.hypot(x,y)
    height=(z>-.15)&(z<1.2)
    # The navigator travels forward. The old all-around annulus also stopped
    # it for objects to the side and behind, outside its forward corridor.
    ahead=height&(x>.45)&(x<1.1)&(abs(y)<.55)
    nearby=height&(r>.45)&(r<.95)
    hits=int(np.count_nonzero(ahead))
    profiles={}
    for kind,ceiling in (('stairs',.16),('platform',.44)):
        top=terrain_surface(p,ahead,ceiling)
        terrain=ahead & (z<=top) if top is not None else np.zeros(len(p),dtype=bool)
        remaining=int(np.count_nonzero(ahead & ~terrain))
        profiles[kind]=dict(blocked=remaining>=8,hits=remaining,terrain_hits=int(np.count_nonzero(terrain)),
            surface_top_body_m=top,method='local_support_surface')
    def bounded(mask):
        q=p[mask]
        if len(q)>24:q=q[np.linspace(0,len(q)-1,24,dtype=int)]
        return np.round(q,4).tolist()
    return dict(mono=float(mono),epoch=int(epoch),valid=len(p)>=300,blocked=hits>=8,hits=hits,
        zone='forward_corridor',bounds_m=dict(x=[.45,1.1],y=[-.55,.55],z=[-.15,1.2]),
        hit_points_body=bounded(ahead),policy_clearance=profiles,nearby_hits=int(np.count_nonzero(nearby)),
        nearby_points_body=bounded(nearby))
