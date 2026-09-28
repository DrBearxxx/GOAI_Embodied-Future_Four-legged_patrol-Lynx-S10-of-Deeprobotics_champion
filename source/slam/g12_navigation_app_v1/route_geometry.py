"""Polyline geometry for calibrated edges, without changing mission markers."""
import math


def edge_points(route,index):
    return [route['waypoints'][index]['xyz'],
            *[p['xyz'] for p in route['edges'][index].get('control_points',[])],
            route['waypoints'][index+1]['xyz']]


def project_polyline(pose,points):
    best=None;offset=0.
    for i,(a,b) in enumerate(zip(points,points[1:])):
        v=[y-x for x,y in zip(a,b)];length=math.dist(a,b)
        raw=sum((pose[k]-a[k])*v[k] for k in range(3))/max(length*length,1e-12)
        u=max(0.,min(1.,raw));q=[x+u*d for x,d in zip(a,v)]
        score=(math.dist(pose[:3],q),-offset-u*length)
        if best is None or score<best[0]:best=(score,dict(index=i,u=u,raw_u=raw,xyz=q,arc=offset+u*length))
        offset+=length
    result=best[1];result['remaining']=max(0.,offset-result['arc']);result['length']=offset
    return result


def advance_polyline(points,projection,distance):
    """Return a forward join and all remaining vertices, including the end."""
    q=list(projection['xyz']);i=projection['index']
    while i<len(points)-1:
        b=points[i+1];length=math.dist(q[:2],b[:2])
        if distance<length and length>1e-9:
            u=distance/length
            return [[a+u*(v-a) for a,v in zip(q,b)],*[list(p) for p in points[i+1:]]]
        distance-=length;q=list(b);i+=1
    return [list(points[-1])]
