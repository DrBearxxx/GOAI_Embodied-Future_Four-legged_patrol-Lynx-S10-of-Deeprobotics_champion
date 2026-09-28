"""Continuous Hermite route reference with velocity feedforward.

Waypoints shape one path; they are not individual zero-velocity goals. The
cruise speed is a reference, not an output clamp. PID corrections are added
afterward and the robot's selected native policy owns its operating range.
"""
import bisect
import math


def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return [x-y for x,y in zip(a,b)]
def unit(a):
    length=math.sqrt(dot(a,a))
    return [x/length for x in a] if length>1e-9 else [0.,0.,0.]


class RouteTrajectory:
    def __init__(self,points,cruise_mps=.8,preview_s=.15,control_points=None):
        self.points=[list(p) for p in points]
        if len(points)<2:points=[*points,*points];self.points=[list(p) for p in points]
        groups=[]
        if control_points and any(control_points):
            expanded=[points[0]]
            for i,end in enumerate(points[1:]):
                begin=len(expanded)-1
                expanded.extend(control_points[i]);expanded.append(end)
                groups.append((begin,len(expanded)-2))
            points=expanded
        self.cruise_mps=cruise_mps;self.preview_s=preview_s
        self.samples=[];self.arcs=[];self.bounds=[];self.total=0.
        directions=[unit(sub(b,a)) for a,b in zip(points,points[1:])]
        tangents=[directions[0]]+[[.5*(a+b) for a,b in zip(directions[i-1],directions[i])]
            for i in range(1,len(points)-1)]+[directions[-1]]
        for index,(a,b) in enumerate(zip(points,points[1:])):
            length=math.dist(a,b);steps=max(2,math.ceil(length/.025))
            m0=[v*length for v in tangents[index]];m1=[v*length for v in tangents[index+1]]
            begin=len(self.samples)-1 if self.samples else 0
            for j in range(steps+1):
                if index and j==0:continue
                t=j/steps;t2=t*t;t3=t2*t
                q=[(2*t3-3*t2+1)*x+(t3-2*t2+t)*u+(-2*t3+3*t2)*y+(t3-t2)*v
                    for x,y,u,v in zip(a,b,m0,m1)]
                d=[(6*t2-6*t)*x+(3*t2-4*t+1)*u+(-6*t2+6*t)*y+(3*t2-2*t)*v
                    for x,y,u,v in zip(a,b,m0,m1)]
                dd=[(12*t-6)*x+(6*t-4)*u+(-12*t+6)*y+(6*t-2)*v for x,y,u,v in zip(a,b,m0,m1)]
                if math.hypot(d[0],d[1])<1e-9:d=sub(b,a)
                yaw=math.atan2(d[1],d[0])
                curvature=(d[0]*dd[1]-d[1]*dd[0])/max(math.hypot(d[0],d[1])**3,1e-9)
                if self.samples:self.total+=math.dist(q[:2],self.samples[-1]['xyz'][:2])
                self.samples.append(dict(xyz=q,yaw=yaw,curvature=curvature));self.arcs.append(self.total)
            self.bounds.append((begin,len(self.samples)-1))
        if groups:
            bounds=self.bounds
            self.bounds=[(bounds[a][0],bounds[b][1]) for a,b in groups]

    def at(self,arc):
        right=min(len(self.samples)-1,max(1,bisect.bisect_left(self.arcs,arc)))
        left=right-1;u=max(0.,min(1.,(arc-self.arcs[left])/max(self.arcs[right]-self.arcs[left],1e-9)))
        a,b=self.samples[left],self.samples[right]
        delta=(b['yaw']-a['yaw']+math.pi)%(2*math.pi)-math.pi
        return dict(xyz=[x+u*(y-x) for x,y in zip(a['xyz'],b['xyz'])],
            yaw=a['yaw']+u*delta,curvature=a['curvature']+u*(b['curvature']-a['curvature']))

    def project(self,pose,segment,next_segments=0):
        # Include the previous leg because passage is recorded within the
        # waypoint tolerance before its geometric vertex. This avoids a
        # reference jump when the next waypoint becomes the UI target.
        begin=self.bounds[max(0,segment-1)][0]
        end=self.bounds[min(len(self.bounds)-1,segment+next_segments)][1]
        best=None
        for i in range(begin,end):
            a,b=self.samples[i]['xyz'],self.samples[i+1]['xyz'];v=sub(b,a)
            u=max(0.,min(1.,dot(sub(pose[:2],a[:2]),v[:2])/max(dot(v[:2],v[:2]),1e-12)))
            q=[x+u*d for x,d in zip(a,v)]
            candidate=(math.dist(pose[:2],q[:2]),self.arcs[i]+u*(self.arcs[i+1]-self.arcs[i]))
            if best is None or candidate<best:best=candidate
        q=self.at(best[1])['xyz']
        return dict(arc_m=best[1],xyz=q,cross_track_m=math.hypot(pose[0]-q[0],pose[1]-q[1]),
                    height_error_m=abs(pose[2]-q[2]))

    def reference(self,pose,segment,minimum_arc=0.):
        projection=self.project(pose,segment)
        # The waypoint index is ordered; measured arc progress is reversible.
        # A transient forward pose error must not leave a permanent reference
        # ahead of the robot and create a large fictitious catch-up demand.
        progress=projection['arc_m'];remaining=max(0.,self.total-progress)
        # Terminal braking belongs only to the end of the continuous path.
        speed=self.cruise_mps*math.tanh(remaining/.45)
        arc=min(self.total,progress+speed*self.preview_s);ref=self.at(arc)
        velocity=[speed*math.cos(ref['yaw']),speed*math.sin(ref['yaw']),speed*ref['curvature']]
        return dict(xyz=ref['xyz'],yaw=ref['yaw'],velocity=velocity,progress_m=progress,
            reference_arc_m=arc,remaining_m=remaining,cruise_mps=self.cruise_mps,
            reference_speed_mps=speed,curvature=ref['curvature'],method='hermite_path_feedforward_pid')
