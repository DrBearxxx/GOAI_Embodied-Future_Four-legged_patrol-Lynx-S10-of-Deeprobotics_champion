"""Ordered route follower producing diagnostic proposals only, never motor messages."""
import numpy as np
from .util import wrap,yaw

class RouteFollower:
    def __init__(self,route,cfg):
        self.route=route;self.cfg=cfg;self.xyz=np.array([p['xyz'] for p in route['waypoints']])
        self.target=0;self.reached=[];self.hold_since=None;self.last_arrival=-1e9;self.need_reassociation=False

    def update(self,T,health,now,obstacle,fresh_forward,initialization_changed=False):
        out=dict(target_index=self.target,target_name=self.route['waypoints'][self.target]['name'],
                 reached_indices=self.reached.copy(),vx=0.,vy=0.,wz=0.,motion_authorized=False,mode='shadow_only')
        if initialization_changed and self.reached: self.need_reassociation=True
        if T is None or not health['valid']:
            self.hold_since=None; return {**out,'state':'HOLD_LOCALIZATION'}
        if self.need_reassociation:
            # Never silently skip mandatory points after a localization reset.
            return {**out,'state':'HOLD_REASSOCIATION','reason':'operator must validate current route segment; restart does not skip waypoints'}
        p=T[:3,3]; target=self.xyz[self.target]; dist=float(np.linalg.norm(p-target))
        out['distance_to_target_m']=dist
        if self.target==0 and dist>self.cfg['goal_radius_m']:
            return {**out,'state':'WAIT_ROUTE_ENTRY','reason':'reach first waypoint manually; no unreviewed approach path'}
        if self.target>0:
            a,b=self.xyz[self.target-1],target;v=b-a;u=float(np.clip((p-a)@v/max(v@v,1e-12),0,1))
            closest=a+u*v; lateral=float(np.linalg.norm((p-closest)[:2])); height=float(abs(p[2]-closest[2]))
            out.update(segment=self.target-1,cross_track_m=lateral,height_error_m=height)
            if lateral>self.cfg['corridor_half_width_m'] or height>self.cfg['height_tolerance_m']:
                self.hold_since=None;return {**out,'state':'HOLD_OFF_ROUTE'}
        if obstacle: self.hold_since=None;return {**out,'state':'HOLD_OBSTACLE'}
        if not fresh_forward: self.hold_since=None;return {**out,'state':'HOLD_FORWARD_PERCEPTION'}
        if dist<=self.cfg['goal_radius_m'] and abs(p[2]-target[2])<=self.cfg['height_tolerance_m']:
            if self.hold_since is None:self.hold_since=now
            if now-self.hold_since>=.2 and now-self.last_arrival>=.2:
                if not self.reached or self.reached[-1]!=self.target:self.reached.append(self.target)
                self.last_arrival=now;self.hold_since=None
                if self.target==len(self.xyz)-1:return {**out,'state':'COMPLETE','reached_indices':self.reached.copy()}
                self.target+=1
            return {**out,'state':'CONFIRM_WAYPOINT'}
        self.hold_since=None
        if self.target==0:return {**out,'state':'WAIT_ROUTE_ENTRY'}
        edge=self.route['edges'][self.target-1]
        if edge['warnings']:return {**out,'state':'HOLD_ROUTE_VALIDATION','reason':';'.join(edge['warnings'])}
        look=a+min(1.,u+self.cfg['lookahead_m']/max(np.linalg.norm(v),.01))*v
        heading=np.arctan2(look[1]-p[1],look[0]-p[0]); error=wrap(heading-yaw(T))
        speed=min(self.cfg['max_shadow_speed_mps'],edge['speed_limit_mps'],dist*.5)
        if health['single_lidar']:speed*=.5
        speed*=max(0.,np.cos(error))
        if abs(error)>np.deg2rad(60):speed=0.
        return {**out,'state':'FOLLOW_SHADOW','vx':float(speed),'wz':float(np.clip(error*1.2,-self.cfg['max_yaw_rate_radps'],self.cfg['max_yaw_rate_radps']))}
