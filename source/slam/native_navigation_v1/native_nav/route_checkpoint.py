"""Atomic bounded progress only; never stores an armed/resume permission."""
import hashlib
import json
from pathlib import Path


class RouteCheckpoint:
    def __init__(self,path,route,asset_id):
        self.path=Path(path);self.asset_id=asset_id
        self.route_id=hashlib.sha256(json.dumps(route,sort_keys=True).encode()).hexdigest()
        self.last=None;self.error=None

    def save(self,guard):
        state=dict(schema='s10.route.progress.v1',asset_id=self.asset_id,route_id=self.route_id,
                   target=guard.target,reached=list(guard.reached),complete=guard.complete)
        if state==self.last:return True
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps(state,allow_nan=False)+'\n');temp.replace(self.path)
            self.last=state;self.error=None;return True
        except OSError as exc:self.error=str(exc);return False

    def restore(self,guard):
        if self.path.stat().st_size>16384:raise ValueError('OVERSIZED_CHECKPOINT')
        s=json.loads(self.path.read_text());target=s['target'];reached=s['reached'];complete=s['complete']
        if s.get('schema')!='s10.route.progress.v1' or s['asset_id']!=self.asset_id or s['route_id']!=self.route_id:
            raise ValueError('CHECKPOINT_MAP_OR_ROUTE_MISMATCH')
        if type(target) is not int or not 0<=target<len(guard.xyz) or type(complete) is not bool:raise ValueError('INVALID_PROGRESS')
        if complete and target!=len(guard.xyz)-1:raise ValueError('INVALID_COMPLETE_TARGET')
        if (not isinstance(reached,list) or any(type(v) is not int for v in reached) or
                reached!=list(range(target+(1 if complete else 0)))):raise ValueError('INVALID_REACHED_ORDER')
        guard.target=target;guard.reached=reached;guard.complete=complete;guard.stop('CHECKPOINT_RESTORED_REQUIRES_ARM')
        self.last=s
