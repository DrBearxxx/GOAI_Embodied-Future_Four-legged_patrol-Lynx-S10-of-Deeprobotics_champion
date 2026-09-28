"""Causal diagnostic capture; never loads reference query poses."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,write
from s10nav.engine import Engine
from s10nav.matching import Matcher
from s10nav.replay_io import Replay
cfg=read(ROOT/'config.json');m=Matcher(ROOT/'assets',cfg);T=np.eye(4);T[:3,3]=[.5,.1,0]
engine=Engine(ROOT/'assets',cfg,m,T);directory=ROOT/'results/matching_debug_v3';directory.mkdir(exist_ok=False)
original=m.register;now=0;count=0
def capture(points,initial,wide=False):
    global count
    r=original(points,initial,wide)
    if 104<now<110:
        name=f'{now:.3f}-{count}';count+=1
        np.savez_compressed(directory/(name+'.npz'),points=points,initial=initial,estimate=r.get('T',initial))
        write(directory/(name+'.json'),dict(t=now,**{k:v for k,v in r.items() if k!='T'}))
    return r
m.register=capture
for e in Replay(ROOT/'replay_data/B').events():
    now=max(now,e['received'])
    if now>130:break
    r=engine.event(e,now)
    if r and 104<now<115:
        print(json.dumps({k:v for k,v in r.items() if k not in ('pose','health')}),flush=True)
print('COUNTS',engine.counts,flush=True)
