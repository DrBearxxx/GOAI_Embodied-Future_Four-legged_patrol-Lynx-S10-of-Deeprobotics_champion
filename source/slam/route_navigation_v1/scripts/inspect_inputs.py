import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read
from s10nav.engine import Engine
from s10nav.replay_io import Replay
e=Engine(ROOT/'assets',read(ROOT/'config.json'),None);found={}
for item in Replay(ROOT/'replay_data/B').events():
    if item['received']>20:break
    if item['kind']=='cloud':continue
    e.event(item)
    if item['received']>5 and (item['kind'],item['sensor']) not in found:
        found[item['kind'],item['sensor']]=True
        print({k:v for k,v in item.items() if k!='points'})
print('vio',len(e.motion.vio),'epoch',e.motion.vio_epoch,'rejects',e.motion.vio_rejects)
print('clocks',{s:c.__dict__ for s,c in e.clocks.items() if s=='camera'})
print('vio_samples',list(e.motion.vio)[-2:])
