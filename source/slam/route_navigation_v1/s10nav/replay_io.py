import json
import heapq
from collections import OrderedDict
from pathlib import Path
import numpy as np

class Replay:
    def __init__(self,directory):self.directory=Path(directory);self.chunks=OrderedDict()
    def events(self):
        with (self.directory/'events.jsonl').open() as f:
            for line in f:
                e=json.loads(line)
                if e['kind']=='cloud':
                    i=e.pop('chunk');slot=e.pop('slot')
                    if i not in self.chunks:
                        with np.load(self.directory/f'clouds-{i:04d}.npz') as z:self.chunks[i]={k:z[k] for k in z.files}
                        if len(self.chunks)>2:self.chunks.popitem(last=False)
                    z=self.chunks[i];a,b=z['offsets'][slot:slot+2];e['points']=z['points'][a:b];e['rel']=z['rel'][a:b]
                if e['kind']=='depth':e['points']=np.asarray(e['points'])
                yield e

SCENARIOS={
    'baseline':[],
    'mixed_faults':[
        dict(start=40,end=55,sensor='front',action='drop'),
        dict(start=70,end=85,sensor='rear',action='drop'),
        dict(start=105,end=120,sensor='camera',action='drop'),
        dict(start=145,end=150,sensor='all',action='drop'),
        dict(start=185,end=190,sensor='front',action='delay',seconds=.6),
        dict(start=220,end=230,sensor='rear',action='duplicate'),
        dict(start=270,end=280,sensor='front',action='stamp_shift',seconds=1.),
        dict(start=325,end=340,sensor='all',kind='cloud',action='thin',fraction=.05),
        dict(start=405,end=410,sensor='all',kind='imu',action='drop'),
        dict(start=450,end=455,sensor='camera',kind='vio',action='vio_jump',metres=15.),
        dict(start=510,end=530,sensor='all',kind='cloud',action='random_drop',probability=.5),
        dict(start=580,end=584,sensor='all',action='drop')],
}

def inject(events,scenario):
    rng=np.random.default_rng(20260913);pending=[];serial=0
    for original in events:
        e=original.copy();arrival=e['received'];drop=False;duplicate=False
        for rule in SCENARIOS[scenario]:
            if not rule['start']<=arrival<rule['end'] or rule['sensor'] not in ['all',e['sensor']] or ('kind' in rule and rule['kind']!=e['kind']):continue
            action=rule['action']
            if action=='drop':drop=True
            elif action=='random_drop':drop=rng.random()<rule['probability']
            elif action=='delay':e['received']+=rule['seconds']
            elif action=='stamp_shift':e['stamp']+=rule['seconds']
            elif action=='duplicate':duplicate=True
            elif action=='vio_jump':e['pose']=e['pose'].copy();e['pose'][0]+=rule['metres']
            elif action=='thin':e['points']=e['points'][::20];e['rel']=e['rel'][::20]
        if not drop:
            serial+=1;heapq.heappush(pending,(e['received'],serial,e))
            if duplicate:
                serial+=1;again=e.copy();again['received']+=.002;heapq.heappush(pending,(again['received'],serial,again))
        while pending and pending[0][0]<=arrival:yield heapq.heappop(pending)[2]
    while pending:yield heapq.heappop(pending)[2]
