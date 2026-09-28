"""Check foreground RC and a zero-output manual shadow session, then stop."""
import json
import time
import urllib.request
import uuid
from pathlib import Path
from protocol import sign
key=(Path(__file__).parent/'private/pairing.key').read_text().strip()
def rpc(path,data):
    data=dict(data,nonce=str(uuid.uuid4()));raw=json.dumps(data).encode()
    req=urllib.request.Request('http://10.21.33.102:18894'+path,data=raw,headers={'X-S10-MAC':sign(key,path,raw)})
    with urllib.request.urlopen(req,timeout=2) as r:
        value=r.read()
        assert sign(key,path,value)==r.headers.get('X-S10-MAC')
        return json.loads(value)
counts={};states={};started=False;response={}
try:
    for i in range(60):
        s=rpc('/state',{});m=s.get('mission',{});n=s.get('navigation',{})
        counts[str(m.get('rc_fresh'))]=counts.get(str(m.get('rc_fresh')),0)+1
        state=n.get('owner','?')+':'+n.get('state','?');states[state]=states.get(state,0)+1
        if not started and m.get('rc_fresh'):
            response=rpc('/navigation',dict(map_id=s['map_id'],ticket=s['command_ticket'],request_id=str(uuid.uuid4()),action='manual_shadow'))
            started=response.get('ok',False)
        time.sleep(.1)
finally:
    rpc('/stop',{})
    time.sleep(.15)
print(json.dumps(dict(samples=counts,states=states,start=response,final=rpc('/state',{}).get('navigation')),ensure_ascii=False,indent=2))
