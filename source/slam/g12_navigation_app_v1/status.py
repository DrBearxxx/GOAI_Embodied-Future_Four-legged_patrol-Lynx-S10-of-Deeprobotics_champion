"""Authenticated read-only status; never prints pairing material."""
import json
import urllib.request
import uuid
from pathlib import Path
from protocol import sign
key=(Path(__file__).parent/'private/pairing.key').read_text().strip()
path='/state';raw=json.dumps({'nonce':str(uuid.uuid4())}).encode()
req=urllib.request.Request('http://10.21.33.102:18894'+path,data=raw,headers={'X-S10-MAC':sign(key,path,raw)})
with urllib.request.urlopen(req,timeout=2) as r:
    body=r.read()
    if sign(key,path,body)!=r.headers.get('X-S10-MAC'):raise ValueError('BAD_SIGNATURE')
    d=json.loads(body)
print(json.dumps({k:d.get(k) for k in ('mode','valid','pose','sensors','navigation','control')},ensure_ascii=False,indent=2))
