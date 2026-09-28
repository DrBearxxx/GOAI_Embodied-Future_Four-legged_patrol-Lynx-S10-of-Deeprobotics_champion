"""Read authenticated hardware diagnostics without acquiring a control session."""
import hashlib
import hmac
import json
from pathlib import Path
import urllib.request
import uuid


def query():
    key=(Path(__file__).parent/'private/pairing.key').read_text().strip().encode()
    path='/status'
    nonce=str(uuid.uuid4())
    raw=json.dumps(dict(nonce=nonce),separators=(',',':')).encode()
    def signature(data):return hmac.new(key,path.encode()+b'\n'+data,hashlib.sha256).hexdigest()
    request=urllib.request.Request('http://127.0.0.1:18895'+path,data=raw,
                                  headers={'X-S10-MAC':signature(raw)})
    with urllib.request.urlopen(request,timeout=2) as response:
        body=response.read(65537)
        if len(body)>65536 or not hmac.compare_digest(signature(body),response.headers.get('X-S10-MAC','')):
            raise ValueError('BAD_GATEWAY_SIGNATURE')
    result=json.loads(body)
    if result.get('nonce')!=nonce:raise ValueError('STATUS_NONCE_MISMATCH')
    result.pop('nonce',None)
    return result


if __name__=='__main__':print(json.dumps(query(),ensure_ascii=False,indent=2))
