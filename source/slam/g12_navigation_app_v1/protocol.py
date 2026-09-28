"""Authenticated read-only monitoring + idempotent, bounded localization jobs."""
import hashlib
import hmac
import math
import secrets
import time
import uuid
from collections import OrderedDict

def sign(key, path, raw):
    return hmac.new(key.encode(), path.encode()+b'\n'+raw, hashlib.sha256).hexdigest()

def seed_request(body, map_id, bounds):
    if body.get('map_id') != map_id:
        raise ValueError('MAP_MISMATCH')
    request_id = str(uuid.UUID(body['request_id']))
    xyz = body.get('xyz')
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise ValueError('INVALID_XYZ')
    numbers = xyz + [body.get('radius')]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in numbers):
        raise ValueError('NONFINITE_SEED')
    if not 1 <= body['radius'] <= 5:
        raise ValueError('RADIUS_OUT_OF_RANGE_1_5_M')
    if any(not bounds[0][i]-1 <= xyz[i] <= bounds[1][i]+1 for i in range(3)):
        raise ValueError('SEED_OUTSIDE_MAP')
    return dict(request_id=request_id, xyz=xyz, radius=float(body['radius']),
                height_tolerance=.8, issued=time.monotonic(), timeout=60.)

class Tickets:
    def __init__(self,ttl=15.): self.items = OrderedDict();self.ttl=ttl
    def issue(self, now):
        self.items = OrderedDict((k,v) for k,v in self.items.items() if now-v < self.ttl)
        token = secrets.token_hex(20)
        self.items[token] = now
        while len(self.items) > 128: self.items.popitem(last=False)
        return token
    def consume(self, token, now):
        issued = self.items.pop(token, None)
        return issued is not None and 0 <= now-issued < self.ttl
