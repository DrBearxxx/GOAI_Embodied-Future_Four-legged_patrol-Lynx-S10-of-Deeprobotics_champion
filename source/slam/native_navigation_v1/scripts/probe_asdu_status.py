"""Bounded real-device protocol capture: heartbeats only, never motion commands."""
import fcntl
import json
from pathlib import Path
import socket
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_nav.protocol import Codec, heartbeat
from native_nav.gateway import live_preflight

live_preflight()
lock = open('/tmp/s10-native-navigation-gateway.lock', 'a')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(('10.21.33.103', 30004))
sock.settimeout(.1)
codec = Codec()
start = time.monotonic()
last = -1e9
samples = []
shown = set()
try:
    while time.monotonic()-start < 8:
        now = time.monotonic()
        if now-last >= 1:
            sock.send(codec.encode(heartbeat()))
            last = now
        try:
            raw = sock.recv(65536)
        except socket.timeout:
            continue
        sample = dict(received_mono=time.monotonic(), header_hex=raw[:16].hex(), size=len(raw))
        try:
            sample['decoded'] = codec.decode(raw)
        except Exception as exc:
            sample['error'] = repr(exc)
            sample['body_text'] = raw[16:].decode('utf-8', errors='replace')[:12000]
        samples.append(sample)
        d = sample.get('decoded', {})
        key = (d.get('Type'), d.get('Command'), sample.get('error'))
        if key not in shown:
            shown.add(key)
            print(json.dumps(sample, ensure_ascii=False), flush=True)
finally:
    sock.close()
out = Path(__file__).resolve().parents[1]/'results'
out.mkdir(exist_ok=True)
path = out/('asdu-status-'+str(time.time_ns())+'.json')
path.write_text(json.dumps(dict(motion_commands_sent=0, samples=samples), ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(capture=str(path), received=len(samples), motion_commands_sent=0)), flush=True)
