"""Poll loopback gateway. No robot commands, mode changes, or implicit rearming."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_nav.client import Client

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--port', type=int, default=18891)
p.add_argument('--seconds', type=float, default=5)
a = p.parse_args()
c = Client(a.port)
start = time.monotonic()
printed = -1e9
try:
    while not c.closed and time.monotonic()-start < a.seconds:
        now = time.monotonic()
        c.poll(now)
        status = c.get(now)
        if status and now-printed >= 1:
            print(json.dumps({k:v for k,v in status.items() if k not in ('session','ticket','id')}), flush=True)
            printed = now
        time.sleep(.005)
    if c.error:
        print(c.error, file=sys.stderr)
        sys.exit(1)
finally:
    c.close()
