"""Read-only failure-window diagnostics for a completed replay."""
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); lo=float(sys.argv[2]); hi=float(sys.argv[3])
for line in (p/'events.jsonl').open():
    e=json.loads(line);t=e['received']
    if lo<=t<=hi:
        m=e.get('metrics',{});q=m.get('quality',{});h=e.get('health',{})
        print(json.dumps(dict(t=round(t,3),ok=e['accepted'],state=h.get('state'),reason=e.get('reason'),
              sensor=m.get('sensor'),pred=m.get('prediction'),correction=m.get('correction'),
              q=q,source_age=m.get('source_age_s'))))
