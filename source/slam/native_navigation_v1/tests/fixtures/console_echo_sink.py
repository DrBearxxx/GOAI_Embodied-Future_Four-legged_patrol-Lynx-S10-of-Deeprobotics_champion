"""Disposable tmux test fixture: echoes stdin; no network, ROS or robot imports."""
import sys
import json
import os
from pathlib import Path
import time

print('ECHO SINK READY - NO ROBOT CONNECTION', flush=True)
for line in sys.stdin:
    print('ECHO:'+line.strip(), flush=True)
    parts = line.split()
    if len(parts) == 2 and len(parts[1]) == 32:
        path = Path(__file__).resolve().parents[1]/'live_logs/command-replies.jsonl'
        path.parent.mkdir(exist_ok=True)
        with path.open('a') as stream:
            stream.write(json.dumps(dict(pid=os.getpid(), request_id=parts[1], command=parts[0],
                         mono=time.monotonic(), phase='REJECTED', reason='ECHO_FIXTURE_NO_ROBOT',
                         ok=False, final=True))+'\n')
