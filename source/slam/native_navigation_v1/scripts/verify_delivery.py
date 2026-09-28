import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
for name, expected in json.loads((root/'manifest.json').read_text())['files'].items():
    p = (root/name).resolve()
    if not p.is_relative_to(root) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
        raise SystemExit('DELIVERY_MISMATCH: '+name)
print('Native source deployment hashes verified')
