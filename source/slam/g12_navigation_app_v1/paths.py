"""Isolated app overlay; all prior map/control projects remain read-only."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
def project(local, remote):
    p = ROOT.parent / local
    return p if p.is_dir() else Path('/home/wym') / remote
BASE = project('route_navigation_v1', 's10_route_navigation_v1')
INDOOR = project('indoor_navigation_v1', 's10_indoor_navigation_v1')
NATIVE = project('native_navigation_v1', 's10_native_navigation_v1')
ASSETS = BASE / 'assets'
for p in (BASE, INDOOR, NATIVE):
    sys.path.insert(0, str(p))

def verify():
    from native_nav.bootstrap import verify as verify_dependencies
    verify_dependencies()
    m = json.loads((ASSETS / 'manifest.json').read_text())
    for name, digest in m['files'].items():
        if hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() != digest:
            raise ValueError('MAP_ASSET_CHANGED:' + name)
    return hashlib.sha256((ASSETS / 'manifest.json').read_bytes()).hexdigest()
