"""Read-only dependency verification. Never rewrites old manifests or maps."""
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
INDOOR = ROOT.parent / 'indoor_navigation_v1'
if not INDOOR.is_dir():
    INDOOR = Path('/home/wym/s10_indoor_navigation_v1')
MANIFEST_SHA = '3ceb28ba871603586f3e8b5e8d902192ca3474dd0d8bcd294dd79ab7eb628cfa'


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def verify():
    manifest = INDOOR / 'goai_overlay_manifest.json'
    if digest(manifest) != MANIFEST_SHA:
        raise ValueError('FROZEN_V2_MANIFEST_CHANGED')
    m = json.loads(manifest.read_text())
    for relative, sha in {**m['files'], **m['requirements']}.items():
        p = (INDOOR / relative).resolve()
        if not p.is_relative_to(INDOOR.resolve()) or digest(p) != sha:
            raise ValueError('FROZEN_V2_FILE_CHANGED:' + relative)
    assets = INDOOR / 'assets'
    if digest(assets / 'manifest.json') != m['asset_id']:
        raise ValueError('ASSET_MANIFEST_CHANGED')
    for relative, sha in json.loads((assets / 'manifest.json').read_text())['files'].items():
        if digest(assets / relative) != sha:
            raise ValueError('MAP_OR_ROUTE_CHANGED:' + relative)
    base = INDOOR.parent / 'route_navigation_v1'
    if not base.is_dir():
        base = Path('/home/wym/s10_route_navigation_v1')
    lock = json.loads((INDOOR / 'dependency_lock.json').read_text())
    for relative, sha in lock['files'].items():
        if digest(base / relative) != sha:
            raise ValueError('FROZEN_CORE_CHANGED:' + relative)
    if digest(base / 'assets/route.json') != lock['original_66_route_sha256']:
        raise ValueError('ORIGINAL_ROUTE_CHANGED')
    return m['asset_id']


sys.path.insert(0, str(INDOOR))
