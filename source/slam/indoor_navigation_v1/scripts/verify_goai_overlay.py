"""Verify the GOAI overlay plus frozen map/core without rewriting old manifests."""
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((root/'goai_overlay_manifest.json').read_text())
for name,h in {**m['files'],**m['requirements']}.items():
    p=(root/name).resolve()
    if not p.is_relative_to(root) or not p.is_file() or sha(p)!=h:raise SystemExit('GOAI overlay mismatch: '+name)
assets=root/'assets';assert sha(assets/'manifest.json')==m['asset_id']
for name,h in json.loads((assets/'manifest.json').read_text())['files'].items():assert sha(assets/name)==h,name
base=root.parent/'route_navigation_v1'
if not base.is_dir():base=Path('/home/wym/s10_route_navigation_v1')
lock=json.loads((root/'dependency_lock.json').read_text())
for name,h in lock['files'].items():assert sha(base/name)==h,name
assert sha(base/'assets/route.json')==lock['original_66_route_sha256']
print(json.dumps(dict(goai_overlay_verified=True,files=len(m['files']),assets_unchanged=True,baseline_core_unchanged=True,original_route_unchanged=True)))
