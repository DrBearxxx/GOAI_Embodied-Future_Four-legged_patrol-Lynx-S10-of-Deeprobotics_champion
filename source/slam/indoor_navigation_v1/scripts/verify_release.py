import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'delivery_manifest.json').read_text());bad=[]
for name,expected in manifest['files'].items():
    p=(root/name).resolve()
    if not p.is_relative_to(root) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:bad.append(name)
if bad:raise SystemExit('Release mismatch: '+repr(bad))
base=root.parent/'route_navigation_v1'
if not base.is_dir():base=Path('/home/wym/s10_route_navigation_v1')
lock=json.loads((root/'dependency_lock.json').read_text())
for name,expected in lock['files'].items():
    if hashlib.sha256((base/name).read_bytes()).hexdigest()!=expected:raise SystemExit('Dependency changed:'+name)
assert hashlib.sha256((base/'assets/route.json').read_bytes()).hexdigest()==lock['original_66_route_sha256']
f=root/'sdk_ws/install/drdds/share/drdds/msg/MotionInfo.json'
if f.exists():assert json.loads(f.read_text())['type_hashes'][0]['hash_string']=='RIHS01_a7d7f26e0f1152b10643f0e8bed1c9fd7b7f574170f3460335e381c40070830b'
print(json.dumps(dict(files=len(manifest['files']),checksums_passed=True,baseline_core_unchanged=True,original_66_route_unchanged=True)))
