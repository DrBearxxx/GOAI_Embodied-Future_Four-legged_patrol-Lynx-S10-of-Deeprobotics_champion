"""Standard-library verification; no ROS, network or motor interfaces."""
import json,hashlib,sys,math,platform
from pathlib import Path
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'runtime_manifest.json').read_text())
errors=[]
for name,expected in manifest['files'].items():
    path=(root/name).resolve()
    if not path.is_relative_to(root.resolve()):raise ValueError('Invalid manifest path')
    h=hashlib.sha256(path.read_bytes()).hexdigest()
    if h!=expected:errors.append(name)
route=json.loads((root/'assets/route.json').read_text());original=json.loads((root/'assets/original_waypoints.json').read_text())
assert [(p['id'],p['xyz']) for p in route['waypoints']]==[(p['id'],p['xyz']) for p in original['waypoints']]
for a,b in zip(route['waypoints'][:-1],route['waypoints'][1:]):
    angle=math.atan2(b['xyz'][1]-a['xyz'][1],b['xyz'][0]-a['xyz'][0])
    assert abs(math.atan2(math.sin(a['yaw']-angle),math.cos(a['yaw']-angle)))<1e-9
result=dict(platform=platform.machine(),files_verified=len(manifest['files']),mismatches=errors,waypoints_preserved=len(route['waypoints']),
            all_yaws_toward_next=True,mode='shadow_only',autonomous_navigation_released=False)
(root/'delivery_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
if errors:raise SystemExit(1)
