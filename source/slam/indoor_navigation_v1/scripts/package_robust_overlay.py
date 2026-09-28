"""Build a versioned v2 overlay against the exact installed v1 manifest."""
import hashlib,json,tarfile
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
old_path=root/'delivery/goai_overlay_v1_manifest.json'
if not old_path.exists():
    old=json.loads((root/'goai_overlay_manifest.json').read_text());assert old['schema']=='goai.indoor.overlay.v1'
    old_path.parent.mkdir(exist_ok=True);old_path.write_bytes((root/'goai_overlay_manifest.json').read_bytes())
old=json.loads(old_path.read_text());assert old['schema']=='goai.indoor.overlay.v1'
names=list(old['files'])+['indoor/continuity.py','indoor/safety_stream.py','tests/test_continuity.py',
    'scripts/replay_robustness.py','scripts/package_robust_overlay.py','scripts/deploy_robust_overlay.py',
    'scripts/validate_robust_readonly.py','ROBUSTNESS_V2.md','ROBUSTNESS_VALIDATION.md']
previous={**old['files'],'goai_overlay_manifest.json':sha(old_path)}
m=dict(schema='goai.indoor.overlay.v2',files={n:sha(root/n) for n in names},
    requirements=old['requirements'],asset_id=old['asset_id'],previous=previous)
(root/'goai_overlay_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
out=root/'delivery/goai_robust_v2_20260914.tar.gz'
with tarfile.open(out,'w:gz') as tar:
    for name in names+['goai_overlay_manifest.json']:tar.add(root/name,arcname=name,recursive=False)
print(json.dumps(dict(archive=str(out),files=len(names),sha256=sha(out))))
