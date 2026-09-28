"""Generate checksummed release artifacts; do not alter original map or core."""
import hashlib,json,tarfile
from pathlib import Path
root=Path(__file__).resolve().parents[1];base=root.parent/'route_navigation_v1'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
lock=dict(files={p.relative_to(base).as_posix():digest(p) for p in sorted((base/'s10nav').glob('*.py'))},original_66_route_sha256=digest(base/'assets/route.json'))
(root/'dependency_lock.json').write_text(json.dumps(lock,indent=2)+'\n')
paths=[root/n for n in ['config.json','dependency_lock.json','build_sdk.sh','run_localizer.sh','run_sdk.sh','run_isolated_test.sh','start_monitor_tmux.sh','README_ZH.md','SDK_INTERFACE_EVIDENCE.md','VALIDATION_REPORT.md','indoor_route.png']]
for folder in ['indoor','scripts','tests','sdk_ws/src','assets']:
    paths.extend(p for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
files={p.relative_to(root).as_posix():digest(p) for p in sorted(paths)}
delivery=root/'delivery';delivery.mkdir(exist_ok=True)
(root/'delivery_manifest.json').write_text(json.dumps(dict(files=files),indent=2)+'\n')
with tarfile.open(delivery/'indoor_release.tar.gz','w:gz') as tar:
    for p in paths+[root/'delivery_manifest.json']:tar.add(p,arcname=p.relative_to(root).as_posix(),recursive=False)
print(json.dumps(dict(files=len(files),tar_sha256=digest(delivery/'indoor_release.tar.gz'),archive=str(delivery/'indoor_release.tar.gz'))))
