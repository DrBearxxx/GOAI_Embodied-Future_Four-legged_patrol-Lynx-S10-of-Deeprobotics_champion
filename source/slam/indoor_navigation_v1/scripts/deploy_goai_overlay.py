"""Install only verified overlay files, with a recoverable pre-change backup."""
import hashlib,json,shutil,subprocess,tarfile
from pathlib import Path
source=Path('/home/wym/s10_indoor_goai_candidate_20260913').resolve()
target=Path('/home/wym/s10_indoor_navigation_v1').resolve()
backup=Path('/home/wym/s10_indoor_before_goai_nav_20260913.tar.gz')
assert source.is_dir() and target.is_dir() and not backup.exists()
subprocess.run(['python3',str(source/'scripts/verify_goai_overlay.py')],check=True)
m=json.loads((source/'goai_overlay_manifest.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(target/'assets/manifest.json')==m['asset_id']
names=[*m['files'],'goai_overlay_manifest.json']
for name in names:
    dst=(target/name).resolve();src=(source/name).resolve()
    assert dst.is_relative_to(target) and src.is_relative_to(source)
    if dst.exists():
        assert name in m['previous'] and sha(dst)==m['previous'][name], 'Existing destination changed: '+name
for name,h in m['requirements'].items():assert sha(target/name)==h,name
with tarfile.open(backup,'x:gz') as tar:
    for name in [*names,'delivery_manifest.json']:
        if (target/name).is_file():tar.add(target/name,arcname=name,recursive=False)
for name in names:
    dst=target/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/name,dst)
subprocess.run(['python3',str(target/'scripts/verify_goai_overlay.py')],check=True)
print(json.dumps(dict(deployed=str(target),files=len(names),backup=str(backup),backup_sha256=sha(backup),started_hardware=False)))
