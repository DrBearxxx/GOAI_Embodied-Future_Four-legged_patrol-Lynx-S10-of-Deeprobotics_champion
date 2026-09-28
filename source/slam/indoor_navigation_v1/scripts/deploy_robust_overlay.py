"""Explicit versioned deployment, no automatic process launch or map changes."""
import hashlib,json,shutil,subprocess,tarfile
from pathlib import Path
source=Path('/home/wym/s10_indoor_robust_candidate_20260914').resolve()
target=Path('/home/wym/s10_indoor_navigation_v1').resolve()
backup=Path('/home/wym/s10_indoor_before_robust_v2_20260914.tar.gz')
assert source.is_dir() and target.is_dir() and not backup.exists()
# Never replace code under a running route/localizer/GOAI controller.
for p in Path('/proc').glob('[0-9]*/cmdline'):
    try:args=[Path(v.decode()).name for v in p.read_bytes().split(b'\0') if v]
    except (OSError,UnicodeDecodeError):continue
    assert not set(args)&{'goai_node','goai_route.py','localize_mp.py','localize.py'},'Stop active navigation/localization before deployment'
subprocess.run(['python3',str(source/'scripts/verify_goai_overlay.py')],check=True)
m=json.loads((source/'goai_overlay_manifest.json').read_text());assert m['schema']=='goai.indoor.overlay.v2'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(target/'assets/manifest.json')==m['asset_id']
names=[*m['files'],'goai_overlay_manifest.json']
for name in names:
    dst=(target/name).resolve();src=(source/name).resolve()
    assert dst.is_relative_to(target) and src.is_relative_to(source)
    if dst.exists():assert name in m['previous'] and sha(dst)==m['previous'][name],'Existing destination changed: '+name
    else:assert name not in m['previous'],'Missing previous file: '+name
for name,h in m['requirements'].items():assert sha(target/name)==h,name
with tarfile.open(backup,'x:gz') as tar:
    for name in [*names,'delivery_manifest.json']:
        if (target/name).is_file():tar.add(target/name,arcname=name,recursive=False)
for name in names:
    dst=target/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/name,dst)
subprocess.run(['python3',str(target/'scripts/verify_goai_overlay.py')],check=True)
print(json.dumps(dict(deployed=str(target),files=len(names),backup=str(backup),backup_sha256=sha(backup),started_hardware=False)))
