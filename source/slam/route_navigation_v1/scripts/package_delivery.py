import sys,tarfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s10nav.util import ROOT,read,write,sha
files=[ROOT/n for n in ('config.json','run_orin_shadow.sh','README_ZH.md','VALIDATION_REPORT.md')]
files += [p for folder in ['s10nav','scripts','tests','assets'] for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
manifest=dict(schema='s10.runtime_delivery.v1',motion_enabled=False,files={str(p.relative_to(ROOT)):sha(p) for p in sorted(files)})
write(ROOT/'delivery/runtime_manifest.json',manifest)
target=ROOT/'delivery/orin_route_shadow_final_v1.tar.gz'
with tarfile.open(target,'x:gz') as tar:
    for p in files:tar.add(p,arcname=str(p.relative_to(ROOT)),recursive=False)
    tar.add(ROOT/'delivery/runtime_manifest.json',arcname='runtime_manifest.json')
print(target,sha(target),target.stat().st_size,flush=True)
