"""Minimal overlay: preserve unrelated deployed files and all map assets."""
import hashlib,json,tarfile
from pathlib import Path
root=Path(__file__).resolve().parents[1]
names=['indoor/control.py','indoor/goai_control.py','indoor/transport.py','indoor/pipeline.py','scripts/localize_mp.py',
       'scripts/goai_route.py','scripts/test_goai_isolated.py','scripts/package_goai_overlay.py','scripts/verify_goai_overlay.py','scripts/deploy_goai_overlay.py',
       'tests/test_goai_control.py','tests/test_goai_audit.py','run_goai_route.sh','run_goai_localizer.sh','run_goai_isolated_test.sh']
optional=['GOAI导航操作.md','GOAI_NAV_VALIDATION.md','scripts/audit_goai_route.py']
names += [n for n in optional if (root/n).exists()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=dict(schema='goai.indoor.overlay.v1',files={n:sha(root/n) for n in names},
    requirements={n:sha(root/n) for n in ['config.json','dependency_lock.json','run_localizer.sh','indoor/bootstrap.py','indoor/clock.py','indoor/messages.py']},
    asset_id=sha(root/'assets/manifest.json'),
    previous={'indoor/control.py':'7638ab82e4fb3f44550c9285b486e2d31a31d27604d693ed1dac4f889dadd244',
              'indoor/pipeline.py':'bb5db30bbdd5e02e7def1cc6d07d43fdac8ec138f9897c85325ce5f3ad578d53',
              'scripts/localize_mp.py':'d88d44df7dabd0f29d0ede79ad01d414f73e29ab36f705fe9b43894ce5baa51b'})
(root/'goai_overlay_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
out=root/'delivery/goai_overlay.tar.gz';out.parent.mkdir(exist_ok=True)
with tarfile.open(out,'w:gz') as tar:
    for name in names+['goai_overlay_manifest.json']:tar.add(root/name,arcname=name,recursive=False)
print(json.dumps(dict(archive=str(out),files=len(names),sha256=sha(out))))
