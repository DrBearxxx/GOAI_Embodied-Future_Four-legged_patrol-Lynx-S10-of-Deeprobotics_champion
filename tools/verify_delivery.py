"""Read-only SHA-256 verification for the delivery package. No robot access."""
from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parent.parent
manifest=json.loads((root/'PACKAGE_MANIFEST.json').read_text(encoding='utf-8'))
failures=[]
for e in manifest['files']:
 p=(root/e['path']).resolve()
 if not p.is_relative_to(root) or not p.is_file():failures.append(e['path']+': missing/invalid');continue
 if p.stat().st_size!=e['bytes'] or hashlib.sha256(p.read_bytes()).hexdigest()!=e['sha256']:failures.append(e['path']+': modified')
print(json.dumps({'ok':not failures,'checked':len(manifest['files']),'failures':failures},ensure_ascii=False,indent=2))
sys.exit(bool(failures))
