"""Create a source-only deployment archive and file hashes; never includes credentials."""
import hashlib
import json
from pathlib import Path
import tarfile

root = Path(__file__).resolve().parents[1]
files = sorted(p for p in root.rglob('*') if p.is_file() and p.suffix in ('.py', '.sh', '.md')
               and not set(p.relative_to(root).parts) & {'delivery', 'live_logs', 'results', '__pycache__'})
files.append(root/'acceptance.example.json')
files.append(root/'body_known_hosts')
manifest = {'schema': 's10.native.delivery.v1', 'files': {
    p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
delivery = root/'delivery'
delivery.mkdir(exist_ok=True)
manifest_path = delivery/'manifest.json'
manifest_path.write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
with tarfile.open(delivery/'native-navigation.tar.gz', 'w:gz') as tar:
    for p in files:
        tar.add(p, arcname=p.relative_to(root).as_posix())
    tar.add(manifest_path, arcname='manifest.json')
print(json.dumps({'files': len(files), 'archive': str(delivery/'native-navigation.tar.gz')}))
