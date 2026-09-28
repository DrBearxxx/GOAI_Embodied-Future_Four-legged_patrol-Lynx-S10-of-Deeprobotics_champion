"""Stage pinned estimator sources into a new owned runtime; no downloads."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parent


def stage(destination):
    target=Path(destination).resolve()
    if target==ROOT or ROOT.is_relative_to(target) or target.is_relative_to(ROOT):
        raise RuntimeError('Runtime directory must be separate from the deployment package')
    manifest=json.loads((ROOT/'runtime-manifest.json').read_text())
    for name,digest in manifest.items():
        path=(ROOT/'runtime-source'/name).resolve()
        if not path.is_relative_to((ROOT/'runtime-source').resolve()):
            raise RuntimeError('Invalid manifest path')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Runtime package checksum mismatch: '+name)
    marker=target/'goai-runtime-source.json'
    if target.exists() and any(target.iterdir()):
        if not marker.exists() or json.loads(marker.read_text())!=manifest:
            raise RuntimeError('Runtime directory contains a different/unmanaged source; use a new directory')
        for name,digest in manifest.items():
            if hashlib.sha256((target/name).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Runtime source was edited: '+name)
    else:
        target.mkdir(parents=True,exist_ok=True)
        shutil.copytree(ROOT/'runtime-source',target,dirs_exist_ok=True)
        marker.write_text(json.dumps(manifest,indent=2))
    print(target)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination')
    stage(parser.parse_args().destination)
