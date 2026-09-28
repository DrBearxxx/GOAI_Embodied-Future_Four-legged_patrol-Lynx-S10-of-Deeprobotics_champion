import sys,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT.parent/'route_navigation_v1'
if not BASE.is_dir():BASE=Path('/home/wym/s10_route_navigation_v1')
lock=ROOT/'dependency_lock.json'
if lock.exists():
    for relative,expected in json.loads(lock.read_text())['files'].items():
        path=BASE/relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Frozen localization dependency changed: '+relative)
sys.path.insert(0,str(BASE))
