"""Export a calibrated, display-only route snapshot for offline APK startup."""
import argparse
import json
from pathlib import Path
import shutil
import tempfile
import time

from planning import Plans


def export(calibration_dir, output, endpoint, route):
    root = Path(__file__).resolve().parent
    routes = {name: json.loads((root / 'routes' / (name + '.json')).read_text(encoding='utf-8'))
              for name in ('full', 'indoor')}
    # Plans may migrate presets; never write into the operator's saved data.
    with tempfile.TemporaryDirectory() as folder:
        private = Path(folder)
        for name in ('presets.json', 'route-calibration.json', 'recorded-routes.json'):
            source = calibration_dir / name
            if source.exists():
                shutil.copyfile(source, private / name)
            elif name != 'recorded-routes.json':
                raise FileNotFoundError(source)
        plans = Plans(routes, private / 'presets.json')
        catalog = plans.catalog()
        if route not in catalog:
            raise ValueError('Unknown route: ' + route)
        snapshot = dict(schema=1, endpoint=endpoint,
                        asset_id=json.loads((root / 'android/assets/map.json').read_text(encoding='utf-8'))['asset_id'],
                        saved_at=int(time.time() * 1000),
                        mission=dict(route_id=route, catalog=catalog,
                                     calibration_revision=plans.calibration.data['revision'],
                                     presets_revision=plans.revision))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(snapshot, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print(json.dumps(dict(output=str(output), route=route, points=len(catalog[route]['waypoints']),
                         calibration_revision=snapshot['mission']['calibration_revision']), ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--endpoint', default='http://10.21.33.102:18894')
    parser.add_argument('--route', default='full')
    args = parser.parse_args()
    export(args.calibration_dir, args.output, args.endpoint, args.route)
