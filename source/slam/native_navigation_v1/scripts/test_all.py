"""Run isolated tests and retain an explicit non-physical validation report."""
import datetime
import json
from pathlib import Path
import sys
import unittest

root = Path(__file__).resolve().parents[1]
suite = unittest.defaultTestLoader.discover(str(root/'tests'))
result = unittest.TextTestRunner(verbosity=2).run(suite)
out = root/'results'
out.mkdir(exist_ok=True)
stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
report = dict(tests=result.testsRun, success=result.wasSuccessful(), errors=len(result.errors),
              failures=len(result.failures), physical_motion=False,
              scope='unit tests, loopback TCP and ideal route kinematics; not robot control or localization accuracy')
path = out/('software-validation-'+stamp+'.json')
path.write_text(json.dumps(report, indent=2)+'\n')
print(path)
sys.exit(0 if result.wasSuccessful() else 1)
