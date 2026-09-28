"""Run the actual console UI against the disposable test project, never ROS."""
import os
from pathlib import Path
import sys

sys.path.insert(0, os.environ['S10_CONSOLE_TEST_SOURCE'])
from native_nav import operator_console

operator_console.ROOT = Path(os.environ['S10_CONSOLE_TEST_PROJECT'])
operator_console.main()
