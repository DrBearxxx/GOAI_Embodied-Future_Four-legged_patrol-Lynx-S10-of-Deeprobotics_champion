"""One command builds a supervised tmux session. Default is an inert description."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SESSION = 's10-first-trial'


def navigation_running():
    names = {'supervise_localizer.py', 'robust_localize_mp.py', 'route_ros.py'}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry/'cmdline').read_bytes().split(b'\0')[:3]
        except FileNotFoundError:
            continue
        if any(Path(x.decode(errors='replace')).name in names for x in args if x):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true', help='Start sensor/localizer, SSH gateway and interactive trial panes; never auto-ARM')
    args = parser.parse_args()
    if not args.start:
        print(json.dumps(dict(mode='DESCRIPTION_ONLY', command='bash run_first_trial.sh --start',
            distance_m=.50, max_vx_mps=.20, acceptance='UNVERIFIED', automatic_stand=False,
            automatic_arm=False, windows=['body (SSH password)', 'localizer', 'route (explicit commands)']), indent=2))
        return
    if not sys.stdin.isatty() or not shutil.which('tmux'):
        parser.error('Interactive Orin terminal and tmux required')
    from native_nav.bootstrap import verify
    verify()
    subprocess.run([sys.executable, str(ROOT/'scripts/verify_delivery.py')], check=True)
    from native_nav.operator_console import install as install_console
    if subprocess.run(['tmux', 'has-session', '-t', SESSION], capture_output=True).returncode == 0:
        install_console(SESSION)
        print('Existing session: attach only; no processes restarted.', flush=True)
        subprocess.run(['tmux', 'attach-session', '-t', SESSION], check=True)
        return
    from native_nav.gateway import ownership_check
    if not ownership_check():
        parser.error('COMPETING_CONTROLLER: first have the robot lie normally, then stop the existing controller. No controller was killed.')
    if navigation_running():
        parser.error('Existing localizer/route detected: do not start a duplicate instance')
    if not Path('/home/wym/s10_slam/env.sh').is_file():
        parser.error('Orin sensor environment missing')
    pane = str(ROOT/'scripts/first_trial_pane.sh')
    subprocess.run(['tmux', 'new-session', '-d', '-s', SESSION, '-n', 'body', '-c', str(ROOT),
                    'bash', pane, 'body'], check=True)
    subprocess.run(['tmux', 'set-option', '-w', '-t', SESSION+':body', 'remain-on-exit', 'on'], check=True)
    for name in ('localizer', 'route'):
        subprocess.run(['tmux', 'new-window', '-t', SESSION, '-n', name, '-c', str(ROOT),
                        'bash', pane, name], check=True)
        subprocess.run(['tmux', 'set-option', '-w', '-t', SESSION+':'+name, 'remain-on-exit', 'on'], check=True)
    install_console(SESSION)
    subprocess.run(['tmux', 'select-window', '-t', SESSION+':body'], check=True)
    print('Body window: enter SSH password. Ctrl+B then W: choose route, type at lower command> prompt.', flush=True)
    subprocess.run(['tmux', 'attach-session', '-t', SESSION], check=True)


if __name__ == '__main__':
    main()
