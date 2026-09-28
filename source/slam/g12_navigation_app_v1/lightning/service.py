"""Manage only the owned Lightning estimator/bridge pair; never controls motion."""
import argparse
import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid

APP = Path(__file__).resolve().parents[1]
ROOT = APP/'lightning'
SESSION = 'goai-navigation-lightning'
LOGS = APP/'logs'
DEFAULT_RUNTIME = '/home/wym/s10_lightning_runtime_v1'


def command_line(runtime, epoch, sensor):
    binary = Path(runtime)/'install/lightning_lio_validation/lib/lightning_lio_validation/lightning_lio'
    if not binary.is_file():
        raise FileNotFoundError(f'Lightning binary missing: {binary}; run the included build_lightning.sh first')
    return [str(binary), '--ros-args', '-p', f'config:={ROOT / "lightning.yaml"}',
        '-r', '__ns:=/wym/navigation/lightning/estimator',
        '-r', 'points:=/wym/navigation/lightning/input/points',
        '-r', 'imu:=/wym/navigation/lightning/input/imu'], [
        sys.executable, str(ROOT/'bridge.py'), '--epoch', epoch, '--sensor', sensor,
        '--status-file', str(LOGS/'lightning-status.json')]


def stop_children(children):
    # Groups are created by this process, never discovered by a name substring.
    for child in children:
        if child.poll() is None:
            try:os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:pass
    for child in children:
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:pass
            child.wait(timeout=2)


def supervise(runtime):
    LOGS.mkdir(exist_ok=True)
    lock = (LOGS/'lightning-service.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Lightning supervisor already running')
    config = json.loads((APP/'odometry.json').read_text())
    contract = json.loads((ROOT/'contract.json').read_text())
    if config.get('sensor') != contract['sensor'] or config.get('T_body_imu') != contract['T_body_imu']:
        raise SystemExit('Navigation / Lightning calibration mismatch')
    if config['backend'] != 'lightning':
        raise SystemExit('Configured odometry backend is not Lightning')
    logger = logging.getLogger('lightning-service')
    logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(LOGS/'lightning-runtime.log', maxBytes=8_000_000, backupCount=4)
    handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    logger.addHandler(handler)
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())

    def pump(pipe, label):
        for line in iter(pipe.readline, ''):
            logger.info('%s %s', label, line.rstrip())
        pipe.close()

    try:
        while not stop.is_set():
            epoch = str(uuid.uuid4())
            cmds = command_line(runtime, epoch, config['sensor'])
            children, readers = [], []
            logger.info('Starting estimator pair epoch=%s', epoch)
            try:
                for label, cmd in zip(('estimator', 'bridge'), cmds):
                    child = subprocess.Popen(cmd, start_new_session=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True, errors='replace', bufsize=1)
                    children.append(child)
                    thread = threading.Thread(target=pump, args=(child.stdout, label), daemon=True)
                    thread.start()
                    readers.append(thread)
                while not stop.wait(.2):
                    if any(child.poll() is not None for child in children):
                        logger.error('Pair exited: %s; both restart in a new epoch', [c.poll() for c in children])
                        break
            finally:
                stop_children(children)
                for reader in readers:
                    reader.join(2)
            stop.wait(1.)
    finally:
        handler.close()
        lock.close()


def running():
    return subprocess.run(['tmux', 'has-session', '-t', '='+SESSION], capture_output=True).returncode == 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['start', 'stop', 'status', 'run'])
    p.add_argument('--runtime', default=os.environ.get('GOAI_LIGHTNING_RUNTIME', DEFAULT_RUNTIME))
    args = p.parse_args()
    if args.action == 'run':
        supervise(args.runtime)
    elif args.action == 'start':
        command_line(args.runtime, 'preflight', 'front')
        if not running():
            command = 'exec env GOAI_LIGHTNING_RUNTIME='+shlex.quote(args.runtime)+' bash '+shlex.quote(str(ROOT/'run.sh'))
            subprocess.run(['tmux', 'new-session', '-d', '-s', SESSION, command], check=True)
        print('Lightning service started/reused; no motion commands sent')
    elif args.action == 'stop':
        if running():
            # Ctrl-C is handled by supervisor, which terminates both owned groups.
            subprocess.run(['tmux', 'send-keys', '-t', '='+SESSION, 'C-c'], check=True)
            for _ in range(60):
                if not running():
                    break
                time.sleep(.2)
            if running():
                raise SystemExit('Lightning did not stop; inspect its session before retrying')
        print('Lightning stopped')
    else:
        status = LOGS/'lightning-status.json'
        data = json.loads(status.read_text()) if status.exists() else {}
        data['running'] = running()
        data['status_age_s'] = time.monotonic()-data['monotonic'] if 'monotonic' in data else None
        data['pose_age_s'] = time.monotonic()-data['last_pose_mono'] if data.get('last_pose_mono') else None
        print(json.dumps(data, indent=2))


if __name__ == '__main__':
    main()
