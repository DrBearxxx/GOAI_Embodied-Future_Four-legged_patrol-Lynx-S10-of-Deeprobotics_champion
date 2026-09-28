"""Separate tmux command pane. No robot sockets and no automatic commands."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import secrets
import time
from .command_replies import wait_replies, COMMANDS

ROOT = Path(__file__).resolve().parents[1]
PANE_FORMAT = '#{pane_id}\t#{pane_dead}\t#{pane_pid}\t#{pane_in_mode}\t#{pane_start_command}'


def tmux(*args):
    result = subprocess.run(['tmux', *args], capture_output=True, text=True, timeout=2, check=True)
    return result.stdout.strip()


def pane_info(target):
    if not re.fullmatch(r'%[0-9]+', target):
        raise ValueError('Invalid pane id')
    fields = tmux('display-message', '-p', '-t', target, PANE_FORMAT).split('\t', 4)
    if len(fields) != 5 or fields[0] != target or fields[1] != '0':
        raise ValueError('Route pane has ended; no command sent')
    return fields


def route_identity(target):
    fields = pane_info(target)
    if fields[3] != '0':
        raise ValueError('Exit scroll/copy mode in the upper route pane first')
    pid = int(fields[2])
    proc = Path('/proc')/str(pid)
    args = [x.decode() for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
    cwd = (proc/'cwd').resolve(strict=True)
    # The production launchers exec Python, so pane_pid must be that exact
    # route process. Never send a command to a shell, a recycled pane or a CLI.
    if len(args) < 3 or not Path(args[0]).name.startswith('python') or '--execute' not in args[2:]:
        raise ValueError('Target is not the interactive route process; no command sent')
    if (cwd/args[1]).resolve() != (ROOT/'scripts/route_ros.py').resolve():
        raise ValueError('Target route executable does not match this project')
    return pid


def submit(target, expected_pid, command, request_id=None):
    name = command.strip().upper()
    if name not in COMMANDS:
        raise ValueError('Commands: '+', '.join(COMMANDS))
    if route_identity(target) != expected_pid:
        raise ValueError('Route process changed; reopen the console before retrying')
    if request_id is not None and not re.fullmatch(r'[a-f0-9]{32}', request_id):
        raise ValueError('Invalid request id')
    line = name if request_id is None else name+' '+request_id
    # Clear only an unfinished input line; emit one literal allowlisted command.
    # The original route still decides permission, readiness and acknowledgement.
    tmux('send-keys', '-t', target, 'C-u', ';',
         'send-keys', '-t', target, '-l', line, ';',
         'send-keys', '-t', target, 'Enter')
    return name


def install(session='s10-first-trial'):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', session):
        raise ValueError('Invalid session name')
    window = session+':route'
    rows = [x.split('\t', 4) for x in tmux('list-panes', '-t', window, '-F', PANE_FORMAT).splitlines()]
    script = str(ROOT/'scripts/route_console.py')
    inputs = [r for r in rows if len(r) == 5 and script in r[4] and '--target-pane' in r[4]]
    if inputs:
        if len(inputs) != 1 or inputs[0][1] != '0':
            raise ValueError('Command pane has ended; restart the session only after normal test shutdown')
        target = inputs[0][0]
    else:
        if len(rows) != 1 or len(rows[0]) != 5 or rows[0][1] != '0':
            raise ValueError('Expected one live route pane; no existing process was changed')
        original = rows[0][0]
        tmux('set-option', '-w', '-t', window, 'remain-on-exit', 'on')
        tmux('set-option', '-w', '-t', window, 'pane-border-status', 'top')
        tmux('select-pane', '-t', original, '-T', 'STATUS (commands and replies below)')
        target = tmux('split-window', '-v', '-l', '8', '-t', original, '-c', str(ROOT), '-P', '-F', '#{pane_id}',
                      sys.executable, script, '--target-pane', original)
        tmux('select-pane', '-t', target, '-T', 'COMMAND INPUT')
    tmux('select-pane', '-t', target)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--install', action='store_true', help='Split the existing route window; no restart or robot command')
    group.add_argument('--target-pane', help='Internal: exact tmux pane running route_ros.py')
    parser.add_argument('--session', default='s10-first-trial')
    args = parser.parse_args()
    if args.install:
        print('Command pane:', install(args.session))
        return
    if not sys.stdin.isatty() or not os.environ.get('TMUX'):
        parser.error('Use the command pane inside the live tmux session')
    # Import readline for line editing only; no history file and no input thread.
    try:
        import readline  # noqa: F401
    except ImportError:
        pass
    print('在这里输入命令；本次指令的结果和 ACK 直接显示在下方。', flush=True)
    print('SDK_OFF | STAND | MODE_NAV | FLAT | ARM | STOP | LIE | QUIT | STATUS', flush=True)
    print('Ctrl+C：发送 STOP；HELP：说明。启动此输入栏不会发送任何指令。', flush=True)
    pid = None
    while True:
        quitting = False
        try:
            line = input('command> ')
        except KeyboardInterrupt:
            print()
            line = 'STOP'
        except EOFError:
            print()
            line, quitting = 'STOP', True
        name = line.strip().upper()
        if name in ('', 'HELP'):
            if name:
                print('逐条输入；ACK 是已下发，反馈已确认才是目标状态。STATUS 查看当前状态/上一动作结果；QUIT 结束路线。')
            continue
        try:
            if name not in COMMANDS:
                raise ValueError('Unknown command; type HELP')
            if pid is None:
                pid = route_identity(args.target_pane)
            request_id, since = secrets.token_hex(16), time.monotonic()
            sent = submit(args.target_pane, pid, name, request_id=request_id)
            try:
                wait_replies(ROOT/'live_logs/command-replies.jsonl', pid, request_id, since)
            except KeyboardInterrupt:
                # An operator STOP remains available while waiting for an ACK.
                stop_id, stopped_at = secrets.token_hex(16), time.monotonic()
                submit(args.target_pane, pid, 'STOP', request_id=stop_id)
                print('\n已请求 STOP；不重发上一条指令。', flush=True)
                wait_replies(ROOT/'live_logs/command-replies.jsonl', pid, stop_id, stopped_at)
            if sent == 'QUIT':
                break
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            print('未发送：'+str(exc), flush=True)
        if quitting:
            break


if __name__ == '__main__':
    main()
