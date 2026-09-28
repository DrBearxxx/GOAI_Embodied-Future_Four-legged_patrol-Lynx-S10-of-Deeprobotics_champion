"""Linux zero-only companion. Best-effort software fallback, NOT hardware estop.

Shares the gateway's existing UDP socket; no second robot endpoint is opened.
If owned gateway heartbeats expire, kill that exact parent through pidfd before
sending zeros. A whole-host/kernel/network/receiver failure is NOT covered.
"""
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time
from .protocol import Codec, velocity

PULSE_LIMIT = .30
ACK_LIMIT = .15


class ZeroGuardian:
    def __init__(self, udp):
        if sys.platform != 'linux' or not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
            raise RuntimeError('LINUX_PIDFD_GUARDIAN_REQUIRED')
        self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.channel.setblocking(False)
        self.ack_mono = -1e9
        self.ack_owned = False
        self.sequence = 0
        parent_fd = os.pidfd_open(os.getpid())
        try:
            self.process = subprocess.Popen([sys.executable, '-m', 'native_nav.zero_guardian',
                str(udp.fileno()), str(child.fileno()), str(parent_fd)],
                cwd=Path(__file__).resolve().parents[1],
                pass_fds=(udp.fileno(), child.fileno(), parent_fd), start_new_session=True)
        except BaseException:
            self.channel.close()
            raise
        finally:
            child.close()
            os.close(parent_fd)

    def pulse(self, owned, codec):
        now = time.monotonic()
        self.sequence += 1
        try:
            self.channel.send(json.dumps(dict(mono=now, seq=self.sequence, owned=bool(owned),
                message=codec.message, packet=codec.packet)).encode())
            for _ in range(64):
                try:
                    ack = json.loads(self.channel.recv(1024))
                except BlockingIOError:
                    break
                if (type(ack.get('seq')) is int and ack['seq'] <= self.sequence
                        and 0 <= now-ack['mono'] <= ACK_LIMIT):
                    # Keep the issuance timestamp; queued ACKs cannot renew it.
                    self.ack_mono = max(self.ack_mono, ack['mono'])
                    self.ack_owned = ack.get('owned') is True
        except (OSError, ValueError, KeyError, TypeError):
            self.ack_mono = -1e9

    def healthy(self, require_owned=False):
        return (self.process.poll() is None and 0 <= time.monotonic()-self.ack_mono <= ACK_LIMIT
                and (not require_owned or self.ack_owned))

    def close(self):
        try:
            self.channel.send(b'{"finish":true}')
            self.process.wait(timeout=1.)
        except (OSError, subprocess.TimeoutExpired):
            # Do not kill a companion that may be in its zero-only fallback.
            pass
        self.channel.close()


def watch(udp_fd, channel_fd, parent_fd):
    udp = socket.socket(fileno=udp_fd)
    channel = socket.socket(fileno=channel_fd)
    channel.setblocking(False)
    codec = Codec()
    last, seq, owned = time.monotonic(), 0, False
    try:
        while True:
            ready, _, _ = select.select([channel, parent_fd], [], [], .02)
            if parent_fd in ready:
                break
            if channel in ready:
                try:
                    for _ in range(64):
                        try:
                            data = json.loads(channel.recv(1024))
                        except BlockingIOError:
                            break
                        if data.get('finish') is True:
                            return
                        now = time.monotonic()
                        if (type(data['seq']) is not int or data['seq'] <= seq or
                                type(data['owned']) is not bool or not 0 <= now-data['mono'] <= PULSE_LIMIT):
                            raise ValueError('INVALID_GUARDIAN_PULSE')
                        if (type(data['message']) is not int or not 0 <= data['message'] < 65536
                                or type(data['packet']) is not int or not 0 <= data['packet'] < 256):
                            raise ValueError('INVALID_CODEC_STATE')
                        last, seq = data['mono'], data['seq']
                        owned = owned or data['owned']
                        codec.message, codec.packet = data['message'], data['packet']
                        channel.send(json.dumps(dict(mono=last, seq=seq, owned=owned)).encode())
                except (OSError, ValueError, TypeError, KeyError):
                    break
            if time.monotonic()-last > PULSE_LIMIT:
                break
        if owned:
            try:
                signal.pidfd_send_signal(parent_fd, signal.SIGKILL)
            except ProcessLookupError:
                pass
            print('ZERO_GUARDIAN_FALLBACK: gateway ended/stalled; transmitting zeros; NOT stop confirmation', flush=True)
            # Codec state is the last received next-counter state. Repeated new
            # zero packets also advance beyond an in-flight final parent send.
            until = time.monotonic()+2.
            while time.monotonic() < until:
                try:
                    udp.send(codec.encode(velocity(0., 0., 0.)))
                except OSError:
                    pass
                time.sleep(.05)
    finally:
        udp.close()
        channel.close()
        os.close(parent_fd)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise SystemExit('Internal zero-only companion: launched by first-trial gateway')
    watch(*map(int, sys.argv[1:]))
