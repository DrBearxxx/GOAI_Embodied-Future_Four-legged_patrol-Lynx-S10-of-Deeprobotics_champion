"""Bounded, nonblocking localhost client. A late POLL response is never fresh."""
import json
import secrets
import socket
from collections import deque


class Client:
    def __init__(self, port=18891):
        self._initialize(socket.create_connection(('127.0.0.1', port), timeout=2.))

    @classmethod
    def from_socket(cls, connection):
        client=cls.__new__(cls)
        client._initialize(connection)
        return client

    def _initialize(self, connection):
        self.socket = connection
        self.socket.setblocking(False)
        self.incoming = b''
        self.outgoing = b''
        self.pending = None
        self.value = None
        self.received = -1e9
        self.seq = 0
        self.error = 'NO_GATEWAY_STATUS'
        self.ack = None
        self.acks = deque(maxlen=64)
        self.ticket_used = True
        self.session = None
        self.closed = False

    def queue(self, value):
        self.outgoing += (json.dumps(value, allow_nan=False) + '\n').encode()
        if len(self.outgoing) > 8192:
            raise ValueError('CLIENT_OUTPUT_BACKLOG')

    def stop(self):
        # Drop unsent nonzero commands; partial TCP frames are handled by connection
        # shutdown on fatal errors. In normal operation small writes are bounded.
        self.queue({'kind': 'STOP'})

    def poll(self, now):
        if self.closed:
            return
        if self.pending and now - self.pending[1] > .15:
            self.error = 'GATEWAY_RTT_EXPIRED'
            self.value = None
            # Do not pipeline a newer request behind an expired response.
            self.close()
            return
        if self.pending is None:
            identity = secrets.token_hex(8)
            self.pending = (identity, now)
            # A POLL rotates the server ticket. An unused old ticket must not
            # be used after that request has already entered the TCP stream.
            self.ticket_used = True
            self.queue(dict(kind='POLL', id=identity))
        try:
            if self.outgoing:
                n = self.socket.send(self.outgoing)
                self.outgoing = self.outgoing[n:]
            for _ in range(4):
                try:
                    chunk = self.socket.recv(8192)
                except BlockingIOError:
                    break
                if not chunk:
                    raise ValueError('GATEWAY_DISCONNECTED')
                self.incoming += chunk
                if len(self.incoming) > 32768:
                    raise ValueError('GATEWAY_BACKLOG')
                while b'\n' in self.incoming:
                    line, self.incoming = self.incoming.split(b'\n', 1)
                    m = json.loads(line)
                    if m['kind'] == 'ACK':
                        self.ack = m
                        self.acks.append(m)
                        continue
                    if m['kind'] != 'STATUS' or self.pending is None or m['id'] != self.pending[0]:
                        raise ValueError('UNSOLICITED_GATEWAY_STATUS')
                    if not 0 <= now - self.pending[1] <= .15:
                        raise ValueError('GATEWAY_RTT_EXPIRED')
                    if self.session is not None and self.session != m['session']:
                        raise ValueError('GATEWAY_RESTART')
                    self.session = m['session']
                    self.value = m
                    # Anchor to request issuance, not delayed response consumption.
                    self.received = self.pending[1]
                    self.pending = None
                    self.ticket_used = False
                    self.error = None
        except BlockingIOError:
            pass
        except (OSError, ValueError, KeyError, TypeError, UnicodeError) as exc:
            self.error = str(exc)
            self.close()

    def get(self, now):
        return self.value if not self.closed and 0 <= now - self.received <= .15 else None

    def action(self, kind, now, velocity=None):
        s = self.get(now)
        if not s or self.ticket_used:
            return False
        self.seq += 1
        m = dict(kind=kind, session=s['session'], ticket=s['ticket'], seq=self.seq)
        if velocity is not None:
            m['velocity'] = list(velocity)
        self.queue(m)
        self.ticket_used = True
        return True

    def close(self):
        self.closed = True
        self.value = None
        self.socket.close()
