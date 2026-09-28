"""Nonblocking reconnect of monitoring transport, never retransmits actions."""
import errno
import select
import socket
from .client import Client


class ReconnectingClient:
    def __init__(self,port=18891):
        self.port=port;self.current=None;self.connecting=None;self.connect_started=None
        self.closed=False;self.error='GATEWAY_DISCONNECTED';self.next_retry=0.;self.delay=.25
        self.connection_count=0;self.disconnections=0;self.connected_since=None

    @property
    def received(self):return self.current.received if self.current else -1e9

    @property
    def ack(self):return self.current.ack if self.current else None

    def drain_acks(self):
        if not self.current:return []
        result=list(self.current.acks)
        self.current.acks.clear()
        return result

    def _failed(self,now,reason):
        if self.current:self.current.close()
        if self.connecting:self.connecting.close()
        self.current=None;self.connecting=None;self.error=reason;self.disconnections+=1
        self.next_retry=now+self.delay;self.delay=min(4.,self.delay*2)

    def poll(self,now):
        try:self._poll(now)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self._failed(now,'GATEWAY_TRANSPORT:'+str(exc))

    def _poll(self,now):
        if self.closed:return
        if self.current:
            self.current.poll(now)
            if self.current.closed:
                self._failed(now,self.current.error);return
            self.error=self.current.error
            if self.connected_since is not None and now-self.connected_since>10:self.delay=.25
            return
        if self.connecting is None:
            if now<self.next_retry:return
            self.connecting=socket.socket();self.connecting.setblocking(False);self.connect_started=now
            try:code=self.connecting.connect_ex(('127.0.0.1',self.port))
            except OSError as exc:self._failed(now,str(exc));return
            if code not in (0,errno.EINPROGRESS,errno.EWOULDBLOCK,errno.EALREADY):
                self._failed(now,'GATEWAY_CONNECT:'+str(code));return
        if now-self.connect_started>.5:self._failed(now,'GATEWAY_CONNECT_TIMEOUT');return
        _,w,e=select.select([],[self.connecting],[self.connecting],0)
        if w or e:
            code=self.connecting.getsockopt(socket.SOL_SOCKET,socket.SO_ERROR)
            if code:self._failed(now,'GATEWAY_CONNECT:'+str(code));return
            self.current=Client.from_socket(self.connecting);self.connecting=None
            self.connection_count+=1;self.connected_since=now;self.error='NO_GATEWAY_STATUS'
            self.current.poll(now)

    def get(self,now):return self.current.get(now) if self.current else None

    def action(self,kind,now,velocity=None):
        if not self.current:return False
        try:return self.current.action(kind,now,velocity)
        except (OSError,ValueError,TypeError):
            self._failed(now,'CLIENT_ACTION_FAILED');return False

    def stop(self):
        if self.current and not self.current.closed:
            try:self.current.stop()
            except (OSError,ValueError):self.current.close()

    def close(self):
        self.closed=True
        if self.current:self.current.close()
        if self.connecting:self.connecting.close()
