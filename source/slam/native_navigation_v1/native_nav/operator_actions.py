"""One bounded, unsent operator intent; never replay a transmitted action.

POLL rotates the gateway ticket. A healthy connection can therefore have no
usable ticket on the exact control tick when an operator presses Enter.
Waiting here does not extend the transport's 150 ms freshness/RTT limits.
"""
from dataclasses import dataclass
import math
from .dds_ownership import environment_reason

ACTION_NAMES = ('SDK_OFF', 'MODE_NAV', 'STAND', 'FLAT', 'LIE', 'ARM')
TICKET_WAIT_SECONDS = .35


def action_denial(name, armed, pending_arm, command, env, now):
    if armed or pending_arm is not None:
        return 'STOP before changing robot state'
    reason = environment_reason(env, now)
    if reason:
        return reason
    if name == 'ARM' and not command.get('ready'):
        return str(command.get('state', 'ROUTE_NOT_READY'))
    return None


@dataclass
class Intent:
    name: str
    request_id: str
    started: float
    transport: object
    connection: int
    session: str
    context: object
    waiting: bool = False


class OperatorActions:
    def __init__(self, replies, first_trial=False):
        self.replies = replies
        self.first_trial = first_trial
        self.pending = None

    def submit(self, name, request_id, client, now, context=None):
        if name not in ACTION_NAMES:
            raise ValueError('UNKNOWN_OPERATOR_ACTION')
        reason = None
        if self.pending is not None or self.replies.busy:
            reason = 'PREVIOUS_COMMAND_PENDING；请等待上一条指令结果'
        elif (client.closed or client.current is None or client.current.closed
              or not client.current.session):
            reason = 'GATEWAY_NOT_CONNECTED_OR_INITIALIZING；未发送'
        elif not math.isfinite(now):
            reason = 'INVALID_OPERATOR_CLOCK；未发送'
        if reason:
            self.replies.emit(name, request_id, 'REJECTED', reason, False)
            return False
        self.pending = Intent(name, request_id, now, client.current,
                              client.connection_count, client.current.session, context)
        return True

    def cancel(self, reason):
        if self.pending is not None:
            item, self.pending = self.pending, None
            self.replies.emit(item.name, item.request_id, 'CANCELLED', reason+'；未发送', False)

    def tick(self, client, now, denied=None, context=None):
        item = self.pending
        if item is None:
            return None
        # A command may wait only in the same live connection/session and
        # ownership context. Reconnection is monitoring-only, not an intent queue.
        if (client.closed or client.current is not item.transport or item.transport.closed
                or client.connection_count != item.connection or item.transport.session != item.session):
            self.cancel('GATEWAY_CONNECTION_CHANGED')
        elif not math.isfinite(now) or not 0 <= now-item.started < TICKET_WAIT_SECONDS:
            self.cancel('GATEWAY_TICKET_WAIT_EXPIRED')
        elif context != item.context:
            self.cancel('CONTROL_CONTEXT_CHANGED')
        elif denied:
            self.cancel(denied)
        else:
            kind = 'ARM_TRIAL50' if item.name == 'ARM' and self.first_trial else item.name
            if client.action(kind, now):
                self.pending = None  # Only one enqueue; ACK loss never retries.
                self.replies.queued(item.name, item.request_id, item.transport.seq, item.connection, now)
                return item.name
            if client.current is not item.transport or item.transport.closed:
                self.cancel('GATEWAY_TRANSPORT_FAILED')
            elif not item.waiting:
                item.waiting = True
                self.replies.emit(item.name, item.request_id, 'WAIT_TICKET',
                                  '等待下一张有效网关票据（最多 350 ms）；尚未发送', final=False)
        return None
