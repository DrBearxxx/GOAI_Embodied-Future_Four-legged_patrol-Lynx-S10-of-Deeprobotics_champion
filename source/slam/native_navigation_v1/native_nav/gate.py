"""Deterministic robot-side lease gate. All deadlines use one local monotonic clock."""
import secrets
from .protocol import finite, MAX_FORWARD_SPEED

TICKET_MAX_AGE = .15
COMMAND_LEASE = .25


class Gate:
    def __init__(self):
        self.session = secrets.token_hex(16)
        self.ticket = None
        self.issued = -1.
        self.used = True
        self.seq = -1
        self.deadline = -1.
        self.armed = False
        self.owned = False
        self.reason = 'DISARMED'
        self.command = (0., 0., 0.)
        self.feedback = None
        self.basic_mono = -1e9
        self.motion_mono = -1e9
        self.environment = None
        self.env_mono = -1e9
        self.last_tick = None

    def stop(self, reason):
        self.armed = False
        self.command = (0., 0., 0.)
        self.deadline = -1.
        self.reason = reason

    def challenge(self, now):
        self.ticket = secrets.token_hex(16)
        self.issued = now
        self.used = False
        return self.ticket

    def low_idle_ready(self, now):
        return self.feedback['state'] == 4

    def operation_check(self, kind, now):
        """Formal-mode posture policy; commissioning overrides only this hook."""
        state = self.feedback['state']
        if kind in ('SDK_OFF', 'MODE_NAV', 'STAND') and not self.low_idle_ready(now):
            return 'OPERATION_REQUIRES_LYING'
        if kind in ('STAND', 'FLAT') and self.feedback['mode'] != 1:
            return 'NAV_MODE_REQUIRED'
        if kind == 'FLAT' and state != 17:
            return 'STANDING_RL_REQUIRED'
        if kind == 'LIE' and state not in (1, 17, 4):
            return 'LIE_NOT_ALLOWED_IN_CURRENT_STATE'
        return None

    def check(self, now, moving=True):
        f = self.feedback
        if not f or not 0 <= now - self.basic_mono <= 1.2 or not 0 <= now - self.motion_mono <= .30:
            return 'ROBOT_STATUS_STALE'
        if f.get('hard_stop') != 0 or f.get('charge') != 0 or f.get('sleep') is not False:
            return 'ROBOT_NOT_AVAILABLE'
        if not finite(f.get('speed')) or not 0 <= f['speed'] <= .5:
            return 'UNEXPECTED_BODY_SPEED'
        if not finite(f.get('wz')) or abs(f['wz']) > .6:
            return 'UNEXPECTED_YAW_SPEED'
        if f.get('basic_state') != f.get('state') or f.get('basic_gait') != f.get('gait'):
            return 'STATUS_TRANSITION_OR_DISAGREEMENT'
        if self.environment is not True or not 0 <= now - self.env_mono <= 1.2:
            return 'CONTROL_OWNERSHIP_UNVERIFIED'
        if moving and (f.get('state') != 17 or f.get('gait') != 0x3002 or f.get('mode') != 1):
            return 'NOT_FACTORY_FLAT_NAVIGATION'
        return None

    def accept(self, message, now):
        """One fresh ticket per action; delay cannot lengthen a command's lease."""
        try:
            if message.get('kind') == 'STOP':
                self.stop('OPERATOR_STOP')
                return True, self.reason
            if (message['session'] != self.session or message['ticket'] != self.ticket or self.used
                    or not 0 <= now - self.issued <= TICKET_MAX_AGE):
                raise ValueError('EXPIRED_OR_REPLAYED_TICKET')
            self.used = True
            seq = message['seq']
            if type(seq) is not int or seq <= self.seq:
                raise ValueError('REPLAYED_SEQUENCE')
            self.seq = seq
            kind = message['kind']
            reason = self.check(now, moving=kind in ('ARM', 'VELOCITY'))
            if reason:
                raise ValueError(reason)
            if kind == 'ARM':
                if self.armed:
                    raise ValueError('ALREADY_ARMED')
                if self.feedback['speed'] > .03:
                    raise ValueError('ARM_REQUIRES_STATIONARY')
                self.armed = self.owned = True
                self.command = (0., 0., 0.)
                self.reason = 'ARMED'
                self.deadline = self.issued + COMMAND_LEASE
            elif kind == 'VELOCITY':
                if not self.armed:
                    raise ValueError('EXPLICIT_ARM_REQUIRED')
                v = message['velocity']
                if (not isinstance(v, list) or len(v) != 3 or not all(finite(x) for x in v)
                        or not 0 <= v[0] <= MAX_FORWARD_SPEED or v[1] != 0 or abs(v[2]) > .2):
                    raise ValueError('VELOCITY_OUT_OF_RANGE')
                self.command = tuple(v)
                self.deadline = self.issued + COMMAND_LEASE
                self.reason = 'FOLLOW'
            elif kind in ('SDK_OFF', 'MODE_NAV', 'STAND', 'FLAT', 'LIE'):
                if self.armed or self.feedback['speed'] > .03:
                    raise ValueError('OPERATION_REQUIRES_DISARMED_STATIONARY')
                reason = self.operation_check(kind, now)
                if reason:
                    raise ValueError(reason)
                self.owned = True
                self.reason = 'REQUESTED_' + kind
            else:
                raise ValueError('UNKNOWN_ACTION')
            return True, self.reason
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.stop(str(exc))
            return False, self.reason

    def tick(self, now):
        if self.last_tick is not None and (now < self.last_tick or now - self.last_tick > .15):
            self.stop('GATE_LOOP_STALLED')
        self.last_tick = now
        if self.armed:
            reason = self.check(now)
            if reason or now >= self.deadline:
                self.stop(reason or 'COMMAND_LEASE_EXPIRED')
        return self.command if self.armed else (0., 0., 0.)

    def status(self, now):
        return dict(session=self.session, ticket=self.challenge(now), armed=self.armed,
                    reason=self.reason, ready=self.check(now) is None, feedback=self.feedback,
                    feedback_reason=self.check(now), last_seq=self.seq,
                    basic_age=now-self.basic_mono, motion_age=now-self.motion_mono)
