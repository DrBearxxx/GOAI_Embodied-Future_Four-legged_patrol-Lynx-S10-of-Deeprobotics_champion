"""One-shot commissioning envelope, not a physical acceptance certificate.

The independent body budget survives client reconnects. It cannot be selected by
a network message. Formal --execute continues to require acceptance evidence.
"""
from .gate import Gate
from .protocol import finite

PROFILE = 's10.native.first_trial50.v1'


class FirstTrialGate(Gate):
    def __init__(self):
        super().__init__()
        self.guardian_ready = False
        self.low_since = None
        self.standing_since = None
        self.started = None
        self.finished_reason = None
        self.budget_time = None
        self.command_path = 0.
        self.command_turn = 0.
        self.feedback_path = 0.
        self.stand_requested = False

    def stop(self, reason):
        if self.started is not None and self.finished_reason is None:
            self.finished_reason = reason
        super().stop(reason)

    def low_idle_ready(self, now):
        return self.low_since is not None and now-self.low_since >= 2.

    def operation_check(self, kind, now):
        f = self.feedback
        if abs(f['wz']) > .05:
            return 'OPERATION_REQUIRES_DISARMED_STATIONARY'
        if kind == 'STAND':
            # The already-tested factory posture interface supports regular
            # mode 0. A stand transition can restore that mode on this firmware.
            if not self.low_idle_ready(now):
                return 'OPERATION_REQUIRES_LYING'
            return None if f['mode'] in (0, 1) else 'UNKNOWN_CONTROL_MODE'
        if kind == 'MODE_NAV':
            if self.low_idle_ready(now):
                return None
            if self.standing_since is not None and now-self.standing_since >= 2.:
                return None
            return 'MODE_NAV_REQUIRES_LOW_IDLE_OR_STABLE_STANDING_2S'
        return super().operation_check(kind, now)

    def check(self, now, moving=True):
        reason = super().check(now, moving)
        if reason:
            return reason
        if not self.guardian_ready:
            return 'ZERO_GUARDIAN_NOT_READY'
        f = self.feedback
        if not all(finite(f.get(k)) for k in ('height', 'roll', 'pitch')):
            return 'POSTURE_TELEMETRY_REQUIRED'
        if abs(f['roll']) > .25 or abs(f['pitch']) > .25:
            return 'POSTURE_TILT_EXCEEDED'
        if moving and not .20 <= f['height'] <= .65:
            return 'STANDING_HEIGHT_REQUIRED'
        if f['speed'] > .30 or abs(f['wz']) > .35:
            return 'TRIAL_BODY_SPEED_OUT_OF_RANGE'
        return None

    def tick(self, now):
        f = self.feedback or {}
        low = (self.check(now, moving=False) is None and f.get('state') in (0, 4) and f.get('mode') in (0, 1)
               and .02 <= f['height'] <= .15 and f['speed'] <= .03 and abs(f['wz']) <= .05)
        if low:
            if self.low_since is None:
                self.low_since = now
        else:
            self.low_since = None
        standing = (self.check(now, moving=False) is None and f.get('state') == 17
                    and f.get('mode') in (0, 1) and .20 <= f['height'] <= .65
                    and f['speed'] <= .03 and abs(f['wz']) <= .05)
        if standing:
            if self.standing_since is None:
                self.standing_since = now
        else:
            self.standing_since = None
        if self.armed and self.budget_time is not None:
            dt = now-self.budget_time
            if 0 <= dt <= .15:
                self.command_path += self.command[0]*dt
                self.command_turn += abs(self.command[2])*dt
                if self.check(now) is None:
                    self.feedback_path += f['speed']*dt
            if now-self.started >= 60.:
                self.stop('BODY_TRIAL_TIMEOUT')
            elif self.command_path >= .50:
                self.stop('BODY_TRIAL_COMMAND_PATH')
            elif self.command_turn >= 1.20:
                self.stop('BODY_TRIAL_TURN_BUDGET')
            elif self.feedback_path >= .60:
                self.stop('BODY_TRIAL_FEEDBACK_PATH')
        self.budget_time = now
        return super().tick(now)

    def accept(self, message, now):
        # Enforce budgets even between timer ticks, before extending a lease.
        self.tick(now)
        kind = message.get('kind')
        if kind == 'ARM':
            self.stop('TRIAL50_ARM_REQUIRED')
            return False, self.reason
        if self.finished_reason and kind not in ('STOP', 'LIE'):
            return False, 'TRIAL_FINISHED_RESTART_REQUIRED'
        if kind == 'STAND' and self.stand_requested:
            return False, 'STAND_ALREADY_REQUESTED_NO_RETRY'
        if kind == 'ARM_TRIAL50':
            message = dict(message, kind='ARM')
        ok, why = super().accept(message, now)
        if ok and kind == 'ARM_TRIAL50':
            self.started = self.budget_time = now
        if ok and kind == 'STAND':
            self.stand_requested = True
        return ok, why

    def status(self, now):
        value = super().status(now)
        value['ready'] = value['ready'] and self.finished_reason is None
        if self.finished_reason:
            value['feedback_reason'] = self.finished_reason
        value['first_trial'] = dict(schema=PROFILE, acceptance='UNVERIFIED', max_distance_m=.50,
            max_vx_mps=.20, max_seconds=60., one_shot=True, guardian_ready=self.guardian_ready,
            low_idle_ready=self.low_idle_ready(now), finished_reason=self.finished_reason,
            standing_mode_ready=self.standing_since is not None and now-self.standing_since >= 2.,
            command_path_m=self.command_path, feedback_path_m=self.feedback_path,
            elapsed_s=None if self.started is None else now-self.started)
        return value
