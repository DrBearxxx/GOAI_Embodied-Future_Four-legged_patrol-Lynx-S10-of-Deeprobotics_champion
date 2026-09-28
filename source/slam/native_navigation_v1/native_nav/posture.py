"""Bounded stationary posture check, separate from and unable to arm navigation.

Only STAND/LIE in existing regular mode 0. No SDK, mode, gait or axis commands.
Does not certify emergency stopping or bypass navigation acceptance.
"""
from .protocol import finite, operation


def posture_reason(gate, now, action):
    if action not in ('STAND', 'LIE'):
        return 'POSTURE_ACTION_NOT_ALLOWED'
    reason = gate.check(now, moving=False)
    if reason:
        return reason
    f = gate.feedback
    if f.get('mode') != 0:
        return 'REGULAR_MODE_REQUIRED'
    if f['speed'] > .03 or abs(f['wz']) > .05:
        return 'STATIONARY_REQUIRED'
    if not all(finite(f.get(k)) for k in ('height', 'roll', 'pitch')):
        return 'POSTURE_TELEMETRY_REQUIRED'
    if abs(f['roll']) > .25 or abs(f['pitch']) > .25:
        return 'POSTURE_TILT_EXCEEDED'
    # Idle 0 is documented query-only, not a command or an alias for lying.
    # Permit starting only with low measured body height and stable idle state.
    if action == 'STAND' and (f['state'] not in (0, 4) or not .02 <= f['height'] <= .15):
        return 'LOW_IDLE_POSTURE_REQUIRED'
    if action == 'LIE' and f['state'] not in (1, 17, 4):
        return 'LYING_TRANSITION_NOT_ALLOWED'
    return None


def posture_packet(gate, now, action):
    reason = posture_reason(gate, now, action)
    if reason:
        raise ValueError(reason)
    return operation(action)
