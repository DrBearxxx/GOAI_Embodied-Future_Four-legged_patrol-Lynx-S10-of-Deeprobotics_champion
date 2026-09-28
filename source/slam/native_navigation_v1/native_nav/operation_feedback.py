"""Read-only confirmation of observed state, distinct from gateway delivery ACK."""
import math

EFFECT_TIMEOUTS = {'MODE_NAV': 5., 'STAND': 40., 'FLAT': 5., 'LIE': 15.}


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def feedback_text(feedback):
    f = feedback or {}
    gait = f.get('gait')
    gait_text = hex(gait) if type(gait) is int else '?'
    height = f"{f['height']:.3f} m" if finite(f.get('height')) else '?'
    return f"state={f.get('state', '?')}，mode={f.get('mode', '?')}，gait={gait_text}，高度={height}"


class OperationFeedback:
    def __init__(self, name, request_id, connection, ack_at):
        self.name, self.request_id, self.connection = name, request_id, connection
        self.ack_at = ack_at
        self.since = None
        self.last_status = None
        self.count = 0

    def observe(self, status, status_mono, now, connection):
        status = status or {}
        if connection != self.connection:
            return 'UNCONFIRMED', '连接改变；动作结果未确认，不自动重发', None
        if not finite(now) or now < self.ack_at or now-self.ack_at > EFFECT_TIMEOUTS[self.name]:
            return 'UNCONFIRMED', '等待目标反馈超时；不代表动作未执行，不自动重发', None
        f = (status or {}).get('feedback') or {}
        valid = (finite(status_mono) and 0 <= now-status_mono <= .15
                 and finite(status.get('basic_age')) and 0 <= status['basic_age'] <= 1.2
                 and finite(status.get('motion_age')) and 0 <= status['motion_age'] <= .30
                 # Request issuance minus age is a conservative bound: cached
                 # pre-ACK telemetry cannot prove the new action completed.
                 and status_mono-max(status['basic_age'], status['motion_age']) > self.ack_at
                 and f.get('state') == f.get('basic_state')
                 and f.get('gait') == f.get('basic_gait')
                 and f.get('hard_stop') == 0 and f.get('charge') == 0 and f.get('sleep') is False
                 and all(finite(f.get(k)) for k in ('height', 'speed', 'wz', 'roll', 'pitch'))
                 and 0 <= f['speed'] <= .03 and abs(f['wz']) <= .05
                 and abs(f['roll']) <= .25 and abs(f['pitch']) <= .25)
        target = False
        if valid:
            if self.name == 'MODE_NAV': target = f.get('mode') == 1
            if self.name == 'STAND': target = f['state'] == 17 and .20 <= f['height'] <= .65
            if self.name == 'FLAT': target = f['state'] == 17 and f.get('mode') == 1 and f['gait'] == 0x3002
            if self.name == 'LIE': target = f['state'] in (0, 4) and .02 <= f['height'] <= .15
        if not target:
            self.since, self.last_status, self.count = None, None, 0
        elif status_mono != self.last_status:
            self.last_status = status_mono
            self.count += 1
            if self.since is None: self.since = now
            if self.count >= 3 and now-self.since >= .25:
                return 'CONFIRMED', '新鲜反馈已达到目标状态；'+feedback_text(f), True
        return None
