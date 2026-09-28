"""Correlated operator receipts; telemetry and motor control never wait for UI."""
import json
import os
import re
import secrets
import time
from .bounded_log import BoundedLog
from .operation_feedback import OperationFeedback, EFFECT_TIMEOUTS, feedback_text

COMMANDS = ('SDK_OFF', 'STAND', 'MODE_NAV', 'FLAT', 'ARM', 'STOP', 'LIE', 'QUIT', 'STATUS')


def parse_line(line):
    parts = line.strip().split()
    if not parts or parts[0].upper() not in COMMANDS or len(parts) > 2:
        raise ValueError('UNKNOWN_COMMAND')
    token = parts[1] if len(parts) == 2 else secrets.token_hex(16)
    if not re.fullmatch(r'[a-f0-9]{32}', token): raise ValueError('INVALID_REQUEST_ID')
    return parts[0].upper(), token


class CommandReplies:
    def __init__(self, path):
        self.log = BoundedLog(path, max_bytes=256*1024, backups=1)
        self.pending = {}
        self.arm_request = None
        self.echo = False
        self.confirmation = None
        self.latest_action = None

    @property
    def busy(self):
        return bool(self.pending or self.confirmation)

    def emit(self, command, request_id, phase, reason, ok=None, final=True, **extra):
        event = dict(pid=os.getpid(), request_id=request_id, command=command, phase=phase,
                     reason=reason, ok=ok, final=final, mono=time.monotonic(), **extra)
        self.log.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
        if command not in ('STATUS', 'STOP', 'QUIT'):
            self.latest_action = event
        if self.echo: print(format_reply(event), flush=True)
        return event

    def queued(self, name, request_id, seq, connection, now):
        if name == 'ARM': self.arm_request = (name, request_id)
        self.pending[(connection, seq)] = (name, request_id, now)
        self.emit(name, request_id, 'QUEUED', '等待网关 ACK', final=False)

    def poll(self, acks, connection, now, feedback=None, status=None, status_mono=None):
        for ack in acks:
            if not isinstance(ack, dict) or type(ack.get('seq')) is not int: continue
            item = self.pending.pop((connection, ack.get('seq')), None)
            if not item: continue
            name, request_id, started = item
            ok = ack.get('ok') is True
            confirming = ok and name in EFFECT_TIMEOUTS
            if confirming:
                self.confirmation = OperationFeedback(name, request_id, connection, now)
            self.emit(name, request_id, 'ACK', str(ack.get('reason')), ok,
                      final=not ok or (name != 'ARM' and not confirming), feedback=feedback)
            if name == 'ARM' and not ok: self.arm_request = None
        for key, (name, request_id, started) in list(self.pending.items()):
            if key[0] != connection or now-started > 1.:
                self.emit(name, request_id, 'UNCONFIRMED', '未收到对应 ACK；不自动重发', None)
                self.pending.pop(key)
                if name == 'ARM': self.arm_request = None
        if self.confirmation:
            item = self.confirmation
            result = item.observe(status, status_mono, now, connection)
            if result:
                self.confirmation = None
                phase, reason, ok = result
                self.emit(item.name, item.request_id, phase, reason, ok, feedback=feedback)

    def cancel_waits(self, reason):
        for name, request_id, _ in self.pending.values():
            self.emit(name, request_id, 'UNCONFIRMED', reason+'；已发送动作可能仍在执行，不重发', None)
        self.pending.clear()
        if self.confirmation:
            item, self.confirmation = self.confirmation, None
            self.emit(item.name, item.request_id, 'UNCONFIRMED', reason+'；仅结束反馈等待，动作可能仍在执行', None)
        self.arm_request = None

    def arm_result(self, ok, reason):
        if self.arm_request:
            self.emit(*self.arm_request, 'ARM_RESULT', str(reason), bool(ok))
            self.arm_request = None

    def close(self):
        self.log.close()


def format_reply(event):
    phase, name = event['phase'], event['command']
    if phase == 'WAIT_TICKET': return name+'：'+str(event.get('reason'))
    if phase == 'QUEUED': return name+'：已加入发送队列，等待网关 ACK…'
    if phase == 'ACK':
        if event.get('ok'):
            if name == 'SDK_OFF':
                return 'SDK_OFF：网关已下发关闭 SDK 请求；无 SDKEnable 回读，不能据此确认开关状态。'
            return f'{name}：网关已下发，等待对应状态反馈（不是动作完成）。'
        return f"{name}：拒绝 — {reason_text(event.get('reason'))}；{feedback_text(event.get('feedback'))}"
    if phase == 'CONFIRMED':
        hint = '；下一步 MODE_NAV，确认 mode=1 后再 FLAT' if name == 'STAND' else ''
        return f"{name}：反馈已确认 — {event.get('reason')}{hint}"
    if phase == 'STATUS': return f"STATUS：{event.get('reason')}"
    suffix = '；'+feedback_text(event['feedback']) if event.get('feedback') else ''
    return f"{name}：{phase} — {reason_text(event.get('reason'))}{suffix}"


def reason_text(reason):
    text = str(reason)
    descriptions = {
        'NAV_MODE_REQUIRED': '尚未进入导航使用模式；站稳后先 MODE_NAV，确认 mode=1 后再 FLAT',
        'FACTORY_GATEWAY:NOT_FACTORY_FLAT_NAVIGATION': '站立、导航模式和平地步态尚未全部满足',
        'MODE_NAV_REQUIRES_LOW_IDLE_OR_STABLE_STANDING_2S': '切导航模式需要趴稳或连续站稳 2 秒',
        'TRIAL_START_NOT_REACHED': '当前定位不在地图试验起点',
        'TRIAL_OUTSIDE_CORRIDOR': '当前定位超出这条 50 cm 路线的走廊范围',
        'TRIAL_START_HEADING': '当前朝向与路线起始朝向不一致',
    }
    return descriptions.get(text, text)+(f' ({text})' if text in descriptions else '')


def wait_replies(path, pid, request_id, since, emit=print, timeout=2.):
    # UI process only. Bounded tail scanning tolerates append and log rotation;
    # UUID+PID+time prevent an old ACK being shown as this command's result.
    seen = set()
    accepted = None
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        try:
            with open(path, 'rb') as stream:
                stream.seek(0, 2)
                length = stream.tell()
                stream.seek(max(0, length-65536))
                data = stream.read(65536)
            for line in data.splitlines():
                try: event = json.loads(line)
                except (ValueError, UnicodeError): continue
                if not isinstance(event, dict) or type(event.get('mono')) not in (float, int): continue
                if (event.get('pid') != pid or event.get('request_id') != request_id
                        or event.get('mono', -1) < since): continue
                if not all(k in event for k in ('phase', 'command', 'reason')): continue
                key = (event.get('mono'), event.get('phase'))
                if key in seen: continue
                seen.add(key)
                emit(format_reply(event))
                if event.get('phase') == 'ACK' and event.get('ok') is True:
                    accepted = event.get('command')
                if event.get('final'): return True
        except OSError:
            pass
        time.sleep(.04)
    if accepted in EFFECT_TIMEOUTS:
        emit(accepted+'：已下发，目标反馈仍在确认；可输入 STATUS 查看，勿重复发送。')
    else:
        emit('未收到这条指令的最终确认；不要据此认为已成功，程序不会自动重发。')
    return False
