"""S10 ASDU, September 19 guide sections 1.2.1--1.2.8. No socket side effects."""
import datetime
import json
import math
import struct

SYNC = b'\xeb\x91\xeb\x90'
HEADER = struct.Struct('<4sHHBBB5s')
MAX_FORWARD_SPEED = .20  # m/s; acceptance evidence must cover this ceiling.


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def body(kind, command, items):
    return {'PatrolDevice': {'Type': kind, 'Command': command,
            'Time': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), 'Items': items}}


def heartbeat():
    return body(0x00100064, 5, {})


def velocity(vx, vy, wz):
    # Real axis, NOT 0x00100002 proportional joystick commands. Trial limits
    # are deliberately much smaller than the guide's hardware maxima.
    if not all(finite(v) for v in (vx, vy, wz)) or not 0 <= vx <= MAX_FORWARD_SPEED or vy != 0 or abs(wz) > .20:
        raise ValueError('VELOCITY_OUT_OF_RANGE')
    return body(0x00100001, 0x00110002,
                dict(X=float(vx), Y=0., Z=0., Roll=0., Pitch=0., Yaw=float(wz)))


def operation(name):
    if name == 'MODE_NAV':
        return body(0x00100002, 0x00500002, {'Mode': 1})
    if name == 'SDK_OFF':
        return body(0x00100005, 0x00300002, {'SDKEnable': False, 'Frequency': 100})
    if name in ('STAND', 'LIE'):
        return body(0x00100001, 0x00200002, {'MotionParam': 1 if name == 'STAND' else 4})
    if name == 'FLAT':
        return body(0x00100001, 0x00300002, {'GaitParam': 0x3002})
    # No joint commands, zeroing, soft-estop (-2 is query-only), or stairs in V1.
    raise ValueError('OPERATION_NOT_ALLOWED')


class Codec:
    def __init__(self):
        self.message = 0
        self.packet = 0

    def encode(self, value):
        payload = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8')
        if len(payload) > 65535:
            raise ValueError('ASDU_TOO_LARGE')
        data = HEADER.pack(SYNC, len(payload), self.message, 1, self.packet, 1, bytes(5)) + payload
        self.message = (self.message + 1) % 65536
        self.packet = (self.packet + 1) % 256
        return data

    @staticmethod
    def decode(data):
        if len(data) < HEADER.size:
            raise ValueError('SHORT_ASDU')
        sync, length, message, fmt, packet, version, reserved = HEADER.unpack(data[:16])
        if sync != SYNC or fmt != 1 or version != 1 or reserved != bytes(5) or length != len(data) - 16:
            raise ValueError('INVALID_ASDU_HEADER')
        value = json.loads(data[16:].decode('utf-8'), parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
        if not isinstance(value, dict) or not isinstance(value.get('PatrolDevice'), dict):
            raise ValueError('INVALID_ASDU_BODY')
        return value['PatrolDevice']
