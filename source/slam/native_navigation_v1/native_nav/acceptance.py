"""Evidence gate, NOT a hardware safety certification or replacement for testing."""
import hashlib
import json
from pathlib import Path
from .protocol import finite, MAX_FORWARD_SPEED


def code_hash():
    root = Path(__file__).resolve().parent
    h = hashlib.sha256()
    for name in ('protocol.py', 'gate.py', 'gateway.py', 'acceptance.py', 'first_trial.py', 'zero_guardian.py'):
        h.update(name.encode())
        h.update((root / name).read_bytes())
    return h.hexdigest()


def verify(path, boot_id):
    d = json.loads(Path(path).read_text(encoding='utf-8'))
    if d.get('schema') != 's10.native.acceptance.v1' or d.get('gateway_sha256') != code_hash():
        raise ValueError('ACCEPTANCE_CODE_MISMATCH')
    if d.get('body_boot_id') != boot_id:
        raise ValueError('ACCEPTANCE_BOOT_MISMATCH')
    if not isinstance(d.get('operator'), str) or not d['operator'].strip():
        raise ValueError('ACCEPTANCE_OPERATOR_REQUIRED')
    for field in ('independent_estop_verified', 'exclusive_control_verified',
                  'plaintext_isolated_network_verified', 'velocity_units_verified',
                  'gateway_kill_stop_verified', 'orin_disconnect_stop_verified'):
        if d.get(field) is not True:
            raise ValueError('PHYSICAL_TEST_NOT_VERIFIED:' + field)
    for field, limit in (('receiver_timeout_s', .30), ('measured_stop_distance_m', .10)):
        v = d.get(field)
        if not finite(v) or not 0 < v <= limit:
            raise ValueError('STOP_TEST_OUT_OF_BUDGET:' + field)
    speed = d.get('verified_forward_speed_mps')
    if not finite(speed) or not MAX_FORWARD_SPEED <= speed <= .5:
        raise ValueError('STOP_TEST_SPEED_NOT_COVERED')
    if not isinstance(d.get('evidence_file'), str) or not d['evidence_file']:
        raise ValueError('PHYSICAL_EVIDENCE_REQUIRED')
    evidence = Path(d['evidence_file'])
    if not evidence.is_file() or hashlib.sha256(evidence.read_bytes()).hexdigest() != d.get('evidence_sha256'):
        raise ValueError('PHYSICAL_EVIDENCE_HASH_MISMATCH')
    return d
