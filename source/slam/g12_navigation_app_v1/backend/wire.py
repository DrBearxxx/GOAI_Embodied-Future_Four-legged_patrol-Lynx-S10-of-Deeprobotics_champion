"""S10 receive compatibility with explicit format/version/length validation.

Remote wall-clock strings are informational, never a freshness clock. Only a
kernel receive timestamp is mapped to this process's monotonic clock.
"""
import json,math,re,socket,struct

HEADER=struct.Struct('<4sHHBBB5s')
MAGICS=(b'\xeb\x91\xeb\x90',b'\xeb\x90\xeb\x90')

def integer(value,name,minimum=0,maximum=0xffffffff):
    if type(value) is str and len(value)<=16:
        if re.fullmatch(r'-?[0-9]+',value):value=int(value,10)
        elif re.fullmatch(r'0[xX][0-9a-fA-F]+',value):value=int(value,16)
    elif type(value) is float and math.isfinite(value) and value.is_integer():value=int(value)
    if type(value) is not int or not minimum<=value<=maximum:raise ValueError('INVALID_'+name)
    return value

def flag(value,name):
    if type(value) is bool:return int(value)
    if type(value) is str and value.lower() in ('true','false'):return int(value.lower()=='true')
    return integer(value,name,0,1)

def decode(data):
    if len(data)<HEADER.size:raise ValueError('SHORT_HEADER')
    magic,length,message,fmt,packet,version,reserved=HEADER.unpack(data[:16])
    if magic not in MAGICS:raise ValueError('BAD_MAGIC')
    if fmt!=1 or version!=1 or reserved!=bytes(5):raise ValueError('UNSUPPORTED_HEADER')
    if length!=len(data)-16:raise ValueError('LENGTH_MISMATCH')
    try:
        doc=json.loads(data[16:].decode('utf-8-sig'),parse_constant=lambda _: (_ for _ in ()).throw(ValueError('NONFINITE_JSON')))
    except (UnicodeError,json.JSONDecodeError) as e:raise ValueError('INVALID_JSON') from e
    if not isinstance(doc,dict) or not isinstance(doc.get('PatrolDevice'),dict):raise ValueError('MISSING_PATROL_DEVICE')
    value=doc['PatrolDevice']
    if not isinstance(value.get('Items'),dict):raise ValueError('INVALID_ITEMS')
    return dict(type=integer(value.get('Type'),'TYPE'),command=integer(value.get('Command'),'COMMAND'),
                items=value['Items'],time=value.get('Time'),magic=magic.hex(),message=message,packet=packet)

def basic_status(message):
    if message['type'] not in (0x100064,0x300064) or message['command']!=0xf00000:return None
    b=message['items'].get('BasicStatus')
    if not isinstance(b,dict):raise ValueError('MISSING_BASIC_STATUS')
    return dict(ControlUsageMode=integer(b.get('ControlUsageMode'),'MODE',0,3),
                MotionState=integer(b.get('MotionState'),'MOTION_STATE',-32768,32767),
                Gait=integer(b.get('Gait'),'GAIT'),HES=flag(b.get('HES'),'HES'),
                Charge=flag(b.get('Charge'),'CHARGE'),Sleep=bool(flag(b.get('Sleep'),'SLEEP')))

def enable_kernel_timestamp(sock):
    # Linux old/new timestamp ABIs; no userspace-receive-time freshness fallback.
    for option,kind in ((getattr(socket,'SO_TIMESTAMPNS',35),'ns'),(getattr(socket,'SO_TIMESTAMP',29),'us')):
        try:sock.setsockopt(socket.SOL_SOCKET,option,1);return kind
        except OSError:continue
    raise RuntimeError('KERNEL_RECEIVE_TIMESTAMP_UNAVAILABLE')

def received_monotonic(ancillary,flags,wall,mono,max_age=.3):
    if flags:raise ValueError('TRUNCATED_DATAGRAM')
    stamp=None
    for level,kind,data in ancillary:
        if level!=socket.SOL_SOCKET:continue
        if kind not in (35,64,29,63):continue
        layout='=qq' if kind in (64,63) else '@ll'
        if len(data)<struct.calcsize(layout):raise ValueError('SHORT_KERNEL_TIMESTAMP')
        seconds,fraction=struct.unpack_from(layout,data)
        divisor=1e9 if kind in (35,64) else 1e6
        if not 0<=fraction<divisor:raise ValueError('BAD_KERNEL_TIMESTAMP')
        stamp=seconds+fraction/divisor;break
    if stamp is None:raise ValueError('MISSING_KERNEL_TIMESTAMP')
    age=wall-stamp
    if not math.isfinite(age) or not 0<=age<=max_age:raise ValueError('STALE_OR_FUTURE_DATAGRAM')
    return mono-age
