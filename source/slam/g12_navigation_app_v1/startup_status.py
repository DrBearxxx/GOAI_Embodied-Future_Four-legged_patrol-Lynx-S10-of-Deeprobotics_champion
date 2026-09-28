"""Wait for service replies; optionally choose outdoor on a fresh service start."""
import argparse
import hmac
import json
import time
import urllib.request
import uuid
from pathlib import Path
from protocol import sign
from control_status import query

ROOT=Path(__file__).resolve().parent


def call(path,body):
    key=(ROOT/'private/pairing.key').read_text().strip()
    nonce=str(uuid.uuid4())
    raw=json.dumps(dict(body,nonce=nonce),separators=(',',':')).encode()
    request=urllib.request.Request('http://10.21.33.102:18894'+path,data=raw,
        headers={'Content-Type':'application/json','X-S10-MAC':sign(key,path,raw)})
    with urllib.request.urlopen(request,timeout=2) as reply:
        data=reply.read()
        if not hmac.compare_digest(sign(key,path,data),reply.headers.get('X-S10-MAC','')):
            raise ValueError('导航响应签名校验失败')
    value=json.loads(data)
    if not value.get('ok') or value.get('nonce')!=nonce:
        raise ValueError('导航服务响应异常')
    return value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--select-outdoor',action='store_true')
    args=parser.parse_args()
    deadline=time.monotonic()+45
    while True:
        try:
            state=call('/state',{});control=query()
            nav=state['navigation']
            if not state.get('navigation_version') or not nav.get('route_id'):
                raise ValueError('导航状态初始化中')
            break
        except Exception as error:
            if time.monotonic()>=deadline:
                raise SystemExit('服务启动检查失败：'+str(error))
            time.sleep(.5)
    # This flag is used only if this invocation started a new navigation server.
    # Existing runs keep their route, progress and motion state untouched.
    if args.select_outdoor and nav['route_id']!='full':
        if nav['owner']=='paused' and not nav['active'] and not control['enabled']:
            call('/navigation',dict(action='select',route='full',map_id=state['map_id'],
                ticket=state['command_ticket'],request_id=str(uuid.uuid4())))
            # /state refreshes asynchronously at the navigation tick rate.
            for _ in range(20):
                state=call('/state',{});nav=state['navigation']
                if nav['route_id']=='full':break
                time.sleep(.1)
            else:raise SystemExit('室外路线选择尚未生效，请在 G12 检查')
        else:print('已有操作者接管，保留其当前路线。')
    print('导航：'+state['navigation_version'])
    print('里程计：'+state.get('odometry_backend','gicp'))
    if state.get('odometry_backend')=='lightning':
        lio=state.get('local_odometry',{})
        print('Lightning 实测帧：'+str(lio.get('accepted',0))+'；数据年龄：'+str(lio.get('age_s'))+'；原因：'+str(lio.get('reason','')))
    print('网关：'+str(control.get('diagnostics',{}).get('version',control.get('backend'))))
    print('路线：'+('室外完整路线' if nav['route_id']=='full' else nav['route_id']))
    print('状态：'+nav['owner']+'；定位：'+state.get('mode','未知'))
    print('运动输出：'+str(control.get('output')))
    sensors=state.get('sensors',{})
    for name in ('front','rear'):
        values=[v for k,v in sensors.items() if '/'+name+'/' in k]
        if values:print(name+' 传感器：'+str(values))


if __name__=='__main__':main()
