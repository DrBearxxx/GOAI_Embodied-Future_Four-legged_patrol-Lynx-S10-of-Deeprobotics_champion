"""Separate bounded sensor worker. Publishes no motor commands or global TF."""
import json
import queue
import time
import numpy as np
from paths import ASSETS, BASE, ROOT
from indoor.pipeline import install_ingest, unpack_cloud, latest_put, diagnostic
from indoor.messages import convert
from indoor.continuity import use_redundancy
from native_nav.robust_timing import use_slewed_clocks
from s10nav.engine import Engine
from seed_matcher import SeedMatcher,AppEngine
from registration_motion import use_registration_motion
from localization_quality import navigation_config
from automatic_localization import AutomaticMatcher,load_config
from registration_log import RegistrationLog

def worker_main(inboxes, outbox, commands, stop, heartbeat, transaction):
    cfg = navigation_config(json.loads((BASE/'config.json').read_text()))
    cfg['threads'] = 2
    matcher = SeedMatcher(ASSETS, cfg)
    automatic=AutomaticMatcher(matcher,cfg,load_config(ROOT))
    automatic_log=RegistrationLog(ROOT/'logs/automatic-map-scans.bin',max_bytes=32_000_000,backups=3)
    def new_engine(): return use_registration_motion(use_slewed_clocks(AppEngine(ASSETS,cfg,matcher)))
    active = new_engine()
    pending = None
    job = dict(state='IDLE', reason='请在地图点选当前位置附近，启动定位')
    side = seq = epoch = 0
    candidate = None
    last_emit = 0.
    while not stop.is_set():
        heartbeat.value = time.monotonic()
        try:
            command = commands.get_nowait()
            automatic.latest=None
            if command.get('cancel'):
                if pending and command['cancel']==pending['request_id']:
                    job = dict(job, state='CANCELLED', reason='用户取消；原定位不被覆盖')
                    pending = candidate = None
                    matcher.finish()
            else:
                try:
                    count = matcher.begin(command)
                    pending = command
                    candidate = new_engine()
                    job = dict(request_id=command['request_id'],state='SEARCHING',reference_count=count,
                               xyz=command['xyz'],radius=command['radius'],reason='限定区域内进行几何匹配',required_confirmations=1)
                except ValueError as exc:
                    pending = candidate = None
                    matcher.finish()
                    job = dict(request_id=command['request_id'],state='FAILED',reason=str(exc))
        except queue.Empty: pass
        now = time.monotonic()
        if pending and now-pending['issued'] > pending['timeout']:
            job = dict(job,state='FAILED',reason='60 秒内没有获得稳定几何匹配；可重新点选')
            pending = candidate = None
            matcher.finish()
        item = None
        for _ in range(2):
            q = inboxes[side]; side = 1-side
            try: item = q.get_nowait(); break
            except queue.Empty: pass
        result = None
        if item and now-item['received'] < .25:
            engine = candidate if pending else active
            install_ingest(engine,item['ingest'])
            local=item.get('local_motion')
            if local and local.get('accepted',0)>=3 and 0<=item['received']-local['t']<=.25:
                v=np.asarray(local.get('velocity',[0.,0.,0.]))
                if v.shape==(3,) and np.isfinite(v).all():
                    # Past measured local velocity, not scan-to-map jitter.
                    engine.velocity=v.copy()
            if pending:
                if (local and pending.get('map_to_odom') is not None and
                        local.get('epoch')==pending.get('heading_odom_epoch')):
                    R=np.asarray(pending['map_to_odom'])[:3,:3]@np.asarray(local['T'])[:3,:3]
                    matcher.scope['heading_prior']=float(np.arctan2(R[1,0],R[0,0]))
                try:
                    e = convert(item['topic'],unpack_cloud(item['cloud']),item['received'])
                    result = engine.event(e,time.monotonic())
                except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
                    job['last_rejection'] = 'MALFORMED_SCAN:'+str(exc)[:160]
                now = time.monotonic()
                if pending:
                    job['last_rejection'] = engine.reason
                    job['confirmations'] = engine.confirmations
                    if (result and result['accepted'] and engine.poll(now)['valid'] and
                            engine.confirmations >= cfg['relocalization']['confirmations']):
                        # Linearize cancellation vs. commit across HTTP/worker.
                        with transaction.get_lock():
                            if transaction.value.decode()=='ACTIVE:'+pending['request_id']:
                                transaction.value=('COMMITTED:'+pending['request_id']).encode()
                                epoch += 1
                                active = candidate
                                job = dict(job,state='SUCCEEDED',reason='几何匹配可用',
                                           pose=active.T[:3,3].tolist(),epoch=epoch,
                                           odometry_epoch=local.get('epoch') if local else None,
                                           quality=active.last_metrics.get('quality'))
                            else:
                                job = dict(job,state='CANCELLED',reason='取消先于提交；没有覆盖原定位')
                        pending = candidate = None
                        matcher.finish()
            else:
                automatic.observe(item,engine,now,automatic_log)
        now = time.monotonic()
        if now-last_emit > .1 or result:
            seq += 1
            s = diagnostic(active, now, seq, {}, {})
            # No candidate is published as a live pose before confirmation.
            if s:
                s['metrics'] = active.last_metrics
                s['generation'] = [epoch, active.generation]
                s['anchor']['generation'] = [epoch, active.generation]
                s['anchor']['odometry_epoch'] = job.get('odometry_epoch')
                s['anchor']['quality_level']=active.last_metrics.get('quality_level','tracking')
            latest_put(outbox,dict(snapshot=s,job=job.copy(),epoch=epoch,pending=bool(pending),
                                  automatic=automatic.latest,automatic_recording=automatic_log.status()))
            last_emit = now
        if item is None: stop.wait(.003)
    automatic_log.close()
