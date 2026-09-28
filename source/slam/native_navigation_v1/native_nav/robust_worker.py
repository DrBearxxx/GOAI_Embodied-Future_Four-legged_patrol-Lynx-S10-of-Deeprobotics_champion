"""Frozen V2 matching loop with V3 clocks, bounded logs and liveness heartbeat."""
from indoor.pipeline import *
from native_nav.bootstrap import ROOT as NATIVE_ROOT
from native_nav.robust_timing import use_slewed_clocks
from native_nav.bounded_log import BoundedLog

def robust_worker_main(inboxes,outbox,stop,run_id,initial,profile=False,heartbeat=None):
    prof=cProfile.Profile() if profile else None
    if prof:prof.enable()
    log=None
    try:
        cfg=read(ROOT/'config.json');assets=ROOT/'assets';T=None
        if initial:
            T=np.eye(4);T[:3,3]=initial[:3];T[:3,:3]=Rotation.from_euler('z',np.radians(initial[3])).as_matrix()
        engine=use_redundancy(use_slewed_clocks(Engine(assets,cfg,IndoorMatcher(assets,cfg),T)));seq=0;side=0;perception={};perception_native={}
        log=BoundedLog(NATIVE_ROOT/'live_logs'/'robust-matching.jsonl')
        while not stop.is_set():
            # Updated even without input; a disconnected sensor is not a hung
            # worker and must not cause an endless restart loop.
            if heartbeat is not None:heartbeat.value=time.monotonic()
            item=None
            for _ in range(2):
                q=inboxes[side];side=1-side
                try:item=q.get_nowait();break
                except queue.Empty:pass
            if item is None:stop.wait(.001);continue
            before=time.monotonic();received=item['received']
            if before-received>.25:continue
            try:
                event=convert(item['topic'],unpack_cloud(item['cloud']),received)
                if event is None:raise ValueError('unrecognized_cloud')
            except (ValueError,KeyError,TypeError,IndexError,OverflowError) as exc:
                log.write(json.dumps(dict(accepted=False,received=received,reason='malformed_cloud:'+repr(exc)))+'\n')
                continue
            converted=time.monotonic()
            install_ingest(engine,item['ingest']);installed=time.monotonic()
            r=engine.event(event,installed);after=time.monotonic();seq+=1
            mapped=engine.clocks[event['sensor']].map_stamp(event['stamp'],after)
            # Obstacle freshness is independent of the slower map-fit rate.
            # Collision support uses current body-frame raw geometry, not the
            # human-filtered map. No extra motion or localization validity is
            # inferred from this stream. Stale/duplicate/bad frames cannot
            # keep perception alive.
            sensor=event['sensor'];native_key=(engine.clocks[sensor].epoch,event['stamp'])
            span=float(np.max(event['rel'])) if len(event['rel']) else 0.
            good=(mapped is not None and -.025<=after-mapped-span<.25 and
                event['frame']==f'wym_{sensor}_lidar' and len(event['points'])>=300 and
                .03<span<.16 and float(np.min(event['rel']))>=-.001 and
                native_key>perception_native.get(sensor,(-1,-1e30)))
            if good:
                perception[sensor]=(mapped+span,apply(np.array(engine.ext['lidar'][sensor]),event['points']))
                perception_native[sensor]=native_key
            if item['ingest']['depth'] is not None:perception['depth']=item['ingest']['depth']
            timings=dict(queue_ms=(before-received)*1000,convert_ms=(converted-before)*1000,copy_ms=(installed-converted)*1000,
                engine_ms=(after-installed)*1000,measurement_age_ms=None if mapped is None else (after-mapped-float(np.max(event['rel'])))*1000,
                raw_points=item['cloud']['width']*item['cloud']['height'],converted_points=len(event['points']))
            if r:r['pipeline']=timings;log.write(json.dumps(r)+'\n')
            latest_put(outbox,dict(snapshot=diagnostic(engine,after,seq,timings,perception),pipeline=timings))
    except Exception as exc:
        latest_put(outbox,dict(error=repr(exc)))
        if log:log.write(json.dumps(dict(error=repr(exc)))+'\n')
    finally:
        if log:log.close()
        if prof:prof.disable();prof.dump_stats(NATIVE_ROOT/'live_logs'/f'robust-worker-{run_id}.prof')
