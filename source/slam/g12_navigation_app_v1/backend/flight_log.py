"""Bounded asynchronous JSONL recording. Recording never controls the robot."""
import json,logging,os,queue,sys,threading,time,uuid
from logging.handlers import RotatingFileHandler
from pathlib import Path


class FlightLog:
    def __init__(self,path,max_bytes=32_000_000,backups=7,capacity=4096):
        self.path=Path(path);self.session=str(uuid.uuid4());self.queue=queue.Queue(capacity)
        self.closed=False;self.written=0;self.dropped=0;self.failed=0;self.error=None
        self.thread=None;self.handler=None
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            owner=self
            class Handler(RotatingFileHandler):
                def handleError(self,record):
                    owner.failed+=1;owner.error=str(sys.exc_info()[1])
            self.handler=Handler(self.path,maxBytes=max_bytes,backupCount=backups,encoding='utf-8')
            self.handler.setFormatter(logging.Formatter('%(message)s'))
            self.thread=threading.Thread(target=self._write,name='flight-log',daemon=True);self.thread.start()
            try:boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            except OSError:boot=None
            self.emit('logger_started',pid=os.getpid(),boot_id=boot,format_version=1)
        except OSError as exc:self.error=str(exc)

    def emit(self,event,**fields):
        if self.closed or self.handler is None:self.dropped+=1;return
        try:
            # Serialize before enqueue so later mutations cannot change evidence.
            record=json.dumps(dict(event=event,log_session=self.session,mono=time.monotonic(),
                wall_ns=time.time_ns(),**fields),ensure_ascii=False,allow_nan=False,separators=(',',':'))
            self.queue.put_nowait(record)
        except (ValueError,TypeError,OverflowError,queue.Full):self.dropped+=1

    def _write(self):
        while not self.closed or not self.queue.empty():
            try:line=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                before=self.failed
                self.handler.handle(logging.LogRecord('flight',logging.INFO,'',0,line,(),None))
                if before==self.failed:self.written+=1;self.error=None
            except Exception as exc:self.failed+=1;self.error=str(exc)
            finally:self.queue.task_done()

    def status(self):
        return dict(path=str(self.path),session=self.session,written=self.written,
                    queued=self.queue.qsize(),dropped=self.dropped,failed=self.failed,error=self.error)

    def close(self):
        self.closed=True
        if self.thread:self.thread.join(2.)
        if self.handler and (self.thread is None or not self.thread.is_alive()):self.handler.close()


def navigation_sample(navigation,solution,perception,control,operator):
    """Explicit fields only: no HTTP tickets, authentication headers or keys."""
    solution=solution or {}
    fields=('mode','reason','pose','estimate_mono','measurement_mono','map_age_s','odom_age_s',
            'generation','xy_budget_m','yaw_budget_rad','correction_remaining_m','correction_remaining_rad',
            'rejected_map_innovations','max_vx','max_wz','arrival_valid','velocity_body',
            'measurement_source','map_association_enabled','method','quality_level','odometry_generation','pose_source_switches','raw_map_pose',
            'map_measurement_mono','map_registration_generation','rejected_map_measurement_mono','prediction','measurement_pose','lidar_continuity','automatic_localization')
    keys=('connected','enabled','ready','policy_ready','requested','confirmed','phase','healthy',
          'health_reason','reason','detail','actual_policy','gait_response','input_kind','output_units','output','command_count','nonzero_count')
    return dict(navigation=navigation,localization={k:solution.get(k) for k in fields},
                perception=perception,control={k:control.get(k) for k in keys},operator=operator)
