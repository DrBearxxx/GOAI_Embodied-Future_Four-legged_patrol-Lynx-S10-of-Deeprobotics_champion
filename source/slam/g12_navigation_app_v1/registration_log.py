"""Bounded asynchronous rejected-GICP fixtures for exact offline replay."""
import pickle,queue,struct,threading,zlib
from pathlib import Path


class RegistrationLog:
    def __init__(self,path,max_bytes=32_000_000,backups=3):
        self.path=Path(path);self.max_bytes=max_bytes;self.backups=backups
        self.queue=queue.Queue(8);self.closed=False;self.written=0;self.dropped=0;self.failed=0;self.error=None
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.thread=threading.Thread(target=self._write,daemon=True,name='rejected-registration-log');self.thread.start()

    def emit(self,record):
        if self.closed:self.dropped+=1;return
        try:self.queue.put_nowait(record)
        except queue.Full:self.dropped+=1

    def _write(self):
        while not self.closed or not self.queue.empty():
            try:record=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                payload=zlib.compress(pickle.dumps(record,protocol=5),1)
                if self.path.exists() and self.path.stat().st_size+len(payload)+4>self.max_bytes:
                    for index in range(self.backups,0,-1):
                        src=self.path.with_name(self.path.name+('.'+str(index-1) if index>1 else ''))
                        dst=self.path.with_name(self.path.name+'.'+str(index))
                        if src.exists():src.replace(dst)
                with self.path.open('ab') as f:f.write(struct.pack('<I',len(payload)));f.write(payload)
                self.written+=1;self.error=None
            except Exception as exc:self.failed+=1;self.error=str(exc)
            finally:self.queue.task_done()

    def status(self):
        return dict(path=str(self.path),written=self.written,dropped=self.dropped,failed=self.failed,error=self.error,queued=self.queue.qsize())

    def close(self):self.closed=True;self.thread.join(2.)


def read_records(path):
    """Read only locally generated trusted fixtures; never untrusted pickle."""
    with Path(path).open('rb') as f:
        while True:
            header=f.read(4)
            if len(header)<4:return
            size=struct.unpack('<I',header)[0];payload=f.read(size)
            if len(payload)<size:return
            yield pickle.loads(zlib.decompress(payload))
