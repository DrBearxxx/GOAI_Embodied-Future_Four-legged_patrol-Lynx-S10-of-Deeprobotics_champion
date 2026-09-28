"""Bounded diagnostic logs; a disk error cannot terminate the sensor loop."""
from pathlib import Path
import sys
import time


class BoundedLog:
    def __init__(self,path,max_bytes=8*1024*1024,backups=3):
        self.path=Path(path)
        self.max_bytes=max_bytes
        self.backups=backups
        self.stream=None
        self.size=0
        self.errors=0
        self.next_retry=0.

    def _rotate(self):
        if self.stream:
            self.stream.close()
            self.stream=None
        # Only our explicit log paths, no wildcard or recursive deletion.
        for n in range(self.backups,0,-1):
            old=self.path if n==1 else Path(str(self.path)+'.'+str(n-1))
            new=Path(str(self.path)+'.'+str(n))
            if old.is_file():
                old.replace(new)
        self.size=0

    def write(self,text):
        if time.monotonic()<self.next_retry:
            return 0
        try:
            if self.stream is None:
                self.path.parent.mkdir(parents=True,exist_ok=True)
                self.stream=self.path.open('a',buffering=1)
                self.size=self.path.stat().st_size
            length=len(text.encode())
            if self.size+length>self.max_bytes:
                self._rotate()
                self.stream=self.path.open('a',buffering=1)
            # Drop an oversized diagnostic, never let one frame exceed the cap.
            if length>self.max_bytes:
                return 0
            self.stream.write(text)
            self.size+=length
            return len(text)
        except OSError as exc:
            self.errors+=1
            self.next_retry=time.monotonic()+10
            if self.stream:
                try:self.stream.close()
                except OSError:pass
            self.stream=None
            print('LOCALIZER_DIAGNOSTIC_LOG_UNAVAILABLE: '+str(exc),file=sys.stderr,flush=True)
            return 0

    def flush(self):
        if self.stream:
            try:self.stream.flush()
            except OSError:self.errors+=1

    def close(self):
        if self.stream:
            try:self.stream.close()
            except OSError:self.errors+=1
            self.stream=None
