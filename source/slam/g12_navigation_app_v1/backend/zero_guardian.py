"""Independent, latched native-zero watchdog. Never sends nonzero or SDK commands."""
import argparse,datetime,json,os,select,socket,struct,time

def zero_packet(sequence,command=0x110002):
    if command not in (0x110002,0x100002):raise ValueError('INVALID_ZERO_CHANNEL')
    data=json.dumps({'PatrolDevice':dict(Type=0x100001,Command=command,
         Time=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
         Items=dict(X=0.,Y=0.,Z=0.,Roll=0.,Pitch=0.,Yaw=0.))},separators=(',',':')).encode()
    return struct.pack('<4sHHBBB5s',b'\xeb\x91\xeb\x90',len(data),sequence%65536,1,sequence%256,1,bytes(5))+data

def child(fd,channel):
    robot=socket.socket(fileno=fd);control=socket.socket(fileno=channel);control.setblocking(False)
    active=False;latched=False;last=time.monotonic();sent=-1e9;seq=32768;disconnected=None;command=0x110002
    while True:
        now=time.monotonic()
        if disconnected is not None:time.sleep(.02)
        elif select.select([control],[],[],.02)[0]:
            try:data=control.recv(64)
            except BlockingIOError:continue
            except ConnectionResetError:data=b''
            if not data:
                if disconnected is None:disconnected=now
            else:
                # A pulse is permission to watch native control, never to move.
                active=data[-1:] in (b'1',b'J');last=now
                if data[-1:]==b'J':command=0x100002
                elif data[-1:]==b'1':command=0x110002
                # Q is an explicit transfer barrier: acknowledge only after
                # native-zero output has been deactivated. A latched fault
                # cannot be cleared by Q. Process coalesced Q/0 pulses together.
                quiet=b'Q' in data and not active
                try:control.send(b'L' if latched else b'Q' if quiet else b'R')
                except (BrokenPipeError,ConnectionResetError):disconnected=now
                except BlockingIOError:pass
        if active and (now-last>.4 or disconnected is not None):latched=True
        if latched and now-sent>=.05:
            try:robot.send(zero_packet(seq,command));seq+=1;sent=now
            except OSError:pass
        if disconnected is not None and (not active or now-disconnected>2.):break
    robot.close();control.close()

class Guardian:
    def __init__(self,robot):
        import subprocess,sys
        from pathlib import Path
        parent,worker=socket.socketpair();parent.setblocking(False)
        self.pipe=parent;self.last_reply=-1e9;self.latched=False;self.quiet_ack=False
        self.proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--socket',str(robot.fileno()),'--channel',str(worker.fileno())],pass_fds=(robot.fileno(),worker.fileno()),start_new_session=True)
        worker.close()
    def pulse(self,active,now,command=0x110002):
        if active:self.quiet_ack=False
        try:self.pipe.send((b'J' if command==0x100002 else b'1') if active else b'0')
        except (BlockingIOError,BrokenPipeError,ConnectionResetError):self.latched=True
        self.receive(now)
    def receive(self,now):
        while True:
            try:data=self.pipe.recv(64)
            except BlockingIOError:break
            if not data:self.latched=True;break
            self.last_reply=now
            if b'L' in data:self.latched=True
            if b'Q' in data:self.quiet_ack=True
    def quiesce(self,now):
        # Drain acknowledgments from any older transaction before issuing Q.
        self.receive(now);self.quiet_ack=False
        try:self.pipe.send(b'Q')
        except (BlockingIOError,BrokenPipeError,ConnectionResetError):self.latched=True
    def quiesced(self,now):return self.quiet_ack and self.healthy(now)
    def healthy(self,now):return self.proc.poll() is None and not self.latched and now-self.last_reply<.3
    def close(self):self.pipe.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--socket',type=int,required=True);p.add_argument('--channel',type=int,required=True);a=p.parse_args();child(a.socket,a.channel)
