"""Authenticated LAN gateway. Default MONITOR: zero robot sockets or ROS commands."""
import argparse,hashlib,hmac,json,secrets,threading,time,os
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from unified_control import Controller,Monitor,Simulator

class Server(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,key,controller):
        super().__init__(address,Handler);self.key=key;self.control=controller
        self.lock=threading.RLock();self.tickets={};self.last_issue={};self.running=True
    def signature(self,path,raw):return hmac.new(self.key,(path+'\n'+raw).encode(),hashlib.sha256).hexdigest()
    def tick(self):
        while self.running:
            with self.lock:
                try:self.control.tick(time.monotonic())
                except Exception as e:
                    self.control.stop('后端故障：'+str(e))
                    if hasattr(self.control.adapter,'record'):self.control.adapter.record('controller_error',error=str(e))
                if hasattr(self.control.adapter,'record'):
                    self.control.adapter.record('controller_tick',enabled=self.control.enabled,axes=list(self.control.axes),
                        output=list(self.control.output),policy_switch=self.control.policy_switch,
                        input_kind=self.control.input_kind,input_age_s=time.monotonic()-self.control.last_input,
                        input_seq=self.control.last_seq,reason=self.control.reason,
                        phase=self.control.adapter.phase,guardian_healthy=self.control.adapter.guardian.healthy(time.monotonic()))
            time.sleep(.02)

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def do_POST(self):
        self.connection.settimeout(1.)
        try:
            n=int(self.headers.get('Content-Length','0'))
            if not 0<n<=8192:raise ValueError('BODY_SIZE')
            raw=self.rfile.read(n).decode('utf-8')
            if not hmac.compare_digest(self.headers.get('X-S10-MAC',''),self.server.signature(self.path,raw)):
                self.send_error(403);return
            req=json.loads(raw.lstrip('\ufeff'),parse_constant=lambda x: (_ for _ in ()).throw(ValueError('NONFINITE')))
            if not isinstance(req,dict):raise ValueError('OBJECT_REQUIRED')
            with self.server.lock:
                # A request may have waited behind DDS callbacks. Never compare
                # their new receipt times with a timestamp sampled before lock.
                now=time.monotonic()
                if self.path=='/status':
                    if set(req)!={'nonce'} or not isinstance(req['nonce'],str) or not 1<=len(req['nonce'])<=80:raise ValueError('READ_ONLY_STATUS_NONCE')
                    result=dict(self.server.control.status(now),nonce=req['nonce'],protocol_version=1)
                    adapter=self.server.control.adapter
                    if hasattr(adapter,'diagnostics'):result['diagnostics']=adapter.diagnostics(now)
                    result['input_age_ms']=None if self.server.control.last_input<0 else round((now-self.server.control.last_input)*1000,2)
                    result['input_sequence']=self.server.control.last_seq
                elif self.path=='/ticket':
                    nonce=req.get('nonce');session=req.get('session')
                    if not isinstance(nonce,str) or len(nonce)>80 or not isinstance(session,str) or len(session)>80:raise ValueError('SESSION')
                    # Unique one-shot receiver-issued tickets defeat delayed/replayed controls.
                    self.server.tickets={k:v for k,v in self.server.tickets.items() if now-v[0]<.30}
                    if len(self.server.tickets)>32:raise ValueError('RATE_LIMIT')
                    token=secrets.token_hex(24);self.server.tickets[token]=(now,self.client_address[0],session)
                    result=dict(ticket=token,nonce=nonce)
                elif self.path=='/control':
                    issued,ip,session=self.server.tickets.pop(req.get('ticket',''),(-1,None,None))
                    if ip!=self.client_address[0] or not 0<=now-issued<=.30:raise ValueError('TICKET_EXPIRED_OR_REPLAY')
                    adapter=self.server.control.adapter
                    if hasattr(adapter,'record'):
                        source=req.get('source') if isinstance(req.get('source'),dict) else {}
                        adapter.trace_context=dict(gateway_session=session,gateway_seq=req.get('sample_seq'),
                            source={k:source.get(k) for k in ('run_id','seq','mono','owner','policy','state','motion_authorized','request_id')})
                        adapter.record('control_received',action=req.get('action'),input_kind=req.get('input_kind'),axes=req.get('axes'))
                    self.server.control.new_session(session)
                    try:self.server.control.accept(req,now)
                    except ValueError as e:self.server.control.stop('操作拒绝：'+str(e))
                    result=dict(self.server.control.status(now),request_ticket=req['ticket'])
                    if hasattr(adapter,'record'):
                        adapter.record('control_accepted',result={k:result.get(k) for k in
                            ('enabled','ready','requested','confirmed','actual_policy','gait_response','phase','healthy','health_reason','reason','detail','input_kind','output','policy_switch_paused','policy_switch')})
                else:raise ValueError('PATH')
            response=json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(',',':'))
            data=response.encode();self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(data)));self.send_header('X-S10-MAC',self.server.signature(self.path,response));self.end_headers();self.wfile.write(data)
        except (ValueError,KeyError,TypeError,UnicodeError,TimeoutError):self.send_error(400)
        except (BrokenPipeError,ConnectionResetError):pass

def main():
    p=argparse.ArgumentParser();p.add_argument('--bind',default='127.0.0.1');p.add_argument('--port',type=int,default=18895)
    p.add_argument('--key-file',type=Path,required=True);p.add_argument('--simulate',action='store_true')
    p.add_argument('--execute',action='store_true');p.add_argument('--ack-physical-safety',action='store_true')
    p.add_argument('--plaintext-config-verified',action='store_true')
    p.add_argument('--initialize-key',action='store_true')
    a=p.parse_args()
    if a.initialize_key:
        a.key_file.parent.mkdir(parents=True,exist_ok=True)
        fd=os.open(a.key_file,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as f:f.write(secrets.token_hex(32))
        print('Pairing key created; keep private. No server started.');return
    key=a.key_file.read_text().strip().encode()
    if len(key)<32:p.error('pairing key too short')
    if a.execute and (a.simulate or not a.ack_physical_safety or not a.plaintext_config_verified):p.error('explicit physical safety and existing plaintext-interface confirmation required; never changes robot TLS')
    # Bind first: an occupied HTTP port must not create a robot transport or an
    # orphan guardian. Monitor construction has no ROS/socket side effects.
    adapter=Simulator() if a.simulate else Monitor()
    server=Server((a.bind,a.port),key,Controller(adapter))
    try:
        if a.execute:
            from hardware import Hardware
            adapter=Hardware();server.control=Controller(adapter)
        threading.Thread(target=server.tick,daemon=True).start()
        print('S10 G12 backend='+adapter.backend+' version=1.0-unified bind='+a.bind+':'+str(a.port),flush=True)
        server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.running=False
        with server.lock:
            server.control.stop('网关退出')
            if hasattr(adapter,'close'):adapter.close()
        server.server_close()

if __name__=='__main__':main()
