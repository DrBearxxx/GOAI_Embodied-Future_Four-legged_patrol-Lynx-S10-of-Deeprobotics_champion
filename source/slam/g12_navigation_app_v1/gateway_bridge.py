"""Nonblocking caller side of the sole velocity/waypoint gateway."""
import collections
import hashlib
import hmac
import json
import threading
import time
import uuid
import urllib.request


class GatewayBridge:
    def __init__(self,url=None,key=None,trace=None):
        self.url=url; self.key=key; self.client=bool(url)
        self.lock=threading.RLock(); self.actions=collections.deque(); self.value={}
        self.payload=None; self.received=-1e9; self.error='SHADOW_ONLY' if not url else 'CONNECTING'
        self.last_action=None; self.command_count=0; self.nonzero_count=0
        self.closed=False; self.session=str(uuid.uuid4()); self.seq=0; self.epoch=0
        self.thread=None
        self.trace=trace
        if url:
            self.thread=threading.Thread(target=self._loop,daemon=True);self.thread.start()

    def _rpc(self,path,value):
        raw=json.dumps(value,separators=(',',':')).encode()
        sign=lambda b:hmac.new(self.key.encode(),path.encode()+b'\n'+b,hashlib.sha256).hexdigest()
        req=urllib.request.Request(self.url+path,data=raw,headers={'Content-Type':'application/json','X-S10-MAC':sign(raw)})
        with urllib.request.urlopen(req,timeout=.25) as r:
            b=r.read(65537)
            if len(b)>65536 or not hmac.compare_digest(sign(b),r.headers.get('X-S10-MAC','')):raise ValueError('BAD_GATEWAY_SIGNATURE')
            return json.loads(b)

    def action(self,kind,request_id,now,input_kind=None):
        if not self.client: raise ValueError('SHADOW_NO_HARDWARE_GATEWAY')
        with self.lock:
            if len(self.actions)>=8: raise ValueError('ACTION_BUSY')
            self.actions.append((kind,request_id,now,self.epoch,input_kind))
            self.last_action=dict(kind=kind,request_id=request_id,state='QUEUED')

    def stop(self):
        with self.lock:
            self.epoch+=1; self.payload=None; self.actions.clear()
            if self.client:self.actions.append(('pause',str(uuid.uuid4()),time.monotonic(),self.epoch,None))
            self.value={**self.value,'enabled':False}

    def poll(self,now): pass

    def ready(self):
        with self.lock: return bool(time.monotonic()-self.received<.4 and
            (self.value.get('ready') or self.value.get('live_input_ready')) and self.value.get('enabled'))

    def policy_ready(self,policy,input_kind=None):
        with self.lock: return bool(time.monotonic()-self.received<.4 and self.value.get('ready') and self.value.get('confirmed')==policy
            and (input_kind is None or self.value.get('input_kind')==input_kind))

    def send(self,proposal,now):
        with self.lock:self.payload=(dict(proposal),now,self.epoch)

    def state(self):
        with self.lock:
            fresh=time.monotonic()-self.received<.4
            return dict(self.value,connected=bool(fresh and self.client),enabled=bool(fresh and self.value.get('enabled')),backend=self.value.get('backend','disconnected' if self.client else 'shadow'),
                ready=self.ready(),armed=self.ready(),policy_ready=bool(fresh and self.value.get('ready')),
                confirmed=self.value.get('confirmed','none') if fresh else 'none',reason=self.error or self.value.get('reason'),
                actual_policy=self.value.get('actual_policy',self.value.get('confirmed','none')) if fresh else 'none',
                last_action=self.last_action,command_count=self.command_count,nonzero_count=self.nonzero_count)

    def _loop(self):
        while not self.closed:
            started=time.monotonic(); op=None
            try:
                nonce=str(uuid.uuid4()); offer=self._rpc('/ticket',dict(nonce=nonce,session=self.session))
                if offer.get('nonce')!=nonce:raise ValueError('TICKET_NONCE_MISMATCH')
                with self.lock:
                    now=time.monotonic(); item=self.payload
                    op=self.actions.popleft() if self.actions else None
                    if op and (op[3]!=self.epoch or now-op[2]>1.):op=None
                    p,at,epoch=item if item else ({},-1e9,self.epoch)
                    fresh=epoch==self.epoch and 0<=now-at<=.20
                    active=fresh and p.get('active') and p.get('execution_requested')
                    self.seq+=1
                    kind=p.get('input_kind','velocity')
                    producer_kind=kind
                    if op and op[4] is not None:kind=op[4]
                    v=p.get('axes',[0.,0.,0.]) if kind in ('waypoint_axes','native_axes') else [p.get('vx',0.),p.get('vy',0.),p.get('wz',0.)]
                    action=op[0] if op else ''
                    values=v if active and action!='run' and kind==producer_kind else [0.,0.,0.]
                    # A stopped producer may still issue explicit posture/policy actions.
                    body=dict(ticket=offer['ticket'],fresh=True,sample_seq=self.seq,sample_age_ms=0,
                        axes=values,input_kind=kind,action=action,
                        source={k:p.get(k) for k in ('run_id','seq','mono','owner','policy','state','motion_authorized')})
                    if op:body['source']['request_id']=op[1]
                    if self.trace:self.trace.emit('gateway_request',gateway_session=self.session,gateway_seq=self.seq,
                        source=body['source'],action=action,input_kind=kind,axes=values,producer_fresh=fresh)
                    reply=self._rpc('/control',body)
                    if reply.get('request_ticket')!=offer['ticket']:raise ValueError('CONTROL_REPLY_MISMATCH')
                    self.received=time.monotonic();self.value=reply;self.error=''
                    self.command_count+=1;self.nonzero_count+=int(any(values))
                    if self.trace:self.trace.emit('gateway_reply',gateway_seq=self.seq,gateway_session=self.session,
                        source=body['source'],elapsed_s=time.monotonic()-now,
                        result={k:reply.get(k) for k in ('enabled','ready','requested','confirmed','phase','healthy','health_reason','reason','input_kind','output','policy_switch_paused','policy_switch')})
                    if op:self.last_action=dict(kind=op[0],request_id=op[1],state='RECEIVED',reason=reply.get('reason'))
                    # Producer failure is latched; zero heartbeats must not renew enable.
                    if not active and not op and reply.get('enabled'):self.stop()
            except Exception as exc:
                if self.trace:self.trace.emit('gateway_error',gateway_session=self.session,gateway_seq=self.seq,error=str(exc))
                with self.lock:
                    self.error=str(exc);self.value={};self.payload=None;self.actions.clear();self.epoch+=1
                    if op:self.last_action=dict(kind=op[0],request_id=op[1],state='UNCONFIRMED_NO_RETRY',reason=str(exc))
            time.sleep(max(.005,.05-(time.monotonic()-started)))

    def close(self):
        self.stop()
        if self.thread:time.sleep(.1)
        self.closed=True
        if self.thread:self.thread.join(.6)
