"""Fresh DDS JSON ingress; queued/replayed messages cannot refresh a lease."""
import json,math


class JsonIngress:
    def __init__(self):
        self.writer=None;self.stamp=0;self.value=None;self.received=-1e9;self.error='NO_MESSAGE'

    def receive(self,text,info,wall_ns,mono,graph_writer=None):
        try:
            source=info['source_timestamp'];receipt=info['received_timestamp']
            # Jazzy's Python MessageInfo on this Orin omits publisher_gid.
            # The caller must then provide the sole current graph endpoint;
            # source/receipt timestamps remain the actual per-message DDS values.
            writer=bytes(info.get('publisher_gid',graph_writer))
            if not writer or not any(writer):raise ValueError('MISSING_WRITER_ID')
            if not all(type(v) is int and v>0 for v in (source,receipt)):raise ValueError('MISSING_DDS_TIMESTAMPS')
            if not -.02<=(wall_ns-receipt)*1e-9<=.1 or not -.02<=(wall_ns-source)*1e-9<=.2:raise ValueError('DDS_MESSAGE_STALE')
            if self.writer is not None and writer!=self.writer:
                self.writer=writer;self.stamp=source;raise ValueError('DDS_WRITER_CHANGED')
            self.writer=writer
            if source<=self.stamp:raise ValueError('DDS_REPLAY_OR_CLOCK_RESET')
            self.stamp=source
            if len(text)>50000:raise ValueError('OVERSIZED_JSON')
            obj=json.loads(text)
            if not isinstance(obj,dict) or not math.isfinite(obj['mono']) or not -.02<=mono-obj['mono']<=.25:raise ValueError('PAYLOAD_STALE')
            self.value=obj;self.received=mono;self.error=None
            return True
        except (KeyError,TypeError,ValueError,OverflowError) as exc:
            self.value=None;self.error=str(exc)
            return False

    def get(self,mono):
        return self.value if 0<=mono-self.received<=.25 else None
