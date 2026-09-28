"""Quarantine a bad packet without restamping the last valid observation."""
import json
from . import bootstrap
from indoor.transport import JsonIngress


class ResilientIngress(JsonIngress):
    HARD_ERRORS={'DDS_WRITER_CHANGED','MISSING_WRITER_ID','AMBIGUOUS_LOCALIZATION_PUBLISHERS'}

    def __init__(self):
        super().__init__();self.dropped=0;self.hard_fault=None

    def receive(self,text,info,wall_ns,mono,graph_writer=None):
        old_value,old_received=self.value,self.received
        self.hard_fault=None
        ok=super().receive(text,info,wall_ns,mono,graph_writer)
        if ok:
            try:
                # Python's default JSON parser accepts NaN/Infinity. Do not let
                # those poison downstream calculations or strict JSON logging.
                json.dumps(self.value,allow_nan=False)
            except (ValueError,OverflowError):
                self.value=None;self.error='NONFINITE_JSON';ok=False
        if not ok:
            self.dropped+=1
            if self.error in self.HARD_ERRORS:self.hard_fault=self.error
            else:
                # Original receipt and measurement times remain unchanged. get()
                # and route gates still expire the old evidence independently.
                self.value,self.received=old_value,old_received
        return ok
