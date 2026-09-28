"""Process liveness only; a heartbeat never counts as fresh sensor evidence."""


class HeartbeatWatchdog:
    def __init__(self, started, startup_timeout=45., timeout=5.):
        self.started=started
        self.startup_timeout=startup_timeout
        self.timeout=timeout
        self.last=None

    def beat(self, now):
        self.last=now

    def expired(self, now):
        return now-(self.started if self.last is None else self.last) > (
            self.startup_timeout if self.last is None else self.timeout)
