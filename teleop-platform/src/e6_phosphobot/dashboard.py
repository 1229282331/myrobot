"""Single official-SDK dashboard owner with a separately receivable Stop reply.

One ordinary transaction may be outstanding. Stop may be sent while that
transaction awaits its reply; no movement is queued behind it. Raw feedback
is on its own reader and never depends on this command connection.
"""
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
import socket
import threading
import time

from dobot_sdk.core.connection import DobotConnection


class Busy(RuntimeError):
    pass


class MotionDeadlineExpired(RuntimeError):
    """No bytes written: an original motion proof expired before the socket."""
    pass


@dataclass
class Pending:
    command: str
    event: dict
    done: threading.Event = field(default_factory=threading.Event)
    reply: str | None = None
    error: Exception | None = None
    expired: bool = False


class Dashboard(DobotConnection):
    def __init__(self, host, *, reply_timeout=.2, stop_timeout=1.5, enable_timeout=10.):
        super().__init__(host,29999)
        self.reply_timeout=reply_timeout
        self.stop_timeout=stop_timeout
        self.enable_timeout=enable_timeout
        self.pending=[]
        self.events=deque(maxlen=8192)
        self.uncertain=False
        self._transaction_lock=threading.Lock()
        self._closed=threading.Event()
        self._reader=None
        self.completion_wake=None

    def connect(self, timeout=2):
        super().connect(timeout)
        self._socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self._start_receiver()

    def _start_receiver(self):
        self._socket.settimeout(.05)
        self._closed.clear()
        self._reader=threading.Thread(target=self._receive,daemon=True,name='e6-dashboard-replies')
        self._reader.start()

    def send_receive_text(self, text, recv_size=4096):
        return self.wait(self.submit(text))

    def submit(self, text, *, deadline_ns=None):
        if any(c in text for c in ('\r','\n',';')):
            raise ValueError('One official SDK command required')
        is_stop=text=='Stop()'
        wire=(text+'\n').encode('ascii')
        item=Pending(text,dict(command=text,acknowledged=False,accepted=False))
        with self._transaction_lock:
            if not self._connected or self._closed.is_set():
                raise ConnectionError('Dashboard disconnected')
            if is_stop:
                if any(p.command=='Stop()' for p in self.pending):
                    raise Busy('Stop already pending')
            elif self.uncertain:
                raise RuntimeError('Uncertain execution; Stop and explicitly reconnect')
            elif self.pending:
                raise Busy('A command is awaiting its reply; do not queue another target')
            sent_ns=time.perf_counter_ns()
            if deadline_ns is not None and sent_ns>=deadline_ns:
                raise MotionDeadlineExpired('Motion proof expired before write')
            item.event['sent_ns']=sent_ns
            self.events.append(item.event)
            self.pending.append(item)
            try:
                self._socket.sendall(wire)
                item.event['send_completed_ns']=time.perf_counter_ns()
            except Exception as exc:
                self.uncertain=True
                item.expired=True
                item.event.update(error=str(exc),uncertain=True)
                raise
        return item

    def timeout_for(self, command):
        # Brake/drive activation is a separate SDK operation, never a streamed
        # target. Its response budget must not relax Servo or Stop deadlines.
        return (self.stop_timeout if command=='Stop()' else
                self.enable_timeout if command.startswith('EnableRobot(') else self.reply_timeout)

    def poll(self, item):
        if item.error:
            raise item.error
        if item.expired:
            raise TimeoutError('Dashboard reply remains uncertain; never resend this command')
        if item.done.is_set():
            return True
        duration=self.timeout_for(item.command)
        if time.perf_counter_ns()-item.event['sent_ns']>duration*1e9:
            with self._transaction_lock:
                if not item.done.is_set():
                    item.expired=True
                    self.uncertain=True
                    item.event.update(uncertain=True,failed_ns=time.perf_counter_ns(),error='reply_timeout')
                    raise TimeoutError('Dashboard reply timeout; never resend this command')
        return item.done.is_set()

    def wait(self, item):
        duration=self.timeout_for(item.command)
        while not self.poll(item):
            remaining=duration-(time.perf_counter_ns()-item.event['sent_ns'])/1e9
            item.done.wait(max(.0001,remaining))
        return item.reply

    def _receive(self):
        buffer=bytearray()
        sock=self._socket
        try:
            while not self._closed.is_set():
                try:
                    chunk=sock.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError('Dashboard stream closed')
                buffer.extend(chunk)
                if len(buffer)>65536:
                    raise ValueError('Dashboard reply exceeded 64 KiB')
                while b';' in buffer:
                    end=buffer.index(b';')+1
                    reply=bytes(buffer[:end]).decode('ascii').strip()
                    del buffer[:end]
                    with self._transaction_lock:
                        matches=[p for p in self.pending if reply.endswith(','+p.command+';')]
                        if len(matches)!=1:
                            self.uncertain=True
                            self.events.append(dict(unmatched_reply=reply,received_ns=time.perf_counter_ns()))
                            continue
                        item=matches[0]
                        self.pending.remove(item)
                        if item.expired:
                            item.event.update(late_reply=reply,late_reply_ns=time.perf_counter_ns())
                        else:
                            item.reply=reply
                            item.event.update(reply=reply,reply_ns=time.perf_counter_ns(),
                                acknowledged=True,accepted=int(reply.split(',',1)[0])==0)
                            item.done.set()
                            if self.completion_wake is not None:self.completion_wake.set()
        except Exception as exc:
            if not self._closed.is_set():
                self.uncertain=True
                with self._transaction_lock:
                    for item in self.pending:
                        item.error=exc
                        item.event.update(error=str(exc),uncertain=True)
                        item.done.set()
                    if self.completion_wake is not None:self.completion_wake.set()
        finally:
            self._connected=False

    def disconnect(self):
        self._closed.set()
        super().disconnect()
        with self._transaction_lock:
            for item in self.pending:
                item.error=ConnectionError('Dashboard disconnected')
                item.done.set()
        if self._reader is not None:
            self._reader.join(timeout=1)
            if self._reader.is_alive():
                raise RuntimeError('Dashboard receiver failed to exit')


class SubmissionPort:
    """Official SDK command builders submit without waiting on their replies.

    This deliberately returns Pending to the executor; it never reports that
    a command is accepted before the broker has correlated its real reply.
    """
    def __init__(self,dashboard):
        self.dashboard=dashboard
        self.local=threading.local()

    @contextmanager
    def deadline(self,stamp):
        previous=getattr(self.local,'deadline_ns',None)
        self.local.deadline_ns=stamp
        try:yield
        finally:self.local.deadline_ns=previous

    def send_receive_text(self,text,recv_size=4096):
        deadline=getattr(self.local,'deadline_ns',None)
        if deadline is None:return self.dashboard.submit(text)
        return self.dashboard.submit(text,deadline_ns=deadline)
