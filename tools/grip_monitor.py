"""Continuous CAN contact holding guard; no arm commands or auto-recovery."""
import threading,time
try:
    from omnipicker.tools import grasp_can as grip
except ImportError:
    import grasp_can as grip

class HoldingMonitor:
    def __init__(self,channel,position,report,cancelled=lambda:False,*,gate=None,command=None,target_percent=0,force_raw=32):
        self.channel=channel;self.gate=grip.HoldGate(position) if gate is None else gate;self.report=report;self.cancelled=cancelled
        self.last=None;self.count=0;self.error=None;self.end=threading.Event();self.thread=None
        if not 0<=target_percent<=10:raise ValueError('Invalid holding target')
        self.command=grip.can.encode_command(target_percent,grip.bounded_limits(force_raw),bounds=(0,60)) if command is None else command
        report['holding_feedback']=[]
    def worker(self):
        try:
            next_send=time.monotonic()
            while not self.end.is_set():
                if self.cancelled():raise grip.can.CheckFailed('Cancellation')
                f=self.channel.exchange(self.command);self.gate.update(f)
                self.last=time.monotonic();self.count+=1;self.report['holding_feedback'].append(f)
                next_send+=.05;self.end.wait(max(0,next_send-time.monotonic()))
        except BaseException as exc:self.error=exc
    def start(self):
        self.thread=threading.Thread(target=self.worker,daemon=True);self.thread.start()
        deadline=time.monotonic()+3
        while self.count<20:
            if self.error is not None:raise grip.can.CheckFailed(f'Holding monitor failed: {self.error!r}')
            if self.cancelled() or time.monotonic()>deadline:raise grip.can.CheckFailed('Holding monitor failed to establish 20 fresh stable frames')
            time.sleep(.01)
        self.guard()
    def guard(self):
        if self.cancelled() or self.error is not None or self.count<20 or self.last is None or time.monotonic()-self.last>.3:
            raise grip.can.CheckFailed(f'Lost fresh stable gripping feedback: {self.error!r}')
    def stop(self):
        self.end.set()
        if self.thread is not None:
            self.thread.join(1)
            if self.thread.is_alive():raise grip.can.CheckFailed('Holding monitor did not terminate')

class OpenGate:
    def __init__(self,target):self.target=target
    def update(self,f):
        if f.get('fault')!=0 or f.get('state')!=0 or abs(f['position_raw']-self.target)>1:
            raise grip.can.CheckFailed('Opening is not held at its verified target')

class OpeningMonitor(HoldingMonitor):
    def __init__(self,channel,target_percent,report,cancelled=lambda:False,force_raw=32):
        if not 15<=target_percent<=60:raise ValueError('Invalid opening target')
        command=grip.can.encode_command(target_percent,grip.bounded_limits(force_raw),bounds=(0,60))
        super().__init__(channel,command[1],report,cancelled,gate=OpenGate(command[1]),command=command)
