"""Bounded diagnostics for one periodic worker; never a control policy.

Wall gaps, process CPU time and Python GC intervals retain original timestamps.
Process CPU includes all worker threads, so it is not a scheduling certificate.
"""
from collections import deque
from contextlib import contextmanager
import gc
import sys
import time


@contextmanager
def cooperative_thread_scheduling():
    """Shorter CPython handoff quantum for this gateway's I/O workers.

    Apply before threads start and restore after they close. This is not a
    real-time guarantee or an OS scheduling priority. Long native calls can
    still delay handoff; device/input freshness and Stop remain independent.
    """
    previous = sys.getswitchinterval()
    interval = min(previous, .001)
    sys.setswitchinterval(interval)
    try:
        yield dict(policy='gateway_cpython_thread_handoff',
                   previous_s=previous, interval_s=sys.getswitchinterval())
    finally:
        sys.setswitchinterval(previous)


@contextmanager
def frozen_startup_heap():
    """Exclude the process's imported, long-lived graph from later full GC.

    Enter before any device/worker starts, exit after all owners close. Runtime
    allocations still undergo ordinary cyclic GC; deadlines and feedback are
    unchanged. CPython reference counting is unchanged, too. Never freeze a
    live session's recorder/controller graph or change the global thresholds.
    """
    if gc.get_freeze_count():
        raise RuntimeError('Startup heap already frozen by another owner')
    enabled = gc.isenabled()
    gc.collect()
    gc.freeze()
    report = dict(policy='freeze_import_graph_before_device_start',
                  frozen_objects=gc.get_freeze_count(), automatic_gc=enabled,
                  thresholds=gc.get_threshold())
    try:
        yield report
    finally:
        gc.unfreeze()


class RuntimeTiming:
    def __init__(self, clock=time.perf_counter_ns, cpu=time.process_time_ns, *, gap_slow_ms=20):
        self.clock=clock;self.cpu=cpu
        self.gap_slow_ms=gap_slow_ms
        self.previous_start=None;self.previous_end=None;self.previous_cpu=None
        self.started=None;self.started_cpu=None
        self.marked=None;self.marked_cpu=None;self.segments=[]
        self.pumps=0;self.max_gap_ms=0.;self.max_work_ms=0.
        self.slow=deque(maxlen=8);self.gc_slow=deque(maxlen=8)
        self.gc_started={};self.gc_count=[0,0,0];self.gc_max_ms=[0.,0.,0.]
        self.installed=False

    def install(self):
        if not self.installed:
            gc.callbacks.append(self.on_gc);self.installed=True

    def close(self):
        if self.installed:
            gc.callbacks.remove(self.on_gc);self.installed=False

    def on_gc(self, phase, info):
        generation=info['generation'];now=self.clock()
        if phase=='start':
            self.gc_started[generation]=now
        elif phase=='stop':
            start=self.gc_started.pop(generation,None)
            if start is None:return
            duration=(now-start)/1e6
            self.gc_count[generation]+=1
            self.gc_max_ms[generation]=max(self.gc_max_ms[generation],duration)
            if duration>=20:
                self.gc_slow.append(dict(generation=generation,start_ns=start,end_ns=now,
                    duration_ms=duration,collected=info.get('collected'),uncollectable=info.get('uncollectable')))

    def begin(self):
        now=self.clock();cpu=self.cpu()
        if self.previous_end is not None:
            gap=(now-self.previous_end)/1e6
            self.max_gap_ms=max(self.max_gap_ms,gap)
            if gap>=self.gap_slow_ms:
                self.slow.append(dict(kind='between_callbacks',start_ns=self.previous_end,end_ns=now,
                    wall_ms=gap,process_cpu_ms=(cpu-self.previous_cpu)/1e6))
        self.started=now;self.started_cpu=cpu;self.pumps+=1
        self.marked=now;self.marked_cpu=cpu;self.segments=[]

    def mark(self, name):
        now=self.clock();cpu=self.cpu();duration=(now-self.marked)/1e6
        if duration>=20:
            self.segments.append(dict(stage=name,start_ns=self.marked,end_ns=now,
                wall_ms=duration,process_cpu_ms=(cpu-self.marked_cpu)/1e6))
        self.marked=now;self.marked_cpu=cpu

    def end(self):
        self.mark('callback_end')
        now=self.clock();cpu=self.cpu();work=(now-self.started)/1e6
        self.max_work_ms=max(self.max_work_ms,work)
        if work>=20:
            self.slow.append(dict(kind='callback_work',start_ns=self.started,end_ns=now,
                wall_ms=work,process_cpu_ms=(cpu-self.started_cpu)/1e6,segments=list(self.segments)))
        self.previous_start=self.started;self.previous_end=now;self.previous_cpu=cpu

    def snapshot(self):
        return dict(pumps=self.pumps,max_callback_gap_ms=self.max_gap_ms,
            max_callback_work_ms=self.max_work_ms,slow=list(self.slow),gc_slow=list(self.gc_slow),
            gc_count=list(self.gc_count),gc_max_ms=list(self.gc_max_ms),
            interpretation='Process CPU includes all worker threads; timings do not change deadlines or controller state.')
