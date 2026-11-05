"""Optional Linux startup residency, before any device or control thread opens.

Use the OS mlockall(MCL_CURRENT) primitive; no future-allocation lock, scheduler
priority, global VM tuning, or relaxed control deadlines. This is not a hard
real-time guarantee. The owner releases exactly its process-wide lock on exit.
"""
from contextlib import contextmanager
import ctypes
from pathlib import Path
import sys
import time


def _fields(text):
    return {line.split(':', 1)[0]: int(line.split()[1])*1024
            for line in text.splitlines() if ':' in line and line.rstrip().endswith('kB')}


def _memory():
    return (_fields(Path('/proc/self/status').read_text()),
            _fields(Path('/proc/meminfo').read_text()))


@contextmanager
def resident_current(mode='off'):
    if mode not in ('off', 'current'):
        raise ValueError('Unknown memory residency mode')
    if mode == 'off':
        yield dict(mode='off', locked_bytes=0)
        return
    if sys.platform != 'linux':
        raise RuntimeError('Current-runtime residency requires Linux')
    import resource
    status, memory = _memory()
    if status.get('VmLck', 0):
        raise RuntimeError('Another owner already locked process memory')
    size = status['VmSize']
    soft, _ = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    budget = 350*1024**2
    if soft != resource.RLIM_INFINITY:
        budget = min(budget, soft)
    if size > budget or memory['MemAvailable']-size < 512*1024**2:
        raise RuntimeError('Insufficient reserved memory for runtime residency before device startup')
    libc = ctypes.CDLL(None, use_errno=True)
    started = time.perf_counter_ns()
    if libc.mlockall(1):  # MCL_CURRENT; intentionally exclude MCL_FUTURE.
        error = ctypes.get_errno()
        # A failed syscall may have made partial progress. The precondition
        # ruled out an earlier lock owner; release only this attempt's pages.
        libc.munlockall()
        raise OSError(error, 'Cannot lock current runtime before device startup')
    try:
        current, _ = _memory()
        if current.get('VmLck', 0) <= 0:
            raise RuntimeError('Memory residency returned success without locked pages')
        yield dict(mode='current', mechanism='mlockall(MCL_CURRENT)',
                   locked_bytes=current['VmLck'], vm_size_bytes=size,
                   startup_ms=(time.perf_counter_ns()-started)/1e6,
                   future_allocations_locked=False, hard_realtime=False)
    finally:
        if libc.munlockall():
            raise OSError(ctypes.get_errno(), 'Cannot release owned runtime residency')
