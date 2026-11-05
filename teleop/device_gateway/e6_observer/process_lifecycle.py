"""Process shutdown helpers, independent of SDKs and platform CAN APIs."""
import queue
import time


def join_draining_results(process,results,timeout=5):
    """Drain a multiprocessing result pipe after its consumer has stopped.

    Device cleanup runs in the child before exit. A full pipe must not leave
    the child's feeder (and the parent's atexit join) waiting indefinitely.
    """
    deadline=time.monotonic()+timeout
    while process.is_alive() and time.monotonic()<deadline:
        try:results.get(timeout=min(.05,max(0,deadline-time.monotonic())))
        except queue.Empty:pass
        process.join(0)
    return not process.is_alive()
