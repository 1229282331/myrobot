"""Reproduce a real process waiting on its result-queue feeder at exit."""
import multiprocessing
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.process_lifecycle import join_draining_results


def emit_large_results(results,finished):
    for _ in range(3):results.put(bytes(512*1024))
    finished.set()


def test_shutdown_drains_results_instead_of_hanging_at_atexit():
    ctx=multiprocessing.get_context('spawn');results=ctx.Queue(4);finished=ctx.Event()
    process=ctx.Process(target=emit_large_results,args=(results,finished));process.start()
    try:
        assert finished.wait(5)
        process.join(.1)
        assert process.is_alive()  # Child body ended, pipe feeder cannot flush.
        assert join_draining_results(process,results,5)
        assert process.exitcode==0
    finally:
        if process.is_alive():process.terminate();process.join(2)
        results.close();results.join_thread()
