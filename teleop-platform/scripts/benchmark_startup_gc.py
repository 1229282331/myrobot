"""Device-free GC comparison using the gateway's real import graph.

Run in separate processes with/without --freeze. No Executor, socket, camera,
thread or hardware object is constructed. Retain all timings, including warmup.
"""
import argparse
import gc
import json
import time
import weakref
from contextlib import nullcontext

from e6_phosphobot import gateway, audit, episodes, episode_http, gripper
from e6_phosphobot import jazzy_host, latest_json_files, moveit_bridge
from e6_phosphobot.runtime_timing import frozen_startup_heap


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--freeze', action='store_true')
    args = parser.parse_args()
    before = dict(enabled=gc.isenabled(), thresholds=gc.get_threshold())
    samples = []

    class Cycle:
        pass

    context = frozen_startup_heap() if args.freeze else nullcontext({'policy': 'baseline'})
    with context as policy:
        for _ in range(30):
            refs = []
            for _ in range(1000):
                value = Cycle()
                value.cycle = value
                refs.append(weakref.ref(value))
            del value
            start = time.perf_counter_ns()
            collected = gc.collect(2)
            end = time.perf_counter_ns()
            samples.append(dict(duration_ms=(end-start)/1e6, collected=collected,
                                dynamic_cycles_reclaimed=all(ref() is None for ref in refs)))
        unchanged = before == dict(enabled=gc.isenabled(), thresholds=gc.get_threshold())
    result = dict(policy=policy, samples=samples, gc_configuration_unchanged=unchanged,
                  freeze_count_after_exit=gc.get_freeze_count(), device_connections=0)
    print(json.dumps(result))
    assert unchanged and all(s['dynamic_cycles_reclaimed'] for s in samples)
    assert result['freeze_count_after_exit'] == 0


if __name__ == '__main__':
    main()
