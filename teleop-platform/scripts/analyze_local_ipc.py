"""Compare isolated production-bridge runs; makes no real-device claim."""
import argparse
import json
from pathlib import Path

import numpy as np


def stats(values):
    return dict(n=len(values), **dict(zip(('p50', 'p95', 'p99', 'max'),
                map(float, np.percentile(values, [50, 95, 99, 100]))))) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('evidence', type=Path)
    args = parser.parse_args()
    base = args.evidence
    result = dict(hardware_io=False, scope='X5 virtual plant, not browser or physical latency', groups={})
    sources = {}
    for name in ('startup-baseline', 'startup-candidate'):
        path = base / name
        row = json.loads((path / 'summary.json').read_text(encoding='utf8'))
        sources[name] = json.loads((path / 'source.json').read_text(encoding='utf8'))
        trials = row['trials']
        valid = [t for t in trials if t['first_candidate']]
        result['groups'][name] = dict(passed=row['passed'], error=row['error'], trials=len(trials),
            computed_ms=row['computed_ms'], host_received_ms=row['host_received_ms'], cpu=row['cpu'],
            binary_sha256=row['bridge']['cpp_binary_sha256'],
            computed_to_host_ms=stats([(t['first_candidate']['host_received_ns'] -
                                       t['first_candidate']['computed_ns']) / 1e6 for t in valid]),
            first_take_ms=stats([t['first_take_ms'] for t in trials if t['first_take_ms'] is not None]),
            child_exit=row['child_exit'], forbidden_imports=row['forbidden_imports'])
    before, after = sources.values()
    result['source_differences'] = {key: dict(baseline=before.get(key), candidate=after.get(key))
        for key in sorted(before.keys() | after.keys()) if before.get(key) != after.get(key)}
    result['source_difference_expected'] = set(result['source_differences']) == {
        'python/e6_phosphobot/ipc_snapshot.py', 'python/e6_phosphobot/jazzy_host.py',
        'python/e6_phosphobot/jazzy_cpp_worker.py'}
    for name in ('speed-candidate', 'directions-candidate', 'lifecycle'):
        path = base / name / 'summary.json'
        if not path.exists():
            result['groups'][name] = dict(passed=False, error='Not yet run')
            continue
        row = json.loads(path.read_text(encoding='utf8'))
        group = dict(passed=row.get('passed', False), error=row['error'], child_exit=row['child_exit'])
        if 'cases' in row:
            group['cases'] = row['cases']
        else:
            group['events'] = [{k: v for k, v in event.items() if k != 'info'} for event in row['events']]
        result['groups'][name] = group
    (base / 'comparison.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
