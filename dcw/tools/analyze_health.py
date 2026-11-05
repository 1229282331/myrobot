"""Offline timing checks. Never equate receive-time pairing with exposure sync."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def timing(rows):
    output = {}
    for stream in sorted({r['stream'] for r in rows}):
        group = [r for r in rows if r['stream'] == stream]
        if len(group) < 2:
            output[stream] = {'error': 'Fewer than two frames'}
            continue
        clock = 'host_perf_counter_ns' if group[0].get('host_perf_counter_ns') else 'host_monotonic_ns'
        host = np.array([int(r[clock]) for r in group], dtype=np.int64)
        delta = np.diff(host) / 1e6
        item = {'frames': len(group), 'host_clock': clock,
                'host_gap_ms': dict(zip(['p50','p95','p99','max'], map(float, np.percentile(delta, [50,95,99,100]))))}
        elapsed_us = (host[-1]-host[0])/1000
        for key in ('device_timestamp_us', 'sdk_system_timestamp_us'):
            stamp = np.array([int(r[key]) for r in group], dtype=np.int64)
            ratio = float((stamp[-1]-stamp[0])/elapsed_us) if elapsed_us > 0 else None
            item[key] = {'median_step_us': float(np.median(np.diff(stamp))), 'duration_vs_host_ratio': ratio,
                         'rate_plausible_as_microseconds': ratio is not None and .95 < ratio < 1.05,
                         'nonincreasing_steps': int(np.sum(np.diff(stamp) <= 0))}
        output[stream] = item
    return output


def analyze(path):
    report = json.loads((path/'report.json').read_text(encoding='utf-8'))
    with (path/'frames.csv').open(encoding='utf-8') as f:
        result = {'timing': timing(list(csv.DictReader(f)))}
    sample_path = path/'samples.jsonl'
    if sample_path.exists():
        samples = [json.loads(x) for x in sample_path.read_text().splitlines()]
        depths = [r['depth'] for r in samples if 'depth' in r]
        if depths:
            fraction = [d['valid_fraction'] for d in depths]
            result['sampled_depth'] = {'sample_count': len(depths), 'valid_fraction_min_median_max': list(map(float, [min(fraction),np.median(fraction),max(fraction)])),
                                       'center_median_mm_first_last': [depths[0]['center_median_mm'], depths[-1]['center_median_mm']]}
    result['caveat'] = 'Timestamps are recorded as returned by SDK. Plausible clock rate does not validate exposure time, RGB-depth synchronization or robot synchronization.'
    (path/'analysis.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return {'run': path.name, 'status': report.get('status'), 'streams': report.get('streams'), 'depth_quality': report.get('depth_quality'), **result}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run), ensure_ascii=False, indent=2))
