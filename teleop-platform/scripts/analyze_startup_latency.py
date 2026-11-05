"""Decompose recorded starts using X5 monotonic timestamps only.

Cycle diagnostics are sampled: first-observed transitions are upper bounds,
not exact service completion times. Retained trajectory provenance identifies
actual publications; it is not an end-to-end browser/network measurement.
"""
import argparse
import gzip
import json
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from e6_phosphobot.clock_probe import input_latency_bound


def analyze(base):
    motion = json.loads((base / 'analysis.json').read_text(encoding='utf-8'))
    groups = motion['groups']
    starts = [g['start_ns'] for g in groups]
    windows = [[] for _ in groups]
    probes = {}; input_clocks = {}
    wanted = {g['first_sequence'] for g in groups}
    from bisect import bisect_right
    with gzip.open(base / 'session/x5/audit/events.jsonl.gz', 'rt', encoding='utf-8') as source:
        for line in source:
            event = json.loads(line)
            if event['event'] == 'clock_sample':
                probes[event['connection_id']] = event['data']
            elif event['event'] == 'input' and event['data'].get('seq') in wanted:
                input_clocks[event['at_ns']] = (event['data'].get('client_sample_ms'),
                    probes.get(event.get('connection_id')))
            index = bisect_right(starts, event['at_ns']) - 1
            if index >= 0 and event['at_ns'] < min(groups[index]['release_ns'], starts[index] + 600_000_000):
                windows[index].append(event)
    rows = []
    for group, events in zip(groups, windows):
        start = group['start_ns']
        def relative(stamp):
            return None if stamp is None else (stamp - start) / 1e6
        sent = [e for e in events if e['event'] == 'motion_submitted'
                and group['first_sequence'] <= e['candidate']['input_sequence'] < group['release_sequence']]
        processed = [e for e in events if e['event'] == 'input_processed' and e['sequence'] == group['first_sequence']]
        cycles = [e for e in events if e['event'] == 'cycle']
        transitions = []
        for e in cycles:
            model = e['state'].get('model') or {}
            sampling = model.get('sampling') or {}
            transitions.append(dict(ms=relative(e['at_ns']), transition=model.get('transition'),
                                    sampling=sampling.get('last'), take=model.get('last_take')))
        row = dict(first_sequence=group['first_sequence'], commands=group['servo_commands'],
                   input_processing_ms=relative(processed[0]['processing_ns']) if processed else None,
                   first_motion_ms=group.get('x5_input_to_position_change_ms'), transitions=transitions)
        client_ms, clock_sample = input_clocks.get(start, (None, None))
        row['browser_to_motion'] = None
        if clock_sample and row['first_motion_ms'] is not None:
            row['browser_to_motion'] = input_latency_bound(client_ms,
                start+round(row['first_motion_ms']*1e6), clock_sample)
            row['clock_sample'] = clock_sample
        if sent:
            first = sent[0]
            row['first_motion_command_sent_ms']=relative(first['receipt']['sent_ns'])
            row['motion_command']=first['receipt']['command'].split('(')[0]
            candidate = first['candidate'].get('moveit')
            if candidate is not None:
                publications = [s['input']['at_ns'] for s in candidate['trajectory_sources'] if s.get('input')]
                row.update(earliest_retained_publication_ms=relative(min(publications)) if publications else None,
                           adopted_candidate_computed_ms=relative(candidate['computed_ns']),
                           adopted_candidate_host_received_ms=relative(candidate['host_received_ns']),
                           first_servoj_sent_ms=relative(first['receipt']['sent_ns']),
                           candidate_to_send_ms=(first['receipt']['sent_ns'] - candidate['computed_ns']) / 1e6)
            if row['first_motion_ms'] is not None:
                row['sent_to_first_motion_ms'] = row['first_motion_ms'] - row['first_motion_command_sent_ms']
        rows.append(row)
    metrics = {}
    for key in ('input_processing_ms', 'earliest_retained_publication_ms', 'adopted_candidate_computed_ms',
                'candidate_to_send_ms', 'first_servoj_sent_ms', 'first_motion_command_sent_ms', 'sent_to_first_motion_ms', 'first_motion_ms'):
        values = [r[key] for r in rows if r.get(key) is not None]
        metrics[key] = dict(n=len(values), p50=float(np.percentile(values, 50)),
                            p95=float(np.percentile(values, 95)), max=max(values)) if values else None
    bounds = [r['browser_to_motion'] for r in rows if r['browser_to_motion']]
    end_to_end = dict(covered=len(bounds), total=len(rows), missing=len(rows)-len(bounds),
        method='Browser performance.now input sample to first raw-feedback joint change >0.01 degree; '
               'per-connection four timestamp offset interval; sample <=3s old; 1000ppm drift allowance. '
               'Includes wireless but not physical gamepad sampling delay before browser input mapping.',
        intervals={k:dict(p50=float(np.percentile([b[k] for b in bounds],50)),
                         p95=float(np.percentile([b[k] for b in bounds],95)),
                         p99=float(np.percentile([b[k] for b in bounds],99)),
                         max=max(b[k] for b in bounds))
                   for k in ('lower_ms','upper_ms','clock_width_ms')} if bounds else {})
    return dict(clock='Stage metrics: X5 same-kernel only. browser_to_motion: bounded cross-host measurement.',
                browser_to_motion=end_to_end,
                transition_resolution='sampled cycle upper bounds; not exact service timing',
                motion_threshold='same original joint change >0.01 degree; unchanged',
                metrics=metrics, groups=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.directory)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result['metrics'], indent=2))
