"""Read-only first-candidate timing from archived formal-page shadow inputs."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    base = args.directory
    analysis = json.loads((base / 'analysis.json').read_text(encoding='utf-8'))
    groups = analysis['groups']
    rows = [dict(first_sequence=g['first_sequence'], input_start_ns=g['start_ns'],
                 release_ns=g['release_ns'], epoch=None, computed_ms=None,
                 host_received_ms=None, observed_cycle_ms=None) for g in groups]
    with gzip.open(base / 'session/x5/audit/events.jsonl.gz', 'rt', encoding='utf-8') as source:
        for line in source:
            event = json.loads(line)
            if event['event'] != 'cycle':
                continue
            state = event['state']
            if state['stage'] != 'shadow':
                raise ValueError('Only shadow evidence is accepted')
            candidate = (state.get('candidate') or {}).get('moveit')
            first = (state.get('model') or {}).get('first_candidate')
            if not candidate or not first or candidate['epoch'] != first['epoch']:
                continue
            for group, row in zip(groups, rows):
                if (row['epoch'] is None and group['first_sequence'] <= candidate['input_sequence'] < group['release_sequence']
                        and group['start_ns'] <= first['computed_ns'] <= event['at_ns'] < group['release_ns']):
                    row.update(epoch=first['epoch'], computed_ms=(first['computed_ns'] - group['start_ns']) / 1e6,
                        host_received_ms=(first['host_received_ns'] - group['start_ns']) / 1e6,
                        observed_cycle_ms=(event['at_ns'] - group['start_ns']) / 1e6)
    metrics = {}
    for name in ('computed_ms', 'host_received_ms', 'observed_cycle_ms'):
        values = [r[name] for r in rows if r[name] is not None]
        metrics[name] = dict(n=len(values), p50=float(np.percentile(values, 50)),
                            p95=float(np.percentile(values, 95)), max=max(values)) if values else None
    result = dict(groups=rows, metrics=metrics,
        scope='formal keyboard page to first MoveIt candidate, measured from X5 input reception; '
              'cycle observation is an upper bound; no device motion or browser-to-motion claim',
        motion_commands=analysis['motion_submitted'], missing=sum(r['epoch'] is None for r in rows))
    (base / 'shadow-startup.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'groups'}, indent=2))


if __name__ == '__main__':
    main()
