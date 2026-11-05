"""Inspect recorded device-local receive/apply timing; no hardware access."""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path

import numpy as np


def quantiles(values):
    return dict(zip(('p50', 'p95', 'p99', 'max'),
                    np.percentile(values, [50, 95, 99, 100]).tolist())) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    inputs = {}
    processed = []
    faults = Counter()
    with gzip.open(args.directory / 'session/x5/audit/events.jsonl.gz', 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['event'] == 'input':
                inputs[row['at_ns']] = row
            elif row['event'] == 'input_processed':
                processed.append(row)
            elif row['event'] == 'cycle':
                reason = row['state'].get('reason')
                if reason:
                    faults[reason] += 1
    wait, work, echo, active_wait = [], [], [], []
    missing = []
    rejected = []
    zero_echo = 0
    for row in processed:
        incoming = inputs.get(row['received_ns'])
        if incoming is None:
            missing.append(row['received_ns'])
            continue
        delay = (row['processing_ns'] - row['received_ns']) / 1e6
        wait.append(delay)
        work.append((row['at_ns'] - row['processing_ns']) / 1e6)
        data = incoming['data']
        echo_age = (row['processing_ns'] - data['echo_ns']) / 1e6 if data['echo_ns'] else None
        if echo_age is None:
            zero_echo += 1
        else:
            echo.append(echo_age)
        if (data.get('intent') or {}).get('deadman'):
            active_wait.append(delay)
        if row.get('reason') or delay > 50:
            rejected.append(dict(row, operation=data['op'],
                                 echo_age_at_processing_ms=echo_age))
    result = dict(input_count=len(inputs), processed_count=len(processed),
                  missing_receive_events=missing, receive_to_apply_ms=quantiles(wait),
                  active_receive_to_apply_ms=quantiles(active_wait),
                  apply_duration_ms=quantiles(work), echo_age_at_apply_ms=quantiles(echo),
                  zero_echo_messages=zero_echo,
                  cycle_reasons=dict(faults), notable_processing=rejected,
                  clock_scope='All differences use X5 perf_counter_ns. Echo age is round-trip clock-token age, not one-way input delay.')
    (args.directory / 'input-transport.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in result.items() if key != 'notable_processing'}, indent=2))


if __name__ == '__main__':
    main()
