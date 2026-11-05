"""Correlate perf CLOCK_MONOTONIC major-fault events with feedback wait windows."""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import re


def parse_perf(text):
    rows = []
    current = None
    pattern = re.compile(r'^\s*(\S+)\s+(\d+)/(\d+)\s+(\d+)\.(\d+):\s+major-faults:\s*(.*)$')
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            comm, pid, tid, seconds, fraction, location = match.groups()
            current = dict(comm=comm, pid=int(pid), tid=int(tid),
                at_ns=int(seconds)*1_000_000_000+int(fraction.ljust(9, '0')),
                reported_location=location, callchain=[])
            rows.append(current)
        elif current is not None and line.startswith('\t'):
            current['callchain'].append(line.strip())
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    parser.add_argument('audit', type=Path)
    args = parser.parse_args()
    rows = parse_perf((args.capture/'perf-script.txt').read_text())
    identity = json.loads((args.capture/'identity.json').read_text())
    gateway = identity['pids']['gateway']
    windows = []
    events = []
    with gzip.open(args.audit/'events.jsonl.gz', 'rt', encoding='utf8') as source:
        for line in source:
            event = json.loads(line)
            if event['event'] == 'feedback_transport':
                events.append(event['data'])
    for event in events:
        if event['event'] != 'receive_timing':
            continue
        begin = event['recv_started_ns']
        end = event['publish_finished_ns']
        windows.append(dict(receive=event, faults=[row for row in rows if begin <= row['at_ns'] <= end],
                            nearby_faults=[row for row in rows if begin-50_000_000 <= row['at_ns'] <= end+50_000_000]))
    sample_path = args.capture/'samples.jsonl.gz'
    opener = gzip.open
    if not sample_path.exists():
        sample_path, opener = args.capture/'samples.jsonl', open
    with opener(sample_path, 'rt', encoding='utf8') as source:
        samples = [json.loads(line) for line in source]
    tasks = {}
    for sample in samples:
        for task in sample['tasks']:
            tid = task['tid']
            if tid not in tasks:
                tasks[tid] = dict(first=task['stat']['majflt'], last=task['stat']['majflt'])
            tasks[tid]['last'] = task['stat']['majflt']
    result = dict(events=len(rows), gateway_events=sum(row['pid']==gateway for row in rows),
        event_counts_by_pid_tid=dict(Counter(str((row['pid'], row['tid'])) for row in rows)),
        observed_thread_fault_counters=tasks, windows=windows, all_events=rows,
        clock='perf --clockid mono and feedback perf_counter_ns use the same X5 kernel monotonic clock.',
        limitations=['Major-fault event samples locate thread and callchain; they do not directly measure fault duration or GIL ownership.',
                     'One-second task counters may miss short-lived inherited child processes or threads.'])
    (args.capture/'fault-correlation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('windows','all_events','observed_thread_fault_counters')}, indent=2))
    for window in windows:
        event = window['receive']
        print('receive_ms', (event['publish_finished_ns']-event['recv_started_ns'])/1e6,
              'faults', len(window['faults']), 'nearby', len(window['nearby_faults']))


if __name__ == '__main__':
    main()
