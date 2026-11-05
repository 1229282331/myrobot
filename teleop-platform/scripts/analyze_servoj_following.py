"""Offline comparison of sent ServoJ targets, internal targets and actual joints.

Uses the pinned official SDK and X5 receive/send monotonic times. Device
velocities/currents are retained verbatim; no diagnosis of contact or saturation
is inferred from them. Does not connect to a device or alter source evidence.
"""
import argparse
import base64
from collections import Counter
import csv
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentiles(values):
    return np.percentile(values, [50, 95, 99, 100]).tolist() if len(values) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_evidence', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root/'upstream/dobot-sdk'))
    from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC
    from dobot_sdk.protocol.feedback import FeedbackParser
    sdk = FeedbackParser()
    audit = args.session_evidence/'session/x5/audit'
    report_path = args.session_evidence/'continuous.json'
    phases = json.loads(report_path.read_text(encoding='utf-8'))
    hashes = {name: sha(audit/name) for name in ('raw.jsonl.gz', 'events.jsonl.gz')}
    assert hashes == phases['source_hashes'], 'Phase report must refer to the same raw evidence'
    vectors = ('QActual', 'QTarget', 'QDActual', 'QDTarget', 'QDDTarget',
               'IActual', 'ITarget', 'MActual', 'MTarget', 'ToolVectorActual',
               'ToolVectorTarget', 'MotorTemperatures', 'JointModes')
    scalars = ('RobotMode', 'EnableStatus', 'ErrorStatus', 'CollisionState',
               'RunningStatus', 'RunQueuedCmd', 'PauseCmdFlag', 'CurrentCommandId',
               'BrakeStatus', 'DragStatus', 'AutoManualMode', 'J4ApproachState',
               'SafetyState', 'SafeState', 'Load', 'CenterX', 'CenterY', 'CenterZ')
    raw = []
    for event in records(audit/'raw.jsonl.gz'):
        data = base64.b64decode(event['raw_b64'], validate=True)
        assert len(data) == 1440
        frame = np.frombuffer(data, dtype=MyType, count=1)[0]
        assert frame['len'] == 1440 and frame['TestValue'] == TEST_VALUE_MAGIC
        parsed = sdk.parse(data)
        assert parsed is not None and np.array_equal(parsed.joint_state.q_actual, frame['QActual'])
        assert all(np.isfinite(frame[key]).all() for key in vectors + scalars)
        raw.append(dict(sequence=event['sequence'], received_ns=event['received_ns'],
                        generation=event['generation'],
                        **{key: frame[key].tolist() for key in vectors + scalars}))
    times = np.array([r['received_ns'] for r in raw], dtype=np.int64)
    # All frames completed by one socket recv share its receive timestamp.
    # Keep their sequence order and choose the last completed frame at a send.
    assert np.all(np.diff(times) >= 0)
    assert all(b['generation'] == a['generation'] and b['sequence'] == a['sequence']+1
               for a,b in zip(raw,raw[1:])), 'This diagnosis requires one complete feedback generation'
    events = list(records(audit/'events.jsonl.gz'))
    commands = []
    confirmed = {e['target']['receipt']['sent_ns']: e['target']['receipt']
                 for e in events if e['event'] == 'motion_confirmed'}
    for event in events:
        if event['event'] != 'motion_submitted':
            continue
        receipt = event['receipt']
        match = re.fullmatch(r'ServoJ\(([^()]*)\)', receipt['command'])
        assert match, 'This analyzer is only for ServoJ evidence'
        fields = match[1].split(',')
        joints = np.array([float(v) for v in fields[:6]])
        options = {k: float(v) for k, v in (item.split('=') for item in fields[6:])}
        candidate = event['candidate']
        assert np.max(abs(joints-np.rad2deg(candidate['joints_rad']))) <= .000001
        sent = receipt['sent_ns']
        ack = confirmed.get(sent)
        index = int(np.searchsorted(times, sent, side='right')-1)
        assert index >= 0 and raw[index]['received_ns'] <= sent
        commands.append(dict(sent_ns=sent, joints_deg=joints.tolist(), sdk_options=options,
                             acknowledged=bool(ack and ack['accepted'] and ack['acknowledged']),
                             receipt=ack or receipt,
                             latest_received_before_send=raw[index]))
    commands.sort(key=lambda c: c['sent_ns'])
    stop_times = [e['receipt']['sent_ns'] for e in events if e['event'] == 'stop_requested']
    result = dict(raw_frames=len(raw), invalid_frames=0,
                  adjacent_equal_receive_times=int(np.sum(np.diff(times)==0)),
                  source_hashes=hashes, phase_report_sha256=sha(report_path),
                  analyzer_sha256=sha(Path(__file__)),
                  clock='X5 monotonic send/receive only; no device/PC clock subtraction',
                  interpretation='Offline evidence only. Diagnostic thresholds are not new control protections. '
                  'Raw currents and mode bits do not establish contact, current saturation or firmware cause.',
                  sdk_dtype_sha256=sha(root/'upstream/dobot-sdk/dobot_sdk/protocol/data_types.py'), groups=[])
    args.output.mkdir(parents=True, exist_ok=False)
    for number, group in enumerate(phases['groups']):
        start = group['start_ns']
        finish = min((t for t in stop_times if start <= t < group['end_ns']), default=group['end_ns'])
        indexes = np.flatnonzero((times >= start) & (times < finish))
        if not len(indexes):
            continue
        actual = np.array([raw[i]['QActual'] for i in indexes])
        target = np.array([raw[i]['QTarget'] for i in indexes])
        error = abs(actual-target)
        selected = [c for c in commands if start <= c['sent_ns'] < finish]
        gaps = np.diff([c['sent_ns'] for c in selected])/1e6
        summary = dict(index=number, start_ns=start, cutoff_ns=finish,
                       cutoff='first Stop sent' if finish != group['end_ns'] else 'input release received',
                       duration_ms=(finish-start)/1e6, pre_cutoff_frames=len(indexes),
                       max_actual_internal_error_deg=error.max(axis=0).tolist(),
                       command_count=len(selected), all_commands_acknowledged=all(c['acknowledged'] for c in selected),
                       command_gap_ms_p50_p95_p99_max=percentiles(gaps),
                       command_gap_exceeding_prior_sdk_t=int(sum(
                           gap > c['sdk_options']['t']*1000 for gap, c in zip(gaps, selected))),
                       onset=[], status_counts={key:dict(Counter(str(raw[i][key]) for i in indexes)) for key in scalars})
        for threshold in (.1, .5, 1., 2., 3.):
            locations = np.flatnonzero(error[:, 3] > threshold)
            if not len(locations):
                summary['onset'].append(dict(j4_error_exceeds_deg=threshold, first=None))
                continue
            index = indexes[locations[0]]
            prior = [c for c in selected if c['sent_ns'] <= times[index]]
            command = prior[-1] if prior else None
            summary['onset'].append(dict(j4_error_exceeds_deg=threshold,
                first=dict(since_input_ms=(int(times[index])-start)/1e6, frame=raw[index],
                    previous_send_ns=command['sent_ns'] if command else None,
                    command_age_ms=(int(times[index])-command['sent_ns'])/1e6 if command else None,
                    sent_joints_deg=command['joints_deg'] if command else None)))
        command_rows = []
        for i, command in enumerate(selected):
            before = command['latest_received_before_send']
            command_rows.append(dict(command, since_input_ms=(command['sent_ns']-start)/1e6,
                feedback_age_ms=(command['sent_ns']-before['received_ns'])/1e6,
                sent_minus_actual_deg=(np.array(command['joints_deg'])-before['QActual']).tolist(),
                step_deg=None if not i else (np.array(command['joints_deg'])-selected[i-1]['joints_deg']).tolist(),
                previous_send_gap_ms=None if not i else (command['sent_ns']-selected[i-1]['sent_ns'])/1e6))
        summary['commands'] = command_rows
        # Retain every raw sample surrounding the first segment, with original
        # sequence/time. Include post-Stop samples but do not mix them into
        # the pre-Stop diagnostic maxima/onsets above.
        window = [r for r in raw if start-250_000_000 <= r['received_ns'] < finish+500_000_000]
        columns = ['sequence', 'received_ns', 'since_input_ms'] + list(scalars)
        columns += [f'{key}_{j+1}' for key in vectors for j in range(6)]
        path = args.output/f'group-{number}-raw.csv'
        with path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, columns); writer.writeheader()
            for r in window:
                row = dict(sequence=r['sequence'], received_ns=r['received_ns'],
                           since_input_ms=(r['received_ns']-start)/1e6, **{k:r[k] for k in scalars})
                row.update({f'{key}_{j+1}':r[key][j] for key in vectors for j in range(6)})
                writer.writerow(row)
        summary['raw_csv'] = dict(file=path.name, sha256=sha(path), rows=len(window))
        result['groups'].append(summary)
    (args.output/'analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps([dict(index=g['index'], duration_ms=g['duration_ms'],
        max_error_deg=g['max_actual_internal_error_deg'], gap_ms=g['command_gap_ms_p50_p95_p99_max'],
        gaps_exceeding_t=g['command_gap_exceeding_prior_sdk_t'],
        j4_onsets=[dict(threshold=o['j4_error_exceeds_deg'],
            ms=o['first']['since_input_ms'] if o['first'] else None,
            command_age_ms=o['first']['command_age_ms'] if o['first'] else None) for o in g['onset']])
        for g in result['groups']], ensure_ascii=False))


if __name__ == '__main__':
    main()
