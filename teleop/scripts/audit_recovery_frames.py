"""Independently read back the live zero-input bounded-recovery evidence."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np

from audit_live_frames import audit as audit_live


def audit(directory):
    directory = Path(directory)
    summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
    assert summary['passed'] and summary['suite'] == 'bounded-recovery-zero'
    assert summary['injected_fault']['reason'] == '输入看门狗超时'
    assert summary['enable_commands'] == summary['can_commands'] == 0
    packet_audit = audit_live(directory)
    path = directory / 'frames.jsonl'
    opener = open
    if not path.exists():
        path = path.with_suffix('.jsonl.gz')
        opener = gzip.open
    seen = set()
    actions = []
    prepared = []
    healthy_since = None
    first_stop = None
    ready_ticks = 0
    begin_ns = None
    commands = {}
    with opener(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            frame = json.loads(line)
            begin_ns = begin_ns or frame['observation']['gateway_monotonic_ns']
            control = frame['control']
            assert control['stats']['can_tx'] == 0
            for event in control.get('command_events', []):
                if event['x5_sent_ns'] >= begin_ns:
                    commands[event['x5_sent_ns']] = event
            tick = control.get('tick') or {}
            identity = tick.get('control_tick_id')
            if identity is None or identity in seen:
                continue
            seen.add(identity)
            recovery = tick['status']['recovery']
            if recovery.get('healthy_since_ns') is not None:
                healthy_since = recovery['healthy_since_ns']
            for event in tick.get('events', []):
                if event['kind'] == 'stop' and event['reason'] == '输入看门狗超时':
                    first_stop = event['x5_monotonic_ns']
                if event['kind'] == 'automatic_prepared':
                    assert first_stop is not None and healthy_since is not None
                    assert event['x5_monotonic_ns'] - healthy_since >= 5_000_000_000
                    assert not event['old_action_replayed'] and event['attempt'] == 1
                    prepared.append(event['x5_monotonic_ns'])
            input = tick.get('input')
            action = tick.get('effective_action')
            if action:
                assert input['deadman'] and input['neutral']
                assert not any(input['twist']) and input['gripper_rate'] == 0
                assert action['gripper'] is None
                assert action['input_seq'] == input['seq']
                assert first_stop is None, 'No old target may be sent after the injected stop'
                actions.append(action)
            if recovery['state'] == 'ready':
                ready_ticks += 1
                assert input['neutral'] and not input['deadman'] and not action
                assert tick['status']['stop_acknowledged'] and tick['status']['stopped_feedback']
    assert len(prepared) == 1 and ready_ticks > 0 and actions
    reference = actions[0]['requested_tcp']['controller_pose_mm_deg']
    for action in actions:
        np.testing.assert_allclose(action['requested_tcp']['controller_pose_mm_deg'], reference, atol=1e-9, rtol=0)
    # Read-only native IK/FK queries are part of preparation, not motion.
    assert all(e['command'] == 'Stop()' or e['command'].startswith(('ServoJ(', 'InverseKin(', 'PositiveKin(')) for e in commands.values())
    assert all(e['x5_sent_ns'] < first_stop for e in commands.values() if e['command'].startswith('ServoJ('))
    result = dict(passed=True,raw_sdk_frames_checked=packet_audit['raw_sdk_frames_checked'],
        confirmed_zero_action_ticks=len(actions),automatic_preparations=len(prepared),
        ready_neutral_ticks=ready_ticks,old_targets_sent_after_stop=0,can_tx=0,enable_tx=0,
        stop_send_ms=packet_audit['stop_send_ms'],stop_confirmation_ms=packet_audit['stop_confirmation_ms'],
        tested_source='gamepad synthetic production mapping',scope='zero increments and one lease recovery; no nonzero resumed motion')
    (directory / 'recovery_readback.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('directory',type=Path)
    print(json.dumps(audit(parser.parse_args().directory),indent=2))
