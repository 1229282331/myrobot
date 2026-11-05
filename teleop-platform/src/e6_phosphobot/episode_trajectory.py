"""Read-only trajectory review from sealed native episodes; no execution path.

Observed positions and sent commands stay separate. This is an offline review
format, not an executable trajectory or a new set of action labels.
"""
import argparse
import base64
from collections import Counter
import json
import math
from pathlib import Path

from .dobot_transport import FeedbackStream
from .episode_transfer import digest_file, rows, verify_bundle
from .native_dataset import wire_joints


def vector(value, size):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError('Wrong observation vector size')
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Non-finite observation')
    return result


def timestamp(value):
    if type(value) is not int or value <= 0:
        raise ValueError('Missing or invalid original monotonic timestamp')
    return value


def review(events, raw):
    observations, cycles, commands = [], [], []
    issues = Counter()
    streams, identities, previous = {}, {}, None
    for row in raw:
        generation, sequence = row['generation'], row['sequence']
        if not isinstance(generation, str) or not generation or type(sequence) is not int or sequence < 1:
            raise ValueError('Invalid original feedback identity')
        identity = (generation, sequence)
        if identity in identities:
            raise ValueError('Duplicate original feedback identity')
        when = timestamp(row['received_ns'])
        wire = base64.b64decode(row['raw_b64'], validate=True)
        if len(wire) != 1440:
            raise ValueError('Expected original 1440-byte SDK feedback')
        stream = streams.setdefault(generation, FeedbackStream())
        frames = stream.feed(wire, when)
        if len(frames) != 1:
            raise ValueError('Invalid original SDK feedback framing')
        status = frames[0].status
        joints = [math.radians(v) for v in vector(status.joint_state.q_actual, 6)]
        continuous = previous is not None
        if previous:
            if when < previous['received_ns']:
                raise ValueError('Original feedback receive clock regressed')
            if generation != previous['generation']:
                issues['feedback_generation_changes'] += 1
                continuous = False
            elif sequence != previous['sequence'] + 1:
                issues['feedback_sequence_discontinuities'] += 1
                continuous = False
            if when - previous['received_ns'] > 100_000_000:
                issues['feedback_gaps_over_100ms'] += 1
                continuous = False
        fault = bool(status.error_status or status.collision_state)
        if fault:
            issues['faulted_feedback_frames'] += 1
        observation = dict(generation=generation, sequence=sequence, received_ns=when,
                           joints_rad=joints, robot_mode=status.robot_mode,
                           enabled=status.enable_status, error=status.error_status,
                           collision=status.collision_state, continuous_from_previous=continuous)
        observations.append(observation)
        identities[identity] = observation
        previous = observation
    for stream in streams.values():
        stream.eof()
    if not observations:
        raise ValueError('No original feedback to review')

    submitted, confirmed = {}, {}
    command_names = {'motion_submitted', 'motion_confirmed', 'motion_uncertain',
                     'stop_requested', 'stop_result', 'can', 'workspace_blocked', 'input'}
    last_cycle = None
    for event in events:
        kind = event.get('event')
        if kind in command_names:
            commands.append(event)  # Original event, not a reconstructed action.
        if kind == 'motion_submitted':
            receipt = event['receipt']
            key = (timestamp(receipt['sent_ns']), receipt['command'])
            if key in submitted:
                raise ValueError('Duplicate submitted command identity')
            submitted[key] = receipt
        elif kind == 'motion_confirmed':
            receipt = event['target']['receipt']
            key = (timestamp(receipt['sent_ns']), receipt['command'])
            if key in confirmed:
                raise ValueError('Duplicate command confirmation')
            confirmed[key] = receipt
        elif kind == 'motion_uncertain':
            issues['uncertain_motion_events'] += 1
        elif kind == 'cycle':
            when = timestamp(event['at_ns'])
            if last_cycle is not None and when <= last_cycle:
                raise ValueError('Cycle timestamps must be strictly increasing')
            last_cycle = when
            state = event['state']
            feedback, actual = state.get('feedback') or {}, state.get('actual')
            original = identities.get((feedback.get('generation'), feedback.get('sequence')))
            valid = bool(original and feedback.get('received_ns') == original['received_ns']
                         and 0 <= when - original['received_ns'] <= 100_000_000)
            if valid and actual:
                q = vector(actual.get('joints_rad'), 6)
                valid = all(abs(a - b) <= 1e-8 for a, b in zip(q, original['joints_rad']))
            else:
                valid = False
            if not valid:
                issues['unverified_cycle_observations'] += 1
            gripper = state.get('gripper') or {}
            opening = (actual or {}).get('gripper')
            gripper_valid = (gripper.get('valid') is True and type(opening) in (int, float)
                             and math.isfinite(opening) and 0 <= opening <= 1)
            if not gripper_valid:
                issues['cycles_without_valid_gripper_feedback'] += 1
            cycles.append(dict(at_ns=when, actual=actual, feedback_identity={
                k: feedback.get(k) for k in ('generation', 'sequence', 'received_ns')},
                arm_observation_verified=valid, gripper=gripper,
                gripper_observation_valid=gripper_valid,
                reason=state.get('reason'), model_context={
                    k: (state.get('model') or {}).get(k)
                    for k in ('urdf_sha256', 'cpp_binary_sha256', 'tcp')}))

    accepted = 0
    sent_targets = []
    for key, receipt in submitted.items():
        ack = confirmed.get(key)
        valid = bool(ack and ack.get('accepted') is True and ack.get('acknowledged') is True
                     and type(ack.get('reply_ns')) is int and ack['reply_ns'] >= key[0])
        if valid:
            accepted += 1
        else:
            issues['unconfirmed_or_rejected_sends'] += 1
        try:
            target = wire_joints(receipt['command'])
        except ValueError:
            target = None
            issues['unsupported_motion_commands'] += 1
        sent_targets.append(dict(sent_ns=key[0], command=key[1], joints_rad=target,
                                 acknowledged_accepted=valid, receipt=ack))
    issues['orphan_confirmations'] += len(confirmed.keys() - submitted.keys())
    sent_targets.sort(key=lambda row: row['sent_ns'])
    return dict(observations=observations, cycles=cycles, commands=commands, sent_targets=sent_targets,
                report=dict(raw_frames=len(observations), cycle_samples=len(cycles),
                            submitted_arm_commands=len(submitted), accepted_arm_commands=accepted,
                            issues={k: v for k, v in issues.items() if v},
                            origin_ns=observations[0]['received_ns'],
                            duration_s=(observations[-1]['received_ns']-observations[0]['received_ns'])/1e9))


def export_episode(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination or source in destination.parents or destination in source.parents:
        raise ValueError('Export must be outside the retained source tree')
    if destination.exists():
        raise FileExistsError('Export destination already exists')
    verification = json.loads((source/'verification.json').read_text(encoding='utf-8'))
    if not (source/'associations.jsonl').is_file():
        raise ValueError('Missing retained associations')
    # Reuse native immutable verification; never repair/extract into the source.
    checked = verify_bundle(source/'bundle.zip', source, verification['bundle_sha256'], read_only=True)
    metadata = json.loads((source/'metadata.json').read_text(encoding='utf-8'))
    summary = json.loads((source/'summary.json').read_text(encoding='utf-8'))
    result = review(rows(source/'audit/events.jsonl'), rows(source/'audit/raw.jsonl'))
    source_files = ('metadata.json', 'summary.json', 'audit/events.jsonl', 'audit/raw.jsonl',
                    'associations.jsonl', 'media/manifest.jsonl', 'media/camera_samples.jsonl')
    source_hashes = {name: digest_file(source/name) for name in source_files}
    manifest = dict(schema='e6-offline-trajectory-review-v1', episode=metadata.get('episode'),
                    task=metadata.get('task'), result=summary.get('result'),
                    source_bundle_sha256=verification['bundle_sha256'], source_files=source_hashes,
                    original_metadata=metadata, original_summary=summary,
                    native_verification=checked, review=result['report'],
                    clock='Original X5 monotonic nanoseconds; no retiming or interpolation',
                    units=dict(joints='rad', recorded_tcp='m', gripper='native raw/255, not finger distance'),
                    physical_replay_allowed=False,
                    limitations=['Offline review only; no device connection or executable command entry point',
                                 'Successful recording/acknowledgement does not prove grasp success or arrival',
                                 'Gripper validity is the recorded gateway flag, not independent CAN requalification',
                                 'No current-model path, start-pose, velocity or environment qualification',
                                 'Original model/TCP are retained; no substitution with current configuration'])
    destination.mkdir(parents=True)
    # Manifest is written last; a partial export never appears complete.
    for key, name in [('observations', 'raw-feedback.jsonl'), ('cycles', 'cycle-observations.jsonl'),
                      ('commands', 'command-events.jsonl'), ('sent_targets', 'sent-arm-targets.jsonl')]:
        with (destination/name).open('x', encoding='utf-8') as output:
            for row in result[key]:
                output.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    manifest['export_files'] = {p.name: digest_file(p) for p in destination.iterdir() if p.is_file()}
    (destination/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, allow_nan=False, indent=2), encoding='utf-8')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Retained verified native episode directory')
    parser.add_argument('destination', type=Path, help='New separate offline export directory')
    args = parser.parse_args()
    manifest = export_episode(args.source, args.destination)
    print(json.dumps(dict(destination=str(args.destination.resolve()), review=manifest['review'],
                          physical_replay_allowed=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
