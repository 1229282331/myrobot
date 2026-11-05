import base64
from copy import deepcopy
import hashlib
import json
import math
import socket
import zipfile

import numpy as np
import pytest
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC

from e6_phosphobot.episode_trajectory import export_episode, review
from e6_phosphobot.episode_transfer import associate, digest_file


def source():
    raw = []
    for seq in (1, 2):
        packet = np.zeros(1, dtype=MyType)
        packet['len'] = 1440
        packet['TestValue'] = TEST_VALUE_MAGIC
        packet['RobotMode'] = 5
        packet['EnableStatus'] = 1
        packet['QActual'][0] = [1, 2, 3, 4, 5, 6]
        raw.append(dict(generation='fixture', sequence=seq, received_ns=1_000_000_000+seq*8_000_000,
                        raw_b64=base64.b64encode(packet.tobytes()).decode()))
    receipt = dict(sent_ns=1_010_000_000, command='ServoJ(2,3,4,5,6,7,t=.1,aheadtime=50,gain=500)')
    events = [dict(event='cycle', at_ns=1_009_000_000, state=dict(
        feedback={k: raw[0][k] for k in ('generation', 'sequence', 'received_ns')},
        actual=dict(joints_rad=[math.radians(v) for v in [1, 2, 3, 4, 5, 6]],
                    gripper=.4, pose_frame='grasp_tcp', position_m=[.1, .2, .3]),
        gripper=dict(valid=True, actual=.4, sent_raw=90, contact='closing'),
        candidate=dict(joints_rad=[99]*6))),
        dict(event='motion_submitted', at_ns=1_010_000_000, receipt=receipt),
        dict(event='motion_confirmed', at_ns=1_013_000_000, target=dict(receipt=dict(
            receipt, accepted=True, acknowledged=True, reply_ns=1_012_000_000)))]
    return events, raw


def test_actual_and_original_sent_targets_stay_separate():
    events, raw = source()
    result = review(events, raw)
    assert result['observations'][0]['joints_rad'][0] == math.radians(1)
    assert result['sent_targets'][0]['joints_rad'][0] == math.radians(2)
    assert result['cycles'][0]['arm_observation_verified']
    assert result['cycles'][0]['gripper']['sent_raw'] == 90
    assert result['commands'] == events[1:]
    assert result['report']['accepted_arm_commands'] == 1
    assert result['report']['issues'] == {}


@pytest.mark.parametrize('opening', [None, -1, 2, float('nan')])
def test_missing_or_invalid_gripper_is_never_invented(opening):
    events, raw = source()
    events[0]['state']['actual']['gripper'] = opening
    result = review(events, raw)
    assert not result['cycles'][0]['gripper_observation_valid']
    assert result['report']['issues']['cycles_without_valid_gripper_feedback'] == 1


def test_gripper_only_recording_does_not_invent_arm_commands():
    events, raw = source()
    events = events[:1] + [dict(event='can', data=dict(kind='tx', data_hex='0059001919190000'))]
    result = review(events, raw)
    assert result['sent_targets'] == []
    assert result['report']['submitted_arm_commands'] == 0
    assert result['commands'] == events[1:]


@pytest.mark.parametrize('change,issue', [
    ('gap', 'feedback_gaps_over_100ms'), ('sequence', 'feedback_sequence_discontinuities'),
    ('generation', 'feedback_generation_changes'), ('missing_ack', 'unconfirmed_or_rejected_sends'),
    ('negative_ack', 'unconfirmed_or_rejected_sends'), ('early_ack', 'unconfirmed_or_rejected_sends'),
    ('actual_mismatch', 'unverified_cycle_observations'), ('future_feedback', 'unverified_cycle_observations'),
    ('unknown_command', 'unsupported_motion_commands'), ('fault', 'faulted_feedback_frames'),
])
def test_gaps_faults_and_unconfirmed_data_remain_visible(change, issue):
    events, raw = source()
    if change == 'gap':raw[1]['received_ns'] += 200_000_000
    elif change == 'sequence':raw[1]['sequence'] = 4
    elif change == 'generation':raw[1]['generation'] = 'reconnected'
    elif change == 'missing_ack':events.pop()
    elif change == 'negative_ack':events[-1]['target']['receipt']['accepted'] = False
    elif change == 'early_ack':events[-1]['target']['receipt']['reply_ns'] = 1
    elif change == 'actual_mismatch':events[0]['state']['actual']['joints_rad'][0] = 99
    elif change == 'future_feedback':events[0]['at_ns'] = 1_007_000_000
    elif change == 'unknown_command':events[1]['receipt']['command'] = 'ServoP(1,2,3,4,5,6)'
    elif change == 'fault':
        packet = np.frombuffer(base64.b64decode(raw[0]['raw_b64']), dtype=MyType).copy()
        packet['ErrorStatus'] = 1
        raw[0]['raw_b64'] = base64.b64encode(packet.tobytes()).decode()
    result = review(events, raw)
    assert result['report']['issues'][issue] >= 1
    if change in ('gap', 'sequence', 'generation'):
        assert not result['observations'][1]['continuous_from_previous']


@pytest.mark.parametrize('change', ['duplicate_raw', 'duplicate_send', 'duplicate_ack',
                                   'truncated', 'magic', 'clock', 'nonfinite', 'empty', 'cycle_clock'])
def test_malformed_sources_do_not_become_review_trajectories(change):
    events, raw = source()
    if change == 'duplicate_raw':raw.append(deepcopy(raw[0]))
    elif change == 'duplicate_send':events.append(deepcopy(events[1]))
    elif change == 'duplicate_ack':events.append(deepcopy(events[2]))
    elif change == 'truncated':raw[0]['raw_b64'] = base64.b64encode(b'bad').decode()
    elif change == 'magic':
        wire = bytearray(base64.b64decode(raw[0]['raw_b64']))
        wire[48:56] = bytes(8)
        raw[0]['raw_b64'] = base64.b64encode(wire).decode()
    elif change == 'clock':raw[1]['received_ns'] = raw[0]['received_ns']-1
    elif change == 'nonfinite':events[0]['state']['actual']['joints_rad'][0] = float('nan')
    elif change == 'empty':raw.clear()
    elif change == 'cycle_clock':events.append(deepcopy(events[0]))
    with pytest.raises(ValueError):review(events, raw)


def sealed_episode(path, outcome='success'):
    events, raw = source()
    files = {
        'metadata.json':json.dumps(dict(episode='fixture', task='杯子 / 中文',
            initial_state=dict(model=dict(tcp=dict(translation_m=[0, 0, .1504]))))),
        'summary.json':json.dumps(dict(result=outcome, complete=outcome != 'aborted')),
        'audit/events.jsonl':''.join(json.dumps(row)+'\n' for row in events),
        'audit/raw.jsonl':''.join(json.dumps(row)+'\n' for row in raw),
        'media/manifest.jsonl':'', 'media/camera_samples.jsonl':''}
    path.mkdir()
    for name, data in files.items():
        target = path/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(data, encoding='utf-8')
    # Zip the exact retained bytes, including the platform's line endings.
    with zipfile.ZipFile(path/'bundle.zip', 'w') as archive:
        manifest = []
        for name in files:
            data = (path/name).read_bytes()
            archive.writestr(name, data)
            manifest.append(dict(file=name, bytes=len(data), sha256=hashlib.sha256(data).hexdigest()))
        archive.writestr('bundle_manifest.json', json.dumps(manifest))
    (path/'verification.json').write_text(json.dumps(dict(bundle_sha256=digest_file(path/'bundle.zip'))))
    associate(path, [])


@pytest.mark.parametrize('outcome', ['success', 'failure', 'aborted'])
def test_export_is_offline_preserves_sources_and_never_authorizes_motion(tmp_path, monkeypatch, outcome):
    source_dir, output = tmp_path/'original', tmp_path/'review'
    sealed_episode(source_dir, outcome)
    before = {p.relative_to(source_dir):p.read_bytes() for p in source_dir.rglob('*') if p.is_file()}
    def no_network(*args, **kwargs):raise AssertionError('Offline export attempted network access')
    monkeypatch.setattr(socket.socket, 'connect', no_network)
    result = export_episode(source_dir, output)
    assert result['physical_replay_allowed'] is False
    assert result['result'] == outcome
    assert result['original_metadata']['initial_state']['model']['tcp']['translation_m'][2] == .1504
    assert result['task'] == '杯子 / 中文'
    assert {p.relative_to(source_dir):p.read_bytes() for p in source_dir.rglob('*') if p.is_file()} == before
    assert all(digest_file(output/name) == digest for name, digest in result['export_files'].items())
    with pytest.raises(FileExistsError):export_episode(source_dir, output)


@pytest.mark.parametrize('damage', ['bundle', 'retained_bytes', 'missing_raw', 'missing_association'])
def test_failed_integrity_check_never_repairs_source_or_creates_output(tmp_path, damage):
    source_dir, output = tmp_path/'original', tmp_path/'review'
    sealed_episode(source_dir)
    if damage == 'bundle':(source_dir/'bundle.zip').write_bytes(b'bad')
    elif damage == 'retained_bytes':(source_dir/'audit/raw.jsonl').write_text('{}\n')
    elif damage == 'missing_raw':(source_dir/'audit/raw.jsonl').unlink()
    else:(source_dir/'associations.jsonl').unlink()
    before = {p.relative_to(source_dir):p.read_bytes() for p in source_dir.rglob('*') if p.is_file()}
    with pytest.raises(ValueError):export_episode(source_dir, output)
    assert not output.exists()
    assert {p.relative_to(source_dir):p.read_bytes() for p in source_dir.rglob('*') if p.is_file()} == before


def test_source_and_output_must_not_overlap(tmp_path):
    source_dir = tmp_path/'original'
    sealed_episode(source_dir)
    for output in (source_dir, source_dir/'nested', tmp_path):
        with pytest.raises(ValueError, match='outside'):export_episode(source_dir, output)
