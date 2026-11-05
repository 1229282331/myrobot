"""Read saved formal-page shadow evidence; never connects to devices."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def analyze(path, episode):
    doc = json.loads(path.read_text(encoding='utf8'))
    events = doc['selected']
    inputs = [e for e in events if e['event'] == 'input']
    groups = defaultdict(list)
    for event in events:
        if event['event'] != 'cycle':
            continue
        state = event['state']
        assert state['stage'] == 'shadow' and state['accepted'] is None
        candidate = state['candidate']
        assert candidate['pose_frame'] == state['actual']['pose_frame'] == 'grasp_tcp'
        assert candidate['flange_pose']['pose_frame'] == 'controller_flange'
        assert candidate['tcp'] == state['capabilities']['tcp'] == state['model']['tcp']
        row = candidate['moveit']
        source = row['source_input']
        matches = [e for e in inputs if e['data'].get('seq') == source['sequence']
                   and 0 <= source['received_ns'] - e['at_ns'] < 250_000_000]
        assert len(matches) == 1
        sent = matches[0]['data']
        assert sent['intent'] == source['intent']
        point = np.array(candidate['flange_pose']['position_m']) + Rotation.from_quat(
            candidate['flange_pose']['quaternion_xyzw']).apply([0, 0, .1504])
        np.testing.assert_allclose(point, candidate['position_m'], atol=1e-12)
        groups[(sent['source'], source['intent']['frame'], row['epoch'])].append(candidate)
    results = []
    for (source, frame, epoch), rows in groups.items():
        points = np.array([r['position_m'] for r in rows])
        flanges = np.array([r['flange_pose']['position_m'] for r in rows])
        drift = float(np.linalg.norm(points-points[0], axis=1).max()*1000)
        flange = float(np.linalg.norm(flanges-flanges[0], axis=1).max()*1000)
        assert len(rows) >= 2 and drift < .01 and flange > .01
        results.append(dict(source=source, frame=frame, epoch=epoch, candidates=len(rows),
                            tcp_span_from_first_candidate_mm=drift,
                            flange_span_from_first_candidate_mm=flange))
    cycles = [json.loads(s)['state'] for s in (episode/'audit/events.jsonl').read_text(
        encoding='utf8').splitlines() if json.loads(s)['event'] == 'cycle']
    assert all(c['actual']['pose_frame'] == 'grasp_tcp' and c['accepted'] is None for c in cycles)
    verification = json.loads((episode/'verification.json').read_text(encoding='utf8'))
    assert verification['actions'] == 0 and verification['cycle_samples'] == len(cycles)
    assert doc['audit']['error'] is None
    assert set(doc['counts']) <= {'feedback_transport','cycle','input','input_processed',
                                 'clock_sample','prepared','recording_request_result','shutdown_state'}
    return dict(passed=True, scope='formal page shadow; not actual motion or physical TCP calibration',
                groups=results, raw_frames_in_session=doc['audit']['counts']['raw'],
                accepted_hardware_actions=0, episode=episode.name, verification=verification,
                episode_actual_joint_peak_to_peak_deg=np.ptp(
                    np.rad2deg([c['actual']['joints_rad'] for c in cycles]), axis=0).tolist())


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--evidence', type=Path, required=True)
    p.add_argument('--episode', type=Path, required=True)
    args = p.parse_args()
    result = analyze(args.evidence/'shadow-audit-selected.json', args.episode)
    (args.evidence/'shadow-analysis.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps(result, indent=2))
