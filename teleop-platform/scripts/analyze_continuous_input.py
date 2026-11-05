"""Read-only phase analysis of formal-page inputs, including every zero phase.

Uses X5 monotonic timestamps only. Candidate motion is never real motion.
Transient-excluded slopes are explicitly separate from whole-phase slopes.
"""
import argparse
import base64
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    base = args.directory
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'upstream/dobot-sdk'))
    from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC
    from dobot_sdk.protocol.feedback import FeedbackParser
    sdk = FeedbackParser()
    audit = base / 'session/x5/audit'
    raw = []
    controller_targets = []
    seqs = []
    statuses = Counter()
    for event in records(audit / 'raw.jsonl.gz'):
        data = base64.b64decode(event['raw_b64'])
        row = np.frombuffer(data, dtype=MyType, count=1)[0]
        assert len(data) == 1440 and row['len'] == 1440 and row['TestValue'] == TEST_VALUE_MAGIC
        parsed = sdk.parse(data)
        assert parsed is not None and np.array_equal(parsed.joint_state.q_actual, row['QActual'])
        values = [*row['QActual'], *row['ToolVectorActual']]
        assert np.all(np.isfinite(values))
        raw.append([event['received_ns'], *values])
        controller_targets.append(row['QTarget'].tolist())
        seqs.append(event['sequence'])
        statuses[str(tuple(int(row[k]) for k in ('RobotMode','EnableStatus','ErrorStatus','CollisionState')))] += 1
    raw = np.array(raw)
    controller_targets = np.array(controller_targets)
    counts = Counter()
    groups = []
    active = None
    candidates = {}
    motions = []
    confirmed = []
    stops = []
    faults = []
    reasons = Counter()
    prepared = None
    for event in records(audit / 'events.jsonl.gz'):
        kind = event['event']
        counts[kind] += 1
        if kind == 'prepared':
            prepared = event['base']
        if kind == 'input' and event['data'].get('op') == 'intent':
            intent = event['data']['intent']
            stamp = event['at_ns']
            if intent['deadman']:
                if active is None:
                    active = dict(start_ns=stamp, phases=[], base=prepared)
                if not active['phases'] or active['phases'][-1]['intent'] != intent:
                    if active['phases']:
                        active['phases'][-1]['end_ns'] = stamp
                    active['phases'].append(dict(start_ns=stamp, first_seq=event['data']['seq'], intent=intent))
            elif active is not None:
                active['end_ns'] = stamp
                active['phases'][-1]['end_ns'] = stamp
                groups.append(active)
                active = None
        if kind == 'cycle':
            state = event['state']
            candidate = (state.get('candidate') or {}).get('moveit')
            if candidate:
                candidates.setdefault((candidate['epoch'],candidate['id']), candidate)
            if state.get('reason'):
                reasons[state['reason']] += 1
        if kind == 'motion_submitted':
            motions.append(event)
            assert event['pre_action_observation']['received_ns'] < event['receipt']['sent_ns']
        if kind == 'motion_confirmed':
            confirmed.append(event)
            assert event['target']['receipt']['accepted'] and event['target']['receipt']['acknowledged']
        if kind == 'stop_result':
            assert event['receipt']['accepted'] and event['receipt']['acknowledged']
        if kind == 'stop_requested':
            stops.append(event)
        if kind in ('motion_uncertain','gripper_fault','enable_uncertain'):
            faults.append(event)
    assert active is None, 'Unreleased input: evidence cannot be called complete'
    assert len(motions)==len(confirmed), 'Unconfirmed actions must not pass this analysis'
    cs = sorted(candidates.values(), key=lambda c:c['computed_ns'])

    def slope(times, values):
        if len(times) < 3:
            return None
        t = (times - times[0]) / 1e9
        t -= np.mean(t)
        return (t @ values / (t @ t)).tolist()

    for group_index, group in enumerate(groups):
        begin, end = group['start_ns'], group['end_ns']
        selected = [c for c in cs if begin <= c['computed_ns'] < end]
        group.update(duration_ms=(end-begin)/1e6, candidate_count=len(selected),
                     epochs=sorted(set(c['epoch'] for c in selected)),
                     intermediate_stops=[s['at_ns'] for s in stops if begin <= s['at_ns'] < end],
                     servo_commands=sum(begin <= m['at_ns'] < end for m in motions))
        first_stop = min(group['intermediate_stops'], default=end)
        prior_stop = (raw[:,0]>=begin)&(raw[:,0]<first_stop)
        if np.any(prior_stop):
            tracking = np.abs(raw[prior_stop,1:7]-controller_targets[prior_stop])
            group['pre_stop_actual_minus_controller_target_deg_max'] = np.max(tracking,axis=0).tolist()
            largest = int(np.argmax(np.max(tracking,axis=1)))
            index = np.flatnonzero(prior_stop)[largest]
            group['pre_stop_largest_following_sample'] = dict(
                sequence=seqs[index],received_ns=int(raw[index,0]),
                actual_deg=raw[index,1:7].tolist(),controller_target_deg=controller_targets[index].tolist())
        cmds = [m for m in motions if begin<=m['at_ns']<first_stop]
        if len(cmds)>1:
            gaps = np.diff([m['receipt']['sent_ns'] for m in cmds])/1e6
            group['pre_stop_command_gap_ms_p50_p95_max'] = np.percentile(gaps,[50,95,100]).tolist()
        if len(selected) > 1:
            group['candidate_gap_ms_max'] = max(np.diff([c['computed_ns'] for c in selected])) / 1e6
        if group['base'] and selected:
            b = group['base']; br = Rotation.from_quat(b['quaternion_xyzw'])
            group['candidate_envelope'] = dict(
                position_mm=max(np.linalg.norm(np.array(c['position_m'])-b['position_m'])*1000 for c in selected),
                rotation_deg=max((Rotation.from_quat(c['quaternion_xyzw'])*br.inv()).magnitude()*180/np.pi for c in selected),
                joint_deg=max(np.max(np.abs(np.array(c['joints_rad'])-b['joints_rad']))*180/np.pi for c in selected))
        group['zero_phases'] = 0
        for phase in group['phases']:
            i = phase['intent']; start, finish = phase['start_ns'], phase['end_ns']
            phase['duration_ms'] = (finish-start)/1e6
            kind = 'joint' if any(i['joint_rad_s']) else 'translation' if any(i['linear_m_s']) else 'rotation'
            vector = np.array(i['joint_rad_s'] if kind=='joint' else i['linear_m_s'] if kind=='translation' else i['angular_rad_s'])
            phase['kind'] = kind if np.any(vector) else 'zero'
            if not np.any(vector):
                group['zero_phases'] += 1
                continue
            expected = vector * (1000 if kind=='translation' else 180/np.pi)
            axis = expected / np.linalg.norm(expected)
            phase['requested_speed'] = float(np.linalg.norm(expected))
            phase['speed_unit'] = 'mm/s' if kind=='translation' else 'deg/s'
            before = raw[max(0,np.searchsorted(raw[:,0], start)-1)]
            rotation = Rotation.from_euler('xyz', before[10:13], degrees=True)
            observed = raw[(raw[:,0]>=start)&(raw[:,0]<finish)]
            ct = [c for c in selected if start<=c['computed_ns']<finish]
            for name, times, values in (
                ('actual', observed[:,0], observed[:,1:7] if kind=='joint' else observed[:,7:10] if kind=='translation' else observed[:,10:13]),
                ('candidate', np.array([c['computed_ns'] for c in ct]), np.array([
                    np.rad2deg(c['joints_rad']) if kind=='joint' else np.array(c['position_m'])*1000 if kind=='translation'
                    else Rotation.from_quat(c['quaternion_xyzw']).as_euler('xyz',degrees=True) for c in ct]))):
                if not len(times):
                    phase[name] = {'samples':0}
                    continue
                if kind=='rotation':
                    values=(Rotation.from_euler('xyz',values,degrees=True)*rotation.inv()).as_rotvec(degrees=True)
                if i['frame']=='tool' and kind!='joint':
                    values=rotation.inv().apply(values)
                middle = (times>=start+350_000_000)&(times<finish-100_000_000)
                whole=slope(times,values); center=slope(times[middle],values[middle])
                phase[name]=dict(samples=len(times),whole_phase_velocity=whole,
                    middle_samples=int(sum(middle)),middle_velocity=center,
                    middle_projected_speed=float(np.array(center)@axis) if center else None)
        following = [s for s in stops if end<=s['at_ns']<end+1_500_000_000]
        group['normal_release_observed'] = bool(following) and not group['intermediate_stops']
        group['release_to_stop_send_ms'] = (following[0]['receipt']['send_completed_ns']-end)/1e6 if following else None
        a = int(np.searchsorted(raw[:,0],begin)); b = int(np.searchsorted(raw[:,0],end))
        stop_end = min(end+1_500_000_000, groups[group_index+1]['start_ns'] if group_index+1<len(groups) else raw[-1,0])
        tail = raw[(raw[:,0]>=end)&(raw[:,0]<stop_end)]
        baseline = np.median(raw[max(0,a-12):a+1,1:],axis=0)
        whole = raw[(raw[:,0]>=begin)&(raw[:,0]<stop_end)]
        if len(whole):
            group['actual_joint_excursion_deg'] = np.max(np.abs(whole[:,1:7]-baseline[:6]),axis=0).tolist()
            group['actual_position_excursion_mm'] = float(np.max(np.linalg.norm(whole[:,7:10]-baseline[6:9],axis=1)))
            group['actual_end_position_delta_mm'] = (np.median(whole[-12:,7:10],axis=0)-baseline[6:9]).tolist()
        if len(tail):
            group['post_release_observation_ms'] = (tail[-1,0]-end)/1e6
            group['release_to_stationary_window_ms'] = None
            for j in range(len(tail)):
                k = int(np.searchsorted(tail[:,0],tail[j,0]+250_000_000))
                if k>=len(tail):
                    break
                if np.max(np.ptp(tail[j:k+1,1:7],axis=0))<=.01:
                    group['release_to_stationary_window_ms'] = (tail[j,0]-end)/1e6
                    break
            group['stop_travel_mm'] = float(np.max(np.linalg.norm(tail[:,7:10]-raw[b,7:10],axis=1)))
    result = dict(raw_frames=len(raw),raw_invalid=0,raw_sequence_gaps=int(sum(np.maximum(np.diff(seqs)-1,0))),
        status_counts=statuses,events=counts,faults=faults,cycle_reasons=reasons,groups=groups,
        notes='X5 monotonic only. Every zero phase retained. Middle slopes explicitly exclude first350ms and final100ms; not sustained qualification. Shadow actual slopes are static noise. Provenance checked separately.',
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_hashes={n:hashlib.sha256((audit/n).read_bytes()).hexdigest() for n in ('events.jsonl.gz','raw.jsonl.gz')})
    (base/'continuous.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='groups'},ensure_ascii=False))
    print(json.dumps([{k:v for k,v in g.items() if k not in ('phases','base')} for g in groups],ensure_ascii=False))


if __name__ == '__main__':
    main()
