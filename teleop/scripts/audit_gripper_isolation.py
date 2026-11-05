"""Offline readback of isolated diagnostic CAN exchanges and retained E6 packets."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'teleop/device_gateway')]
from omnipicker.tools import verify_can_control as can
from e6_observer.protocol import decode


def audit(directory):
    path = Path(directory) / 'result.json'
    result = json.loads(path.read_bytes())
    pending = None
    sent = received = 0
    rejection = []
    latencies = []
    for event in result['events']:
        if event['kind'] == 'tx':
            if pending is not None:
                raise ValueError('Unresolved CAN request followed by another send')
            pending = event
            sent += 1
            assert event['send_confirmed']
        elif event['kind'] == 'rx':
            assert pending is not None
            try:
                reply = can.decode_feedback(bytes.fromhex(event['frame_hex']), event['flags'],
                                            pending['can_id'], bytes.fromhex(pending['data_hex']))
            except can.CheckFailed as exc:
                rejection.append(str(exc))
                pending = None
                continue
            if reply is None:
                continue
            elapsed = (event['host_perf_counter_ns'] - pending['host_perf_counter_ns']) / 1e6
            assert 0 <= elapsed <= result['reply_timeout_ms']
            latencies.append(elapsed)
            received += 1
            pending = None
    frames = result.get('arm_feedback', {}).get('frames', [])
    max_error = [0.] * 6
    max_baseline = [0.] * 6
    transient=result.get('arm_guard_policy','strict')=='transient'
    since={};warning_frames=0;longest_ms=0.;guard_stops=0;last_ns=None
    if transient:
        assert result['arm_guard_limits']==dict(soft_deg=.05,hard_deg=.25,persistence_ms=100)
    for frame in frames:
        decoded = decode(base64.b64decode(frame['raw_base64'], validate=True), True)
        for name, value in decoded.items():
            saved = frame['decoded'][name]
            if isinstance(value, list):
                assert len(value) == len(saved)
                assert all(math.isclose(a, b, rel_tol=0, abs_tol=1e-12) for a, b in zip(value, saved))
            else:
                assert value == saved
        for i, (actual, target, baseline) in enumerate(zip(decoded['q_actual_deg'],
                decoded['q_target_deg'], result['arm_baseline_deg'])):
            max_error[i] = max(max_error[i], abs(actual - target))
            max_baseline[i] = max(max_baseline[i], abs(actual - baseline))
        if transient:
            now=frame['x5_receive_monotonic_ns'];assert last_ns is None or now>last_ns;last_ns=now
            warning=False;stop=False
            for kind,reference in [('target',decoded['q_target_deg']),('baseline',result['arm_baseline_deg'])]:
                for i,(actual,target) in enumerate(zip(decoded['q_actual_deg'],reference)):
                    error=abs(actual-target);key=(kind,i)
                    if error>.05:
                        warning=True;since.setdefault(key,now);duration=(now-since[key])/1e6
                        longest_ms=max(longest_ms,duration);stop=stop or error>.25 or duration>=100
                    else:since.pop(key,None)
            warning_frames+=int(warning);guard_stops+=int(stop)
    stages = []
    tolerance=result.get('settle_tolerance_counts',1)
    assert tolerance in (1,5)
    for stage in result.get('stages', []):
        feedback = stage['feedback']
        target = can.encode_command(stage['target_percent'], result['limits_raw'])[1]
        arrived = len(feedback) >= 3 and all(row['state']==0 and abs(row['position_raw']-target)<=tolerance for row in feedback[-3:])
        stages.append(dict(target_percent=stage['target_percent'], hold_replies=len(feedback),
            ramp_replies=len(stage.get('ramp_feedback', [])), last_feedback=feedback[-1] if feedback else None,
            ended_with_three_at_target_replies=arrived,
            endpoint_error_percent=abs(feedback[-1]['position_raw']-target)*100/255 if feedback else None))
    completed = result['status'] == 'completed'
    if completed:
        assert sent == received and pending is None and not rejection
        assert all(s['ended_with_three_at_target_replies'] for s in stages)
        assert result['arm_feedback']['error'] is None
        assert max(max_error + max_baseline) <= (.25 if transient else .05)
        if transient:assert guard_stops==0 and result['arm_feedback']['all_received_frames_retained']
    after = result.get('after', {})
    return dict(status=result['status'], error=result.get('error'),
        diagnostic_completed=completed, continuous_teleoperation_passed=False, operator_unlocked=False,
        result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        executed_source_sha256=result['source_sha256'], limits_raw=result['limits_raw'],
        ramp_percent_s=result.get('ramp_percent_s'), stages=stages,
        settle_tolerance_counts=tolerance,settle_tolerance_percent=tolerance*100/255,
        can_sent=sent, can_valid_replies=received, can_rejections=rejection,
        unresolved_last_request=pending is not None,
        can_rtt_max_ms=max(latencies) if latencies else None,
        retained_arm_packets_checked=len(frames),
        arm_packet_scope='all received packets from preflight through diagnostic end' if result.get('arm_feedback',{}).get('all_received_frames_retained') else 'bounded tail; not all guarded packets from the complete diagnostic',
        arm_guard_policy=result.get('arm_guard_policy','strict'),arm_guard_limits=result.get('arm_guard_limits'),
        arm_warning_frames=warning_frames if transient else None,arm_longest_soft_deviation_ms=longest_ms if transient else None,
        arm_guard_stop_frames=guard_stops if transient else None,
        arm_target_error_max_deg=max_error, arm_baseline_error_max_deg=max_baseline,
        arm_error=result.get('arm_feedback', {}).get('error'),
        e6_writes=result['e6_writes'], can_final_state=after.get('linkinfo', {}).get('info_data', {}).get('state'),
        gripper_current_opening=None, gripper_power_measured=False,
        automatic_retry=False, physical_fault_rarity_estimate=None,
        elapsed_s=(result['events'][-1]['host_perf_counter_ns'] - result['events'][0]['host_perf_counter_ns']) / 1e9
            if result['events'] else 0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.directory)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
