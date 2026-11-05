"""Build v59's restricted operator profile from checked original proofs.

Offline only. Previous executions retain their source hashes, and no failed
gripper run is counted as a passed test. Run once after the v59 short checks.
"""
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'teleop/src'), str(ROOT / 'teleop/device_gateway')]
from arm_scope_acceptance import require_arm_scope_shadow
from control_acceptance import source_hashes
from e6_teleop.acceptance_profile import verify_profile

E = ROOT / 'docs/evidence/teleop_local_integration_20261005'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    sources = source_hashes()
    review = read(E / 'arm_scope_review_v59.json')
    shadow = require_arm_scope_shadow(E / 'shadow_arm_v59_40s/summary.json',
                read(ROOT / 'output/control_mapping_arm_v59.json'), E / 'arm_scope_review_v59.json')
    prior = read(E / 'dependency_reuse_v57.json')
    originals = {p['run']: p for p in prior['reused_proofs']}
    old_sources = review['previous_source_sha256']
    framing_sources = read(E / 'shadow_protocol_reuse_v58.json')['original_source_sha256']
    selected = read(ROOT / 'output/delivery_v58_selected.json')
    selected.pop('gripper_15_35_15')
    selected['shadow_600s'].append('shadow_arm_v59_40s')
    selected['zero_servoj'].append('live_arm_zero_v59')
    selected['j6_return'].append('live_arm_j6_v59')
    selected['faults'][-1] = 'live_ssh_cut_v58_retry1'
    selected['gamepad_30s'] = ['live_record_gamepad_v58_retry1']
    selected['raw_media_readback'] = ['live_record_gamepad_v58_retry1', 'live_record_keyboard_v58']
    selected['gripper_disabled'] = ['live_arm_zero_v59', 'live_arm_j6_v59']
    destination = E / 'delivery_arm_v59'
    profile_path = ROOT / 'teleop/config/operator_acceptance.json'
    if destination.exists() or profile_path.exists():
        raise ValueError('New immutable release paths required')
    proofs = {}
    for name in {v for rows in selected.values() for v in rows}:
        folder = E / name
        result = read(folder / 'summary.json')
        assert result['passed'] and not result.get('source_changed_during_run')
        assert read(folder / 'frame_readback.json')['passed']
        manifest = result['source_sha256']
        if manifest == sources:
            provenance = 'current_arm_only_source_execution'
        elif manifest in (old_sources, framing_sources):
            provenance = 'reviewed_original_v57_or_v58_arm_execution'
        else:
            proof = originals[name]
            assert manifest == proof['original_source_sha256']
            assert sha(folder / 'summary.json') == proof['summary_sha256']
            assert sha(folder / 'frame_readback.json') == proof['audit_sha256']
            provenance = 'previous_scoped_motion_dependency_reuse'
        assert result['final_status']['kinematics']['profile_sha256'] == shadow['final_status']['kinematics']['profile_sha256']
        # The disabled capability has a different unused gripper rate. No arm
        # bound may increase through this restricted release.
        for key, value in shadow['final_status']['limits'].items():
            if key != 'gripper_s':assert result['final_status']['limits'][key] == value
        proofs[name] = dict(original_source_sha256=manifest, provenance=provenance)
    for name in selected['raw_media_readback']:
        assert read(E / name / 'episode_readback.json')['passed']
    for name in selected['bounded_recovery']:
        assert read(E / name / 'recovery_readback.json')['passed']
    axes = set()
    for category in ('cartesian_axes_return', 'frames_multi_fine'):
        for name in selected[category]:
            for check in read(E / name / 'summary.json').get('checks', []):
                outward = check.get('outward', {})
                if 'axis' in outward and not check.get('sustained_rotation'):
                    axes.add(tuple(outward[k] for k in ('frame', 'fine', 'mode', 'axis')))
            if name.startswith('sustained_'):assert read(E / name / 'sustained_rates.json')['passed']
    assert axes >= {(frame, fine, mode, axis) for frame in ('base', 'tool') for fine in (False, True) for mode in ('translation', 'rotation') for axis in range(3)}
    for name in selected['gripper_disabled']:
        result = read(E / name / 'summary.json')
        assert result['stats']['can_tx'] == 0
        assert any(c.get('gripper_disabled') and c.get('initialization_rejected') for c in result['checks'])
    resources = E / 'after_live_arm_v59_resources.json'
    assert read(resources)['passed']
    # All checks are complete before creating a release directory.
    destination.mkdir()
    checks = {}
    for category, names in selected.items():
        files = []
        for name in names:
            filenames = ['summary.json', 'frame_readback.json']
            if category == 'raw_media_readback':filenames.append('episode_readback.json')
            if category == 'bounded_recovery':filenames.append('recovery_readback.json')
            files += [dict(file='../' + name + '/' + f, sha256=sha(E / name / f)) for f in filenames]
        if category == 'gripper_disabled':files.append(dict(file='../' + resources.name, sha256=sha(resources)))
        file = destination / (category + '.json')
        write(file, dict(passed=True, control_scope='arm-only', proofs=files,
            source_provenance={name: proofs[name] for name in names},
            scope_review=dict(file='../arm_scope_review_v59.json', sha256=sha(E / 'arm_scope_review_v59.json')),
            original_600s_retained=category == 'shadow_600s', current_600s_execution=False,
            synthetic_inputs_only=True))
        checks[category] = dict(passed=True, file='../../docs/evidence/teleop_local_integration_20261005/delivery_arm_v59/' + file.name, sha256=sha(file))
    offline = []
    for name in ('offline_v57_complete_windows.xml', 'offline_v57_complete_wsl.xml',
                 'header_v58_scoped.xml', 'shadow_protocol_reuse_v58.xml',
                 'delivery_harness_v58.xml', 'ssh_injection_v58_retry1.xml',
                 'arm_only_v59_scoped_retry2.xml', 'arm_scope_gate_v59.xml'):
        file = ROOT / 'docs/evidence/teleop_resume_20261006' / name
        suites = ET.parse(file).findall('.//testsuite')
        assert suites and all(int(s.get('failures', 0)) == 0 and int(s.get('errors', 0)) == 0 for s in suites)
        offline.append(dict(file='../../teleop_resume_20261006/' + name, sha256=sha(file)))
    for name in ('offline_mapping_v56_full.json', 'simulation_official_readback_v50_20261006.json'):
        file = E / name;assert read(file).get('passed', read(file).get('validated', False))
        offline.append(dict(file='../' + name, sha256=sha(file)))
    file = destination / 'offline.json';write(file, dict(passed=True, proofs=offline,
        retained_gripper_failures=['../live_gripper_cycle_v58_retry1/summary.json', '../live_gripper_fine_v58_retry1/summary.json'],
        full_stage_three_complete=False))
    checks['offline'] = dict(passed=True, file='../../docs/evidence/teleop_local_integration_20261005/delivery_arm_v59/offline.json', sha256=sha(file))
    limits = {k: shadow['final_status']['limits'][k] for k in ('translation_m_s','rotation_rad_s','gripper_s','radius_m','orientation_rad','joint_window_rad','joint_velocity_rad_s','joint_accel_rad_s2')}
    bundle = E / 'enabled_calibration_chord_v16/profile.json'
    profile = dict(schema_version=1, complete=True, control_scope='arm-only', full_stage_three_complete=False,
        robot_ip='192.168.1.4', camera_serial='AY4A1520111', tool=0, user=0,
        source_sha256=dict(sources, **{p: sha(ROOT / 'teleop' / p) for p in ('web/src/HardwareApp.tsx', 'web/src/style.css')}),
        can_driver_sha256=sha(ROOT / 'omnipicker/tools/verify_can_control.py'), checks=checks, limits=limits,
        kinematics_kind='controller-local', kinematics_profile_file='../../docs/evidence/teleop_local_integration_20261005/enabled_calibration_chord_v16/profile.json',
        kinematics_profile_sha256=sha(bundle), feedback_framing='length-envelope', recovery_policy='bounded',
        gripper_command_profile='disabled', wifi_driver_ps_on=shadow['wifi_latency']['driver_ps_on'],
        wifi_low_latency=shadow['wifi_latency']['requested'], wifi_reorder_ms=10, ssh_rto_min_ms=20, e6_rto_min_ms=None,
        pc_transport=shadow['pc_tcp_configuration'], e6_network=shadow['e6_network'], media_hz=shadow['media_hz'],
        physical_gamepad_acceptance_required=True, physical_gamepad_live_acceptance='pending',
        real_lerobot_action_export=False, fixed_servoj=dict(t=.05, aheadtime=50, gain=500))
    write(profile_path, profile)
    verify_profile(profile_path)
    print('Restricted arm-only operator profile verified; gripper remains disabled.')


if __name__ == '__main__':main()
