"""Qualify an arm-only release using explicit, immutable regression reuse.

The original 600-second run keeps its original SHA. A new restricted matrix
must exercise the current code; it is never relabelled as a 600-second run.
This entry point cannot commission, initialize CAN or run a gripper suite.
"""
import argparse
import asyncio
import hashlib
import json
from pathlib import Path

from control_acceptance import Harness, matrix_rows, source_hashes
from live_signal_acceptance import LiveSignals


def read(path):
    return json.loads(Path(path).read_bytes())


def proof(parent, item):
    path = (parent / item['file']).resolve()
    if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
        raise ValueError('Arm scope evidence changed: ' + str(path))
    return path


def require_arm_scope_shadow(path, fixtures, review_path):
    review_path = Path(review_path).resolve()
    review = read(review_path)
    current = source_hashes()
    if (review.get('passed') is not True
            or review.get('scope') != 'remove_CAN_and_gripper_only'
            or review.get('source_sha256') != current
            or review.get('arm_math_IK_FK_ServoJ_Stop_leases_unchanged') is not True
            or review.get('gripper_command_profile') != 'disabled'):
        raise ValueError('Current source does not match the restricted scope review')
    parent = review_path.parent
    original = read(proof(parent, review['original_stress']))
    framing = read(proof(parent, review['header_review']))
    proof(parent, review['prior_motion_review'])
    proof(parent, review['scoped_tests'])
    if (original.get('passed') is not True or original.get('duration_s', 0) < 600
            or original.get('stage') != 'shadow' or original.get('suite') != 'matrix'
            or original.get('invalid') != 0 or original.get('actions') != 0
            or original.get('source_changed_during_run') is not False
            or original.get('hardware_sources_verified') is not True
            or original.get('test_fixture') is not False
            or any(original.get('stats', {}).get(k) != 0 for k in ('motion_tx', 'can_tx', 'stop_tx'))
            or framing.get('passed') is not True
            or framing.get('original_source_sha256') != original['source_sha256']
            or framing.get('current_source_sha256') != review['previous_source_sha256']):
        raise ValueError('Original long shadow or framing review is incomplete')
    for item in framing['proofs']:
        proof(parent, item)
    result = read(path)
    coverage = lambda rows: {tuple(row[k] for k in ('source', 'mode', 'frame', 'fine', 'axis', 'sign')) for row in rows}
    status = result.get('final_status') or {}
    if (fixtures.get('control_scope') != 'arm-only'
            or fixtures.get('gripper_command_profile') != 'disabled'
            or any(row['input']['gripper_rate'] != 0 for row in matrix_rows(fixtures))
            or result.get('passed') is not True or result.get('duration_s', 0) < 40
            or result.get('stage') != 'shadow' or result.get('suite') != 'matrix'
            or result.get('source_sha256') != current
            or result.get('source_changed_during_run') is not False
            or result.get('production_input_sha256') != fixtures['production_input_sha256']
            or result.get('hardware_sources_verified') is not True
            or result.get('test_fixture') is not False
            or result.get('diagnostic_transport', False)
            or result.get('invalid') != 0 or result.get('actions') != 0
            or coverage(result.get('cases', [])) != coverage(matrix_rows(fixtures))
            or any(result.get('stats', {}).get(k) != 0 for k in ('motion_tx', 'can_tx', 'stop_tx'))
            or status.get('gripper_command_profile', {}).get('enabled') is not False
            or status.get('gripper_command_profile', {}).get('name') != 'disabled'
            or any(status.get('limits', {}).get(k) != v for k, v in fixtures['profile'].items())):
        raise ValueError('Current restricted shadow is incomplete or has enabled CAN')
    before = original['final_status']
    arm_limits = {k: v for k, v in before['limits'].items() if k != 'gripper_s'}
    if (arm_limits != {k: v for k, v in status['limits'].items() if k != 'gripper_s'}
            or before['kinematics']['profile_sha256'] != status['kinematics']['profile_sha256']
            or any(original.get(k) != result.get(k) for k in ('e6_network', 'pc_tcp_configuration', 'media_hz'))):
        raise ValueError('Arm limits, calibration or link differ from the original long run')
    return result


class ArmOnlySignals(LiveSignals):
    def __init__(self, fixtures, evidence, shadow, review):
        if evidence.exists():
            raise ValueError('Use a new immutable evidence directory')
        self.shadow = require_arm_scope_shadow(shadow, fixtures, review)
        self.checks = []
        self.episode = None
        self.recording = False
        self.last_gripper_action = None
        Harness.__init__(self, fixtures, evidence)

    async def execute(self, suite):
        # Test the deployed denial before any new preparation or movement.
        # A rejected initialization must not create a thread or touch CAN.
        before = dict(self.latest.get('control', {}).get('stats') or {})
        try:
            await self.rpc(dict(operation='initialize-gripper'))
        except RuntimeError as exc:
            if '禁止初始化' not in str(exc):
                raise
        else:
            raise ValueError('Disabled gripper initialization was accepted')
        await self.phase(.2, self.neutral())
        after = self.latest.get('control', {}).get('stats') or {}
        if before.get('can_tx') != 0 or after.get('can_tx') != 0 or self.status.get('gripper_initialized'):
            raise ValueError('Arm-only session sent CAN or initialized the gripper')
        self.checks.append(dict(passed=True, gripper_disabled=True,
                                initialization_rejected=True, can_tx_before=0, can_tx_after=0))
        await super().execute(suite)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--suite', choices=('zero', 'j6'), required=True)
    parser.add_argument('--fixtures', type=Path, required=True)
    parser.add_argument('--shadow-evidence', type=Path, required=True)
    parser.add_argument('--scope-review', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    harness = ArmOnlySignals(read(args.fixtures), args.evidence, args.shadow_evidence, args.scope_review)
    asyncio.run(harness.run_live(args.suite))


if __name__ == '__main__':
    main()
