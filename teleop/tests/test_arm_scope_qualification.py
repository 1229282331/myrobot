"""Restricted regression reuse cannot hide new limits or enable CAN."""
import copy
import hashlib
import json
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import arm_scope_acceptance as gate


@pytest.fixture
def evidence(tmp_path, monkeypatch):
    current = {'arm.py': 'current'}
    monkeypatch.setattr(gate, 'source_hashes', lambda: current)
    def put(name, value):
        path = tmp_path / name
        path.write_text(json.dumps(value))
        return dict(file=name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    row = dict(source='gamepad', mode='translation', frame='base', fine=False,
               axis=0, sign=1, input=dict(gripper_rate=0))
    fixtures = dict(rows=[row], multi=[], control_scope='arm-only',
                    gripper_command_profile='disabled', production_input_sha256='mapper',
                    profile=dict(translation_m_s=.002, gripper_s=.05))
    old_status = dict(limits=dict(translation_m_s=.002, gripper_s=.1),
                      kinematics=dict(profile_sha256='model'))
    original = dict(passed=True, duration_s=600, stage='shadow', suite='matrix',
                    invalid=0, actions=0, source_changed_during_run=False,
                    hardware_sources_verified=True, test_fixture=False,
                    stats=dict(motion_tx=0, can_tx=0, stop_tx=0),
                    source_sha256={'arm.py': 'original'}, final_status=old_status)
    review = dict(passed=True, scope='remove_CAN_and_gripper_only',
                  source_sha256=current, previous_source_sha256={'arm.py': 'framing'},
                  arm_math_IK_FK_ServoJ_Stop_leases_unchanged=True,
                  gripper_command_profile='disabled',
                  original_stress=put('original.json', original),
                  header_review=put('framing.json', dict(passed=True, proofs=[],
                    original_source_sha256=original['source_sha256'],
                    current_source_sha256={'arm.py': 'framing'})),
                  prior_motion_review=put('motion.json', dict(passed=True)),
                  scoped_tests=put('tests.json', dict(passed=True)))
    put('review.json', review)
    new = dict(original, duration_s=40, source_sha256=current,
               production_input_sha256='mapper', cases=[row],
               final_status=dict(limits=dict(translation_m_s=.002, gripper_s=.05),
                 kinematics=dict(profile_sha256='model'),
                 gripper_command_profile=dict(name='disabled', enabled=False)))
    return tmp_path, fixtures, new


def test_current_short_shadow_keeps_original_long_proof(evidence):
    root, fixtures, new = evidence
    path = root / 'new.json';path.write_text(json.dumps(new))
    assert gate.require_arm_scope_shadow(path, fixtures, root / 'review.json')['duration_s'] == 40
    assert gate.read(root / 'original.json')['duration_s'] == 600


@pytest.mark.parametrize('mutation', ['can', 'source', 'coverage', 'short', 'limits', 'active-profile'])
def test_changed_or_incomplete_release_stays_locked(evidence, mutation):
    root, fixtures, original = evidence
    new = copy.deepcopy(original)
    if mutation == 'can':new['stats']['can_tx'] = 1
    if mutation == 'source':new['source_sha256']['arm.py'] = 'changed'
    if mutation == 'coverage':new['cases'] = []
    if mutation == 'short':new['duration_s'] = 39
    if mutation == 'limits':new['final_status']['limits']['translation_m_s'] = .005
    if mutation == 'active-profile':new['final_status']['gripper_command_profile']['enabled'] = True
    path = root / 'new.json';path.write_text(json.dumps(new))
    with pytest.raises(ValueError):gate.require_arm_scope_shadow(path, fixtures, root / 'review.json')
