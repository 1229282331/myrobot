import json
from pathlib import Path

from e6_phosphobot.control_state import for_operator
from e6_phosphobot.pc_bridge import StateCache


def test_real_large_state_preserves_operator_safety_and_execution_fields():
    root=Path(__file__).resolve().parents[2]
    state=json.loads((root/'docs/teleop-rebuild/evidence/live-jazzy-v54/after-2d5-rx-minus/state.json').read_text())
    original=json.dumps(state)
    small=for_operator(state)
    for key in ('actual','arm','reason','ready','echo_ns','camera','receipt','capabilities',
                'stop_acknowledged','stationary','stop_uncertain','recording'):
        assert small[key]==state[key]
    for field in ('candidate','accepted'):
        for key in ('position_m','quaternion_xyzw','joints_rad','input_sequence'):
            assert small[field][key]==state[field][key]
        assert 'moveit' not in small[field]
    assert small['feedback']['status']['error_status']==state['feedback']['status']['error_status']
    assert len(json.dumps(small))<len(original)*.25
    assert json.dumps(state)==original  # Raw audit/diagnostic state is untouched.


def test_compact_control_frames_cannot_replace_full_diagnostic_cache():
    cache=StateCache();full={'kind':'state','actual':None,'feedback':{'raw':'retained'}}
    assert cache.update(full)
    stamp=cache.received_ns
    assert not cache.update(for_operator(full))
    assert cache.current() is full and cache.received_ns==stamp
    compact=for_operator({'kind':'state','reason':'已有控制者','ready':False})
    assert compact['reason']=='已有控制者' and compact['ready'] is False
    assert compact['candidate'] is None and compact['feedback'] is None


def test_camera_restart_state_reaches_operator_without_changing_readiness():
    state=dict(kind='state',ready=False,camera_restart=dict(available=True,phase='complete',can_restart=True))
    assert for_operator(state)['camera_restart']==state['camera_restart']
    assert not for_operator(state)['ready']
