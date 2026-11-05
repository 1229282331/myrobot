import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('joint_reversals',Path(__file__).parents[1]/'scripts/analyze_joint_reversals.py')
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)


def candidate(ident,q,signs):
    return dict(id=ident,epoch=1,sample_ros_ns=ident*10_000_000,joints_rad=[0]*5+[q],
        trajectory_sources=[dict(kind='j',input=dict(intent=dict(deadman=True,
            joint_rad_s=[0]*5+[sign],linear_m_s=[0]*3,angular_rad_s=[0]*3))) for sign in signs])


def test_returning_positive_target_has_negative_velocity():
    result=analysis.compare(candidate(1,.04,[-1,-1]),candidate(2,.03,[-1,-1]))
    assert result['classification']=='steady_direction' and result['passed']
    assert result['delta_rad'][-1]<0


def test_wrong_increment_is_not_hidden_by_positive_absolute_position():
    result=analysis.compare(candidate(1,.04,[1,1]),candidate(2,.03,[1,1]))
    assert result['classification']=='steady_direction' and not result['passed']


def test_mixed_window_is_retained_without_claiming_direction_pass():
    result=analysis.compare(candidate(1,.04,[1,1]),candidate(2,.03,[1,-1]))
    assert result['classification']=='transition_window' and 'passed' not in result
    assert result['delta_rad'][-1]<0
