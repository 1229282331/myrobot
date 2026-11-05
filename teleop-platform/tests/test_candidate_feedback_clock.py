from types import SimpleNamespace
import pytest
from e6_phosphobot.moveit_bridge import MoveItBridge


def bridge():
    now=[100_000_000]
    b=MoveItBridge('.',None,clock=lambda:now[0],api='cpp')
    b.submit(SimpleNamespace(model_dump=lambda:{}),1)
    b.latest=dict(id=1,epoch=b.epoch,received_ns=now[0],valid_until_ns=150_000_000,
        cpp={'gripper_check':{'fallback':False,'arm_range_raw':[51,53]}})
    return b,now


def observed(stamp):
    return dict(valid=True,actual=52/255,received_ns=stamp,sent_raw=52,inflight_raw=None)


def test_feedback_received_during_take_is_checked_after_its_snapshot():
    b,now=bridge()
    def snapshot():
        now[0]+=1_000_000
        return observed(now[0])
    b.gripper_snapshot=snapshot
    assert b.take()['id']==1
    assert b.last_take['at_ns']==now[0]
    assert b.take() is None


@pytest.mark.parametrize('delay,reason',[(80_000_000,'sample_deadline'),(101_000_000,'candidate_age')])
def test_slow_snapshot_does_not_extend_original_candidate_deadline(delay,reason):
    b,now=bridge()
    def snapshot():
        now[0]+=delay
        return observed(now[0])
    b.gripper_snapshot=snapshot
    assert b.take() is None
    assert b.last_take['reason']==reason
    assert b.latest['valid_until_ns']==150_000_000
    assert b.take() is None


def test_actual_future_feedback_is_still_rejected():
    b,now=bridge();b.gripper_snapshot=lambda:observed(now[0]+1)
    assert b.take() is None
    assert b.last_take['reason']=='gripper_feedback_expired'
