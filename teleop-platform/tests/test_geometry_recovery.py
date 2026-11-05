"""A computational direction block must not mask the subsequent lifecycle."""
import pytest
from test_gateway import setup
from test_moveit_bridge import Bridge, row


@pytest.mark.parametrize('ending', ['stop', 'disconnect', 'feedback_fault'])
def test_geometry_timeout_keeps_prepare_but_later_stop_reason_wins(ending):
    e, d, send, step, now = setup()
    e.moveit = Bridge()
    send(deadman=True, linear_m_s=[.002, 0, 0])
    e.moveit.row = row(now[0]); step()
    e.moveit.info.update(status=5, gripper_check={'arm_safe': False, 'arm_reason': 'callback_deadline'})
    send(deadman=True, linear_m_s=[.002, 0, 0]); step()
    assert e.session.ready and '几何检查超时' in e.state()['reason']
    assert [x[0] for x in d.calls] == ['ServoJ', 'Stop']
    if ending == 'stop':
        send('stop'); expected = 'operator_stop'
    elif ending == 'disconnect':
        e.release('page'); expected = '控制连接关闭'
    else:
        e.suspend('feedback disconnected'); expected = 'feedback disconnected'
    assert not e.state()['ready'] and e.state()['reason'] == expected
    assert [x[0] for x in d.calls] == ['ServoJ', 'Stop']


def test_new_prepare_failure_replaces_old_geometry_warning():
    e, d, send, step, now = setup()
    e.reason = '几何检查超时：当前方向已停止'
    send('stop')
    e.stationary = False
    send('prepare')
    assert not e.state()['ready'] and e.state()['reason'] == '等待5秒稳定反馈'
