from concurrent.futures import ThreadPoolExecutor
import socket
import time
import pytest
from dobot_sdk.api.robot_control import RobotControl
from e6_phosphobot.dashboard import Dashboard, Busy, SubmissionPort


def setup(reply_timeout=.2):
    controller,client=socket.socketpair()
    controller.settimeout(1)
    connection=Dashboard('test',reply_timeout=reply_timeout)
    connection._socket=client
    connection._connected=True
    connection._start_receiver()
    return connection,controller


def test_stop_is_sent_before_pending_query_completes_and_replies_are_separate():
    connection,peer=setup()
    robot=RobotControl(connection)
    try:
        with ThreadPoolExecutor(2) as pool:
            pending=pool.submit(robot.GetPose)
            assert peer.recv(1024)==b'GetPose()\n'
            started=time.perf_counter()
            stopping=pool.submit(robot.Stop)
            assert peer.recv(1024)==b'Stop()\n'
            assert time.perf_counter()-started<.05
            assert not pending.done()
            peer.sendall(b'0,{1,2,3,4,5,6},GetPose();0,{},Sto')
            peer.sendall(b'p();')
            assert pending.result(timeout=1)=='0,{1,2,3,4,5,6},GetPose();'
            assert stopping.result(timeout=1)=='0,{},Stop();'
    finally:
        connection.disconnect();peer.close()


def test_timeout_does_not_relabel_late_reply_as_accepted_or_retry():
    connection,peer=setup(reply_timeout=.02)
    robot=RobotControl(connection)
    try:
        with pytest.raises(TimeoutError):robot.GetPose()
        assert peer.recv(1024)==b'GetPose()\n'
        with pytest.raises(RuntimeError,match='Uncertain'):robot.GetPose()
        with ThreadPoolExecutor(1) as pool:
            stopping=pool.submit(robot.Stop)
            assert peer.recv(1024)==b'Stop()\n'
            peer.sendall(b'0,{1,2,3,4,5,6},GetPose();0,{},Stop();')
            assert stopping.result(timeout=1)=='0,{},Stop();'
        assert not connection.events[0]['accepted']
        assert 'late_reply' in connection.events[0]
        assert connection.uncertain
    finally:
        connection.disconnect();peer.close()


def test_second_target_is_not_queued():
    connection,peer=setup()
    robot=RobotControl(connection)
    try:
        with ThreadPoolExecutor(1) as pool:
            pending=pool.submit(robot.GetPose)
            assert peer.recv(1024)==b'GetPose()\n'
            with pytest.raises(Busy):robot.GetAngle()
            peer.sendall(b'0,{1,2,3,4,5,6},GetPose();')
            pending.result(timeout=1)
    finally:
        connection.disconnect();peer.close()


def test_expired_motion_deadline_sends_no_bytes_and_does_not_poison_stop():
    import time
    from dobot_sdk.api.motion import Motion
    from e6_phosphobot.dashboard import MotionDeadlineExpired
    connection,peer=setup();port=SubmissionPort(connection)
    try:
        with port.deadline(time.perf_counter_ns()-1):
            with pytest.raises(MotionDeadlineExpired):Motion(port).ServoJ([0]*6)
        assert not connection.pending and not connection.events and not connection.uncertain
        peer.settimeout(.01)
        with pytest.raises(TimeoutError):peer.recv(1024)
        p=RobotControl(port).Stop();assert peer.recv(1024)==b'Stop()\n'
        peer.sendall(b'0,{},Stop();');assert p.done.wait(1)
        assert connection.poll(p) and p.event['accepted']
    finally:connection.disconnect();peer.close()


def test_official_enable_uses_no_parameter_writes_and_has_separate_reply_budget():
    connection,peer=setup(reply_timeout=.001)
    robot=RobotControl(SubmissionPort(connection))
    try:
        pending=robot.EnableRobot()
        assert peer.recv(1024)==b'EnableRobot()\n'
        assert connection.timeout_for(pending.command)==10
        assert connection.timeout_for('ServoJ(1,2,3,4,5,6)')==.001
        assert connection.timeout_for('Stop()')==1.5
        pending.event['sent_ns']-=1_000_000_000
        assert not connection.poll(pending)  # 1 second is valid only for Enable.
        with pytest.raises(Busy):robot.EnableRobot()
        peer.sendall(b'0,{},EnableRobot();');assert pending.done.wait(1)
        assert connection.poll(pending) and pending.event['accepted']
    finally:
        connection.disconnect();peer.close()
