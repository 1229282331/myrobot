import socket
import struct
from collections import deque
import numpy as np
import pytest

from dobot_sdk.api.motion import Motion
from dobot_sdk.protocol.data_types import MyType
from e6_phosphobot.dobot_transport import FeedbackStream, FramedDobotConnection
from test_sdk_upstream import packet


@pytest.mark.parametrize('split', [1, 7, 48, 711, 1439, 1440, 1500, 2881])
def test_all_feedback_packets_survive_fragmentation_and_coalescing(split):
    stream = FeedbackStream()
    raw = packet(1) + packet(2) + packet(3)
    rows = stream.feed(raw[:split], 10) + stream.feed(raw[split:], 20)
    assert [r.status.timestamp for r in rows] == [1, 2, 3]
    assert b''.join(r.raw for r in rows) == raw
    stream.eof()


def test_explicit_length_envelope_preserves_wire_prefix():
    raw = struct.pack('<Q', 1440) + packet(42)
    stream = FeedbackStream(allow_length_envelope=True)
    assert stream.feed(raw[:1440], 10) == []
    row, = stream.feed(raw[1440:], 20)
    assert row.prefix + row.raw == raw and row.status.timestamp == 42


def test_model_domain_does_not_limit_actual_feedback():
    data = np.frombuffer(packet(), dtype=MyType).copy()
    data['QActual'] = [100, 20, -15, 30, -80, 12]
    row, = FeedbackStream().feed(data.tobytes(), 1)
    assert row.status.joint_state.q_actual == [100, 20, -15, 30, -80, 12]


def test_nonfinite_and_truncated_feedback_remain_explicit():
    data = np.frombuffer(packet(), dtype=MyType).copy()
    data['QActual'][0][0] = float('nan')
    with pytest.raises(ValueError, match='Non-finite'):
        FeedbackStream().feed(data.tobytes(), 1)
    stream = FeedbackStream()
    stream.feed(packet()[:100], 1)
    with pytest.raises(ValueError, match='truncated'):
        stream.eof()


def test_device_restart_is_not_old_session_data():
    stream = FeedbackStream()
    stream.feed(packet(20), 1)
    with pytest.raises(ValueError, match='restarted'):
        stream.feed(packet(10), 2)


class FakeSocket:
    def __init__(self, *chunks):
        self.chunks = deque(chunks)
        self.sent = []
    def recv(self, _size):
        item = self.chunks.popleft()
        if isinstance(item, BaseException):
            raise item
        return item
    def sendall(self, data):
        self.sent.append(data)
    def settimeout(self, _timeout):
        pass
    def shutdown(self, _how):
        pass
    def close(self):
        pass


def connected(*chunks):
    conn = FramedDobotConnection('127.0.0.1')
    conn._socket = FakeSocket(*chunks)
    conn._connected = True
    return conn


def test_reply_fragments_are_buffered_but_unsolicited_ack_is_rejected():
    conn = connected(b'0,{},Ro', b'botMode();0,{},Stop();')
    assert conn.send_receive_text('RobotMode()') == '0,{},RobotMode();'
    with pytest.raises(ValueError, match='Unsolicited'):
        conn.send_receive_text('Stop()')
    assert conn._socket.sent == [b'RobotMode()\n']


def test_official_motion_builder_is_used_unchanged():
    expected = 'ServoP(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,t=0.050,aheadtime=50.0,gain=500.0)'
    conn = connected(('0,{21},' + expected + ';').encode())
    assert Motion(conn).ServoP([1,2,3,4,5,6], t=.05) == '0,{21},' + expected + ';'
    assert conn._socket.sent == [(expected+'\n').encode()]


def test_timeout_never_resends_motion_and_late_reply_cannot_ack_stop():
    conn = connected(socket.timeout(), b'0,{},RobotMode();', b'0,{},Stop();')
    with pytest.raises(socket.timeout):
        conn.send_receive_text('RobotMode()')
    with pytest.raises(RuntimeError, match='Unconfirmed'):
        conn.send_receive_text('RobotMode()')
    assert conn.send_receive_text('Stop()') == '0,{},Stop();'
    assert conn.events[-1]['unmatched_replies'] == ['0,{},RobotMode();']
    assert conn._socket.sent == [b'RobotMode()\n', b'Stop()\n']
    assert conn.uncertain  # Restore requires a new explicit connection.
