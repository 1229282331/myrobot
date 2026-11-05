"""Reproduce upstream transport behavior without contacting any device."""
from collections import deque
import numpy as np
from dobot_sdk.api.robot import DobotRobot
from dobot_sdk.core.connection import DobotConnection
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC
from dobot_sdk.protocol.feedback import FeedbackParser


def packet(timestamp=1):
    data = np.zeros(1, dtype=MyType)
    data['len'] = 1440
    data['TestValue'] = TEST_VALUE_MAGIC
    data['RobotMode'] = 5
    data['TimeStamp'] = timestamp
    return data.tobytes()


class Chunks:
    def __init__(self, *chunks):
        self.chunks = deque(chunks)
    def recv(self, _size):
        return self.chunks.popleft()


def test_upstream_parser_decodes_complete_official_packet():
    result = FeedbackParser().parse(packet(42))
    assert result is not None and result.timestamp == 42


def test_upstream_dashboard_returns_fragment_as_complete_reply():
    connection = DobotConnection('127.0.0.1', 29999)
    connection._socket = Chunks(b'0,{17},Ser', b'voP(1,2,3,4,5,6);')
    connection._connected = True
    try:
        # Reproduces the defect: caller receives an incomplete successful prefix.
        assert connection.receive_text() == '0,{17},Ser'
        assert connection._socket.chunks
    finally:
        connection._socket = None
        connection._connected = False


def test_upstream_monitor_discards_all_coalesced_packets():
    robot = DobotRobot('127.0.0.1')
    reads = 0
    observed = []
    def receive(_size):
        nonlocal reads
        reads += 1
        if reads == 1:
            return packet(1) + packet(2)
        robot._feedback_running = False
        return packet(3)
    robot._feedback_conn.receive_bytes = receive
    robot._callback = observed.append
    robot._feedback_running = True
    robot._feedback_loop()
    assert [status.timestamp for status in observed] == [3]
