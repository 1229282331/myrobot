import base64
import json
from pathlib import Path
import socket
import struct

import pytest

from e6_phosphobot.dobot_transport import FeedbackStream, FeedbackFramingError
from e6_phosphobot.feedback import FeedbackReader
from test_feedback_reader import until
from test_sdk_upstream import packet


def captured_bad_packet():
    root=Path(__file__).resolve().parents[2]
    data=json.loads((root/'docs/teleop-rebuild/evidence/v12_feedback_outlier_neighbours.json').read_text())
    return base64.b64decode(data[2]['raw']['raw_b64'])


@pytest.mark.parametrize('split',[1,79,80,128,1439])
def test_captured_nested_header_is_not_a_joint_observation(split):
    raw=captured_bad_packet()
    stream=FeedbackStream()
    assert stream.feed(raw[:split],1)==[]
    with pytest.raises(FeedbackFramingError,match='offset 80') as exc:
        stream.feed(raw[split:],2)
    assert exc.value.wire==raw


def test_marker_without_header_magic_is_ordinary_payload():
    raw=bytearray(packet())
    struct.pack_into('<Q',raw,80,1440)
    assert len(FeedbackStream().feed(raw,1))==1


def test_reader_preserves_rejection_then_reconnects_without_publishing_bad_frame():
    incoming1,outgoing1=socket.socketpair()
    incoming2,outgoing2=socket.socketpair()
    sockets=iter([incoming1,incoming2])
    reader=FeedbackReader('test',socket_factory=lambda *a,**k:next(sockets),reconnect_delay=.01)
    raw=captured_bad_packet()
    subscription=reader.subscribe(10)
    with reader:
        outgoing1.sendall(packet(1))
        until(lambda:reader.snapshot()['sequence']==1)
        generation=reader.snapshot()['generation']
        outgoing1.sendall(raw)
        until(lambda:reader.snapshot()['generation']!=generation)
        assert reader.snapshot()['status'] is None
        outgoing2.sendall(packet(2))
        until(lambda:reader.snapshot()['sequence']==1)
        assert subscription.queue.qsize()==2
        event=next(e for e in reader.drain_events() if 'rejected_wire_b64' in e)
        assert base64.b64decode(event['rejected_wire_b64'])==raw
        assert not event['evidence_truncated']
        assert reader.snapshot()['status']['timestamp']==2
    outgoing1.close();outgoing2.close()
