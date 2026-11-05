"""Latest-only SDK projection: no timestamp renewal, mutable aliases or RX lock."""
from dataclasses import asdict
import threading
from unittest.mock import patch
from e6_phosphobot.feedback import FeedbackReader
from e6_phosphobot.dobot_transport import FeedbackFrame
from test_sdk_upstream import packet


def reader():
    r=FeedbackReader('no-device');r.connected=True;r.generation='first'
    r._publish(FeedbackFrame(packet(42),b'',10))
    return r


def test_latest_projection_reuses_sdk_decode_without_sharing_mutable_values():
    r=reader();original=FeedbackFrame.status.fget;calls=[]
    def decode(frame):calls.append(frame);return original(frame)
    with patch.object(FeedbackFrame,'status',property(decode)):
        before=r.snapshot()
        mutated=r.snapshot()
        mutated['status']['joint_state']['q_actual'][0]=999
        mutated['status']['user_value'][0]=999
        mutated['status']['tool_vector_actual']['x']=999
        mutated['status']['actual_quaternion']['qw']=999
        later=r.snapshot()
        assert later['status']==before['status'] and len(calls)==1
        assert later['received_ns']==10 and later['age_ms']>=before['age_ms']
        r._publish(FeedbackFrame(packet(43),b'',20))
        fresh=r.snapshot()
        assert fresh['status']['timestamp']==43 and fresh['sequence']==2
        assert fresh['received_ns']==20 and len(calls)==2
        assert r.snapshot(full=True)['status']==asdict(original(r.latest))


def test_decoding_cannot_block_raw_publication_or_mix_new_metadata_with_old_frame():
    r=reader();old=r.latest;original=FeedbackFrame.status.fget
    entered,resume=threading.Event(),threading.Event();result=[]
    def decode(frame):
        if frame is old:entered.set();assert resume.wait(2)
        return original(frame)
    with patch.object(FeedbackFrame,'status',property(decode)):
        t=threading.Thread(target=lambda:result.append(r.snapshot()));t.start()
        try:
            assert entered.wait(1)
            r._publish(FeedbackFrame(packet(43),b'',20))
            new=r.snapshot()
            assert new['status']['timestamp']==43 and new['sequence']==2
        finally:resume.set();t.join(2)
    assert not t.is_alive()
    assert result[0]['status']['timestamp']==42 and result[0]['sequence']==1
    assert result[0]['received_ns']==10
    assert r.snapshot()['status']['timestamp']==43
    # Reconnect with no frame must not expose the retained cache.
    r.latest=None;r.generation='second'
    assert r.snapshot()['status'] is None
