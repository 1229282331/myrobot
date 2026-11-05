import io
import pytest
from e6_phosphobot.episode_http import copy_limited


def test_bulk_media_pacing_preserves_bytes_and_cannot_catch_up_after_stall():
    now=[0.];sent=[];waits=[];payload=bytes(range(256))*256
    class Destination(io.BytesIO):
        def write(self,block):
            sent.append(now[0])
            if len(sent)==2:now[0]+=2
            return super().write(block)
    def wait(seconds):waits.append(seconds);now[0]+=seconds
    destination=Destination()
    copy_limited(io.BytesIO(payload),destination,65536,clock=lambda:now[0],wait=wait)
    assert destination.getvalue()==payload
    assert sent==[0.,.25,2.5,2.75]
    assert waits==[.25,.25,.25]


def test_unbounded_or_negative_bulk_rate_is_rejected():
    for rate in (0,-1):
        with pytest.raises(ValueError):copy_limited(io.BytesIO(b'data'),io.BytesIO(),rate)
