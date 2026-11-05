"""Exact observed framing grammar; wrong lengths/markers never resynchronize."""
from pathlib import Path
import struct
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.protocol import FeedbackParser,decode

WIRE=(Path(__file__).resolve().parents[2]/'docs/evidence/teleop_local_integration_20261005/feedback_length_envelope.bin').read_bytes()


def test_real_capture_requires_explicit_opt_in_and_keeps_exact_outer_header():
    p=FeedbackParser(extended=True)
    with pytest.raises(ValueError,match='marker'):p.feed(WIRE)
    assert bytes(p.buffer)==WIRE
    p=FeedbackParser(extended=True,allow_length_envelope=True)
    rows=p.feed(WIRE);assert len(rows)==1 and rows[0][0]==WIRE[8:]
    assert rows[0][1]==decode(WIRE[8:],True)
    assert p.last_transport_prefixes==[WIRE[:8]] and p.length_envelopes==1
    assert p.last_transport_prefixes[0]+rows[0][0]==WIRE;p.eof()


@pytest.mark.parametrize('boundary',[1,7,8,16,48,55,56,1440,1447])
def test_split_envelope_and_coalesced_standard_frames(boundary):
    p=FeedbackParser(extended=True,allow_length_envelope=True)
    assert not p.feed(WIRE[:boundary])
    rows=p.feed(WIRE[boundary:]+WIRE[8:]+WIRE)
    assert len(rows)==3 and p.last_transport_prefixes==[WIRE[:8],b'',WIRE[:8]] and p.length_envelopes==2;p.eof()


@pytest.mark.parametrize('kind',['length','inner-marker','nan','unknown-prefix'])
def test_invalid_envelope_never_accepted_or_scanned(kind):
    raw=bytearray(WIRE)
    if kind=='length':struct.pack_into('<Q',raw,0,1448)
    elif kind=='inner-marker':struct.pack_into('<Q',raw,56,0)
    elif kind=='nan':struct.pack_into('<d',raw,8+432,float('nan'))
    else:raw=b'unknown!'+WIRE[8:]
    p=FeedbackParser(extended=True,allow_length_envelope=True)
    with pytest.raises(ValueError):p.feed(bytes(raw)+WIRE[8:])
    assert p.length_envelopes==0


def test_normal_frame_with_input_bitmap_equal_to_length_keeps_standard_precedence():
    raw=bytearray(WIRE[8:]);struct.pack_into('<Q',raw,8,1440)
    p=FeedbackParser(extended=True,allow_length_envelope=True);rows=p.feed(raw)
    assert rows[0][0]==bytes(raw) and p.last_transport_prefixes==[b''] and p.length_envelopes==0
