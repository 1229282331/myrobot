import json
import struct
import pytest
from studio.protocol import crc16, encode_request, FibreChannel, ProtocolError
from studio.device import NativeDevice, ROOT, flatten_schema, opening_fraction


def test_crc_known_check_vector():
    # EN-13757 check value is C2B7 after xorout=FFFF (we return raw CRC).
    assert crc16(b"123456789",0) ^ 0xFFFF == 0xC2B7
    assert crc16(b"",1) == 1
    assert crc16((ROOT/'assets/schema-fw330.json').read_bytes())==9774


def test_wire_packet():
    assert encode_request(0x81,65,struct.pack('<f',.15),0,0x1234).hex() == '8100418000009a99193e3412'
    assert encode_request(0x81,0,b'\0'*4,512,99)[-2:] == b'\x01\0'


class FakeTransport:
    def __init__(self,replies): self.replies=list(replies); self.writes=[]
    def write(self,data,timeout): self.writes.append(data)
    def read(self,timeout): return self.replies.pop(0)


def test_stale_ack_ignored_and_no_repeated_write():
    t=FakeTransport([b'\x80\x80bad',b'\x81\x80'+struct.pack('<I',330)])
    assert FibreChannel(t).operation(3,length=4)==struct.pack('<I',330)
    assert len(t.writes)==1


@pytest.mark.parametrize('reply',[b'',b'\x81\x80\x00',b'\x81\x80'+b'0'*5])
def test_bad_response_rejected(reply):
    t=FakeTransport([reply])
    with pytest.raises(ProtocolError): FibreChannel(t).operation(3,length=4)
    assert len(t.writes)==1


def test_timeout_write_never_retried():
    class Timeout(FakeTransport):
        def read(self,timeout): raise TimeoutError()
    t=Timeout([])
    with pytest.raises(TimeoutError): FibreChannel(t).operation(64)
    assert len(t.writes)==1


def test_function_argument_trigger_and_cancel():
    class Channel:
        def __init__(self): self.calls=[]
        def operation(self,*args,**kwargs): self.calls.append((args,kwargs)); return b''
    d=NativeDevice(); d.compatible=True
    d.entries=flatten_schema(json.loads((ROOT/'assets/schema-fw330.json').read_text()))
    d.channel=Channel()
    assert d.set_opening(15)
    assert [x[0][0] for x in d.channel.calls]==[65,64]
    assert d.channel.calls[0][0][1]==struct.pack('<f',.15)
    d.channel.calls.clear(); d.cancel_motion=lambda:True
    assert not d.set_opening(35)
    assert [x[0][0] for x in d.channel.calls]==[65]
    d.compatible=False
    with pytest.raises(ProtocolError): d.set_opening(15)


@pytest.mark.parametrize('p,f',[(0,0),(15,.15),(35.1,.351),(100,1),(15.12,.151)])
def test_percent(p,f): assert opening_fraction(p)==pytest.approx(f)


@pytest.mark.parametrize('p',[-.1,100.1,float('nan'),float('inf')])
def test_bad_percent(p):
    with pytest.raises(ValueError): opening_fraction(p)
