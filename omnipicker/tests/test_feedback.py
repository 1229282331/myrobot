from types import SimpleNamespace
import time
import pytest

from studio.feedback import CsvDecoder, CdcFeedback
from studio.control import ControlEngine
from studio.device import MockDevice


def test_decoder_keeps_host_arrival_time_and_raw_signed_values():
    decoder = CsvDecoder()
    assert decoder.feed(b"edge\n-0.12,0.", 1) == []
    rows = decoder.feed(b"18,0.15\r\n0.02,-0.18,0.35\n", 1.01)
    assert len(rows) == 2
    assert rows[0]["monotonic"] == rows[1]["monotonic"] == 1.01
    assert rows[0]["raw"] == "-0.12,0.18,0.15\r"
    assert rows[1]["velocity_fraction_s"] == -.18
    assert rows[1]["position_fraction"] == .35
    assert rows[1]["host_sequence"] == 2


@pytest.mark.parametrize("bad", [b"nan,0.00,0.00\n", b"1,2,3\n", b"0.00,0.00,0.00,0.00\n",
                               b"0.00,0.00,10.00\n", b"x"*1025])
def test_decoder_rejects_changed_or_unbounded_format(bad):
    decoder = CsvDecoder()
    decoder.feed(b"edge\n", 0)
    with pytest.raises(ValueError): decoder.feed(bad, 1)


def test_serial_identity_and_readonly_open_and_close():
    class Port:
        def __init__(self, **kwargs):
            assert kwargs['port'] is None and kwargs['timeout'] == .05
            self.closed = False
            self.data = b"edge\n0.10,-0.18,0.15\n"
        def open(self): assert self.dtr is False and self.rts is False
        @property
        def in_waiting(self): return len(self.data)
        def read(self, size):
            time.sleep(.002)
            data, self.data = self.data, b""
            return data
        def close(self): self.closed = True
        def write(self, data): raise AssertionError("CDC must never send application bytes")
    match = SimpleNamespace(vid=0x1001,pid=0x03ef,serial_number="TEST",device="COM_TEST")
    reader = CdcFeedback("TEST", port_factory=Port, enumerate_ports=lambda:[match])
    reader.start()
    deadline = time.monotonic()+1
    rows=[]
    try:
        while not rows and time.monotonic()<deadline:
            rows,error=reader.poll()
            time.sleep(.002)
        assert not error and rows[0]['position_fraction']==.15
    finally: reader.close()
    assert reader.port.closed and not reader.thread.is_alive()
    other = CdcFeedback("OTHER", port_factory=Port, enumerate_ports=lambda:[match])
    with pytest.raises(RuntimeError): other.start()
    assert other.port is None


class Clock:
    value = 0.
    def __call__(self): return self.value


class Feed:
    port_name = "TEST"
    def __init__(self): self.rows=[];self.error=None;self.closed=False
    def start(self): pass
    def close(self): self.closed=True
    def poll(self):
        rows,self.rows=self.rows,[]
        return rows,self.error
    def add(self, t, p=.15):
        self.rows.append(dict(monotonic=t,host_sequence=1,raw=f"0.00,0.00,{p:.2f}",
                         position_fraction=p,velocity_fraction_s=0.,column1_unverified=0.))


def rig(initial=True):
    clock,device,feed,events=Clock(),MockDevice(),Feed(),[]
    if initial: feed.add(0)
    engine=ControlEngine(lambda k,d:events.append((k,d)),clock=clock,factory=lambda:device,
                         feedback_factory=lambda d:feed)
    engine.submit("connect",(True,"TEST"));engine.tick()
    return engine,device,clock,feed,events


def command(e,k,v=None): e.submit(k,v);e.tick()


def test_initial_missing_feedback_never_changes_existing_motor_state():
    e,d,c,f,events=rig(False)
    command(e,'enable')
    c.value=2.1;e.tick()
    assert not d.calls
    assert not e.state['feedbackValid'] and e.state['feedbackPosition'] is None
    assert e.state['connected'] and e.state['enableState']=='unknown'


@pytest.mark.parametrize('failure',['stale','error','gap'])
def test_loss_stops_owned_motion_once_clears_targets_and_requires_reconnect(failure):
    e,d,c,f,events=rig()
    assert e.state['feedbackPosition']==15 and e.state['ackTarget']==-1
    command(e,'enable');command(e,'test',(15,35,6,2));e.tick()
    old_connection=e.state['connectionId']
    e.submit('target',100)
    c.value=.31
    if failure=='error': f.error='USB disconnected'
    if failure=='gap': f.add(.31)
    e.tick()
    assert e.state['locked'] and not e.state['testActive']
    assert not e.state['feedbackValid'] and e.state['feedbackPosition'] is None
    assert d.calls[-1]==('state',False)
    f.error=None;f.add(.32);c.value=.32;e.tick()
    assert not e.state['feedbackValid']
    assert len([x for x in d.calls if x==('state',False)])==1
    command(e,'enable')
    assert e.state['enableState']=='disabled'
    command(e,'disconnect','keep');f.add(.33);c.value=.33
    command(e,'connect',(True,'TEST'));command(e,'enable');e.tick()
    assert e.state['connectionId']!=old_connection
    assert [x[1] for x in d.calls if x[0]=='opening']==[.15]


def test_feedback_failure_after_readonly_connect_does_not_disable():
    e,d,c,f,_=rig()
    f.error='port closed';e.tick()
    assert not d.calls
    assert e.state['enableState']=='unknown' and not e.state['feedbackValid']


def test_simulated_observation_has_dynamics_and_explicit_source():
    c,d,events=Clock(),MockDevice(),[]
    e=ControlEngine(lambda k,v:events.append((k,v)),clock=c,factory=lambda:d)
    command(e,'connect',(True,'TEST'));command(e,'enable');e.submit('target',35);e.tick()
    c.value=.1;e.tick()
    assert 0 < e.state['feedbackPosition'] < 35
    assert e.state['ackTarget']==35
    assert all(row['source']=='simulation' for kind,rows in events if kind=='observation' for row in rows)
