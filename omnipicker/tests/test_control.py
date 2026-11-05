import pytest
from studio.control import ControlEngine, Mailbox
from studio.device import MockDevice


class Clock:
    value=0.
    def __call__(self): return self.value
    def advance(self,value=.051): self.value+=value


@pytest.fixture
def rig():
    c=Clock(); d=MockDevice(); events=[]
    e=ControlEngine(lambda k,v:events.append((k,v)),clock=c,factory=lambda:d)
    e.submit('connect',(True,'TEST'));e.tick()
    return e,d,c,events


def command(e,kind,value=None): e.submit(kind,value);e.tick()


def test_connect_readonly_and_no_stale_reconnect(rig):
    e,d,c,_=rig
    assert not d.calls and e.state['enableState']=='unknown'
    command(e,'enable');e.submit('target',35)
    command(e,'disconnect','keep')
    e.submit('target',100)
    command(e,'connect',(True,'TEST'));command(e,'enable');e.tick()
    assert not any(x[0]=='opening' for x in d.calls)


def test_latest_target_rate_limit_and_final(rig):
    e,d,c,_=rig;command(e,'enable')
    for x in range(101):e.submit('target',x)
    e.tick();assert d.calls[-1]==('opening',1.)
    e.submit('target',15);e.tick();assert d.calls[-1]==('opening',1.)
    e.submit('target',35);c.advance();e.tick()
    assert d.calls[-1]==('opening',.35)
    assert len([x for x in d.calls if x[0]=='opening'])==2


def test_stop_priority_and_drops_queue(rig):
    e,d,c,_=rig;command(e,'enable')
    e.submit('target',100);e.submit('enable');e.submit('stop');e.tick();e.tick()
    assert d.calls[-1]==('state',False)
    assert not any(x[0]=='opening' for x in d.calls)
    b=Mailbox();b.put('stop');b.put('close','keep')
    assert b.take_command()[0]=='stop'


def test_test_and_manual_are_exclusive(rig):
    e,d,c,_=rig;command(e,'enable');command(e,'test',(15,35,.5,1))
    for _ in range(4):
        e.submit('target',100);e.tick();c.advance(.6)
    e.tick()
    assert [x[1] for x in d.calls if x[0]=='opening']==[.15,.35,.15]
    assert not e.state['testActive']


def test_fault_aborts_and_clear_does_not_enable(rig):
    e,d,c,_=rig;command(e,'enable');command(e,'test',(15,35,6,100))
    d.error=7;c.advance(.21);e.tick()
    assert e.state['locked'] and not e.state['testActive']
    assert d.calls[-1]==('state',False)
    command(e,'clear');assert not e.state['locked']
    assert e.state['enableState']=='disabled'


def test_timeout_locks_and_no_replay(rig):
    e,d,c,_=rig;command(e,'enable');e.submit('target',15);d.fail_next=True;e.tick()
    assert not e.state['connected'] and e.state['locked']
    command(e,'connect',(True,'TEST'));command(e,'enable');e.tick()
    assert not any(x[0]=='opening' for x in d.calls)


@pytest.mark.parametrize('kind',['close','disconnect'])
@pytest.mark.parametrize('mode',['disable','keep'])
def test_exit_branches_restore_only_after_disable(rig,kind,mode):
    e,d,c,_=rig;baseline=dict(e.baseline);command(e,'stop')
    command(e,'apply',{n:v*.1 for n,v in baseline.items()});command(e,'enable')
    command(e,kind,mode)
    assert not e.state['connected']
    assert e.quit_requested==(kind=='close')
    assert d.values==(baseline if mode=='disable' else {n:v*.1 for n,v in baseline.items()})
    if mode=='disable':
        assert d.calls[-7]==('state',False)


def test_exit_failed_stop_not_retried_or_closed(rig):
    e,d,c,_=rig;command(e,'enable')
    count=0
    def fail(enabled):
        nonlocal count
        count+=1
        raise TimeoutError('failed')
    d.request_state=fail
    command(e,'close','disable')
    assert not e.quit_requested and count==1
    assert '停止未确认' in e.state['status']


def test_limits_require_disabled_and_bounded(rig):
    e,d,c,_=rig
    command(e,'apply',e.baseline);assert not d.calls
    command(e,'stop');before=list(d.calls)
    command(e,'apply',{n:v*2 for n,v in e.baseline.items()})
    assert d.calls==before
    command(e,'apply',{n:v*.1 for n,v in e.baseline.items()});command(e,'restore')
    assert d.values==e.baseline


def test_unknown_firmware_cannot_write(rig):
    e,d,c,_=rig;d.compatible=False
    command(e,'enable');command(e,'stop');e.submit('target',35);e.tick()
    assert not d.calls


def test_cancel_exit_is_no_command(rig):
    e,d,c,_=rig;command(e,'enable');before=list(d.calls);e.tick()
    assert e.state['connected'] and not e.quit_requested and d.calls==before


def test_temporary_baseline_survives_transport_reconnect(rig):
    e,d,c,_=rig; baseline=dict(e.baseline);command(e,'stop')
    command(e,'apply',{n:v*.1 for n,v in baseline.items()})
    d.fail_next=True;c.advance(.3);e.tick()
    assert e.modified and not e.state['connected']
    command(e,'close','disable');assert not e.quit_requested
    command(e,'connect',(True,'TEST'));assert e.baseline==baseline
    command(e,'close','disable');assert e.quit_requested and d.values==baseline


def test_unconfirmed_stop_requires_explicit_keep_to_close(rig):
    e,d,c,_=rig
    d.request_state=lambda *_: (_ for _ in ()).throw(TimeoutError())
    command(e,'stop')
    assert e.state['mayBeEnabled'] and not e.state['connected']
    command(e,'close','disable');assert not e.quit_requested
    command(e,'close','keep');assert e.quit_requested


def test_close_after_confirmed_stop_does_not_repeat_disable(rig):
    e,d,c,_=rig;command(e,'enable');command(e,'stop')
    command(e,'apply',{n:v*.1 for n,v in e.baseline.items()})
    def reject_repeat(*_): raise RuntimeError('firmware rejects redundant state transition')
    d.request_state=reject_repeat
    command(e,'close','disable')
    assert e.quit_requested and not e.modified and d.values==e.baseline


def test_lost_enable_ack_does_not_reuse_previous_disabled_result(rig):
    e,d,c,_=rig;command(e,'stop');attempts=[]
    def uncertain(enabled):
        attempts.append(enabled)
        if enabled: raise TimeoutError('enable accepted, ACK lost')
    d.request_state=uncertain
    command(e,'enable')
    assert attempts==[True,False]
    assert e.state['enableState']=='disabled' and not e.state['mayBeEnabled']
