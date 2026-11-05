from types import SimpleNamespace
import pytest
from test_gateway import setup
from e6_phosphobot.control_state import for_operator


def request(e,source='keyboard',op='record_start',**fields):
    return dict(op=op,seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
                source=source,client_sample_ms=0,task='explicit recording',**fields)


@pytest.mark.parametrize('source',['keyboard','gamepad'])
def test_expired_record_click_is_visible_without_retry_or_motion_and_new_request_can_succeed(source):
    e,d,send,step,now=setup('shadow')
    e.session.ready=False;e.session.source=source
    calls=[];value={'phase':'idle','error':None}
    e.episodes=SimpleNamespace(state=lambda:dict(value),request=lambda op,**data:calls.append((op,data)))
    data=request(e,source);now[0]+=1_230_000_000
    e.receive(data)
    state=for_operator(e.state())
    assert state['recording']['phase']=='idle' and not calls and not d.calls
    assert state['recording']['request']['sequence']==data['seq']
    assert state['recording']['request']['accepted'] is False
    assert '未被接受' in state['recording']['error']
    assert state['reason'] is None and e.session.ready is False
    assert e.session.dropped==1
    # Ordinary fresh neutral packets cannot hide or replay the rejected click.
    e.session.source='keyboard';send();e.session.source=source
    assert e.state()['recording']['error']==state['recording']['error'] and not calls
    e.receive(request(e,source))
    state=e.state()['recording']
    assert len(calls)==1 and calls[0][0]=='start'
    assert state['request']['accepted'] and state['error'] is None
    assert not d.calls and not e.session.ready


def test_duplicate_recording_start_is_not_executed_and_does_not_replace_device_failure():
    e,d,send,step,now=setup('shadow');calls=[]
    e.episodes=SimpleNamespace(state=lambda:dict(phase='aborted',error='disk_full'),
                              request=lambda *a,**k:calls.append(a))
    data=request(e);data['seq']=e.session.sequence
    e.receive(data)
    state=e.state()['recording']
    assert not calls and not d.calls and e.session.ready
    assert state['error']=='disk_full' and not state['request']['accepted']
    assert '未被接受' in state['request']['reason']


def test_finish_rejection_is_visible_and_a_later_explicit_finish_clears_it():
    e,d,send,step,now=setup('shadow');calls=[]
    def fail(*a,**kw):raise ValueError('没有正在录制的片段')
    e.episodes=SimpleNamespace(state=lambda:dict(phase='idle'),request=fail,abort=lambda reason:None)
    e.receive(request(e,op='record_finish',result='success'))
    assert e.state()['recording']['error']=='没有正在录制的片段'
    assert not e.state()['recording']['request']['accepted']
    e.episodes.request=lambda *a,**kw:calls.append(a)
    e.receive(request(e,op='record_finish',result='aborted'))
    assert e.state()['recording']['request']['accepted']
    assert e.state()['recording'].get('error') is None
    assert len(calls)==1 and not d.calls
