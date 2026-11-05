import asyncio
import copy
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from control_acceptance import Harness
import control_acceptance


def test_shadow_matrix_includes_original_multi_axis_mapping_signals():
    single={'axis':0,'input':{'twist':[1,0,0,0,0,0]}}
    multi={'axes':[0,1,2],'input':{'twist':[.5,.5,.5,0,0,0]}}
    fixtures={'rows':[single],'multi':[multi]}
    result=control_acceptance.matrix_rows(fixtures)
    assert result[0] is single and result[1]['axis']=='xyz'
    assert result[1]['input'] is multi['input'] and 'axis' not in multi


class DelayedTransition(Harness):
    def __init__(self):
        self.last_input_identity=None;self.ack_identity=None;self.trace=[];self.remote_error=None
        self.latest=None;self.status={'armed':False};self.skipped_input_periods=0
    async def rpc(self,input):
        identity=(input['source'],input['identity'])
        self.trace.append(('neutral-request',identity))
        # More than one input period of return latency. Sending the new
        # source before this acknowledgement would carry its old lease.
        await asyncio.sleep(.075)
        self.ack_identity=identity;self.last_input_identity=identity
        self.trace.append(('acknowledged',identity));return {'ok':True}
    async def send(self,input):
        identity=(input['source'],input['identity'])
        assert identity==self.ack_identity,'Old-token stream started before source acknowledgement'
        self.trace.append(('stream',identity))


def test_changed_source_waits_for_neutral_reply_before_continuing_stream():
    h=DelayedTransition()
    async def run():
        for source in ('gamepad','keyboard'):
            n=dict(source=source,identity=source,neutral=True,deadman=False)
            await h.phase(.11,n);await h.phase(.05,n)
    asyncio.run(run())
    for source in ('gamepad','keyboard'):
        events=[e[0] for e in h.trace if e[1][0]==source]
        assert events[:2]==['neutral-request','acknowledged'] and events.count('neutral-request')==1
        assert events.count('stream')>=3


def test_device_transition_cannot_start_with_an_active_signal():
    h=DelayedTransition()
    with pytest.raises(ValueError,match='neutral handshake'):
        asyncio.run(h.phase(.1,dict(source='gamepad',identity='new pad',neutral=False,deadman=True)))
    assert not h.trace


@pytest.mark.parametrize('operation',['input','arm'])
def test_delayed_new_session_rpc_keeps_fresh_neutral_streaming(operation):
    import time
    async def run():
        h=Harness.__new__(Harness);h.remote_error=None;sends=[];requests=[]
        neutral=dict(operation='input',source='keyboard',identity='new owner',neutral=True,deadman=False)
        async def send(input):
            assert input['neutral'] and not input['deadman']
            sends.append(time.monotonic())
        async def rpc(data):
            # Windows' default timer quantum can turn each 50 ms wait into
            # 62.5 ms. Use a delay beyond one lease that still permits five
            # neutral samples without assuming sub-quantum wall timing.
            requests.append(data['operation']);await asyncio.sleep(.4);return {'ok':True}
        h.send=send;h.rpc=rpc
        start=time.monotonic();result=await h.neutral_rpc(neutral if operation=='input' else {'operation':'arm'},neutral)
        assert result['ok'] and requests==[operation] and len(sends)>=5
        assert max(b-a for a,b in zip([start]+sends,sends))<.15
        with pytest.raises(ValueError,match='neutral released'):await h.neutral_rpc({'operation':'arm'},dict(neutral,deadman=True))
    asyncio.run(run())


@pytest.mark.parametrize('rejection',['已有控制者','Required observations invalid'])
def test_shadow_handover_only_retries_rejected_owner_connections(monkeypatch,rejection):
    class Wire:
        def __init__(self,hello):self.hello=hello;self.closed=False
        async def recv(self):return json.dumps(self.hello)
        async def close(self):self.closed=True
    async def run():
        h=Harness.__new__(Harness);h.latest={'control_stage':'shadow'}
        old=Wire({});h.ws=old;h.requests={1:None};h.remote_error=None
        h.last_input_identity=('gamepad','old');h.session_transitions=[]
        h.receiver=asyncio.create_task(asyncio.sleep(10));calls=[];operations=[]
        async def rpc(data):operations.append(data['operation'])
        async def replies():await asyncio.sleep(10)
        async def connect(*args,**kwargs):
            wire=Wire({'ok':True,'status':{'armed':False}} if calls else {'ok':False,'error':rejection})
            calls.append(wire);return wire
        h.rpc=rpc;h.replies=replies
        monkeypatch.setenv('E6_TELEOP_HARNESS_TOKEN','test-only')
        monkeypatch.setattr(control_acceptance.websockets,'connect',connect)
        try:
            if rejection=='已有控制者':
                await h.replace_input_session()
                assert len(calls)==2 and h.session_transitions[0]['attempts']==2
                assert h.status=={'armed':False} and h.last_input_identity is None
            else:
                with pytest.raises(RuntimeError,match=rejection):await h.replace_input_session()
                assert len(calls)==1 and not h.session_transitions
            assert old.closed and calls[0].closed and operations==['stop']
            assert not h.requests
        finally:
            h.receiver.cancel();await asyncio.gather(h.receiver,return_exceptions=True)
    asyncio.run(run())
