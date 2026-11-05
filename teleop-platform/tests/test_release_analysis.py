import base64
import importlib.util
import json
from pathlib import Path
import numpy as np
from dobot_sdk.protocol.data_types import MyType

spec=importlib.util.spec_from_file_location('release_analysis',Path(__file__).parents[1]/'scripts/analyze_releases.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_delayed_stop_is_measured_from_release_edge_not_later_idle_heartbeat(tmp_path):
    events=[]
    for ms,held in [(0,True),(100,False),(150,False),(200,False),(250,False)]:
        events.append(dict(event='input',at_ns=ms*1_000_000,
                           data=dict(intent=dict(deadman=held))))
    events.append(dict(event='stop_requested',receipt=dict(command='Stop()',sent_ns=260_000_000,
        reply_ns=700_000_000,accepted=True)))
    events.append(dict(event='motion_submitted',receipt=dict(command='ServoP(0)',sent_ns=120_000_000)))
    events.append(dict(event='gripper_feedback',target=.35))
    (tmp_path/'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
    raw=np.zeros(1,dtype=MyType).tobytes()
    rows=[dict(received_ns=ms*1_000_000,raw_b64=base64.b64encode(raw).decode()) for ms in range(0,2100,8)]
    (tmp_path/'raw.jsonl').write_text('\n'.join(json.dumps(e) for e in rows))
    result=module.analyze(tmp_path)
    assert len(result['releases'])==1
    assert result['releases'][0]['stop']['sent_delay_ms']==160
    assert result['releases'][0]['first_deceleration_ms']==20
