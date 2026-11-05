import importlib.util
import gzip
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('relay_analysis', Path(__file__).parents[1]/'tools/analyze_input_relay.py')
analysis=importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def fixture(tmp_path, *, shift=0, missing=False):
    rows=[];events=[]
    # Equal browser phases; the second transition is 100ms late on X5.
    for i,(client,x5,velocity) in enumerate([(0,0,30),(500,600,-30),(1000,1000,0)],1):
        data=dict(op='intent',seq=i,client_sample_ms=client,
                  intent=dict(deadman=bool(velocity),linear_m_s=[0,0,velocity/1000]))
        pc=9_000_000_000+int(client*1e6)
        receive=shift+3_000_000_000+int(x5*1e6)
        rows.append(dict(sequence=i,client_sample_ms=client,connection_id='pc-one',outcome='sent',
                         pc_received_ns=pc,pc_send_started_ns=pc+10,pc_send_finished_ns=pc+20))
        events.extend([dict(event='input',at_ns=receive,connection_id='x5-one',data=data),
                       dict(event='input_processed',connection_id='x5-one',sequence=i,
                            processing_ns=receive+1000,at_ns=receive+2000)])
    event_path=tmp_path/'events.jsonl.gz'
    with gzip.open(event_path,'wt') as f:
        for e in events:f.write(json.dumps(e)+'\n')
    if missing:rows.pop(1)
    pc_path=tmp_path/'pc.json'
    pc_path.write_text(json.dumps(dict(rows=rows,total=len(rows),overwritten=0)))
    return event_path,pc_path


def test_duration_comparison_is_invariant_to_arbitrary_host_clock_offset(tmp_path):
    results=[]
    for offset in (0,10**15):
        result=analysis.analyze(*fixture(tmp_path,shift=offset))
        results.append(result)
        area=result['runs'][0]['z_area_mm']
        assert area['browser']==pytest.approx(0)
        assert area['pc_send']==pytest.approx(0)
        assert area['x5_dequeue']==pytest.approx(6)
        assert area['x5_apply']==pytest.approx(6)
    assert results[0]==results[1]


def test_missing_join_is_not_silently_replaced_by_nearest_packet(tmp_path):
    with pytest.raises(KeyError):analysis.analyze(*fixture(tmp_path,missing=True))
