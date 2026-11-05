import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location('gripper_audit',Path(__file__).parents[1]/'scripts/analyze_gripper_audit.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_audit_does_not_hide_wrong_position_or_a_target_sent_after_release(tmp_path):
    def intent(ns,seq,active):return dict(event='input',at_ns=ns,data=dict(op='intent',seq=seq,
        intent=dict(deadman=active,gripper_rate_s=-.025 if active else 0)))
    def tx(ns,raw):return dict(event='can',data=dict(kind='tx',host_perf_counter_ns=ns,
        data_hex=bytes([0,raw,25,64,25,25,0,0]).hex()))
    rows=[tx(1,89),intent(10,1,True),tx(20,70),intent(30,2,False),tx(40,69)]
    for ns in (800_000_030,850_000_030,900_000_030):
        rows.append(dict(event='gripper_feedback',at_ns=ns,target=70/255,
            feedback=dict(host_perf_counter_ns=ns,state=0,position_raw=74,round_trip_ms=1)))
    path=tmp_path/'events.jsonl';path.write_text('\n'.join(json.dumps(r) for r in rows))
    result=module.analyze(path);group=result['input_groups'][0]
    assert group['final_sent_raw']==70 and group['changed_target_after_release']==[40]
    assert not group['tail_arrived'] and group['tail_samples']==3
    assert result['incomplete_input'] is None
