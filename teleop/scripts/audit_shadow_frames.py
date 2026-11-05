"""Read back every retained shadow frame and its original E6 SDK packet."""
import argparse,base64,gzip,json,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.protocol import decode

def audit(directory):
    directory=Path(directory);summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    assert summary['passed'] and summary['stage']=='shadow' and summary['invalid']==summary['actions']==0
    path=directory/'frames.jsonl';opener=open
    if not path.exists():path=path.with_suffix('.jsonl.gz');opener=gzip.open
    joints=[];seen=set();computations=[];residuals=[];armed_dt=[];flags=set();clocks=[];count=0;released=0
    with opener(path,'rt',encoding='utf-8') as stream:
        for line in stream:
            frame=json.loads(line);count+=1
            assert frame['observation']['valid'] and frame['control_stage']=='shadow'
            robot=frame['sources']['robot'];raw=base64.b64decode(robot['raw_base64'],validate=True);parsed=decode(raw,True)
            actual=robot['decoded'];assert set(parsed)==set(actual)
            for name,value in parsed.items():
                if isinstance(value,list):np.testing.assert_allclose(value,actual[name],atol=1e-12,rtol=0)
                else:assert value==actual[name]
            assert not actual['error_status'] and not actual['collision_state'] and actual['robot_mode'] in (4,5)
            joints.append(actual['q_actual_deg']);flags.add((actual['robot_mode'],actual['run_queued_command'],actual['pause_command_flag']))
            tick=frame['control'].get('tick') or {};assert tick.get('effective_action') is None
            identity=tick.get('control_tick_id')
            if identity is None or identity in seen:continue
            seen.add(identity);clocks.append(tick['control_tick_monotonic_ns'])
            if tick['status']['armed']:armed_dt.append(tick['dt_s'])
            candidate=tick.get('candidate_action')
            if candidate:
                ik=tick['ik_result'];assert ik['position_m']<=.001 and ik['rotation_rad']<=math.radians(1)
                if not ik['accepted']:
                    assert ik['reason']=='允许动作键已释放，候选目标未发送'
                    assert not tick['status']['faulted'] and tick['status']['armed']
                    assert candidate['rejected_reason']==ik['reason'];released+=1
                computations.append(ik['query_duration_ms']);residuals.append((ik['position_m'],ik['rotation_rad']))
                assert np.max(np.abs(ik['joint_velocity_rad_s']))<=math.radians(2)+1e-6
                assert np.max(np.abs(ik['joint_acceleration_rad_s2']))<=math.radians(4)+1e-6
            for key in ('motion_tx','stop_tx','can_tx'):assert frame['control']['stats'][key]==0
    assert count==summary['frames'] and len(computations)==summary['candidate_ticks']
    assert all(b>a for a,b in zip(clocks,clocks[1:]))
    assert not armed_dt or max(armed_dt)<=.075
    def percentiles(values):return dict(zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]).tolist())) if len(values) else {}
    result=dict(passed=True,raw_sdk_frames_checked=count,unique_ticks=len(seen),candidate_ticks=len(computations),
                actual_joint_peak_to_peak_deg=np.ptp(joints,axis=0).tolist(),algorithm_queue_flag_states=sorted(flags),
                local_ik_fk_ms=percentiles(computations),armed_period_ms=percentiles(np.asarray(armed_dt)*1000),
                max_candidate_residual_m_rad=np.max(residuals,axis=0).tolist() if residuals else None,
                normally_released_candidates= released,physical_motion_test=False,motion_tx=0,stop_tx=0,can_tx=0)
    (directory/'frame_readback.json').write_text(json.dumps(result,indent=2),encoding='utf-8');return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();print(json.dumps(audit(a.directory),indent=2))
