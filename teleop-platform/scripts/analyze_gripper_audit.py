"""Read original CAN events; never infer a gripper action from arm state."""
import argparse
import json
from pathlib import Path
import statistics


def analyze(path):
    rows=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    frames=[r for r in rows if r['event']=='gripper_feedback']
    wire=[r['data'] for r in rows if r['event']=='can']
    holds=[];current=[]
    for row in frames:
        if current and row['target']!=current[-1]['target']:
            holds.append(current);current=[]
        current.append(row)
    if current:holds.append(current)
    groups=[];active=None
    for row in rows:
        if row['event']!='input' or row['data']['op']!='intent':continue
        intent=row['data']['intent'];moving=intent['deadman'] and intent['gripper_rate_s']!=0
        if moving and active is None:
            active=dict(start_ns=row['at_ns'],first_sequence=row['data']['seq'],rate_s=intent['gripper_rate_s'])
        elif not moving and active is not None:
            active.update(release_ns=row['at_ns'],release_sequence=row['data']['seq'])
            groups.append(active);active=None
    tx=[r for r in wire if r['kind']=='tx']
    for index,group in enumerate(groups):
        start=group['start_ns'];release=group['release_ns']
        end=min(release+1_000_000_000,groups[index+1]['start_ns'] if index+1<len(groups) else release+1_000_000_000)
        before=[r for r in tx if r['host_perf_counter_ns']<start]
        during=[r for r in tx if start<=r['host_perf_counter_ns']<release]
        after=[r for r in tx if release<=r['host_perf_counter_ns']<end]
        group['duration_ms']=(release-start)/1e6
        group['initial_sent_raw']=bytes.fromhex(before[-1]['data_hex'])[1] if before else None
        group['final_sent_raw']=bytes.fromhex(during[-1]['data_hex'])[1] if during else group['initial_sent_raw']
        group['changed_target_after_release']=[r['host_perf_counter_ns'] for r in after
            if bytes.fromhex(r['data_hex'])[1]!=group['final_sent_raw']]
        tail=[r for r in frames if end-250_000_000<=r['feedback']['host_perf_counter_ns']<end]
        group['tail_samples']=len(tail)
        group['tail_actual_raw']=[r['feedback']['position_raw'] for r in tail]
        group['tail_arrived']=bool(len(tail)>=3 and group['final_sent_raw'] is not None and all(
            r['feedback']['state']==0 and abs(r['feedback']['position_raw']-group['final_sent_raw'])<=1 for r in tail))
    endpoints=[]
    for group in holds:
        duration=(group[-1]['at_ns']-group[0]['at_ns'])/1e9
        if duration<3:continue
        tail=[r for r in group if group[-1]['at_ns']-r['at_ns']<=1_000_000_000]
        target=round(group[-1]['target']*255)
        positions=[r['feedback']['position_raw'] for r in tail]
        endpoints.append(dict(target_percent=group[-1]['target']*100,target_raw=target,
            duration_s=duration,last_second_actual_raw_range=[min(positions),max(positions)],
            last_second_actual_percent_median=statistics.median(positions)*100/255,
            last_second_max_error_counts=max(abs(v-target) for v in positions),
            last_second_states=sorted(set(r['feedback']['state'] for r in tail))))
    return dict(feedback_frames=len(frames),can_tx=sum(r['kind']=='tx' for r in wire),
        can_rx=sum(r['kind']=='rx' for r in wire),
        faults=[r for r in rows if r['event']=='gripper_fault'],
        state_counts={str(s):sum(r['feedback']['state']==s for r in frames) for s in range(4)},
        max_can_rtt_ms=max(r['feedback']['round_trip_ms'] for r in frames) if frames else None,
        stable_holds=endpoints,input_groups=groups,incomplete_input=active,
        scope='Formal-page gripper-only inputs; endpoint mismatches retained; not full-range acceptance')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('events',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();result=analyze(a.events)
    with a.output.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,ensure_ascii=False)
    print(json.dumps(result,indent=2,ensure_ascii=False))
