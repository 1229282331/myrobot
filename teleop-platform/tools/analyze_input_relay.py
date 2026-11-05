"""Compare intent durations on each clock; never infer one-way latency.

Usage: python analyze_input_relay.py EVENTS.jsonl.gz PC_RELAY.json OUT.json
"""
import gzip
import json
from pathlib import Path
import sys
import numpy as np


def stats(values):
    return dict(count=len(values), **({str(k):float(np.percentile(values,q))
        for k,q in [('p50',50),('p95',95),('p99',99),('max',100)]} if values else {}))


def analyze(events_path, pc_path):
    trace=json.loads(Path(pc_path).read_text(encoding='utf-8-sig'))
    pc={}
    for r in trace['rows']:
        if 'sequence' in r and 'client_sample_ms' in r:
            key=(r['sequence'],r['client_sample_ms'])
            if key in pc:raise ValueError('Ambiguous PC connection/sequence/sample join')
            pc[key]=r
    inputs=[];processed={};writes=[]
    opener=gzip.open if str(events_path).endswith('.gz') else open
    with opener(events_path,'rt',encoding='utf8') as source:
        for line in source:
            e=json.loads(line)
            if e['event']=='input' and e['data'].get('op')=='intent':inputs.append(e)
            if e['event']=='input_processed':processed[e['connection_id'],e['sequence']]=e
            if e['event'] in ('motion_submitted','stop_submitted','can_tx'):writes.append(e['event'])
    groups=[];active=[]
    for e in inputs:
        if e['data']['intent']['deadman']:active.append(e)
        elif active:
            groups.append(active+[e]);active=[]
    if active:raise ValueError('Active group has no observed release')
    runs=[]
    for group in groups:
        phases=[];area=dict(browser=0.,pc_receive=0.,pc_send=0.,x5_dequeue=0.,x5_apply=0.)
        send_ms=[];wait_ms=[];apply_ms=[]
        for a,b in zip(group,group[1:]):
            da,db=a['data'],b['data']
            pa,pb=pc[da['seq'],da['client_sample_ms']],pc[db['seq'],db['client_sample_ms']]
            xa,xb=processed[a['connection_id'],da['seq']],processed[b['connection_id'],db['seq']]
            if pa['connection_id']!=pb['connection_id'] or a['connection_id']!=b['connection_id']:
                raise ValueError('Connection changed during one active group')
            if pa['outcome']!='sent' or pb['outcome']!='sent':raise ValueError('Uncompleted PC send')
            dt=dict(browser=(db['client_sample_ms']-da['client_sample_ms'])/1000,
                    pc_receive=(pb['pc_received_ns']-pa['pc_received_ns'])/1e9,
                    pc_send=(pb['pc_send_finished_ns']-pa['pc_send_finished_ns'])/1e9,
                    x5_dequeue=(b['at_ns']-a['at_ns'])/1e9,
                    x5_apply=(xb['processing_ns']-xa['processing_ns'])/1e9)
            if min(dt.values())<0:raise ValueError('Nonmonotonic timestamps within one clock')
            velocity=da['intent']['linear_m_s'][2]*1000
            for k,v in dt.items():area[k]+=velocity*v
            if not phases or phases[-1]['velocity_mm_s']!=velocity:
                phases.append(dict(sequence=da['seq'],velocity_mm_s=velocity,duration_s={k:0. for k in dt}))
            for k,v in dt.items():phases[-1]['duration_s'][k]+=v
            send_ms.append((pa['pc_send_finished_ns']-pa['pc_received_ns'])/1e6)
            wait_ms.append((xa['processing_ns']-a['at_ns'])/1e6)
            apply_ms.append((xa['at_ns']-xa['processing_ns'])/1e6)
        names=list(area)
        runs.append(dict(first_sequence=group[0]['data']['seq'],last_sequence=group[-1]['data']['seq'],
            input_count=len(group)-1,phases=phases,z_area_mm=area,
            area_change_mm={a+'_to_'+b:area[b]-area[a] for a,b in zip(names,names[1:])},
            absolute_phase_duration_error_ms=stats([abs(p['duration_s']['x5_dequeue']-p['duration_s']['pc_send'])*1000 for p in phases]),
            pc_receive_through_send_ms=stats(send_ms),x5_lock_wait_ms=stats(wait_ms),x5_apply_ms=stats(apply_ms)))
    return dict(runs=runs,pc_total=trace['total'],pc_overwritten=trace['overwritten'],
                device_write_events=writes,
                interpretation='Duration/velocity integrals computed separately on each clock; area differences are duration distortions, not absolute one-way latency. X5 dequeue occurs after its WebSocket receive queue.')


if __name__=='__main__':
    result=analyze(*sys.argv[1:3])
    Path(sys.argv[3]).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({**result,'runs':[{k:v for k,v in r.items() if k!='phases'} for r in result['runs']]},indent=2))
