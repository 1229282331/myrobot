"""Offline native ServoP/Stop transition from original SDK frames and receipts.

No device connections. Joint velocities below are finite differences of actual
positions, not an assumption about the SDK QDActual field's unit.
"""
import argparse
import base64
import csv
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'teleop-platform/upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC


def analyze(directory, sequence, output):
    audit = directory/'session/x5/audit'
    events = [json.loads(line) for line in gzip.open(audit/'events.jsonl.gz', 'rt', encoding='utf8')]
    first = next(e for e in events if e['event']=='input' and e['data'].get('seq')==sequence)
    start = first['at_ns']
    stop = next(e for e in events if e['event']=='stop_requested' and e['at_ns']>start)
    stop_time = stop['receipt']['send_completed_ns']
    commands = [e for e in events if e['event']=='motion_submitted' and start<=e['at_ns']<stop_time]
    confirmations = [e for e in events if e['event']=='motion_confirmed' and
                     e['target']['receipt']['sent_ns'] in {c['receipt']['sent_ns'] for c in commands}]
    stop_result = next(e for e in events if e['event']=='stop_result' and e['at_ns']>stop_time)
    next_input = next((e['at_ns'] for e in events if e['event']=='input' and
                      e['at_ns']>stop_time and e['data']['op']=='intent' and
                      e['data']['intent']['deadman'] and any(
                          e['data']['intent']['linear_m_s']+e['data']['intent']['angular_rad_s']+
                          e['data']['intent']['joint_rad_s'])), None)
    end_time=min(stop_time+2_000_000_000,next_input or stop_time+2_000_000_000)
    rows = []
    for line in gzip.open(audit/'raw.jsonl.gz', 'rt', encoding='utf8'):
        entry=json.loads(line)
        if not start-100_000_000<=entry['received_ns']<end_time: continue
        data=base64.b64decode(entry['raw_b64'])
        frame=np.frombuffer(data,dtype=MyType,count=1)[0]
        assert len(data)==1440 and frame['len']==1440 and frame['TestValue']==TEST_VALUE_MAGIC
        rows.append(dict(sequence=entry['sequence'],received_ns=entry['received_ns'],
                         relative_stop_ms=(entry['received_ns']-stop_time)/1e6,
                         q_actual=frame['QActual'].tolist(),q_target=frame['QTarget'].tolist(),
                         tool_pose=frame['ToolVectorActual'].tolist(),
                         mode=int(frame['RobotMode']),error=int(frame['ErrorStatus']),
                         collision=int(frame['CollisionState']),command_id=int(frame['CurrentCommandId'])))
    times=np.array([r['received_ns'] for r in rows])
    actual=np.array([r['q_actual'] for r in rows]);target=np.array([r['q_target'] for r in rows])
    poses=np.array([r['tool_pose'] for r in rows])
    before=int(np.searchsorted(times,stop_time))-1
    tail=np.flatnonzero(times>=stop_time)
    end=tail[-12:]
    r0=Rotation.from_euler('xyz',poses[0,3:],degrees=True)
    rotation=(r0.inv()*Rotation.from_euler('xyz',poses[:,3:],degrees=True)).as_rotvec(degrees=True)
    final_target=Rotation.from_quat(commands[-1]['candidate']['quaternion_xyzw'])
    # Use >=25 ms pairs to avoid treating one recv containing several frames
    # as an impossible zero-time physical velocity. Retain all original rows.
    slopes=[]
    for n in tail:
        j=int(np.searchsorted(times,times[n]-25_000_000,side='right'))-1
        if j>=0 and times[n]>times[j]:slopes.append((actual[n]-actual[j])/((times[n]-times[j])/1e9))
    result=dict(input_sequence=sequence,input=first,commands=commands,confirmations=confirmations,
        stop=stop,stop_result=stop_result,raw_frames=len(rows),
        observation_end_ns=end_time,next_active_input_ns=next_input,
        last_command_to_stop_send_ms=(stop_time-commands[-1]['receipt']['send_completed_ns'])/1e6,
        all_motion_confirmed=len(commands)==len(confirmations),
        last_target_relative_tool_rotation_deg=(r0.inv()*final_target).as_rotvec(degrees=True).tolist(),
        actual_at_stop_relative_tool_rotation_deg=rotation[before].tolist(),
        actual_final_relative_tool_rotation_deg=np.median(rotation[end],axis=0).tolist(),
        actual_post_stop_joint_excursion_deg=np.max(np.abs(actual[tail]-actual[before]),axis=0).tolist(),
        controller_target_post_stop_excursion_deg=np.max(np.abs(target[tail]-target[before]),axis=0).tolist(),
        actual_post_stop_joint_speed_25ms_deg_s=np.max(np.abs(slopes),axis=0).tolist(),
        observed_errors=sorted({(r['error'],r['collision']) for r in rows}),
        command_ids_after_stop=sorted({rows[n]['command_id'] for n in tail}),
        source_sha256={name:hashlib.sha256((audit/name).read_bytes()).hexdigest()
                       for name in ('raw.jsonl.gz','events.jsonl.gz')},
        method='Original SDK QActual/QTarget and ToolVectorActual; X5 same-host receive times. '
               '25ms-or-longer differences are observed average speeds, not certified peak speed. '
               'A post-Stop target change is observed; controller firmware causality is not established.')
    output.mkdir(parents=True,exist_ok=True)
    (output/'transition.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    (output/'frames.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
    with (output/'joint4.csv').open('w',newline='',encoding='utf8') as f:
        writer=csv.writer(f);writer.writerow(['sequence','relative_stop_ms','actual_j4_deg','controller_target_j4_deg','mode','command_id'])
        writer.writerows([r['sequence'],r['relative_stop_ms'],r['q_actual'][3],r['q_target'][3],r['mode'],r['command_id']] for r in rows)
    print(json.dumps({k:v for k,v in result.items() if k not in ('input','commands','confirmations','stop','stop_result')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--sequence',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    analyze(args.directory,args.sequence,args.output)
