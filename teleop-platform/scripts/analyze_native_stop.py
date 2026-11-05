"""Compare native stop probes using original SDK feedback and the fixed criterion."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType


def analyze(path):
    summary=json.loads((path/'summary.json').read_text(encoding='utf-8'))
    frames=[json.loads(line) for line in (path/'frames.jsonl').read_text().splitlines()]
    rows=[np.frombuffer(base64.b64decode(r['wire_base64'])[-1440:],dtype=MyType)[0] for r in frames]
    times=np.array([r['received_ns'] for r in frames],dtype=np.int64)
    q=np.array([r['QActual'] for r in rows]);target=np.array([r['QTarget'] for r in rows])
    pose=np.array([r['ToolVectorActual'] for r in rows])
    release=summary['release_received_ns']
    start=np.searchsorted(times,release);lo=np.searchsorted(times,release+1_250_000_000)
    end=np.searchsorted(times,release+1_500_000_000)
    if end>=len(times):raise ValueError('Probe did not retain complete 1.5 second stop observation')
    rest=np.median(q[lo:end+1],axis=0);settled=None
    for i in range(start,lo):
        j=np.searchsorted(times,times[i]+250_000_000)
        if j<=end and np.max(abs(q[i:j+1]-rest))<=.01:
            settled=float((times[i]-release)/1e6);break
    modes=[float((ts-release)/1e6) for ts,r in zip(times,rows) if ts>=release and r['RobotMode']==5 and r['RunQueuedCmd']==0]
    trace=[]
    for ms in range(-100,1001,50):
        i=np.searchsorted(times,release+ms*1_000_000);r=rows[i]
        trace.append(dict(dt_ms=float((times[i]-release)/1e6),mode=int(r['RobotMode']),
            queued=int(r['RunQueuedCmd']),command_id=int(r['CurrentCommandId']),
            actual_deg=q[i].tolist(),target_deg=target[i].tolist(),actual_pose_mm_deg=pose[i].tolist(),
            target_pose_mm_deg=r['ToolVectorTarget'].tolist()))
    stop=next(c for c in summary['commands'] if c['command']=='Stop()')
    before_rotation=Rotation.from_euler('xyz',[
        summary['before']['status']['tool_vector_actual'][k] for k in ('rx','ry','rz')],degrees=True)
    final_rotation=Rotation.from_euler('xyz',pose[-1,3:],degrees=True)
    return dict(mode=summary['mode'],gain=summary.get('servo_gain',500),frames=len(frames),
        source_sha256=hashlib.sha256((path/'frames.jsonl').read_bytes()).hexdigest(),
        criterion='Within 0.01 deg of release+1.25..1.5s median for 250ms; unchanged',
        qualification_only=True,webpage_acceptance=False,settled_ms=settled,
        stop_send_ms=(stop['sent_ns']-release)/1e6,stop_ack_ms=(stop['reply_ns']-release)/1e6,
        mode5_empty_ms=modes[0] if modes else None,
        travel_after_release_mm=float(np.linalg.norm(pose[end,:3]-pose[start,:3])),
        joint_peak_to_peak_deg=np.ptp(q,axis=0).tolist(),
        final_pose_mm_deg=pose[-1].tolist(),alarms=sum(bool(r['ErrorStatus'] or r['CollisionState']) for r in rows),
        final_joint_deg=q[-1].tolist(),
        final_tool_rotation_delta_deg=(before_rotation.inv()*final_rotation).as_rotvec(degrees=True).tolist(),
        post_stop_actual_joint_excursion_deg=np.max(abs(q[start:]-q[start]),axis=0).tolist(),
        post_stop_controller_target_excursion_deg=np.max(abs(target[start:]-target[start]),axis=0).tolist(),
        last_second_joint_peak_to_peak_deg=np.ptp(q[times>=times[-1]-1_000_000_000],axis=0).tolist(),
        final_status={name:int(rows[-1][name]) for name in ('RobotMode','EnableStatus','ErrorStatus','CollisionState')},
        complete_stop_observation=True,max500_passed=settled is not None and settled<=500,
        max1000_passed=settled is not None and settled<=1000,
        trace=trace)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=analyze(a.directory)
    a.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2))
