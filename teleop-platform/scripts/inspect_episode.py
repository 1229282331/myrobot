"""Inspect original records without changing the participant's outcome label."""
import argparse
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'upstream/dobot-sdk')]
from dobot_sdk.protocol.data_types import MyType
from e6_phosphobot.episode_transfer import rows


def inspect(directory):
    raw=rows(directory/'audit/raw.jsonl');events=rows(directory/'audit/events.jsonl')
    decoded=[np.frombuffer(base64.b64decode(row['raw_b64']),dtype=MyType)[0] for row in raw]
    camera=rows(directory/'media/camera_samples.jsonl')
    cycles=[e for e in events if e['event']=='cycle'];inputs=[e for e in events if e['event']=='input']
    sources={}
    for name in ('rgb','depth'):
        frames=[row['sources'].get(name,{}) for row in camera]
        ids=[row.get('device_frame_id') for row in frames]
        ages=[row.get('age_ms') for row in frames if row.get('age_ms') is not None]
        sources[name]=dict(samples=len(frames),unique_ids=len(set(x for x in ids if x is not None)),
            invalid=sum(not r.get('fresh',False) for r in frames),
            consecutive_reuses=sum(a==b for a,b in zip(ids,ids[1:]) if a is not None),
            receive_age_ms=None if not ages else np.percentile(ages,[50,95,99,100]).tolist())
    commands=[e for e in events if e['event'] in ('motion_submitted','stop_requested') or
              e['event']=='can' and e['data'].get('kind')=='tx']
    result=dict(episode=directory.name,annotation=json.loads((directory/'summary.json').read_text(encoding='utf-8')),
        input_events=len(inputs),active_inputs=sum(bool(e['data'].get('intent',{}).get('deadman')) for e in inputs),
        device_sends=len(commands),raw_frames=len(raw),modes=dict(Counter(int(row['RobotMode']) for row in decoded)),
        alarms=sum(bool(row['ErrorStatus'] or row['CollisionState']) for row in decoded),
        joint_peak_to_peak_deg=np.ptp([row['QActual'] for row in decoded],axis=0).tolist() if decoded else None,
        actual_last_pose_mm_deg=decoded[-1]['ToolVectorActual'].tolist() if decoded else None,
        samples=len(cycles),cycle_duration_s=(cycles[-1]['at_ns']-cycles[0]['at_ns'])/1e9 if len(cycles)>1 else 0,
        camera=sources,clock='X5 receive monotonic; no exposure sync',
        source_quality_passed=bool(camera) and sources['rgb']['invalid']==0,
        complete_platform_acceptance=False)
    result['hashes']={str(path.relative_to(directory)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [directory/'audit/raw.jsonl',directory/'audit/events.jsonl',directory/'metadata.json',directory/'summary.json']}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();result=inspect(args.directory)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('episode','raw_frames','samples','device_sends','alarms','source_quality_passed')},ensure_ascii=False))
