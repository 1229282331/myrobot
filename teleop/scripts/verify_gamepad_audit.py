"""Summarize both real 2.4G input sessions; this never sends device commands."""
import hashlib
import json
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[2]
files=[root/'datasets/e6_teleop/input_audit'/name for name in ('33752485696100.json','34418849522200.json')]
sessions=[json.loads(p.read_text(encoding='utf-8')) for p in files]
first,second=sessions
inputs=[x for x in first if x['operation']=='input' and x['input'].get('source')=='gamepad' and x['input'].get('connected')]
active=[x for x in inputs if x['input']['deadman']]
fine=[x for x in active if x['input']['fine']]
ranges=np.array([x['input']['twist'] for x in active])
assert all(ranges.min(axis=0)<0) and all(ranges.max(axis=0)>0)
assert {x['input']['mode'] for x in active}=={'rotation','translation'}
assert {x['input']['frame'] for x in active}=={'base','tool'}
assert fine and all(max(abs(v) for v in x['input']['twist'][:3])<=.0025+1e-9 for x in fine)
assert min(x['input']['gripper_rate'] for x in active)==-.1 and max(x['input']['gripper_rate'] for x in active)==.1
back=[(i,x) for i,x in enumerate(second) if x['operation']=='input' and x['input'].get('stop') and x['input'].get('raw',{}).get('buttons',[0]*9)[8]>.5]
assert back and all(not x['status']['armed'] for _,x in back)
transitions=[];previous=True
for i,x in enumerate(second):
    if x['operation']!='input':continue
    value=x['input'].get('connected')
    if value!=previous:transitions.append(dict(index=i,connected=value,pc_ns=x['pc_monotonic_ns'],armed=x['status']['armed']))
    previous=value
disconnected=next(x for x in transitions if not x['connected'])
reconnected=next(x for x in transitions if x['connected'] and x['index']>disconnected['index'])
arm_index=next(i for i,x in enumerate(second) if i>reconnected['index'] and x['operation']=='arm')
assert all(not x['status']['armed'] and not (x.get('effective_action') or {}).get('active') for x in second[disconnected['index']:arm_index])
hold_deltas=[]
for a,b in zip(inputs,inputs[1:]):
    if not a['input']['deadman'] and not b['input']['deadman'] and a.get('effective_action') and b.get('effective_action'):
        hold_deltas.append(abs(a['effective_action']['gripper']-b['effective_action']['gripper']))
assert hold_deltas and max(hold_deltas)<1e-9
report=dict(raw_files=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),samples=len(s)) for p,s in zip(files,sessions)],
            user_reports=['验收完成','补测完成'],connection='2.4G receiver, browser standard XInput',bluetooth_tested=False,
            connected_gamepad_samples=len(inputs),active_samples=len(active),fine_samples=len(fine),
            axis_ranges=np.stack([ranges.min(axis=0),ranges.max(axis=0)],axis=1).tolist(),
            modes=['translation','rotation'],coordinate_frames=['base','tool'],gripper_rate_range=[-.1,.1],
            release_hold_max_gripper_target_delta=max(hold_deltas),back_stop_samples=len(back),
            disconnect_duration_s=(reconnected['pc_ns']-disconnected['pc_ns'])/1e9,
            disconnect_index=disconnected['index'],reconnect_index=reconnected['index'],explicit_arm_index=arm_index,
            reconnect_remained_stopped=True,hardware_motion=False)
output=root/'docs/evidence/teleop_observation_20261004/gamepad_live_validation.json'
output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
