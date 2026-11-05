"""Read retained wire feedback and receipts; never connect to a device.

Command-ID appearance is a feedback observation, not proof of completion.
Keep it separate from ACK and physical motion. All times use X5 receive clock.
"""
import argparse
import base64
from bisect import bisect_right
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC


def analyze(base):
    audit = base / 'session/x5/audit'
    groups = json.loads((base / 'analysis.json').read_text(encoding='utf-8'))['groups']
    starts = [g['start_ns'] for g in groups]
    output = [dict(start_ns=g['start_ns'], release_ns=g['release_ns'],
                   first_motion_ms=g['x5_input_to_position_change_ms'], commands=[], feedback=[])
              for g in groups]
    commands = {}
    with gzip.open(audit / 'events.jsonl.gz', 'rt', encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['event'] == 'motion_submitted':
                receipt = event['receipt']; stamp = receipt['sent_ns']
                index = bisect_right(starts, stamp) - 1
                if index < 0 or stamp >= groups[index]['release_ns']:continue
                moveit = event['candidate']['moveit']
                row = dict(sent_ns=stamp, sent_ms=(stamp-starts[index])/1e6,
                    computed_to_send_ms=(stamp-moveit['computed_ns'])/1e6,
                    received_to_send_ms=(stamp-moveit['host_received_ns'])/1e6,
                    command=receipt['command'], acknowledged=False, accepted=False)
                commands[stamp] = row;output[index]['commands'].append(row)
            elif event['event'] == 'motion_confirmed':
                receipt=event['target']['receipt'];row=commands.get(receipt['sent_ns'])
                if row is not None:
                    match=re.match(r'^0,\{(\d+)\},', receipt.get('reply',''))
                    row.update(acknowledged=receipt['acknowledged'], accepted=receipt['accepted'],
                        ack_ms=(receipt['reply_ns']-row['sent_ns'])/1e6,
                        command_id=int(match[1]) if match else None)
    with gzip.open(audit / 'raw.jsonl.gz', 'rt', encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line);stamp=event['received_ns'];index=bisect_right(starts,stamp)-1
            if index < 0 or stamp > groups[index]['release_ns']+300_000_000:continue
            raw=base64.b64decode(event['raw_b64']);frame=np.frombuffer(raw,dtype=MyType,count=1)[0]
            assert len(raw)==1440 and frame['len']==1440 and frame['TestValue']==TEST_VALUE_MAGIC
            assert np.all(np.isfinite(frame['QActual'])) and np.all(np.isfinite(frame['QTarget']))
            output[index]['feedback'].append(dict(received_ns=stamp,ms=(stamp-starts[index])/1e6,
                command_id=int(frame['CurrentCommandId']),q_actual_deg=frame['QActual'].tolist(),
                q_target_deg=frame['QTarget'].tolist()))
    gaps=[]
    for group in output:
        previous=None
        for row in group['commands']:
            row['previous_send_gap_ms']=None if previous is None else (row['sent_ns']-previous)/1e6
            if previous is not None:gaps.append(row['previous_send_gap_ms'])
            previous=row['sent_ns']
            seen=next((f for f in group['feedback'] if f['received_ns']>=row['sent_ns']
                       and f['command_id']==row.get('command_id')),None)
            row['first_id_seen_ms']=None if seen is None else (seen['received_ns']-row['sent_ns'])/1e6
    return dict(method='Same X5 monotonic receive clock; ID appearance is not target completion. '
        'Physical first motion uses unchanged >0.01 degree criterion from analysis.json.',
        source_sha256={name:hashlib.sha256((audit/name).read_bytes()).hexdigest()
                       for name in ('events.jsonl.gz','raw.jsonl.gz')},
        dispatch_gap_ms=dict(n=len(gaps),minimum=min(gaps) if gaps else None,
            below_50_ms=sum(g<50 for g in gaps),maximum=max(gaps) if gaps else None),groups=output)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=analyze(args.directory)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result['dispatch_gap_ms']))
