"""Offline release metrics from immutable page events and official raw feedback.

Never associate a delayed Stop with a later idle heartbeat. All release edges
are retained. No synchronization of clocks across hosts is assumed.
"""
import argparse
import base64
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType


def analyze(path):
    events=[json.loads(x) for x in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    raw=[json.loads(x) for x in (path/'raw.jsonl').read_text().splitlines()]
    times=np.array([r['received_ns'] for r in raw],dtype=np.int64)
    rows=[np.frombuffer(base64.b64decode(r['raw_b64']),dtype=MyType)[0] for r in raw]
    q=np.array([r['QActual'] for r in rows]);xyz=np.array([r['ToolVectorActual'][:3] for r in rows])
    commands={}
    for event in events:
        state=event.get('state',{})
        target=event.get('target')
        if not isinstance(target,dict):target={}
        for item in [event.get('receipt'),state.get('receipt'),state.get('pending'),
                     (state.get('accepted') or {}).get('receipt'),target.get('receipt')]:
            if item and 'sent_ns' in item:commands.setdefault(item['sent_ns'],{}).update(item)
    inputs=[e for e in events if e['event']=='input']
    releases=[];previous=False;last_active=None
    for event in inputs:
        intent=event['data'].get('intent')
        if intent is None:continue
        held=intent['deadman']
        if held:last_active=event['at_ns']
        if previous and not held:releases.append(event['at_ns'])
        previous=held
    result=[]
    for release in releases:
        next_active=next((e['at_ns'] for e in inputs if e['at_ns']>release and e['data'].get('intent',{}).get('deadman')),None)
        limit=next_active or int(times[-1])
        subsequent=[c for t,c in sorted(commands.items()) if release<=t<limit]
        stop=next((c for c in subsequent if c['command']=='Stop()'),None)
        decel=next((c for c in subsequent if c['command'].startswith(('ServoP(','ServoJ('))),None)
        start=int(np.searchsorted(times,release));end=int(np.searchsorted(times,release+1_500_000_000))
        entry=dict(release_ns=release,stop=None if stop is None else dict(
            sent_delay_ms=(stop['sent_ns']-release)/1e6,ack_ms=None if 'reply_ns' not in stop else
            (stop['reply_ns']-stop['sent_ns'])/1e6,accepted=stop.get('accepted',False)),
            first_deceleration_ms=None if decel is None else (decel['sent_ns']-release)/1e6,
            settled_ms=None,travel_after_release_mm=None,complete_observation=False)
        # Same criterion as the native failure, including the same rest window.
        lo=int(np.searchsorted(times,release+1_250_000_000))
        if end<len(times) and end>lo and limit>release+1_500_000_000:
            entry['complete_observation']=True
            rest=np.median(q[lo:end+1],axis=0)
            for i in range(start,lo):
                j=int(np.searchsorted(times,times[i]+250_000_000))
                if j<=end and np.max(np.abs(q[i:j+1]-rest))<=.01:
                    entry['settled_ms']=(times[i]-release)/1e6;break
            entry['travel_after_release_mm']=float(np.linalg.norm(xyz[end]-xyz[start]))
        result.append(entry)
    return dict(criterion='Within 0.01 deg of release+1.25..1.5s median for 250ms; original criterion unchanged',
                release_clock='X5 input receive monotonic',frames=len(raw),releases=result,
                warning='Deceleration submission is not Stop submission; both times are retained',
                counts=dict(motion=sum(c['command'].startswith(('ServoP(','ServoJ(')) for c in commands.values()),
                            stop=sum(c['command']=='Stop()' for c in commands.values())))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runtime',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();result=analyze(args.runtime)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
