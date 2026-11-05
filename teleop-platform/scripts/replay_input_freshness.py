"""Read-only replay of v278 audit input ages using verified clock samples."""
import argparse, gzip, json, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from e6_phosphobot.clock_probe import InputAgeBounds

p=argparse.ArgumentParser();p.add_argument('audit');p.add_argument('output');a=p.parse_args()
owners={};pending={};last_original={};latest={};rows=[];pauses=[];previous_reason=None
with gzip.open(a.audit,'rt',encoding='utf8') as f:
    for line in f:
        r=json.loads(line);event=r.get('event');owner=r.get('connection_id')
        if event=='clock_sample':owners.setdefault(owner,InputAgeBounds()).add(r['data'])
        elif event=='input':pending[(owner,r['data']['seq'])]=r['data']
        elif event=='input_processed':
            d=pending.pop((owner,r.get('sequence')),None)
            if not d or d.get('op')!='intent':continue
            now=r['processing_ns'];accepted=r.get('last_accepted_input_ns',0)
            was_accepted=accepted>last_original.get(owner,0)
            last_original[owner]=accepted
            bound=owners.setdefault(owner,InputAgeBounds()).bound(d['client_sample_ms'],now)
            proven=bound is not None and bound['upper_ms']<=250
            if was_accepted or proven:latest[owner]=now
            if not was_accepted:
                rows.append(dict(sequence=d['seq'],echo_age_ms=(now-d['echo_ns'])/1e6,age=bound,new_valid=proven))
        elif event=='cycle':
            reason=r['state'].get('reason')
            if reason=='input_stale' and previous_reason!='input_stale':
                pauses.append(dict(at_ns=r['at_ns'],counterfactual_latest_age_ms=(r['at_ns']-max(latest.values(),default=0))/1e6))
            previous_reason=reason
result=dict(rejections=len(rows),provably_fresh=sum(r['new_valid'] for r in rows),
            retained_rejections=sum(not r['new_valid'] for r in rows),pauses=pauses,rows=rows)
Path(a.output).write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
