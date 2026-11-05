"""Read immutable probe captures; never rewrite a capture or its derived summary."""
import argparse
import hashlib
import json
from pathlib import Path


def distribution(values):
    values=sorted(values)
    if not values:return None
    def quantile(p):
        pos=(len(values)-1)*p;lower=int(pos);upper=min(lower+1,len(values)-1)
        return values[lower]+(values[upper]-values[lower])*(pos-lower)
    return dict(count=len(values),p50=quantile(.5),p95=quantile(.95),p99=quantile(.99),maximum=values[-1])


def analyze(root):
    def rows(name):return [json.loads(line) for line in (root/name).read_text().splitlines() if line.strip()]
    calls=rows('calls.jsonl');pts=rows('pts.jsonl')
    state=json.loads((root/'summary.json').read_text())
    collected=json.loads((root/'collected_summary.json').read_text())
    names=['camera_probe.py','summary.json','collected_summary.json','calls.jsonl','pts.jsonl','uvc_metadata.jsonl','camera.log']
    hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names}
    frames=[frame for call in calls for frame in call.get('frames',[])]
    sources={}
    for stream in ('color','depth'):
        selected=[f for f in frames if f['stream']==stream]
        received=[f['host_perf_counter_ns'] for f in selected]
        gaps=[(b-a)/1e6 for a,b in zip(received,received[1:])]
        missing=sum(max(0,b['index']-a['index']-1) for a,b in zip(selected,selected[1:]))
        backwards=sum(b['index']<=a['index'] for a,b in zip(selected,selected[1:]))
        sources[stream]=dict(frames=len(selected),fps=len(selected)/state['elapsed_s'],
                            receive_gap_ms=distribution(gaps),missing_source_ids=missing,nonincreasing_ids=backwards)
    writes=[w for c in calls for w in c.get('status_writes',[])]
    longest=sorted(calls,key=lambda c:c.get('consumer_duration_ns',0),reverse=True)[:8]
    return dict(capture_directory=str(root),capture=state,sha256=hashes,
                collected_sha_matches_snapshot=collected.get('probe_sha256')==hashes['camera_probe.py'],
                sha_note='Deployed local camera_probe.py snapshot is authoritative. Original collected summary retained.',
                resources_after=collected.get('resources_after'),sources=sources,
                read_ms=distribution([c['read_duration_ns']/1e6 for c in calls if 'read_duration_ns' in c]),
                read_cpu_ms=distribution([c['read_cpu_ns']/1e6 for c in calls if 'read_cpu_ns' in c]),
                consumer_ms=distribution([c['consumer_duration_ns']/1e6 for c in calls if 'consumer_duration_ns' in c]),
                between_calls_ms=distribution([c['consumer_gap_ns']/1e6 for c in calls]),
                status_write_ms=distribution([w['duration_ns']/1e6 for w in writes]),
                pts_errors=[p for p in pts if 'error' in p],call_errors=[c for c in calls if 'error' in c],
                longest_consumers=longest)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=analyze(args.root)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('longest_consumers','sha256')},indent=2))
