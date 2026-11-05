"""Read-only analysis of exported standard receiver/video presentation counters."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def quantiles(values):
    values=sorted(values)
    return {key:values[max(0,math.ceil(len(values)*p)-1)]
        for key,p in [('p50',.5),('p95',.95),('p99',.99),('max',1)]} if values else None


def analyze(report):
    stats=report.get('receiverStats',[])
    intervals=[]
    for previous,current in zip(stats,stats[1:]):
        if previous['connection']!=current['connection']:continue
        a,b=previous['values'],current['values']
        seconds=(current['atMs']-previous['atMs'])/1000
        if seconds<=0:continue
        delta={key:b[key]-a[key] for key in b.keys()&a.keys()
            if key not in ('jitter',)}
        if any(delta.get(key,0)<0 for key in ('framesDecoded','framesReceived','bytesReceived')):continue
        row=dict(start_ms=previous['atMs'],end_ms=current['atMs'],connection=current['connection'],
            packets_lost=delta.get('packetsLost'),
            decoder=current.get('decoder',{}))
        if 'jitter' in b:row['jitter_ms']=b['jitter']*1000
        for key,label in [('framesDecoded','decode_fps'),('framesReceived','receive_fps')]:
            if key in delta:row[label]=delta[key]/seconds
        for time,count,label in [('totalDecodeTime','framesDecoded','decode_ms_per_frame'),
            ('totalProcessingDelay','framesDecoded','processing_ms_per_frame'),
            ('jitterBufferDelay','jitterBufferEmittedCount','buffer_ms_per_frame')]:
            if time in delta and delta.get(count,0)>0:row[label]=delta[time]*1000/delta[count]
        intervals.append(row)
    windows=report.get('presentationWindows',[])
    samples=report.get('samples',[])
    return dict(method='All counter intervals within each connection; no slow-window filtering. Decode/processing times are browser pipeline counters, not measured CPU execution time.',
        counter_intervals=len(intervals),summary={key:quantiles([r[key] for r in intervals if key in r])
            for key in ('receive_fps','decode_fps','jitter_ms','decode_ms_per_frame','processing_ms_per_frame','buffer_ms_per_frame')},
        packet_loss_changes=sum(r['packets_lost'] for r in intervals if r['packets_lost'] is not None)
            if any(r['packets_lost'] is not None for r in intervals) else None,
        presentation_fps=quantiles([w['fps'] for w in windows]),presentation_windows=len(windows),
        presentation_windows_under_15=sum(w['fps']<15 for w in windows),
        latency_lower_ms=quantiles([s['lowerMs'] for s in samples]),latency_upper_ms=quantiles([s['upperMs'] for s in samples]),
        pixel_samples=len(samples),receiver_failures=report.get('receiverFailures'),
        receiver_types=report.get('receiverTypes'),intervals=intervals)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('report',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();raw=args.report.read_bytes()
    result=analyze(json.loads(raw));result['input_sha256']=hashlib.sha256(raw).hexdigest()
    with args.output.open('x',encoding='utf-8') as output:json.dump(result,output,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='intervals'},indent=2))
