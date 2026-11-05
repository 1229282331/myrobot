"""Audit the real workbench's QR/frame/clock report; never reads a device.

Nearest-rank quantiles match the UI. Includes startup, jitter and every exported
sample; the origin is a source receipt, never exposure or display photons.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def percentile(values, percent):
    ordered=sorted(values)
    return ordered[max(0,math.ceil(len(ordered)*percent/100)-1)] if ordered else None


def distribution(values):
    return dict(min=min(values),p50=percentile(values,50),p95=percentile(values,95),
                p99=percentile(values,99),max=max(values)) if values else None


def analyze(report):
    assert report['schema']==2, 'Requires explicit source basis and per-sample clock bounds'
    samples=report['samples'];windows=report['presentationWindows']
    assert samples and windows
    # The UI describes the active connection. The exported report deliberately
    # retains prior connections and failures; audit the complete history below.
    connection=report['summary'].get('connection')
    current=[s for s in samples if s.get('connection')==connection] if connection is not None else samples
    assert len(current)==report['summary']['samples'] and current
    assert report['attempts']==len(samples)+sum(report[k] for k in
        ('decodeFailures','unsynchronized','invalidBounds','truncated'))
    for s in samples:
        c=s['clock'];error=1+(s['presented']-c['startedAt'])*.001
        assert c['startedAt']<=c['at']<=s['presented']<=c['at']+3000
        assert c['lower']<=c['upper'] and s['basis'] in ('uvc_kernel_receipt','sdk_wait_return')
        assert abs(s['driftAndQuantizationMs']-error)<1e-6
        assert abs(s['lowerMs']-(s['presented']+c['lower']-s['sourceMs']-error))<1e-6
        assert abs(s['upperMs']-(s['presented']+c['upper']-s['sourceMs']+error))<1e-6
        assert abs(s['clockWidthMs']-(c['upper']-c['lower']+2*error))<1e-6
        assert math.isfinite(s['upperMs']) and s['upperMs']>=0
    for w in windows:
        assert w['endMs']>w['startMs'] and w['frames']>=0
        assert abs(w['fps']-w['frames']*1000/(w['endMs']-w['startMs']))<1e-6
    lower=distribution([s['lowerMs'] for s in samples]);upper=distribution([s['upperMs'] for s in samples])
    assert abs(distribution([s['lowerMs'] for s in current])['p95']-report['summary']['p95Lower'])<1e-6
    assert abs(distribution([s['upperMs'] for s in current])['p95']-report['summary']['p95Upper'])<1e-6
    keys=[(s['captureEpoch'],s['frameId'],s['sourceMs']) for s in samples]
    seconds=sum(w['endMs']-w['startMs'] for w in windows)/1000
    frames=sum(w['frames'] for w in windows)
    return dict(method=report['method'],quantile='nearest rank ceil(n*p)-1, identical to UI',
        measurement_scope='Source receipt → browser expectedDisplayTime; no exposure/panel timing claim',
        samples=len(samples),attempts=report['attempts'],
        displayed_connection=connection,displayed_connection_samples=len(current),
        counters={k:report[k] for k in ('decodeFailures','unsynchronized','invalidBounds','truncated','connections')},
        source_bases=dict(Counter(s['basis'] for s in samples)),capture_epochs=sorted(set(s['captureEpoch'] for s in samples)),
        source_time_backwards=sum(a['captureEpoch']==b['captureEpoch'] and b['sourceMs']<a['sourceMs']
                                  for a,b in zip(samples,samples[1:])),
        repeated_frame_samples=len(keys)-len(set(keys)),
        sample_span_s=(samples[-1]['presented']-samples[0]['presented'])/1000,
        lower_ms=lower,upper_ms=upper,clock_width_ms=distribution([s['clockWidthMs'] for s in samples]),
        presentation=dict(windows=len(windows),seconds=seconds,frames=frames,mean_fps=frames/seconds,
            window_fps=distribution([w['fps'] for w in windows]),windows_below_15fps=sum(w['fps']<15 for w in windows),
            playback_dropped_latest=windows[-1]['dropped']),
        measured_checks=dict(sampled_p95_upper_at_most_300ms=upper['p95']<=300,
            aggregate_at_least_15fps=frames/seconds>=15,all_exported_windows_at_least_15fps=all(w['fps']>=15 for w in windows)),
        exclusions='None. Startup windows, decode failures and clock failures reported; no failed samples removed.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze(json.loads(args.report.read_text(encoding='utf-8')))
    result.update(source_sha256=hashlib.sha256(args.report.read_bytes()).hexdigest(),
                  analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
