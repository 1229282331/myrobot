"""Summarize all retained presentation samples, preserving failed intervals."""
import argparse
import hashlib
import json
from pathlib import Path
from analyze_camera_probe import distribution


def analyze(path,episode=None):
    report=json.loads(path.read_text(encoding='utf-8'))
    samples=report['samples']
    if samples and isinstance(samples[0],list):samples=[dict(zip(report['sample_columns'],s)) for s in samples]
    def stats(rows):
        return dict(samples=len(rows),lower_ms=distribution([s['lowerMs'] for s in rows]),
                    upper_ms=distribution([s['upperMs'] for s in rows]),
                    clock_width_ms=distribution([s['clockWidthMs'] for s in rows]),
                    first_source_ms=rows[0]['sourceMs'] if rows else None,
                    last_source_ms=rows[-1]['sourceMs'] if rows else None,
                    presented_duration_s=(rows[-1]['presented']-rows[0]['presented'])/1000 if len(rows)>1 else 0)
    result=dict(method=report['method'],all=stats(samples),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                counts={k:report[k] for k in ('attempts','decodeFailures','unsynchronized','invalidBounds','truncated')})
    if episode:
        metadata=json.loads((episode/'metadata.json').read_text(encoding='utf-8'))
        summary=json.loads((episode/'summary.json').read_text(encoding='utf-8'))
        start=metadata['x5_requested_ns']/1e6;end=summary['x5_finished_ns']/1e6
        result.update(recording_source_interval_ms=[start,end],
                      during_recording=stats([s for s in samples if start<=s['sourceMs']<=end]),
                      after_recording_including_transfer_and_idle=stats([s for s in samples if s['sourceMs']>end]),
                      phase_note='X5 source receipt selects recording interval. Post-recording includes bundle, transfer and idle; no PC/X5 direct subtraction.')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('report',type=Path)
    parser.add_argument('--episode',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=analyze(args.report,args.episode)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
