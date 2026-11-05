"""Direction of continuous shadow joint targets, not real joint motion.

Absolute target minus a static shadow plant is not a velocity after a reversal.
Compare successive target samples only when their complete provenance windows
contain the same joint intent. Retain every transition/duplicate explicitly.
Input identity must be audited separately against the original input journal.
"""
import argparse
import gzip
import json
import math
from pathlib import Path


def window_direction(candidate):
    directions = set()
    for source in candidate['trajectory_sources']:
        record = source.get('input')
        if source['kind'] != 'j' or not record:
            return None
        intent = record['intent']
        joint = intent['joint_rad_s']
        if not intent['deadman'] or any(intent['linear_m_s'] + intent['angular_rad_s']):
            return None
        direction = tuple(0 if value == 0 else (1 if value > 0 else -1) for value in joint)
        # This diagnostic covers the workbench's single selected joint. It
        # must not treat the sign of a multi-axis velocity as its vector ratio.
        if sum(value != 0 for value in direction) != 1:
            return None
        directions.add(direction)
    return next(iter(directions)) if len(directions) == 1 else None


def compare(previous, current):
    row = dict(id=current['id'],epoch=current['epoch'],sample_ros_ns=current['sample_ros_ns'],
               joints_rad=current['joints_rad'],classification='initial')
    if previous is None or previous['epoch'] != current['epoch']:
        return row
    delta = [a-b for a,b in zip(current['joints_rad'],previous['joints_rad'])]
    row.update(previous_id=previous['id'],delta_rad=delta,
               sample_interval_ns=current['sample_ros_ns']-previous['sample_ros_ns'])
    direction = window_direction(current)
    if row['sample_interval_ns'] < 0:
        row['classification'] = 'time_regression'
    elif direction is None or direction != window_direction(previous):
        row['classification'] = 'transition_window'
    elif math.sqrt(sum(x*x for x in delta)) <= 1e-12:
        row['classification'] = 'unchanged_target'
    else:
        norm = math.sqrt(sum(x*x for x in delta)*sum(x*x for x in direction))
        cosine = sum(a*b for a,b in zip(delta,direction))/norm
        row.update(classification='steady_direction',direction=direction,cosine=cosine,
                   passed=cosine > .97)
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('events',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    opener = gzip.open if args.events.suffix == '.gz' else open
    seen = set(); previous = None; rows = []
    with opener(args.events,'rt',encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['event'] != 'cycle':continue
            candidate = (event['state'].get('candidate') or {}).get('moveit')
            if not candidate or candidate['id'] in seen:continue
            seen.add(candidate['id'])
            rows.append(compare(previous,candidate)); previous = candidate
    stable = [row for row in rows if row['classification'] == 'steady_direction']
    result = dict(scope='Successive shadow joint targets only; not actual motion, latency, or source identity qualification.',
        candidates=len(rows),classifications={name:sum(r['classification']==name for r in rows)
            for name in sorted({r['classification'] for r in rows})},
        checked_directions=sorted({tuple(r['direction']) for r in stable}),
        steady_direction_passed=bool(stable) and all(r['passed'] for r in stable),
        has_time_regression=any(r['classification']=='time_regression' for r in rows),rows=rows)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
    return int(not result['steady_direction_passed'] or result['has_time_regression'])


if __name__=='__main__':raise SystemExit(main())
