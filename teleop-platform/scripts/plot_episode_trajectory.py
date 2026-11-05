"""Plot an offline trajectory-review export. No robot or network connection."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def plot(directory, destination):
    directory, destination = Path(directory).resolve(), Path(destination).resolve()
    manifest = json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('schema') != 'e6-offline-trajectory-review-v1' or manifest.get('physical_replay_allowed') is not False:
        raise ValueError('Expected an offline review export')
    allowed = {'raw-feedback.jsonl', 'cycle-observations.jsonl', 'command-events.jsonl', 'sent-arm-targets.jsonl'}
    if set(manifest['export_files']) != allowed:
        raise ValueError('Unexpected export files')
    for name, digest in manifest['export_files'].items():
        if hashlib.sha256((directory/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Export changed: '+name)
    if destination.exists():
        raise FileExistsError('Plot already exists')
    raw = read_rows(directory/'raw-feedback.jsonl')
    cycles = read_rows(directory/'cycle-observations.jsonl')
    sends = read_rows(directory/'sent-arm-targets.jsonl')
    origin = manifest['review']['origin_ns']
    seconds = lambda ns: (ns-origin)/1e9
    times = np.array([seconds(row['received_ns']) for row in raw])
    joints = np.rad2deg(np.array([row['joints_rad'] for row in raw]))
    initial = joints[0].copy()
    broken = [index for index, row in enumerate(raw) if index and not row['continuous_from_previous']]
    joints[broken] = np.nan
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, constrained_layout=True)
    colors = plt.get_cmap('tab10').colors
    for index in range(6):
        axes[0].plot(times, joints[:, index]-initial[index], color=colors[index], lw=1.1, label=f'J{index+1} actual')
        accepted = [row for row in sends if row['acknowledged_accepted'] and row['joints_rad'] is not None]
        if accepted:
            axes[0].scatter([seconds(row['sent_ns']) for row in accepted],
                            [np.rad2deg(row['joints_rad'][index])-initial[index] for row in accepted],
                            color=colors[index], marker='x', s=10)
    axes[0].set_ylabel('Joint delta [deg]')
    axes[0].set_title('Solid: original SDK feedback; x: acknowledged sent target (no interpolation)')
    axes[0].legend(ncol=6, fontsize=8)
    tcp, grip, age, ct = [], [], [], []
    for row in cycles:
        actual = row.get('actual') or {}
        position = actual.get('position_m')
        tcp.append(position if row['arm_observation_verified'] and position is not None else [np.nan]*3)
        value = actual.get('gripper') if row['gripper_observation_valid'] else None
        grip.append(np.nan if value is None else value*100)
        received = row['feedback_identity'].get('received_ns')
        age.append(np.nan if received is None else (row['at_ns']-received)/1e6)
        ct.append(seconds(row['at_ns']))
    if cycles:
        tcp = np.array(tcp, dtype=float)*1000
        finite = np.flatnonzero(np.isfinite(tcp).all(axis=1))
        baseline = tcp[finite[0]].copy() if len(finite) else np.zeros(3)
        # Break at missing intervals instead of drawing through absent samples.
        for index in range(1, len(ct)):
            if ct[index]-ct[index-1] > .1:
                tcp[index] = np.nan
                grip[index] = np.nan
        for index, label in enumerate('XYZ'):
            axes[1].plot(ct, tcp[:, index]-baseline[index], label=label, lw=1.2)
        axes[1].legend(ncol=3, fontsize=8)
        axes[2].plot(ct, grip, lw=1.2)
        axes[3].plot(ct, age, lw=1)
        if not np.isfinite(grip).any():
            axes[2].text(.5, .5, 'No valid gripper feedback in this recording',
                         transform=axes[2].transAxes, ha='center', va='center', color='#555555')
    axes[1].set_ylabel('Recorded TCP delta [mm]')
    axes[1].set_title('Original recorded TCP configuration; not recalculated with the current model')
    axes[2].set_ylabel('Native opening [%]')
    axes[2].set_title('Raw / 255; not linear finger distance or usable-stroke percentage')
    axes[3].set_ylabel('Feedback age [ms]')
    axes[3].set_xlabel('Original X5 monotonic time from first retained raw frame [s]')
    for axis in axes:
        axis.grid(alpha=.2)
    fig.suptitle(f"ORIGINAL RECORDING — {manifest['episode']}\nRecorded outcome: {manifest['result']} · offline review only · physical replay not qualified", fontsize=14)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open('xb') as output:
        fig.savefig(output, format='png', dpi=140)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    plot(args.directory, args.destination)
