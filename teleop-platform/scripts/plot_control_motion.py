"""Plot saved synthetic qualification motion, with real observation timestamps.

Reads only saved files. Does not connect to a robot. Feedback in these fixtures
is synthetic; these plots cannot establish physical tracking or J5 stability.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


def segments(rows):
    """A release/restart must not be plotted as continuous commanded motion."""
    result = []
    for row in rows:
        if not result or result[-1][-1]["row"]["epoch"] != row["row"]["epoch"]:
            result.append([])
        result[-1].append(row)
    return result


def plot_case(path, destination):
    summary = json.loads((path / "summary.json").read_text(encoding="utf8"))
    assert summary["synthetic"] and summary["physical_io"] is False
    rows = json.loads((path / "writes.json").read_text(encoding="utf8"))
    if not rows:
        return {"writes": 0, "scope": "synthetic; no physical motion"}
    origin = rows[0]["at_ns"]
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    joint, speed, translation, rotation = axes.flat
    maximum_step = np.zeros(6)
    maximum_speed = np.zeros(6)
    maximum_feedback_error = np.zeros(6)
    maximum_position_span = np.zeros(3)
    maximum_orientation_span = 0.
    boundaries = []
    for number, group in enumerate(segments(rows)):
        at = np.array([x["at_ns"] for x in group], dtype=np.int64)
        observed_at = np.array(
            [x["row"]["pre_action_received_ns"] for x in group], dtype=np.int64)
        q = np.array([x["q"] for x in group])
        measured = np.rad2deg(
            [x["row"]["cpp"]["actual_q_rad"] for x in group])
        target = np.array([x["row"]["joints_rad"] for x in group])
        assert q.shape[1] == measured.shape[1] == 6
        # q is the actual serialized command; full precision solver output is
        # only used to verify that plotting did not select a different target.
        assert np.max(np.abs(q - np.rad2deg(target))) <= .00000051
        assert np.all(np.diff(at) > 0)
        t = (at - origin) / 1e9
        tf = (observed_at - origin) / 1e9
        joint.plot(t, q[:, 4], color="#1769aa", lw=1,
                   label="Serialized J5 command" if number == 0 else None)
        joint.plot(tf, measured[:, 4], color="#c45339", lw=.8,
                   label="Synthetic J5 feedback (sample time)" if number == 0 else None)
        maximum_feedback_error = np.maximum(
            maximum_feedback_error, np.max(np.abs(q - measured), axis=0))
        if len(group) > 1:
            step = np.diff(q, axis=0)
            velocity = step / (np.diff(at)[:, None] / 1e9)
            maximum_step = np.maximum(maximum_step, np.max(np.abs(step), axis=0))
            maximum_speed = np.maximum(maximum_speed, np.max(np.abs(velocity), axis=0))
            for axis in range(6):
                speed.plot(t[1:], velocity[:, axis], lw=1 if axis == 4 else .65,
                           alpha=1 if axis == 4 else .7,
                           color=f"C{axis}", label=f"J{axis+1}" if number == 0 else None)
        position = np.array([x["row"]["position_m"] for x in group])
        delta = 1000 * (position - position[0])
        rotations = Rotation.from_quat([x["row"]["quaternion_xyzw"] for x in group])
        relative = rotations[0].inv() * rotations
        rotvec = relative.as_rotvec(degrees=True)
        maximum_position_span = np.maximum(maximum_position_span, np.ptp(delta, axis=0))
        maximum_orientation_span = max(
            maximum_orientation_span, float(np.rad2deg(relative.magnitude()).max()))
        for axis in range(3):
            label = "xyz"[axis] if number == 0 else None
            translation.plot(t, delta[:, axis], color=f"C{axis}", lw=.9, label=label)
            rotation.plot(t, rotvec[:, axis], color=f"C{axis}", lw=.9, label=label)
        boundaries.append({
            "epoch": group[0]["row"]["epoch"],
            "start_s": float(t[0]), "end_s": float(t[-1]), "writes": len(group)})
    failures = summary["dispatch_rejections"] + summary["solver_barriers"]
    budget_rows = [x for x in rows
                   if x["row"]["cpp"].get("collision_retreat", {}).get("budget_exhausted")]
    for ax in axes.flat:
        for index, event in enumerate(failures):
            ax.axvline((event["at_ns"] - origin) / 1e9, color="#9c167c",
                       linestyle="--", lw=.8,
                       label="Direction rejected" if index == 0 else None)
        for index, event in enumerate(budget_rows):
            ax.axvline((event["at_ns"] - origin) / 1e9, color="#e69100",
                       linestyle=":", lw=1.2,
                       label="Retreat budget fallback" if index == 0 else None)
        for previous, current in zip(boundaries, boundaries[1:]):
            ax.axvspan(previous["end_s"], current["start_s"],
                       color="#999999", alpha=.12)
        ax.set_xlabel("Seconds from first synthetic write")
        ax.grid(alpha=.18)
        ax.legend(fontsize=7)
    joint.set(title="J5 command and delayed synthetic feedback", ylabel="degrees")
    speed.set(title="All joint command velocities; actual send intervals",
              ylabel="degrees/s")
    translation.set(title="TCP target translation relative to each epoch start",
                    ylabel="mm")
    rotation.set(title="TCP target rotation vector relative to each epoch start",
                 ylabel="degrees")
    fig.suptitle(f"{path.name}: {len(rows)} writes, {summary['pauses']} pauses, "
                 f"{len(budget_rows)} retreat budget fallbacks\n"
                 "Synthetic device only — unfiltered samples; shaded gaps preserve "
                 "releases/restarts; feedback plotted at its original sample time")
    fig.tight_layout(rect=(0, 0, 1, .94))
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=140)
    plt.close(fig)
    return {
        "scope": "synthetic; not physical tracking evidence",
        "writes": len(rows), "pauses": summary["pauses"], "epochs": boundaries,
        "maximum_same_epoch_joint_step_deg": maximum_step.tolist(),
        "maximum_same_epoch_command_velocity_deg_s": maximum_speed.tolist(),
        "maximum_target_minus_input_feedback_deg": maximum_feedback_error.tolist(),
        "maximum_epoch_position_span_mm": maximum_position_span.tolist(),
        "maximum_epoch_relative_rotation_deg": maximum_orientation_span,
        "rejected_directions": len(failures),
        "retreat_budget_fallbacks": [
            dict(at_s=(x["at_ns"]-origin)/1e9, candidate=x["row"]["id"],
                 detail=x["row"]["cpp"]["collision_retreat"]) for x in budget_rows],
        "feedback_error_note": "Target minus prior feedback; includes synthetic delay. "
                               "Not a simultaneous physical following-error measurement.",
        "figure": str(destination),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    results = {}
    for path in sorted(args.root.glob("*/summary.json")):
        if (path.parent / "writes.json").is_file():
            results[path.parent.name] = plot_case(
                path.parent, args.output / (path.parent.name + ".png"))
    if not results:
        raise ValueError("No saved synthetic motion cases")
    (args.output / "motion-summary.json").write_text(
        json.dumps(results, indent=2), encoding="utf8")
    print(json.dumps({"cases": len(results), "output": str(args.output)}))


if __name__ == "__main__":
    main()
