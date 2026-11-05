"""Plot complete isolated-chain timing, retaining pauses and epoch gaps.

Usage: python plot_control_chain.py EVIDENCE_ROOT --output OUTPUT.png
Reads only saved qualification files. Never connects to a robot or platform.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    cases = sorted(p.parent for p in args.root.glob("*/summary.json")
                   if (p.parent / "writes.json").is_file())
    if not cases:
        raise ValueError("No complete chain cases")
    fig, axes = plt.subplots((len(cases) + 1) // 2, 2, figsize=(15, 3.4 * ((len(cases) + 1) // 2)),
                             squeeze=False)
    metrics = {}
    for ax, p in zip(axes.flat, cases):
        writes = json.loads((p / "writes.json").read_text())
        summary = json.loads((p / "summary.json").read_text())
        if not writes:
            ax.set_title(p.name + ": no writes")
            continue
        origin = writes[0]["at_ns"]
        t, intervals, ages = [], [], []
        for previous, current in zip(writes, writes[1:]):
            if previous["row"]["epoch"] != current["row"]["epoch"]:
                # A release or failed direction must not look like a measured
                # same-epoch send interval or a continuous plotted trajectory.
                t.append(float("nan")); intervals.append(float("nan")); ages.append(float("nan"))
                continue
            t.append((current["at_ns"] - origin) / 1e9)
            intervals.append((current["at_ns"] - previous["at_ns"]) / 1e6)
            ages.append((current["at_ns"] - current["row"]["pre_action_received_ns"]) / 1e6)
        ax.plot(t, intervals, color="#1769aa", lw=.8, label="Send interval")
        ax.plot(t, ages, color="#c45339", lw=.7, alpha=.8, label="Original feedback age at send")
        failures = summary["dispatch_rejections"] + summary["solver_barriers"]
        budget_rows = [x for x in writes
                       if x["row"]["cpp"].get("collision_retreat", {}).get("budget_exhausted")]
        for i, event in enumerate(failures):
            ax.axvline((event["at_ns"] - origin) / 1e9, color="#9c167c", alpha=.8,
                       linestyle="--", lw=1, label="Rejected direction" if i == 0 else None)
        for i, event in enumerate(budget_rows):
            ax.axvline((event["at_ns"] - origin) / 1e9, color="#e69100",
                       linestyle=":", lw=1.2, label="Retreat budget fallback" if i == 0 else None)
        ax.axhline(100, color="#c45339", linestyle=":", lw=.7, label="Feedback deadline")
        ax.axhline(50, color="#1769aa", linestyle=":", lw=.7, label="Minimum send interval")
        ax.set_title(f"{p.name}: {len(writes)} writes, {summary['pauses']} pauses, "
                     f"{len(budget_rows)} budget fallbacks")
        ax.set_xlabel("Seconds from first synthetic write"); ax.set_ylabel("ms")
        ax.grid(alpha=.15)
        ax.set_ylim(0, max(110, np.nanmax(intervals) * 1.07))
        metrics[p.name] = dict(
            writes=len(writes), pauses=summary["pauses"],
            original_feedback_age_p50_p95_max_ms=np.nanpercentile(ages, [50, 95, 100]).tolist(),
            interval_p50_p95_max_ms=np.nanpercentile(intervals, [50, 95, 100]).tolist(),
            retreat_budget_fallbacks=[
                dict(at_s=(x["at_ns"]-origin)/1e9, candidate=x["row"]["id"],
                     detail=x["row"]["cpp"]["collision_retreat"]) for x in budget_rows],
            failures=[dict(at_s=(x["at_ns"] - origin) / 1e9,
                           reason=x.get("reason", x.get("detail", {}).get("reason"))) for x in failures])
    for ax in list(axes.flat)[len(cases):]:
        ax.set_visible(False)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle("Isolated control-chain timing — synthetic device, no physical motion\n"
                 "Original feedback timestamps; gaps preserve releases/restarts; all direction rejections shown")
    fig.tight_layout(rect=(0, 0, 1, .95))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    args.output.with_suffix(".json").write_text(json.dumps(metrics, indent=2), encoding="utf8")
    print(json.dumps(dict(cases=len(cases), output=str(args.output))))


if __name__ == "__main__":
    main()
