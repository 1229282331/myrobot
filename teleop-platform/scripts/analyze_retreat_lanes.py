"""Analyze isolated native per-thread traces. No robot connections or writes.

Wall minus CPU time includes blocking, preemption and instrumentation skew; it
does not by itself establish a scheduler, mutex or storage root cause.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_rows(path):
    if path.suffix == ".jsonl":
        rows = []
        for line in path.read_text(encoding="utf8").splitlines():
            row = json.loads(line)
            if row.get("kind") == "reply":
                rows.append(dict(response=row["response"], pending=row.get("pending"),
                                 variant="diagnostic", label=path.parent.name,
                                 original_seq=row["response"]["seq"]))
        return rows
    return json.loads(path.read_text(encoding="utf8"))


def summarize(row):
    response = row["response"]
    profiles = response.get("diagnostic_retreat_lanes", [])
    samples = [x for x in profiles if x["stage"] == "sample" and x["executed"]]
    assessments = [x for x in profiles if x["stage"] == "assess"]
    for profile in profiles:
        assert profile["complete"], "Incomplete native diagnostic scope"
        assert profile["end_ns"] >= profile["begin_ns"]
        assert profile["cpu_us"] >= 0
    if not samples:
        return None
    callback = response["diagnostic_callback"]
    assert callback["complete"] and callback["executed"]
    retreat = response["collision_retreat"]
    slowest = max(samples, key=lambda x: x["wall_us"])
    return {
        "variant": row.get("variant"), "iteration": row.get("iteration"),
        "label": row.get("label"), "original_seq": row.get("original_seq"),
        "seq": response["seq"], "status": response["status"],
        "callback_wall_ms": callback["wall_us"] / 1000,
        "callback_main_cpu_ms": callback["cpu_us"] / 1000,
        "retreat_ms": retreat["elapsed_us"] / 1000,
        "sample_batch_ms": retreat["sample_queries_us"] / 1000,
        "reason": retreat["reason"], "budget": retreat["budget_exhausted"],
        "selected_scale": retreat["selected_scale"],
        "samples": len(samples), "assessments": assessments,
        "sample_cpu_sum_ms": sum(x["cpu_us"] for x in samples) / 1000,
        "slowest_sample_wall_ms": slowest["wall_us"] / 1000,
        "slowest_sample_cpu_ms": slowest["cpu_us"] / 1000,
        "slowest_sample_unaccounted_ms": (slowest["wall_us"] - slowest["cpu_us"]) / 1000,
        "sample_voluntary_switches": sum(x["voluntary_switches"] for x in samples),
        "sample_involuntary_switches": sum(x["involuntary_switches"] for x in samples),
        "sample_minor_faults": sum(x["minor_faults"] for x in samples),
        "sample_major_faults": sum(x["major_faults"] for x in samples),
        "profiles": profiles,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = read_rows(args.input)
    selected = [summary for row in rows
                if row["response"].get("active") and
                row["response"].get("diagnostic_retreat_lanes")
                if (summary := summarize(row)) is not None]
    if not selected:
        raise ValueError("No executed native sample profiles")
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    x = np.arange(len(selected))
    axes[0, 0].plot(x, [v["callback_wall_ms"] for v in selected], lw=.8,
                    label="Callback wall")
    axes[0, 0].plot(x, [v["callback_main_cpu_ms"] for v in selected], lw=.8,
                    label="Callback main-thread CPU")
    axes[0, 0].axhline(42, color="#c45339", ls=":", label="Callback deadline")
    axes[0, 0].set(ylabel="ms", xlabel="Active request with retreat samples")
    wall = np.array([v["slowest_sample_wall_ms"] for v in selected])
    cpu = np.array([v["slowest_sample_cpu_ms"] for v in selected])
    axes[0, 1].scatter(cpu, wall, s=12, alpha=.6)
    limit = max(float(wall.max()), float(cpu.max()))
    axes[0, 1].plot([0, limit], [0, limit], lw=.8, color="#888888", label="Wall = CPU")
    axes[0, 1].set(xlabel="Slowest sample thread CPU (ms)",
                   ylabel="Same sample wall (ms)")
    axes[1, 0].plot(x, [v["sample_batch_ms"] for v in selected], lw=.8,
                    label="Sample batch wall")
    axes[1, 0].plot(x, wall, lw=.8, label="Slowest sample wall")
    axes[1, 0].plot(x, cpu, lw=.8, label="Same sample thread CPU")
    axes[1, 0].set(ylabel="ms", xlabel="Active request with retreat samples")
    axes[1, 1].plot(x, [v["sample_involuntary_switches"] for v in selected], lw=.8,
                    label="Involuntary switches (all sample threads)")
    axes[1, 1].plot(x, [v["sample_voluntary_switches"] for v in selected], lw=.8,
                    label="Voluntary switches (all sample threads)")
    axes[1, 1].set(ylabel="count", xlabel="Active request with retreat samples")
    for ax in axes.flat:
        ax.legend(fontsize=7); ax.grid(alpha=.15)
    for index, row in enumerate(selected):
        if row["budget"]:
            for ax in (axes[0, 0], axes[1, 0], axes[1, 1]):
                ax.axvline(index, color="#e69100", ls=":", lw=.8)
    fig.suptitle("Isolated native diagnostics — no physical motion\n"
                 "Elapsed and per-thread CPU clocks; orange markers are budget fallbacks; "
                 "instrumentation is not performance qualification")
    fig.tight_layout(rect=(0, 0, 1, .94))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140)
    result = {
        "diagnostic_only": True, "physical_io": False,
        "requests_with_samples": len(selected),
        "budget_fallbacks": sum(v["budget"] for v in selected),
        "longest_sample_batches": sorted(
            selected, key=lambda x: x["sample_batch_ms"], reverse=True)[:12],
        "all_requests": selected,
        "interpretation": "Wall minus CPU is unaccounted time, not proof of a specific "
                          "wait cause. Sum of parallel thread CPU may exceed wall time.",
    }
    args.output.with_suffix(".json").write_text(json.dumps(result, indent=2), encoding="utf8")
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("longest_sample_batches", "all_requests")}))


if __name__ == "__main__":
    main()
