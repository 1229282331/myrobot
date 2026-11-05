"""Plot retained native ABBA replies without treating them as physical motion."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = json.loads(args.results.read_text(encoding="utf8"))
    active = [r for r in rows if r["response"]["active"]]
    fields = (("state_wait_us", "State delivery wait"),
              ("compute_us", "Servo computation"),
              ("callback_us", "Whole callback"),
              ("roundtrip_ms", "Native IPC round trip"))
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    summary = dict(physical_io=False, rows=len(rows), active=len(active), variants={})
    for variant in ("baseline", "candidate"):
        group = [r for r in active if r["variant"] == variant]
        assert group
        summary["variants"][variant] = dict(samples=len(group), stages={})
        for ax, (field, title) in zip(axes.flat, fields):
            values = np.array([r[field] if field == "roundtrip_ms" else
                               r["response"][field] / 1000 for r in group])
            assert np.isfinite(values).all() and (values >= 0).all()
            ordered = np.sort(values)
            ax.plot(ordered, (np.arange(len(ordered)) + 1) / len(ordered), label=variant)
            ax.set(title=title, xlabel="Milliseconds", ylabel="Fraction of active replies")
            ax.grid(alpha=.2)
            ax.legend()
            summary["variants"][variant]["stages"][field] = dict(
                median_ms=float(np.median(values)), p95_ms=float(np.percentile(values, 95)),
                max_ms=float(values.max()))
    axes[1, 0].axvline(42, color="red", linestyle="--", alpha=.6, label="42 ms deadline")
    axes[1, 0].legend()
    fig.suptitle("Native solver ABBA — original input replay, no physical motion\n"
                 "All active replies retained; startup/reset replies reported separately")
    summary["startup_and_reset"] = [
        dict(iteration=r["iteration"], variant=r["variant"], label=r["label"], index=r["index"],
             callback_ms=r["response"]["callback_us"]/1000, status=r["response"]["status"])
        for r in rows if not r["response"]["active"]]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf8")
    print(json.dumps({k: v for k, v in summary.items() if k != "startup_and_reset"}))


if __name__ == "__main__":
    main()
