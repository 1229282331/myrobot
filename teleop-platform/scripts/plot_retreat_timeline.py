"""Plot measured native scopes without inventing contiguous CPU execution."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from analyze_retreat_lanes import read_rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seq", type=int)
    args = p.parse_args()
    candidates = [
        r for r in read_rows(args.input)
        if r["response"].get("active") and
        r["response"].get("diagnostic_callback", {}).get("complete") and
        any(x.get("executed") and x.get("stage") == "sample"
            for x in r["response"].get("diagnostic_retreat_lanes", []))
    ]
    if args.seq is not None:
        candidates = [r for r in candidates if r["response"]["seq"] == args.seq]
        assert len(candidates) == 1, "Requested sequence is absent or not unique"
    assert candidates, "No completed diagnostic sample scopes"
    row = max(candidates, key=lambda r: r["response"]["collision_retreat"]["sample_queries_us"])
    response = row["response"]
    callback = response["diagnostic_callback"]
    origin = callback["begin_ns"]
    scopes = [callback] + [
        x for x in response["diagnostic_retreat_lanes"]
        if x["executed"] and x["complete"]
    ]
    assert all(s["begin_ns"] >= origin and s["end_ns"] <= callback["end_ns"]
               for s in scopes), "Scope extends beyond callback"
    scopes = [callback] + sorted(scopes[1:], key=lambda x: (x["begin_ns"], x["end_ns"]))
    fig, ax = plt.subplots(figsize=(13, max(4, .38 * len(scopes) + 2.2)))
    labels = []
    for index, scope in enumerate(scopes):
        begin = (scope["begin_ns"] - origin) / 1e6
        duration = (scope["end_ns"] - scope["begin_ns"]) / 1e6
        assert abs(duration - scope["wall_us"] / 1000) < 1e-5
        color = {"callback": "#4778b3", "assess": "#bf8d2b", "sample": "#358f75"}[scope["stage"]]
        ax.barh(index, duration, left=begin, height=.65, color=color, alpha=.7)
        ax.text(begin + duration + .12, index,
                f"wall {duration:.2f}, CPU {scope['cpu_us']/1000:.2f} ms; "
                f"switch {scope['voluntary_switches']}/{scope['involuntary_switches']}",
                va="center", fontsize=8)
        labels.append(f"{scope['stage']} {scope['index']} · TID {scope['tid']}")
    ax.set_yticks(range(len(scopes)), labels=labels)
    ax.invert_yaxis()
    ax.set_xlim(0, callback["wall_us"] / 1000 + 16)
    ax.set_xlabel("Measured scope interval relative to callback start (ms)")
    ax.grid(axis="x", alpha=.2)
    retreat = response["collision_retreat"]
    ax.set_title(
        f"Isolated diagnostic sequence {response['seq']} · {retreat['reason']} · "
        f"scale {retreat['selected_scale']:.5f}\n"
        "Bars show elapsed scope time. CPU totals are labels, not a reconstructed execution timeline."
    )
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140)
    metadata = dict(diagnostic_only=True, physical_io=False, source=str(args.input),
                    sequence=response["seq"], original_seq=row.get("original_seq"),
                    label=row.get("label"), iteration=row.get("iteration"),
                    collision_retreat=retreat, scopes=scopes)
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2), encoding="utf8")
    print(json.dumps({k: v for k, v in metadata.items() if k not in ("scopes", "collision_retreat")}))


if __name__ == "__main__":
    main()
