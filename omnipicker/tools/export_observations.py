"""Offline JSONL to CSV export. Joins confirmed actions by host timestamp only.

Separate connection IDs prevent targets leaking across reconnects. No device
access, resampling, interpolation, or invented values for missing observations.
"""
import argparse
import bisect
import csv
import json
from pathlib import Path


def export(source, output):
    if source.with_suffix(".incomplete.txt").exists():
        raise ValueError("此日志有记录失败标记；未导出，请先核查原始文件")
    actions, observations = {}, []
    with source.open(encoding="utf-8") as f:
        for number, line in enumerate(f, 1):
            try: record = json.loads(line)
            except json.JSONDecodeError as exc: raise ValueError(f"日志第 {number} 行不完整；未导出") from exc
            if record.get("record_type") == "recording_error":
                raise ValueError("日志记录不完整；未导出")
            session = (record.get("run_id"), record.get("connection_id"))
            if record.get("record_type") == "observation":
                observations.append(record)
            if record.get("action") == "set_opening":
                actions.setdefault(session, []).append((record["monotonic"], record["target"]))
    for values in actions.values(): values.sort()
    columns = ["run_id", "connection_id", "simulated", "firmware", "serial", "source", "monotonic",
               "host_sequence", "stream_accepted", "position_fraction", "velocity_fraction_s",
               "column1_unverified", "last_confirmed_target_percent", "last_command_ack_monotonic", "raw"]
    with output.open("x", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for observation in observations:
            row = {name: observation.get(name, "") for name in columns}
            values = actions.get((observation.get("run_id"), observation.get("connection_id")), [])
            index = bisect.bisect_right(values, (observation["monotonic"], float("inf"))) - 1
            if index >= 0:
                row["last_command_ack_monotonic"], row["last_confirmed_target_percent"] = values[index]
            writer.writerow(row)
    return len(observations)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(f"Exported {export(args.source, args.output)} observations to {args.output}")
