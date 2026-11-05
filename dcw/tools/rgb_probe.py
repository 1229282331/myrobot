"""Diagnostic RGB-only DirectShow capture, explicit camera indexes required.

This is not a substitute for depth acquisition or hardware timestamp support.
"""
import argparse
import json
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--indexes", type=int, nargs="+", required=True)
    parser.add_argument("--seconds", type=float, default=5)
    args = parser.parse_args()
    results = []
    output = ROOT / "captures/rgb_probe"
    output.mkdir(parents=True, exist_ok=True)
    for index in args.indexes:
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        record = {"index": index, "backend": "DirectShow", "opened": cap.isOpened(), "frames": 0}
        print(f"Opening camera {index}: {cap.isOpened()}", flush=True)
        try:
            if not cap.isOpened():
                results.append(record)
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            start = time.monotonic()
            frame = None
            while time.monotonic() - start < args.seconds:
                ok, candidate = cap.read()
                if not ok:
                    record["read_failure"] = True
                    break
                frame = candidate
                record["frames"] += 1
            record["elapsed_s"] = time.monotonic() - start
            record["host_fps"] = record["frames"] / record["elapsed_s"]
            if frame is not None:
                record["shape"] = list(frame.shape)
                record["snapshot"] = str(output / f"camera_{index}.png")
                if not cv2.imwrite(record["snapshot"], frame):
                    raise RuntimeError("Snapshot write failed")
        finally:
            cap.release()
        results.append(record)
        print(json.dumps(record), flush=True)
    (ROOT / "logs/rgb_probe.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
