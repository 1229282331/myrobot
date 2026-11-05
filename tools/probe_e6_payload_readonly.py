"""Read E6 payload configuration from passive port 30004; sends no commands."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import struct
import time

from probe_e6_readonly import FEEDBACK_BYTES, SDK_COMMIT, decode_feedback


def decode_payload(data):
    """V4.5.1 p103 / V4.6.6 p154-155, matching pinned SDK MyType.

    These fields are controller payload parameters, not a weighing result.
    Controller firmware compatibility remains unconfirmed.
    """
    state = decode_feedback(data)
    load, x, y, z = struct.unpack_from("<4d", data, 1168)
    if not all(math.isfinite(v) for v in (load, x, y, z)) or load < 0:
        raise ValueError("Invalid payload parameter in feedback")
    return dict(state, payload_parameter_kg=load,
                center_parameter_xyz_mm=[x, y, z],
                tool_value_xyz_mm_rxyz_deg=list(struct.unpack_from("<6d", data, 1248)))


def read_frame(connection, deadline):
    data = bytearray()
    while len(data) < FEEDBACK_BYTES:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Incomplete frame: {len(data)} bytes")
        connection.settimeout(remaining)
        chunk = connection.recv(FEEDBACK_BYTES - len(data))
        if not chunk:
            raise ConnectionError(f"Peer closed after {len(data)} frame bytes")
        data.extend(chunk)
    return bytes(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.1.4")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.frames <= 100 or not 0 < args.timeout <= 10:
        parser.error("Require 1-100 frames and timeout in (0, 10] seconds")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve a new evidence file before connecting; never overwrite a prior run.
    with args.output.open("x", encoding="utf-8") as output:
        report = {"started_at_utc": datetime.now(timezone.utc).isoformat(),
                  "robot_ip": args.ip, "port": 30004, "application_bytes_sent": 0,
                  "layout_source_commit": SDK_COMMIT,
                  "scope": "Passive feedback only; no dashboard commands or device writes",
                  "interpretation": "Payload parameters; not measured mass or validated load capacity",
                  "requested_frames": args.frames, "frames": []}
        try:
            deadline = time.monotonic() + args.timeout
            with socket.create_connection((args.ip, 30004), timeout=args.timeout) as connection:
                report["local_address"] = list(connection.getsockname())
                for index in range(args.frames):
                    data = read_frame(connection, deadline)
                    entry = {"index": index, "received_at_utc": datetime.now(timezone.utc).isoformat(),
                             "host_monotonic_ns": time.monotonic_ns(), "raw_hex": data.hex(),
                             "sha256": hashlib.sha256(data).hexdigest()}
                    report["frames"].append(entry)
                    entry["decoded"] = decode_payload(data)
        except (OSError, ValueError) as error:
            report["error"] = f"{type(error).__name__}: {error}"
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        json.dump(report, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(json.dumps({"output": str(args.output), "frames": len(report["frames"]),
                      "error": report.get("error"),
                      "first_decoded": report["frames"][0].get("decoded") if report["frames"] else None},
                     ensure_ascii=False, indent=2))
    return 1 if report.get("error") else 0


if __name__ == "__main__":
    raise SystemExit(main())
