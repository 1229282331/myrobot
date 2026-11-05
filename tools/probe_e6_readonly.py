"""Probe E6 TCP ports using only fixed, read-only queries (Python stdlib)."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import socket
import struct
import time


READ_ONLY_QUERIES = ("RobotMode()", "GetAngle()", "GetPose()", "GetErrorID()")
MAX_REPLY_BYTES = 65536
FEEDBACK_BYTES = 1440
SDK_COMMIT = "82bce656c6e1a1c71599901baa93ed45bb50963d"


def read_reply(connection, timeout):
    """TCP may split a reply across reads; the protocol terminates with ';'."""
    deadline = time.monotonic() + timeout
    data = bytearray()
    while b";" not in data:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Incomplete reply: {bytes(data)!r}")
        connection.settimeout(remaining)
        try:
            chunk = connection.recv(4096)
        except TimeoutError as error:
            raise TimeoutError(f"Reply timed out; received {len(data)} bytes: {bytes(data)!r}") from error
        if not chunk:
            raise ConnectionError(f"Connection closed with partial reply: {bytes(data)!r}")
        data.extend(chunk)
        if len(data) > MAX_REPLY_BYTES:
            raise ValueError("Reply exceeds size limit")
        # This controller can return a tab-terminated refusal outside TCP mode.
        if bytes(data).strip() == b"Control Mode Is Not Tcp":
            return data.decode("ascii").strip()
    raw = data.decode("utf-8", errors="strict").strip()
    if not raw.endswith(";") or raw.count(";") != 1:
        raise ValueError(f"Unexpected reply framing: {raw!r}")
    return raw


def probe_dashboard(ip, timeout):
    result = {"port": 29999, "connected": False, "queries": []}
    try:
        started = time.monotonic()
        with socket.create_connection((ip, 29999), timeout=timeout) as connection:
            result.update(connected=True, local_address=list(connection.getsockname()),
                          connect_ms=round((time.monotonic() - started) * 1000, 2))
            for command in READ_ONLY_QUERIES:
                entry = {"command": command}
                result["queries"].append(entry)
                started = time.monotonic()
                connection.settimeout(timeout)
                connection.sendall(command.encode("ascii"))
                raw = read_reply(connection, timeout)
                entry.update(raw=raw, elapsed_ms=round((time.monotonic() - started) * 1000, 2))
                if raw == "Control Mode Is Not Tcp":
                    entry["error"] = "Controller is not in TCP/IP control mode"
                    break
                # Preserve the payload without assuming firmware-specific mode meanings.
                match = re.fullmatch(r"\s*(-?\d+)\s*,\s*\{(.*)\}\s*,\s*"
                                     + re.escape(command) + r"\s*;", raw, re.DOTALL)
                if not match:
                    entry["error"] = "Unexpected reply format or command echo; stopping queries"
                    break
                entry.update(error_id=int(match.group(1)), payload=match.group(2))
                if entry["error_id"] != 0:
                    entry["note"] = "Controller rejected this query; no recovery commands sent"
    except (OSError, ValueError, UnicodeError) as error:
        result["error"] = f"{type(error).__name__}: {error}"
    return result


def decode_feedback(data):
    """Selected little-endian MyType fields in the pinned official V4 SDK.

    https://github.com/Dobot-Arm/TCP-IP-Python-V4/blob/
    82bce656c6e1a1c71599901baa93ed45bb50963d/dobot_api.py
    Length and magic validation do not establish full firmware compatibility.
    """
    if len(data) != FEEDBACK_BYTES or struct.unpack_from("<H", data, 0)[0] != FEEDBACK_BYTES:
        raise ValueError("Unexpected feedback length")
    if struct.unpack_from("<Q", data, 48)[0] != 0x123456789ABCDEF:
        raise ValueError("Unexpected feedback test marker")
    q = list(struct.unpack_from("<6d", data, 432))
    tcp = list(struct.unpack_from("<6d", data, 624))
    if not all(math.isfinite(value) for value in q + tcp):
        raise ValueError("Non-finite state in feedback")
    return {
        "layout_source_commit": SDK_COMMIT,
        "robot_mode": struct.unpack_from("<Q", data, 24)[0],
        "q_actual_deg": q,
        "tcp_actual_xyz_mm_rxyz_deg": tcp,
        "user_index": data[1012],
        "tool_index": data[1013],
        "enable_status": data[1026],
        "error_status": data[1029],
        "note": "Decoded with pinned V4 layout; controller firmware version remains unconfirmed",
    }


def probe_feedback(ip, timeout):
    result = {"port": 30004, "connected": False, "application_bytes_sent": 0}
    try:
        started = time.monotonic()
        with socket.create_connection((ip, 30004), timeout=timeout) as connection:
            result.update(connected=True, local_address=list(connection.getsockname()),
                          connect_ms=round((time.monotonic() - started) * 1000, 2))
            deadline = time.monotonic() + timeout
            data = bytearray()
            while len(data) < FEEDBACK_BYTES:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Incomplete feedback: {len(data)} bytes")
                connection.settimeout(remaining)
                chunk = connection.recv(FEEDBACK_BYTES - len(data))
                if not chunk:
                    raise ConnectionError(f"Peer closed after {len(data)} feedback bytes")
                data.extend(chunk)
            result.update(bytes_received=len(data), prefix_hex=data[:32].hex())
            result["decoded"] = decode_feedback(data)
    except (OSError, ValueError) as error:
        result["error"] = f"{type(error).__name__}: {error}"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.1.4")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--output", type=Path, help="Optional JSON report path")
    args = parser.parse_args()
    if not 0 < args.timeout <= 30:
        parser.error("timeout must be greater than 0 and at most 30 seconds")
    report = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "robot_ip": args.ip,
        "scope": "Fixed read-only queries; no enable, clear, motion, mode change or IO writes",
        "dashboard": probe_dashboard(args.ip, args.timeout),
        "feedback": probe_feedback(args.ip, args.timeout),
    }
    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    dashboard = report["dashboard"]
    feedback = report["feedback"]
    ok = (not dashboard.get("error") and len(dashboard["queries"]) == len(READ_ONLY_QUERIES)
          and all(item.get("error_id") == 0 for item in dashboard["queries"])
          and not feedback.get("error") and feedback.get("bytes_received", 0) > 0)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
