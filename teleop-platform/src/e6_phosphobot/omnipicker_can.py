"""Bounded OmniPicker classical-CAN position/feedback verification on Linux.

Default is an offline plan. Only --execute configures a verified peak_usb port
and sends motion frames. This protocol has no documented disable command;
interface shutdown is not motor disable. Never imports vendor constructors.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

FRAME = struct.Struct("=IB3x8s")
CAN_EFF = 0x80000000
CAN_RTR = 0x40000000
CAN_ERR = 0x20000000
SFF_MASK = 0x7FF
SOL_CAN_RAW = 101
FAULT_NAMES = {0: "none", 1: "overheat", 2: "overspeed", 3: "initialization", 4: "limit"}
STATE_NAMES = {0: "at_target", 1: "moving", 2: "stalled", 3: "object_dropped"}
LIMITS = {"velocity": 25, "force": 64, "acceleration": 25, "deceleration": 25}
# Pinned FW335 maps D3 to current_limit = D3 * 3 / 255 A. Match the
# historical native Type-C Studio current limit, without changing its speed
# or acceleration here. Keep the standalone qualification profile unchanged.
FW335_STUDIO_LIMITS = dict(LIMITS, force=255)
# Qualified for this station's pinned FW335 (0c2ea534...4df2d), handler
# 0x0801f000: D2=0 returns before any target/parameter write. The receive path
# still returns a fresh status. This is NOT a stop/disable command and is not
# assumed portable to other firmware. D1=127 cannot match a valid state byte,
# so even an all-zero closed-position feedback cannot be mistaken for TX echo.
FW335_OBSERVATION = bytes((0, 127, 0, 64, 25, 25, 0, 0))


class CheckFailed(RuntimeError):
    pass


def encode_command(percent, limits=None, *, bounds=(15, 35), profile='verification'):
    limits = LIMITS if limits is None else limits
    if len(bounds) != 2 or not all(math.isfinite(x) for x in bounds) or not 0 <= bounds[0] <= bounds[1] <= 100:
        raise ValueError("Invalid explicit motion bounds")
    if not math.isfinite(percent) or not bounds[0] <= percent <= bounds[1]:
        raise ValueError(f"This verification is limited to {bounds[0]}..{bounds[1]} percent")
    ceilings = {"velocity": 51, "force": 85, "acceleration": 51, "deceleration": 51}
    if profile == 'fw335_studio_3a':ceilings['force']=255
    elif profile != 'verification':raise ValueError('Unknown CAN limits profile')
    if set(limits) != set(ceilings):
        raise ValueError("Incorrect control limits")
    if any(type(limits[k]) is not int or not 1 <= limits[k] <= cap for k, cap in ceilings.items()):
        raise ValueError("Control limits exceed bounded test caps")
    return bytes((0, round(percent * 255 / 100), limits["velocity"], limits["force"],
                  limits["acceleration"], limits["deceleration"], 0, 0))


def decode_feedback(packet, flags, can_id, command, *, allow_stalled=False, allow_dropped=False):
    if flags & (getattr(socket, "MSG_DONTROUTE", 4) | getattr(socket, "MSG_CONFIRM", 0x800)):
        return None  # Local transmission/echo is never a device observation.
    if len(packet) != FRAME.size:
        raise CheckFailed("Not a 16-byte classical CAN frame")
    frame_id, dlc, data = FRAME.unpack(packet)
    if frame_id & CAN_ERR:
        raise CheckFailed(f"CAN error frame: {frame_id:08x} {data.hex()}")
    if frame_id & (CAN_EFF | CAN_RTR) or frame_id != can_id:
        return None
    if dlc != 8:
        raise CheckFailed(f"Unexpected DLC {dlc}")
    if data == command:
        return None  # Also reject a possible adapter-side payload echo.
    if data[0] not in FAULT_NAMES or data[1] not in STATE_NAMES:
        raise CheckFailed(f"Unknown fault/state: {data.hex()}")
    if data[5:] != command[5:]:
        raise CheckFailed(f"D5-D7 echo mismatch: {data.hex()} vs {command.hex()}")
    result = {"raw_hex": data.hex(), "fault": data[0], "state": data[1],
              "state_name": STATE_NAMES[data[1]], "position_raw": data[2],
              "position_percent": data[2] * 100 / 255,
              "velocity_raw": data[3], "force_raw": data[4], "echo_hex": data[5:].hex()}
    if result["fault"]:
        raise CheckFailed(f"Gripper fault {result['fault']}: {result}")
    if (result["state"] == 3 and not allow_dropped) or (result["state"] == 2 and not allow_stalled):
        raise CheckFailed(f"Gripper {result['state_name']}: {result}")
    return result


def arrived(feedback, target_byte):
    return feedback["state"] == 0 and abs(feedback["position_raw"] - target_byte) <= 1


def run_sequence(channel, targets, report, *, dwell=6.0, hz=20.0,
                 clock=time.monotonic, sleep=time.sleep, stopped=lambda: False):
    report["stages"] = []
    for percent in targets:
        if stopped():
            raise CheckFailed("Stop requested before next target")
        command = encode_command(percent)
        stage = {"target_percent": percent, "target_byte": command[1],
                 "command_hex": command.hex(), "feedback": []}
        report["stages"].append(stage)
        start = clock()
        stable = 0
        next_send = start
        while clock() - start < dwell:
            if stopped():
                raise CheckFailed("Stop requested; remaining targets cancelled")
            feedback = channel.exchange(command)
            feedback["stage_elapsed_s"] = clock() - start
            stage["feedback"].append(feedback)
            stable = stable + 1 if arrived(feedback, command[1]) else 0
            if stable >= 3 and "first_stable_at_s" not in stage:
                stage["first_stable_at_s"] = clock() - start
            next_send += 1 / hz
            sleep(max(0, next_send - clock()))
        if stable < 3:
            raise CheckFailed(f"Target {percent}% did not finish with three matching at-target reports")
        stage["completed"] = True
        stage["last_feedback"] = stage["feedback"][-1]
        stage["sample_count"] = len(stage["feedback"])
        print(json.dumps({"event": "stage_completed", "target_percent": percent,
                          "samples": stage["sample_count"], "first_stable_at_s": stage.get("first_stable_at_s"),
                          "final_position_percent": stage["last_feedback"]["position_percent"]}), flush=True)
    report["sequence_completed"] = True


class CanChannel:
    def __init__(self, interface, can_id, timeout, report, *, allow_stalled=False, allow_dropped=False):
        self.can_id, self.timeout, self.report = can_id, timeout, report
        self.allow_stalled = allow_stalled
        self.allow_dropped = allow_dropped
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(SOL_CAN_RAW, 3, 0)  # Disable kernel TX loopback.
            self.sock.setsockopt(SOL_CAN_RAW, 4, 0)  # Never receive our own messages.
            self.sock.setsockopt(SOL_CAN_RAW, 1, struct.pack("=II", can_id, SFF_MASK | CAN_EFF | CAN_RTR))
            self.sock.setsockopt(SOL_CAN_RAW, 2, struct.pack("=I", 0x1FFFFFFF))
            self.sock.bind((interface,))
        except BaseException:
            self.sock.close()
            raise

    def exchange(self, command):
        # Drain a bounded number of old frames before issuing one new request.
        self.sock.setblocking(False)
        for _ in range(16):
            try:
                old, _, old_flags, _ = self.sock.recvmsg(72)
            except BlockingIOError:
                break
            self.report["events"].append({"kind": "discard_before_tx", "raw_hex": old.hex(), "flags": old_flags})
            # Stale safety-critical faults/error frames still abort the run.
            if len(old) == FRAME.size:
                old_id, old_dlc, old_data = FRAME.unpack(old)
                if old_id & CAN_ERR or (old_id == self.can_id and old_dlc == 8 and
                                       old_data != command and (old_data[0] != 0 or old_data[1] == 3 or
                                                               (old_data[1] == 2 and not self.allow_stalled))):
                    raise CheckFailed("Fault/error in stale queue before transmission")
        else:
            raise CheckFailed("Receive queue did not drain; another sender may be active")
        self.sock.settimeout(self.timeout)
        tx_ns = time.perf_counter_ns()
        event = {"kind": "tx", "host_perf_counter_ns": tx_ns, "host_time_ns": time.time_ns(),
                 "can_id": self.can_id, "data_hex": command.hex(), "send_confirmed": False}
        self.report["events"].append(event)
        try:
            written = self.sock.send(FRAME.pack(self.can_id, 8, command))
        except OSError as exc:
            event["send_error"] = repr(exc)
            raise
        if written != FRAME.size:
            raise CheckFailed("Short CAN send; do not retry")
        event["send_confirmed"] = True
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.sock.settimeout(max(.001, deadline - time.monotonic()))
            try:
                raw, _, flags, _ = self.sock.recvmsg(72)
            except socket.timeout as exc:
                raise CheckFailed("No valid feedback before deadline; no next target or retry") from exc
            rx_ns = time.perf_counter_ns()
            self.report["events"].append({"kind": "rx", "host_perf_counter_ns": rx_ns,
                                          "host_time_ns": time.time_ns(), "flags": flags, "frame_hex": raw.hex()})
            feedback = decode_feedback(raw, flags, self.can_id, command, allow_stalled=self.allow_stalled,
                                       allow_dropped=self.allow_dropped)
            if feedback is not None:
                feedback.update(host_perf_counter_ns=rx_ns, round_trip_ms=(rx_ns - tx_ns) / 1e6,
                                receive_flags=flags)
                return feedback
        raise CheckFailed("Only echoes/unrelated frames arrived; no retry")

    def close(self):
        self.sock.close()


def ip_snapshot(interface):
    return json.loads(subprocess.run(["ip", "-details", "-statistics", "-json", "link", "show", interface],
                                    check=True, capture_output=True, text=True, timeout=3).stdout)[0]


def configure(interface, *args):
    return subprocess.run(["ip", "link", "set", interface, *args], check=True,
                          capture_output=True, text=True, timeout=3)


def validate_usb_interface(interface):
    if not interface.startswith("can") or not interface[3:].isdigit():
        raise CheckFailed("Invalid CAN interface name")
    base = Path("/sys/class/net") / interface / "device"
    device = base.resolve(strict=True)
    driver = (base / "driver").resolve(strict=True)
    if driver.name != "peak_usb":
        raise CheckFailed("Refuse to operate a non-peak_usb CAN port")
    for parent in (device, *device.parents):
        if (parent / "idVendor").is_file() and (parent / "idProduct").is_file():
            vid = (parent / "idVendor").read_text().strip()
            pid = (parent / "idProduct").read_text().strip()
            if (vid, pid) != ("0c72", "000c"):
                raise CheckFailed(f"Unexpected USB VID/PID {vid}:{pid}")
            return {"device_path": str(device), "driver_path": str(driver), "vid_pid": f"{vid}:{pid}"}
    raise CheckFailed("Cannot establish USB device identity")


def execute(args, targets):
    import fcntl  # Linux only, and only in the explicit hardware path.
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "status": "preflight",
              "interface": args.interface, "can_id": args.can_id, "limits_raw": LIMITS,
              "targets_percent": targets, "dwell_s": args.dwell, "request_hz": args.hz,
              "feedback_timeout_s": args.timeout, "clock_semantics": "host receive time, not device sampling time",
              "events": [], "motor_disable_command_available": False, "motor_disable_requested": False}
    channel = None
    configured = False
    cancelled = [False]
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, lambda _sig, _frame: cancelled.__setitem__(0, True))
    stop_path = Path(args.stop_file)
    with output.open("x", encoding="utf-8") as evidence, open(f"/tmp/omnipicker-{args.interface}.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if stop_path.exists():
                raise CheckFailed("Stop file already exists")
            report["identity"] = validate_usb_interface(args.interface)
            report["before"] = ip_snapshot(args.interface)
            if "UP" in report["before"]["flags"]:
                raise CheckFailed("Interface already UP; do not take it from another controller")
            report["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            configured = True
            configure(args.interface, "type", "can", "bitrate", "1000000", "sample-point", "0.8", "listen-only", "off")
            configure(args.interface, "up")
            report["initialized"] = ip_snapshot(args.interface)
            if report["initialized"].get("linkinfo", {}).get("info_data", {}).get("bittiming", {}).get("bitrate") != 1000000:
                raise CheckFailed("Configured bitrate did not read back as 1 Mbps")
            channel = CanChannel(args.interface, args.can_id, args.timeout, report)
            report["status"] = "running"
            run_sequence(channel, targets, report, dwell=args.dwell, hz=args.hz,
                         stopped=lambda: cancelled[0] or stop_path.exists())
            report["status"] = "completed"
        except BaseException as exc:
            report["status"] = "aborted"
            report["error"] = repr(exc)
            print(json.dumps({"event": "aborted", "error": repr(exc),
                              "notice": "CAN shutdown is not motor disable; remove 24V if motion is unsafe"}), flush=True)
        finally:
            if channel is not None:
                channel.close()
            if configured:
                try:
                    report["before_interface_down"] = ip_snapshot(args.interface)
                except BaseException as exc:
                    report["snapshot_error"] = repr(exc)
                try:
                    configure(args.interface, "down")
                    report["after"] = ip_snapshot(args.interface)
                except BaseException as exc:
                    report["cleanup_error"] = repr(exc)
                    report["status"] = "aborted_cleanup_unconfirmed"
            report["ended_at_utc"] = datetime.now(timezone.utc).isoformat()
            report["final_motor_state"] = "Not disabled via CAN; may hold the last target while powered"
            json.dump(report, evidence, ensure_ascii=False, indent=2)
            evidence.write("\n")
    print(json.dumps({"event": "finished", "status": report["status"], "output": str(output),
                      "tx_frames": sum(e["kind"] == "tx" for e in report["events"]),
                      "rx_frames": sum(e["kind"] == "rx" for e in report["events"])}), flush=True)
    return 0 if report["status"] == "completed" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--can-id", type=lambda s: int(s, 0), default=1)
    parser.add_argument("--targets", default="15,35,15")
    parser.add_argument("--dwell", type=float, default=6.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=.3)
    parser.add_argument("--output")
    parser.add_argument("--stop-file", default="/root/omnipicker-can/STOP")
    args = parser.parse_args()
    targets = [float(x) for x in args.targets.split(",")]
    if not 1 <= args.can_id <= SFF_MASK or not 1 <= len(targets) <= 4:
        parser.error("Use one known standard CAN ID and at most four targets")
    if not all(math.isfinite(v) for v in (args.dwell, args.hz, args.timeout)) or not (
            1 <= args.dwell <= 8 and len(targets) * args.dwell <= 30 and 10 <= args.hz <= 30 and .1 <= args.timeout <= 1):
        parser.error("Unbounded timing parameters")
    commands = [encode_command(target).hex() for target in targets]
    if not args.execute:
        print(json.dumps({"mode": "offline_preview", "interface": args.interface, "can_id": args.can_id,
                          "targets_percent": targets, "commands_hex": commands, "limits_raw": LIMITS,
                          "dwell_s": args.dwell, "hz": args.hz, "feedback_timeout_s": args.timeout,
                          "motor_disable_available": False}, indent=2))
        return 0
    if sys.platform != "linux" or not args.output:
        parser.error("Hardware execution requires Linux and a new --output file")
    return execute(args, targets)


if __name__ == "__main__":
    raise SystemExit(main())
