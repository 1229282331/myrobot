"""Bounded acceptance profiles; offline preview unless --execute is given.

Position stages advance only after matching device at-target reports. A CAN
shutdown cancels future sends, but cannot disable the gripper motor. Full
stroke requires the explicit --full-stroke option and cleared physical travel.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import statistics
import subprocess
import time

import verify_can_control as base


def now():
    return datetime.now(timezone.utc).isoformat()


class Acceptance:
    def __init__(self, args, report, *, clock=time.monotonic, sleep=time.sleep):
        self.args, self.report, self.clock, self.sleep = args, report, clock, sleep
        self.channel = None
        self.cancelled = False
        self.stop = Path(args.stop_file)
        self.last = None
        self.last_progress = 0
        self.profile_started = clock()
        self.configured = False
        self.bounds = (0, 100) if args.full_stroke else (15, 35)

    def check(self):
        if self.cancelled or self.stop.exists():
            raise base.CheckFailed("Stop requested; future targets cancelled")
        if self.clock() - self.profile_started > 900:
            raise base.CheckFailed("Hard overall duration limit 900s")

    def emit(self, event, **fields):
        print(json.dumps({"event": event, **fields}), flush=True)

    def open(self):
        self.check()
        if "UP" in base.ip_snapshot(self.args.interface)["flags"]:
            raise base.CheckFailed("Refuse to take over an active CAN interface")
        self.configured = True
        base.configure(self.args.interface, "type", "can", "bitrate", "1000000", "sample-point", "0.8", "listen-only", "off")
        base.configure(self.args.interface, "up")
        snapshot = base.ip_snapshot(self.args.interface)
        data = snapshot.get("linkinfo", {}).get("info_data", {})
        if data.get("bittiming", {}).get("bitrate") != 1000000 or "LISTEN-ONLY" in data.get("ctrlmode", []):
            raise base.CheckFailed("Normal 1Mbps mode did not read back")
        self.channel = base.CanChannel(self.args.interface, self.args.can_id, .3, self.report)
        self.report["interface_opens"].append({"time_utc": now(), "snapshot": snapshot})

    def close(self):
        if self.channel is not None:
            self.channel.close()
            self.channel = None
        if self.configured:
            base.configure(self.args.interface, "down")

    def step(self, target, *, hz=20, deadline=12, hold=.4, limits=None, duration=None, label="position"):
        self.check()
        command = base.encode_command(target, limits, bounds=self.bounds)
        stage = {"label": label, "target_percent": target, "target_byte": command[1],
                 "command_hex": command.hex(), "hz": hz, "started_at_utc": now(), "feedback": []}
        self.report["stages"].append(stage)
        started = self.clock()
        next_tx = started
        stable = 0
        stable_since = None
        drift_since = None
        limit = duration if duration is not None else deadline
        while self.clock() - started < limit:
            self.check()
            feedback = self.channel.exchange(command)
            feedback["stage_elapsed_s"] = self.clock() - started
            stage["feedback"].append(feedback)
            self.last = feedback
            if duration is not None and "first_stable_at_s" in stage and abs(feedback["position_raw"] - command[1]) > 3:
                drift_since = self.clock() if drift_since is None else drift_since
                if self.clock() - drift_since > .5:
                    raise base.CheckFailed("Position hold drift exceeded three counts for 500ms")
            else:
                drift_since = None
            if base.arrived(feedback, command[1]):
                stable += 1
                if stable == 3:
                    stable_since = self.clock()
                    stage.setdefault("first_stable_at_s", self.clock() - started)
                if duration is None and stable >= 3 and self.clock() - stable_since >= hold:
                    break
            else:
                stable = 0
                stable_since = None
            if self.clock() - self.last_progress >= 20:
                self.last_progress = self.clock()
                self.emit("progress", label=label, elapsed_s=round(self.clock() - started, 1),
                          position_percent=round(feedback["position_percent"], 3),
                          feedback_count=sum(len(s["feedback"]) for s in self.report["stages"]))
            # Do not catch up by bursting after a delayed response.
            next_tx = max(next_tx + 1 / hz, self.clock())
            self.sleep(max(0, next_tx - self.clock()))
        if stable < 3:
            raise base.CheckFailed(f"Stage {label} did not finish at target {target}%")
        stage.update(completed=True, ended_at_utc=now(), sample_count=len(stage["feedback"]),
                     last_feedback=stage["feedback"][-1], elapsed_s=self.clock() - started)
        self.emit("stage_completed", label=label, target_percent=target, samples=stage["sample_count"],
                  first_stable_at_s=stage.get("first_stable_at_s"), final_position_percent=stage["last_feedback"]["position_percent"])

    def profile(self):
        self.open()
        if self.args.profile == "soak":
            self.step(25, label="soak_20Hz_300s", duration=300, hz=20)
        elif self.args.profile == "motion":
            sequence = [15, 35, 20, 30, 18, 32, 25, 15] * 5
            for index, target in enumerate(sequence):
                self.step(target, label=f"repeat_{index + 1:02d}")
            self.step(15, limits={**base.LIMITS, "velocity": 10}, label="speed10_prepare")
            self.step(35, limits={**base.LIMITS, "velocity": 10}, label="speed10_open")
            self.step(15, label="speed25_close")
            self.step(35, label="speed25_open")
            self.step(25, label="motion_final25")
        elif self.args.profile == "rates":
            self.step(25, label="rates_prepare")
            for hz in (5, 20, 50, 100):
                self.step(25, hz=hz, duration=30, label=f"rate_{hz}Hz_30s")
        elif self.args.profile == "reopen":
            self.step(25, label="reopen_prepare")
            for index in range(10):
                self.close()
                self.report["reopen_cycles"].append({"cycle": index + 1, "down": base.ip_snapshot(self.args.interface)})
                self.sleep(.5)
                self.open()
                self.step(25, label=f"reopen_{index + 1:02d}", duration=1, hz=20)
        elif self.args.profile == "full-stroke":
            if not self.args.full_stroke:
                raise base.CheckFailed("Full stroke not explicitly selected")
            for cycle in range(2):
                for index, target in enumerate((5, 30, 55, 80, 100, 75, 50, 25, 0, 15)):
                    self.step(target, label=f"full_{cycle + 1}_{index + 1:02d}")
            self.step(25, label="full_final25")
        elif self.args.profile == "cancel":
            self.step(25, label="cancel_prepare")
            marker = len(self.report["events"])
            self.cancelled = True
            try:
                self.step(35, label="must_not_send35")
            except base.CheckFailed:
                pass
            else:
                raise base.CheckFailed("Cancellation gate failed")
            if any(e["kind"] == "tx" for e in self.report["events"][marker:]):
                raise base.CheckFailed("Sent a command after cancellation")
            self.report["cancel_gate_passed"] = True
            self.cancelled = False
            self.close()
            self.sleep(.5)
            self.open()
            self.step(25, label="cancel_resume_known25")
        elif self.args.profile == "interface-loss":
            self.step(25, label="interface_loss_prepare")
            marker = len(self.report["events"])
            before = base.ip_snapshot(self.args.interface)
            base.configure(self.args.interface, "down")
            try:
                self.step(35, label="blocked35_interface_down")
            except OSError as exc:
                if exc.errno != 100:
                    raise
                self.report["expected_interface_error"] = repr(exc)
            else:
                raise base.CheckFailed("Interface loss did not stop the command path")
            new_events = self.report["events"][marker:]
            after = base.ip_snapshot(self.args.interface)
            if any(e.get("send_confirmed") for e in new_events if e["kind"] == "tx") or after["stats64"]["tx"]["packets"] != before["stats64"]["tx"]["packets"]:
                raise base.CheckFailed("A command escaped while interface was down")
            self.report["stages"][-1]["expected_interface_failure"] = True
            self.report["interface_loss_gate_passed"] = True
            self.close()
            self.sleep(.5)
            self.open()
            self.step(25, label="interface_loss_explicit_resume25")
        else:
            raise ValueError("Unknown bounded profile")


def execute(args):
    import fcntl
    report = {"started_at_utc": now(), "status": "preflight", "profile": args.profile,
              "full_stroke_authorized": args.full_stroke, "interface": args.interface, "can_id": args.can_id,
              "events": [], "stages": [], "interface_opens": [], "reopen_cycles": [],
              "motor_disable_requested": False, "motor_disable_command_available": False,
              "clock_semantics": "host monotonic receive time; no device timestamp"}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    acceptance = Acceptance(args, report)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda _sig, _frame: setattr(acceptance, "cancelled", True))
    with output.open("x", encoding="utf-8") as evidence, open(f"/tmp/omnipicker-{args.interface}.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            report["identity"] = base.validate_usb_interface(args.interface)
            report["before"] = base.ip_snapshot(args.interface)
            report["uptime_start_s"] = float(Path("/proc/uptime").read_text().split()[0])
            report["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            report["protocol_source_sha256"] = hashlib.sha256(Path(base.__file__).read_bytes()).hexdigest()
            report["status"] = "running"
            acceptance.profile()
            report["status"] = "completed"
        except BaseException as exc:
            report.update(status="aborted", error=repr(exc))
            print(json.dumps({"event": "aborted", "error": repr(exc),
                              "notice": "Remaining targets cancelled; interface down does not disable motor"}), flush=True)
        finally:
            try:
                report["before_interface_down"] = base.ip_snapshot(args.interface)
                acceptance.close()
                report["after"] = base.ip_snapshot(args.interface)
            except BaseException as exc:
                report.update(status="aborted_cleanup_unconfirmed", cleanup_error=repr(exc))
            report["ended_at_utc"] = now()
            report["last_feedback"] = acceptance.last
            report["final_motor_state"] = "Powered motor may hold last target; not disabled by CAN"
            report["kernel_log"] = subprocess.run(["dmesg"], capture_output=True, text=True, timeout=3).stdout
            json.dump(report, evidence, indent=2)
            evidence.write("\n")
    samples = [f for s in report["stages"] for f in s["feedback"]]
    print(json.dumps({"event": "finished", "profile": args.profile, "status": report["status"],
                      "tx_frames": sum(e["kind"] == "tx" for e in report["events"]), "feedback_count": len(samples),
                      "rtt_median_ms": statistics.median(f["round_trip_ms"] for f in samples) if samples else None,
                      "output": str(output)}), flush=True)
    return 0 if report["status"] == "completed" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--profile", choices=("soak", "motion", "rates", "reopen", "cancel", "interface-loss", "full-stroke"), required=True)
    parser.add_argument("--full-stroke", action="store_true")
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--can-id", type=int, default=1)
    parser.add_argument("--stop-file", default="/root/omnipicker-can/STOP")
    parser.add_argument("--output")
    args = parser.parse_args()
    if not 1 <= args.can_id <= base.SFF_MASK or args.profile == "full-stroke" and not args.full_stroke:
        parser.error("Use a known standard CAN ID; full stroke requires --full-stroke")
    if not args.execute:
        print(json.dumps({"mode": "offline_preview", "profile": args.profile,
                          "bounds_percent": [0, 100] if args.full_stroke else [15, 35],
                          "limits_raw": base.LIMITS, "hard_limit_seconds": 900,
                          "motor_disable_available": False}, indent=2))
        return 0
    if not args.output:
        parser.error("Explicit --execute requires a new --output path")
    return execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
