"""335-only, bounded 15 -> 35 -> 15 percent CDC/Fibre experiment.

Default is offline preview. Requires the previously authorized clear bench.
No serial application writes, flash, calibration, reset, or fault clearing.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from studio.device import NativeDevice, FORMATS, LIMIT_NAMES

TARGETS = (15.0, 35.0, 15.0)
DWELL = 6.0
ROW = re.compile(rb"[+-]?\d+\.\d{2},[+-]?\d+\.\d{2},[+-]?\d+\.\d{2}\r?")
STOP_FILE = ROOT.parent / "tmp/omnipicker_motion_stop"


class Stream:
    def __init__(self):
        self.pending = b""
        self.aligned = False
        self.rows, self.chunks, self.invalid = [], [], []
        self.error = None
        self.halt = threading.Event()

    def feed(self, data, timestamp):
        self.chunks.append({"t": timestamp, "hex": data.hex()})
        self.pending += data
        while b"\n" in self.pending:
            line, self.pending = self.pending.split(b"\n", 1)
            if not self.aligned:
                self.aligned = True  # first line may start before serial open
                continue
            if not ROW.fullmatch(line) or len(line) > 100:
                self.invalid.append({"t": timestamp, "hex": line.hex()})
                continue
            values = list(map(float, line.split(b",")))
            if not all(math.isfinite(v) for v in values):
                self.invalid.append({"t": timestamp, "hex": line.hex()})
                continue
            self.rows.append({"t": timestamp, "raw": line.decode("ascii"), "values": values})
        if len(self.pending) > 1024:
            raise RuntimeError("CDC line exceeds bounded size")

    def run(self, port):
        try:
            while not self.halt.is_set():
                data = port.read(min(max(port.in_waiting, 1), 512))
                if data:
                    self.feed(data, time.perf_counter())
        except BaseException as exc:
            self.error = repr(exc)

    def check(self, now):
        if self.error or self.invalid:
            raise RuntimeError(f"CDC invalid/disconnected: {self.error}, invalid={len(self.invalid)}")
        if not self.rows or now - self.rows[-1]["t"] > .3:
            raise TimeoutError("CDC feedback stale > 300 ms")


def reduced_limits(baseline):
    if set(baseline) != set(LIMIT_NAMES) or any(not math.isfinite(v) or v <= 0 for v in baseline.values()):
        raise ValueError("Invalid baseline limits")
    return {k: min(v, 1.0) if k == "current_limit" else v * .1 for k, v in baseline.items()}


def snapshot(device):
    return {k: device.read(k) for k, v in device.entries.items()
            if v["type"] in FORMATS and "r" in v.get("access", "")}


def experiment(device, stream, report, stop_file=STOP_FILE, dwell=DWELL, clock=time.perf_counter, sleep=time.sleep):
    """Only this thread owns Fibre; cleanup never replays a failed target."""
    disabled = False
    needs_stop = False
    changed = False
    baseline = device.limits()
    report["baseline_limits"] = baseline
    report["temporary_limits"] = reduced_limits(baseline)
    report["events"] = []
    report["fault_samples"] = []

    def call(kind, fn, **fields):
        event = {"kind": kind, "t_start": clock(), **fields}
        report["events"].append(event)
        try:
            result = fn()
            if result is False:
                raise RuntimeError(f"Request rejected: {kind}")
            event.update(confirmed=True, t_ack=clock())
            print(json.dumps(event), flush=True)
        except BaseException as exc:
            event.update(confirmed=False, t_end=clock(), error=repr(exc))
            raise

    def health():
        if stop_file.exists():
            raise RuntimeError("Stop file present")
        stream.check(clock())
        fault = device.fault()
        report["fault_samples"].append({"t": clock(), "fault": fault})
        if fault:
            raise RuntimeError(f"Device fault {fault}")

    device.cancel_motion = lambda: stop_file.exists() or bool(stream.error or stream.invalid)
    try:
        health()
        call("initial_disable", lambda: device.request_state(False))
        disabled = True
        for name, value in report["temporary_limits"].items():
            health()
            changed = True  # write may succeed even if its ACK is lost
            call("temporary_limit", lambda n=name, v=value: device.set_limit(n, v), name=name, value=value)
        health()
        disabled = False  # outcome is unknown until the request is acknowledged
        needs_stop = True
        call("enable", lambda: device.request_state(True))
        for target in TARGETS:
            health()
            call("target", lambda p=target: device.set_opening(p), percent=target)
            end = clock() + dwell
            while clock() < end:
                health()
                sleep(min(.2, max(0, end - clock())))
        report["motion_sequence_completed"] = True
    finally:
        if needs_stop:
            try:
                call("final_disable", lambda: device.request_state(False))
                disabled = True
            except BaseException as exc:
                report["stop_error"] = repr(exc)
        report["disable_confirmed"] = disabled
        if changed and disabled:
            restore_errors = []
            for name, value in baseline.items():
                try:
                    call("restore_limit", lambda n=name, v=value: device.set_limit(n, v), name=name, value=value)
                except BaseException as exc:
                    restore_errors.append({"name": name, "error": repr(exc)})
            report["restore_errors"] = restore_errors
            report["limits_restored"] = not restore_errors
        elif changed:
            report["limits_restored"] = False
        report["status"] = ("completed" if report.get("motion_sequence_completed") and disabled
                            and report.get("limits_restored") else "aborted_or_cleanup_unconfirmed")


def execute(output):
    import serial
    from serial.tools import list_ports

    report = {"started_at": datetime.now().astimezone().isoformat(), "status": "preflight",
              "clock": "time.perf_counter; host receive time, not device sample time",
              "serial_application_writes": [], "targets_percent": TARGETS, "dwell_s": DWELL}
    # Reserve the evidence path before touching hardware. Never overwrite prior evidence.
    with output.open("x", encoding="utf-8") as f:
        json.dump(report, f)
    device, port, thread, stream = NativeDevice(), None, None, Stream()
    try:
        if STOP_FILE.exists():
            raise RuntimeError("Stop file present; inspect before a new session")
        device.open()
        if not (device.compatible and device.firmware == 335 and device.internal_serial == 0x3954464D):
            raise RuntimeError("Unexpected device/schema")
        before = report["before"] = snapshot(device)
        report["schema_sha256"] = hashlib.sha256(device.raw_schema).hexdigest()
        if (before["motor.error_code"] != 0 or before["motor.config.ctrl_mode"] != 5
                or before["motor.config.calib_valid"] is not True or before["debug_flag"] != 56):
            raise RuntimeError("Unexpected fault/mode/calibration/debug format")
        ports = [p for p in list_ports.comports() if (p.vid, p.pid, p.serial_number)
                 == (0x1001, 0x03EF, device.serial)]
        if len(ports) != 1:
            raise RuntimeError("Expected one matching CDC port")
        port = serial.Serial(port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                             timeout=.05, xonxoff=False, rtscts=False, dsrdtr=False)
        port.dtr = False
        port.rts = False
        port.port = ports[0].device
        report["port"] = port.port
        report["serial_settings"] = {"baud": 115200, "format": "8N1", "dtr": False, "rts": False}
        port.open()
        thread = threading.Thread(target=stream.run, args=(port,), daemon=True)
        thread.start()
        time.sleep(1)
        stream.check(time.perf_counter())
        if len(stream.rows) < 40:
            raise RuntimeError("Insufficient CDC records before motion")
        experiment(device, stream, report)
        time.sleep(.6)  # also observe the disabled tail
        report["after"] = snapshot(device)
        report["property_changes"] = {k: [v, report["after"][k]] for k, v in before.items()
                                      if report["after"][k] != v}
    except BaseException as exc:
        report.update(status="aborted_or_cleanup_unconfirmed", error=repr(exc))
    finally:
        stream.halt.set()
        if thread:
            thread.join(timeout=1)
        if port:
            port.close()
        device.close()
        report.update(rows=stream.rows, chunks=stream.chunks, invalid=stream.invalid,
                      trailing_fragment_hex=stream.pending.hex(), stream_error=stream.error,
                      ended_at=datetime.now().astimezone().isoformat())
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: v for k, v in report.items() if k not in
                          ("rows", "chunks", "before", "after", "events", "fault_samples")}, indent=2))
    return report["status"] == "completed"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({"mode": "offline_preview", "targets": TARGETS, "dwell_s": DWELL,
                          "speed_factor": .1, "current_limit_max": 1, "writes_flash": False}))
    elif args.output is None:
        parser.error("--execute requires a new --output path")
    else:
        sys.exit(0 if execute(args.output) else 1)
