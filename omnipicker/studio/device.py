from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import struct

from .protocol import FibreChannel, ProtocolError, UsbTransport

ROOT = Path(__file__).resolve().parents[1]
LIMIT_NAMES = ("velocity_limit", "traj_vel", "traj_acc", "traj_dcc", "ramp_vel", "current_limit")
FORMATS = {"bool": "?", "uint8": "B", "uint32": "I", "int32": "i", "float": "f"}


def opening_fraction(percent: float) -> float:
    value = float(percent)
    if not math.isfinite(value) or not 0 <= value <= 100:
        raise ValueError("开度必须在 0–100% 之间")
    return round(value, 1) / 100.0


def flatten_schema(entries, prefix=""):
    result = {}
    for item in entries:
        name = prefix + item["name"]
        if item["type"] == "object":
            result.update(flatten_schema(item["members"], name + "."))
        else:
            if name in result:
                raise ProtocolError("Duplicate schema name")
            result[name] = item
    return result


class NativeDevice:
    simulated = False

    def __init__(self, serial="20773954464D", transport_factory=UsbTransport):
        self.serial = serial
        self.transport_factory = transport_factory
        self.transport = None
        self.compatible = False
        self.cancel_motion = lambda: False

    def open(self):
        self.transport = self.transport_factory(self.serial)
        try:
            self.channel = FibreChannel(self.transport)
            self.raw_schema = self.channel.read_schema()
            self.schema = json.loads(self.raw_schema.decode("ascii"))
            self.entries = flatten_schema(self.schema)
            self.firmware = int(self.read("fw_version"))
            self.internal_serial = int(self.read("serial_number"))
            self.can_id = int(self.read("can_node_id"))
            # Each firmware has different endpoint IDs; only exact observed
            # interfaces are approved. Resolve all endpoints from this device.
            expected = None
            if self.firmware in (330, 335):
                expected = json.loads((ROOT / f"assets/schema-fw{self.firmware}.json").read_text(encoding="utf-8"))
            self.compatible = self.schema == expected and int(self.read("device_type")) == 1
            return self
        except BaseException:
            self.close()
            raise

    def _read_entry(self, entry):
        fmt = "<" + FORMATS[entry["type"]]
        return struct.unpack(fmt, self.channel.operation(entry["id"], length=struct.calcsize(fmt)))[0]

    def read(self, path):
        entry = self.entries.get(path)
        if not entry or entry["type"] not in FORMATS or "r" not in entry.get("access", ""):
            raise ProtocolError("不支持读取属性：" + path)
        return self._read_entry(entry)

    def _write_entry(self, entry, value):
        if not self.compatible:
            raise ProtocolError("接口或固件未经验证，仅允许读取")
        if "w" not in entry.get("access", ""):
            raise ProtocolError("属性不可写")
        self.channel.operation(entry["id"], struct.pack("<" + FORMATS[entry["type"]], value))

    def _call(self, name, args=()):
        if not self.compatible:
            raise ProtocolError("设备处于只读模式")
        function = self.entries[name]
        if len(args) != len(function["inputs"]):
            raise ValueError("Incorrect function argument count")
        for entry, value in zip(function["inputs"], args):
            self._write_entry(entry, value)
        if name == "motor.ctrl.set_pos" and self.cancel_motion():
            return False
        # Function arguments are separate endpoints, then a trigger, then result.
        self.channel.operation(function["id"])
        outputs = function["outputs"]
        return self._read_entry(outputs[0]) if outputs else None

    def request_state(self, enabled):
        result = self._call("motor.request_state", (1 if enabled else 0,))
        if result is not True:
            raise ProtocolError("使能状态请求未被设备接受")

    def set_opening(self, percent):
        return self._call("motor.ctrl.set_pos", (opening_fraction(percent),)) is not False

    def fault(self):
        return int(self.read("motor.error_code"))

    def limits(self):
        return {name: float(self.read("motor.config." + name)) for name in LIMIT_NAMES}

    def set_limit(self, name, value):
        if name not in LIMIT_NAMES or not math.isfinite(value) or value <= 0:
            raise ValueError("不支持的运动限幅")
        self._write_entry(self.entries["motor.config." + name], float(value))
        actual = float(self.read("motor.config." + name))
        if not math.isclose(actual, value, rel_tol=1e-5, abs_tol=1e-6):
            raise ProtocolError("参数读回不一致：" + name)
        return actual

    def clear_fault(self):
        self._call("motor.clear_error")

    @property
    def rtt_ms(self):
        return self.channel.last_rtt_ms

    def close(self):
        if self.transport is not None:
            try:
                self.transport.close()
            finally:
                self.transport = None


class MockDevice:
    """Explicit simulation, never presented as observed hardware telemetry."""
    simulated = True
    firmware = 330
    internal_serial = 0x3954464D
    can_id = 1
    compatible = True
    rtt_ms = 1.2

    def __init__(self, serial="DEMO"):
        self.serial = serial
        self.error = 0
        self.calls = []
        self.cancel_motion = lambda: False
        self.values = dict(velocity_limit=2.5, traj_vel=1.8, traj_acc=10.0, traj_dcc=10.0, ramp_vel=10.0, current_limit=3.0)
        self.closed = False
        self.fail_next = False
        self.simulation_target = 0.0
        self.simulation_enabled = False

    def open(self):
        self.closed = False
        return self

    def _check(self):
        if self.closed or self.fail_next:
            self.fail_next = False
            raise TimeoutError("模拟通信超时")

    def read(self, path):
        self._check()
        if path == "motor.config.ctrl_mode": return 5
        if path == "motor.config.calib_valid": return True
        if path == "motor.error_code": return self.error
        raise ValueError(path)

    def request_state(self, enabled):
        self._check()
        self.calls.append(("state", bool(enabled)))
        self.simulation_enabled = bool(enabled)

    def set_opening(self, percent):
        self._check()
        value = opening_fraction(percent)
        if self.cancel_motion(): return False
        self.calls.append(("opening", value))
        self.simulation_target = value
        return True

    def fault(self):
        self._check()
        return self.error

    def limits(self):
        self._check()
        return copy.deepcopy(self.values)

    def set_limit(self, name, value):
        self._check()
        self.values[name] = value
        self.calls.append(("limit", name, value))
        return value

    def clear_fault(self):
        self._check()
        self.error = 0
        self.calls.append(("clear_fault",))

    def close(self):
        self.closed = True
