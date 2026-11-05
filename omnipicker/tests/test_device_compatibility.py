"""Exercise native discovery and writes against both observed USB schemas."""
import copy
import json
import struct

import pytest

from studio.device import FORMATS, NativeDevice, ROOT, flatten_schema
from studio.protocol import ProtocolError, crc16


class EmulatedUsb:
    def __init__(self, schema, firmware, device_type=1):
        self.raw = json.dumps(schema, separators=(",", ":")).encode("ascii")
        self.entries = flatten_schema(schema)
        self.calls = []
        self.closed = False
        self.values = {
            "fw_version": firmware, "serial_number": 0x3954464D,
            "can_node_id": 1, "device_type": device_type,
            "motor.error_code": 0, "motor.config.traj_vel": 1.8,
        }
        self.endpoints = {entry["id"]: (path, entry) for path, entry in self.entries.items()}
        for path, entry in self.entries.items():
            if entry["type"] == "function":
                for child in entry["inputs"] + entry["outputs"]:
                    name = path + "." + child["name"]
                    self.endpoints[child["id"]] = (name, child)
        self.values["motor.request_state.result"] = True

    def write(self, packet, timeout):
        seq, endpoint, length = struct.unpack_from("<HHH", packet)
        endpoint &= 0x7FFF
        payload = packet[6:-2]
        assert struct.unpack("<H", packet[-2:])[0] == (1 if endpoint == 0 else crc16(self.raw))
        if endpoint == 0:
            offset = struct.unpack("<I", payload)[0]
            data = self.raw[offset:offset + length]
        else:
            path, entry = self.endpoints[endpoint]
            self.calls.append((path, endpoint, payload, length))
            if payload:
                self.values[path] = struct.unpack("<" + FORMATS[entry["type"]], payload)[0]
            data = struct.pack("<" + FORMATS[entry["type"]], self.values[path]) if length else b""
        self.reply = struct.pack("<H", seq | 0x8000) + data

    def read(self, timeout):
        return self.reply

    def close(self):
        self.closed = True


def schema_for(firmware):
    return json.loads((ROOT / f"assets/schema-fw{firmware}.json").read_bytes())


@pytest.mark.parametrize("firmware,state_ids,pos_ids,limit_id", [
    (330, [9, 8, 10], [65, 64], 26),
    (335, [14, 13, 15], [70, 69], 31),
])
def test_exact_schema_discovery_and_dynamic_command_endpoints(firmware, state_ids, pos_ids, limit_id):
    transport = EmulatedUsb(schema_for(firmware), firmware)
    device = NativeDevice(transport_factory=lambda _: transport).open()
    try:
        assert device.compatible
        assert all(not payload and length for _, _, payload, length in transport.calls)
        transport.calls.clear()
        device.request_state(False)
        assert [call[1] for call in transport.calls] == state_ids
        assert transport.calls[0][2] == b"\0"
        transport.calls.clear()
        device.set_opening(35.1)
        assert [call[1] for call in transport.calls] == pos_ids
        assert transport.calls[0][2] == struct.pack("<f", .351)
        transport.calls.clear()
        assert device.set_limit("traj_vel", .18) == pytest.approx(.18)
        assert [call[1] for call in transport.calls] == [limit_id, limit_id]
    finally:
        device.close()
    assert transport.closed


@pytest.mark.parametrize("firmware,device_type,mutation", [
    (336, 1, False), (330, 1, False), (335, 2, False), (335, 1, True),
])
def test_unknown_version_wrong_schema_or_type_stays_readonly(firmware, device_type, mutation):
    schema = copy.deepcopy(schema_for(335))
    if mutation:
        flatten_schema(schema)["motor.ctrl.set_pos"]["inputs"][0]["type"] = "uint32"
    transport = EmulatedUsb(schema, firmware, device_type)
    device = NativeDevice(transport_factory=lambda _: transport).open()
    try:
        assert not device.compatible
        assert device.fault() == 0
        count = len(transport.calls)
        for operation in (lambda: device.request_state(False), lambda: device.set_opening(15),
                          lambda: device.set_limit("traj_vel", .18), device.clear_fault):
            with pytest.raises(ProtocolError):
                operation()
        assert len(transport.calls) == count
    finally:
        device.close()
