"""Bounded native Type-C CDC probe: FC03 only, registers 20..24 and 0.

Default is offline preview. --execute opens only the enumerated gripper port.
Never enables, moves, clears faults, resets, or changes gripper configuration.
Serial line coding is configured on the host; DTR/RTS stay deasserted.
"""
from datetime import datetime
import argparse
import json
from pathlib import Path
import re
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from studio.device import NativeDevice

READS = ((20, 5), (0, 1), (20, 5))


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def read_request(address, start, count):
    if not 1 <= address <= 247 or (start, count) not in READS:
        raise ValueError("Only documented read-only register ranges are permitted")
    data = struct.pack(">BBHH", address, 3, start, count)
    return data + struct.pack("<H", crc16_modbus(data))


def decode_response(data, address, count, request):
    if not data:
        return {"kind": "no_response"}
    if data == request:
        return {"kind": "request_echo_not_feedback"}
    if len(data) < 5 or crc16_modbus(data[:-2]) != int.from_bytes(data[-2:], "little"):
        return {"kind": "invalid_length_or_crc"}
    if data[0] != address:
        return {"kind": "wrong_address"}
    if data[1] == 0x83 and len(data) == 5:
        return {"kind": "modbus_exception", "exception_code": data[2]}
    if data[1] != 3 or data[2] != count * 2 or len(data) != 5 + count * 2:
        return {"kind": "unexpected_function_or_length"}
    return {"kind": "registers", "values": list(struct.unpack(">" + "H" * count, data[3:-2]))}


def csv_stream_summary(data):
    """Recognize only the observed three-decimal-column stream; label no units."""
    if not data or any(byte not in b"0123456789-+.,\r\n" for byte in data):
        return None
    complete = data.split(b"\n")[1:-1]  # edge fragments may be partial records
    pattern = rb"[+-]?\d+\.\d{2},[+-]?\d+\.\d{2},[+-]?\d+\.\d{2}\r?"
    if len(complete) < 3 or any(re.fullmatch(pattern, line) is None for line in complete):
        return None
    values = [list(map(float, line.split(b","))) for line in complete]
    return {"complete_rows_excluding_edges": len(values), "first_rows": values[:5],
            "minimum": [min(row[i] for row in values) for i in range(3)],
            "maximum": [max(row[i] for row in values) for i in range(3)],
            "column_units": "unverified"}


def find_response_in_stream(data, address, count, request):
    matches = []
    for offset in range(len(data) - 4):
        if data[offset] != address or data[offset + 1] not in (3, 0x83):
            continue
        size = 5 if data[offset + 1] == 0x83 else 5 + data[offset + 2]
        candidate = data[offset:offset + size]
        decoded = decode_response(candidate, address, count, request)
        if decoded["kind"] in ("registers", "modbus_exception"):
            matches.append({"offset": offset, "hex": candidate.hex(" "), "decoded": decoded})
    return matches


def collect(port, seconds):
    begin = time.monotonic()
    chunks, data = [], bytearray()
    while time.monotonic() - begin < seconds:
        part = port.read(min(max(port.in_waiting, 1), 256))
        if part:
            chunks.append({"elapsed_ms": (time.monotonic() - begin) * 1000, "hex": part.hex(" ")})
            data.extend(part)
            if len(data) > 4096:
                raise RuntimeError("Unexpected serial stream exceeds bounded probe size")
    return bytes(data), chunks


def native_status(device):
    names = ("fw_version", "serial_number", "can_node_id", "modbus_mode", "rs485_mode",
             "uart_baudrate", "uart_stopbits", "motor.error_code", "motor.config.calib_valid")
    return {name: device.read(name) for name in names}


def execute(output, allow_csv_stream=False):
    import serial
    from serial.tools import list_ports

    if output.exists():
        raise FileExistsError("Refusing to overwrite probe evidence")
    report = {"started_at": datetime.now().astimezone().isoformat(), "events": [],
              "allow_previously_inspected_csv_stream": allow_csv_stream,
              "host_serial": {"baudrate": 115200, "bytesize": 8, "parity": "N", "stopbits": 1,
                              "dtr": False, "rts": False, "flow_control": False},
              "device_configuration_writes": [], "motion_commands": [],
              "limitations": ["DTR/RTS deasserted; no other line state, baud rate or address scanned",
                              "Host serial driver may issue CDC line coding/control requests on open",
                              "No hardware UART/RS485 positive control in this test"]}
    device, port = NativeDevice(), None
    try:
        device.open()
        if not (device.compatible and device.firmware == 335 and device.internal_serial == 0x3954464D):
            raise RuntimeError("Unexpected device identity/schema")
        before = report["before"] = native_status(device)
        if (before["can_node_id"] != 1 or before["modbus_mode"] is not True
                or before["uart_baudrate"] != 115200 or before["uart_stopbits"] != 0
                or before["motor.error_code"] != 0):
            raise RuntimeError("Unexpected current device settings; no configuration changes allowed")
        candidates = [p for p in list_ports.comports() if (p.vid, p.pid, p.serial_number)
                      == (0x1001, 0x03EF, device.serial)]
        if len(candidates) != 1:
            raise RuntimeError("Expected one matching native CDC port")
        match = candidates[0]
        report["port"] = {"name": match.device, "hwid": match.hwid}
        port = serial.Serial(port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                             timeout=.05, write_timeout=.5, xonxoff=False, rtscts=False, dsrdtr=False)
        port.dtr = False
        port.rts = False
        port.port = match.device
        port.open()
        passive_seconds = 2 if allow_csv_stream else .5
        passive, chunks = collect(port, passive_seconds)
        csv_summary = csv_stream_summary(passive)
        report["passive"] = {"hex": passive.hex(" "), "chunks": chunks,
                             "collection_seconds": passive_seconds, "csv": csv_summary}
        if passive and not (allow_csv_stream and csv_summary):
            raise RuntimeError("Unexpected unsolicited serial data; inspect before transmitting")
        after_open = report["after_open"] = native_status(device)
        if after_open != before:
            raise RuntimeError("Status changed on opening port; abort probe")
        for start, count in READS:
            frame = read_request(1, start, count)
            event = {"time": datetime.now().astimezone().isoformat(), "monotonic": time.monotonic(),
                     "start_register": start, "register_count": count, "tx_hex": frame.hex(" ")}
            report["events"].append(event)
            if port.write(frame) != len(frame):
                raise RuntimeError("Incomplete serial write; not retrying")
            event["write_accepted_by_host"] = True
            received, chunks = collect(port, 1.2)
            event.update(rx_hex=received.hex(" "), rx_chunks=chunks,
                         decoded=decode_response(received, 1, count, frame))
            if allow_csv_stream:
                event["modbus_frames"] = find_response_in_stream(received, 1, count, frame)
                event["csv"] = csv_stream_summary(received)
                if len(event["modbus_frames"]) == 1:
                    event["decoded"] = event["modbus_frames"][0]["decoded"]
                elif event["csv"] and not event["modbus_frames"]:
                    event["decoded"] = {"kind": "csv_stream_only_no_modbus_response"}
            event["fibre_status"] = native_status(device)
            if event["fibre_status"] != before:
                raise RuntimeError("Device properties changed during probe")
            if event["decoded"]["kind"] not in ("no_response", "registers", "modbus_exception",
                                                 "csv_stream_only_no_modbus_response"):
                raise RuntimeError("Unexpected serial reply; inspect before further requests")
            time.sleep(.2)
        report["after"] = native_status(device)
        registers = [e for e in report["events"] if e["decoded"]["kind"] == "registers"]
        version_matches = any(e["start_register"] == 0 and e["decoded"]["values"] == [335] for e in registers)
        report["status"] = ("version_and_status_response_received" if version_matches and len(registers) == 3
                            else "limited_probe_no_confirmed_modbus_support")
    except BaseException as exc:
        report.update(status="aborted_requires_inspection", error=repr(exc))
        raise
    finally:
        if port is not None:
            port.close()
        device.close()
        report["ended_at"] = datetime.now().astimezone().isoformat()
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"status": report.get("status"), "error": report.get("error"),
                          "port": report.get("port"), "before": report.get("before"),
                          "after": report.get("after"),
                          "passive_csv": report.get("passive", {}).get("csv"),
                          "events": [{k: e[k] for k in ("tx_hex", "decoded", "csv") if k in e}
                                     for e in report["events"]], "evidence": str(output)}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-inspected-csv-stream", action="store_true",
                        help="Continue only when unsolicited data matches the inspected three-column stream")
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({"mode": "offline_preview", "requests": [read_request(1, *r).hex(" ") for r in READS]}))
    elif args.output is None:
        parser.error("--execute requires --output")
    else:
        execute(args.output, args.allow_inspected_csv_stream)
