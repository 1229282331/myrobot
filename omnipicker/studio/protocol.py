"""Small, serial Fibre packet client for the native USB bulk interface.

Wire layout follows REF-CLI 1.0.3's inspected protocol, not the CAN protocol.
Every transaction has one attempt. Motion/function writes are never replayed.
"""
from __future__ import annotations

import struct
import time


class ProtocolError(RuntimeError):
    pass


def crc16(data: bytes, initial: int = 1) -> int:
    """Non-reflected CRC16, polynomial 0x3D65; JSON starts with version 1."""
    value = initial
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x3D65 if value & 0x8000 else 0)) & 0xFFFF
    return value


def encode_request(seq: int, endpoint: int, payload: bytes, length: int, crc: int) -> bytes:
    if not 0 <= endpoint <= 0x7FFF or len(payload) >= 128 or not 0 <= length <= 512:
        raise ValueError("Invalid Fibre transaction")
    return struct.pack("<HHH", seq, endpoint | 0x8000, length) + payload + struct.pack("<H", 1 if endpoint == 0 else crc)


class FibreChannel:
    def __init__(self, transport, timeout: float = 0.5):
        self.transport = transport
        self.timeout = timeout
        self.sequence = 0
        self.interface_crc = 0
        self.last_rtt_ms = 0.0

    def operation(self, endpoint: int, payload: bytes = b"", length: int = 0) -> bytes:
        self.sequence = (self.sequence + 1) & 0x7FFF
        # This bit is set by the official implementation to avoid reserved IDs.
        sequence = self.sequence | 0x80
        packet = encode_request(sequence, endpoint, payload, length, self.interface_crc)
        begin = time.perf_counter()
        deadline = begin + self.timeout
        self.transport.write(packet, self.timeout)
        while time.perf_counter() < deadline:
            reply = bytes(self.transport.read(max(0.001, deadline - time.perf_counter())))
            if len(reply) < 2:
                raise ProtocolError("Truncated USB response")
            ack = struct.unpack_from("<H", reply)[0]
            if ack != (sequence | 0x8000):
                continue  # Never interpret a stale or unsolicited ACK as ours.
            data = reply[2:]
            if len(data) > length or (endpoint != 0 and len(data) != length):
                raise ProtocolError(f"Unexpected response length: {len(data)} / {length}")
            self.last_rtt_ms = (time.perf_counter() - begin) * 1000
            return data
        raise TimeoutError("USB response timed out; command outcome unknown")

    def read_schema(self) -> bytes:
        data = bytearray()
        for _ in range(2048):
            chunk = self.operation(0, struct.pack("<I", len(data)), 512)
            if not chunk:
                result = bytes(data)
                self.interface_crc = crc16(result)
                return result
            data.extend(chunk)
        raise ProtocolError("Device schema exceeds 1 MiB")


class UsbTransport:
    """Claims only vendor interface 2. No reset, set_configuration or serial I/O."""
    def __init__(self, serial: str = "20773954464D"):
        import usb.core
        import usb.util
        import libusb_package

        self.util = usb.util
        self.device = None
        backend = libusb_package.get_libusb1_backend()
        devices = list(usb.core.find(find_all=True, idVendor=0x1001, idProduct=0x03EF, backend=backend))
        matching = []
        for d in devices:
            try:
                if usb.util.get_string(d, d.iSerialNumber) == serial:
                    matching.append(d)
            except usb.core.USBError:
                pass
        if len(matching) != 1:
            raise ProtocolError(f"未找到唯一的 OmniPicker {serial}；检查 USB 或驱动")
        device = matching[0]
        try:
            interface = device.get_active_configuration()[(2, 0)]
            if interface.bInterfaceClass != 0xFF:
                raise ProtocolError("REF Interface 类型不匹配")
            self.out_ep = usb.util.find_descriptor(interface, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
            self.in_ep = usb.util.find_descriptor(interface, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
            if self.out_ep is None or self.in_ep is None:
                raise ProtocolError("缺少 USB bulk 端点")
            if any(usb.util.endpoint_type(e.bmAttributes) != usb.util.ENDPOINT_TYPE_BULK for e in (self.out_ep, self.in_ep)):
                raise ProtocolError("USB 端点不是 bulk 类型")
            usb.util.claim_interface(device, 2)
            self.device = device
        except BaseException:
            usb.util.dispose_resources(device)
            raise

    def write(self, packet, timeout):
        written = self.out_ep.write(packet, timeout=max(1, int(timeout * 1000)))
        if written != len(packet):
            raise ProtocolError("USB short write; command outcome unknown")

    def read(self, timeout):
        return self.in_ep.read(1024, timeout=max(1, int(timeout * 1000)))

    def close(self):
        if self.device is not None:
            try:
                self.util.release_interface(self.device, 2)
            finally:
                self.util.dispose_resources(self.device)
                self.device = None
