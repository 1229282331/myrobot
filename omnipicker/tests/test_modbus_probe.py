import struct

import pytest
from tools.probe_typec_modbus import crc16_modbus, read_request, decode_response
from tools.probe_typec_modbus import csv_stream_summary, find_response_in_stream


def frame(data):
    return data + struct.pack("<H", crc16_modbus(data))


def test_official_read_vector_and_allowed_ranges():
    assert read_request(1, 20, 5).hex() == "010300140005c5cd"
    assert crc16_modbus(b"123456789") == 0x4B37
    for request in [(0, 20, 5), (1, 10, 6), (1, 15, 1), (1, 20, 6)]:
        with pytest.raises(ValueError):
            read_request(*request)


def test_valid_version_and_state_readback():
    assert decode_response(frame(b"\x01\x03\x02\x01\x4f"), 1, 1, read_request(1, 0, 1)) == {
        "kind": "registers", "values": [335]}
    values = [0, 0, 35, 0, 0]
    data = frame(b"\x01\x03\x0a" + struct.pack(">5H", *values))
    assert decode_response(data, 1, 5, read_request(1, 20, 5))["values"] == values


def test_echo_garbage_bad_crc_and_exception_are_not_telemetry():
    request = read_request(1, 20, 5)
    cases = [(b"", "no_response"), (request, "request_echo_not_feedback"),
             (b"debug text", "invalid_length_or_crc"),
             (frame(b"\x02\x03\x02\x01\x4f"), "wrong_address"),
             (frame(b"\x01\x03\x02\x01\x4f"), "unexpected_function_or_length"),
             (frame(b"\x01\x83\x02"), "modbus_exception")]
    for data, expected in cases:
        assert decode_response(data, 1, 5, request)["kind"] == expected


def test_csv_is_not_modbus_but_binary_response_can_be_found_amid_text():
    csv = b"0.01,-0.00,0.00\n" * 10
    assert csv_stream_summary(csv)["column_units"] == "unverified"
    assert find_response_in_stream(csv, 1, 1, read_request(1, 0, 1)) == []
    reply = frame(b"\x01\x03\x02\x01\x4f")
    mixed = csv + reply + csv
    assert csv_stream_summary(mixed) is None
    found = find_response_in_stream(mixed, 1, 1, read_request(1, 0, 1))
    assert len(found) == 1 and found[0]["decoded"]["values"] == [335]
    assert csv_stream_summary(b"1,2,3\n" * 10) is None
    assert find_response_in_stream(csv + read_request(1, 20, 5), 1, 5, read_request(1, 20, 5)) == []
