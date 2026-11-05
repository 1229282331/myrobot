"""Narrow TCP framing extensions to the pinned official Dobot SDK.

Motion, RobotControl, wire data types and FeedbackParser remain upstream code.
The upstream transport defects are reproduced in tests/test_sdk_upstream.py.
No kinematics, model, camera, recording, or motion policy belongs in this module.
"""
from dataclasses import dataclass
from collections import deque
import hashlib
import socket
import struct
import time

import numpy as np
from dobot_sdk.core.connection import DobotConnection
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC
from dobot_sdk.protocol.feedback import FeedbackParser


@dataclass(frozen=True)
class FeedbackFrame:
    raw: bytes
    prefix: bytes
    received_ns: int

    @property
    def status(self):
        # Retain wire bytes, not thousands of SDK dataclasses/lists. Decoding
        # belongs to the consumer; its allocations cannot hold the reader lock.
        result = FeedbackParser().parse(self.raw)
        if result is None:
            raise ValueError('Official SDK rejected feedback packet')
        return result


class FeedbackStream:
    """Assemble every frame, then delegate decoding to the official parser."""
    def __init__(self, allow_length_envelope=False):
        self.buffer = bytearray()
        self.parser = FeedbackParser()
        self.allow_length_envelope = allow_length_envelope
        self.last_clock = None

    def feed(self, data: bytes, received_ns: int):
        self.buffer.extend(data)
        if len(self.buffer) > 1024 * 1024:
            raise ValueError('Feedback buffer exceeded 1 MiB')
        result = []
        while len(self.buffer) >= 1440:
            if struct.unpack_from('<H', self.buffer)[0] != 1440:
                raise ValueError('Invalid feedback length; no guessed resynchronization')
            offset = 0
            if struct.unpack_from('<Q', self.buffer, 48)[0] != TEST_VALUE_MAGIC:
                envelope = self.buffer[:16] == struct.pack('<QQ', 1440, 1440)
                if not self.allow_length_envelope or not envelope:
                    raise ValueError('Invalid feedback magic')
                if len(self.buffer) < 1448:
                    break
                offset = 8
            raw = bytes(self.buffer[offset:offset + 1440])
            # A real E6 stream contained an 80-byte partial packet followed by
            # a new header. Both the old length/magic and all shifted floats
            # looked valid. Never publish this as an actual joint sample, and
            # never guess an offset to "repair" it. Reconnect with evidence.
            marker = struct.pack('<Q', 1440)
            nested = raw.find(marker, 8)
            while nested >= 0 and nested + 56 <= len(raw):
                if struct.unpack_from('<Q', raw, nested + 48)[0] == TEST_VALUE_MAGIC:
                    raise FeedbackFramingError(
                        f'Nested feedback header at offset {nested}; truncated or interleaved packet',
                        self.buffer)
                nested = raw.find(marker, nested + 1)
            # Model applicability is deliberately absent. An out-of-domain
            # display/IK model can never kill reception of valid device data.
            row = np.frombuffer(raw, dtype=MyType)[0]
            for name in ('QActual', 'QTarget', 'QDActual', 'ToolVectorActual', 'ToolVectorTarget'):
                if not np.isfinite(row[name]).all():
                    raise ValueError('Non-finite device feedback: ' + name)
            clock = (int(row['TimeStamp']), int(row['RunTime']))
            if self.last_clock is not None and any(a < b for a, b in zip(clock, self.last_clock)):
                raise ValueError('Device clock restarted; establish a new connection')
            self.last_clock = clock
            result.append(FeedbackFrame(raw, bytes(self.buffer[:offset]), received_ns))
            del self.buffer[:offset + 1440]
        return result

    def eof(self):
        if self.buffer:
            raise ValueError('Disconnected with truncated feedback')


class FeedbackFramingError(ValueError):
    """Bounded original byte evidence; rejected packets never become state."""
    def __init__(self, message, wire):
        super().__init__(message)
        self.wire = bytes(wire[:65536])
        self.wire_length = len(wire)
        self.wire_sha256 = hashlib.sha256(wire).hexdigest()


class FramedDobotConnection(DobotConnection):
    """Retain upstream connection API, fixing partial writes and reply framing.

    A timed-out command is never retried. Its connection is marked uncertain;
    only Stop may be issued before reconnection. A late original reply is
    retained as diagnostic evidence and cannot acknowledge Stop.
    """
    def __init__(self, ip, port=29999):
        super().__init__(ip, port)
        self.reply_buffer = bytearray()
        self.uncertain = False
        self.events = deque(maxlen=4096)
        self.set_timeout(connect_timeout=2, receive_timeout=.25)

    def connect(self, timeout=None):
        super().connect(timeout)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.reply_buffer.clear()
        self.uncertain = False

    def send_text(self, text):
        if not self._connected or self._socket is None:
            raise ConnectionError('Dashboard disconnected')
        if any(c in text for c in ('\n', '\r', ';')):
            raise ValueError('One SDK command required')
        self._socket.sendall((text + '\n').encode('ascii'))

    def receive_text(self, buffer_size=4096, *, deadline=None):
        if deadline is None:
            deadline = time.monotonic() + self._receive_timeout
        while True:
            end = self.reply_buffer.find(b';')
            if end >= 0:
                reply = bytes(self.reply_buffer[:end + 1])
                del self.reply_buffer[:end + 1]
                return reply.decode('ascii').strip()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Dashboard reply deadline')
            if len(self.reply_buffer) > 65536:
                raise ValueError('Dashboard reply exceeded 64 KiB')
            self._socket.settimeout(remaining)
            data = self._socket.recv(buffer_size)
            if not data:
                raise ConnectionError('Dashboard closed before complete reply')
            self.reply_buffer.extend(data)

    def send_receive_text(self, text, recv_size=4096):
        with self._lock:
            if self.uncertain and text != 'Stop()':
                raise RuntimeError('Unconfirmed command: Stop and reconnect before new motion')
            event = dict(command=text, sent_ns=time.perf_counter_ns(), acknowledged=False)
            self.events.append(event)
            try:
                if self.reply_buffer and not self.uncertain:
                    raise ValueError('Unsolicited reply bytes before command')
                self.send_text(text)
                event['send_completed_ns'] = time.perf_counter_ns()
                deadline = time.monotonic() + self._receive_timeout
                while True:
                    reply = self.receive_text(recv_size, deadline=deadline)
                    if reply.endswith(',' + text + ';'):
                        event.update(reply=reply, reply_ns=time.perf_counter_ns(), acknowledged=True)
                        # A successful Stop does not make a previously uncertain
                        # motion certain, or automatically restore control.
                        return reply
                    event.setdefault('unmatched_replies', []).append(reply)
                    if text != 'Stop()' or not self.uncertain:
                        raise ValueError('Dashboard reply does not match SDK command')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Stop acknowledgement deadline')
            except Exception as exc:
                event.update(error=str(exc), failed_ns=time.perf_counter_ns(), uncertain=True)
                self.uncertain = True
                raise
