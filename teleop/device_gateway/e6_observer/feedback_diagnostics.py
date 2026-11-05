"""Bounded exact receive history for a fail-closed feedback parser.

This is evidence capture, never a resynchronizer. No bytes are discarded from
the parser and a malformed stream still terminates the control session.
"""
import base64
from collections import deque
import json
import os
from pathlib import Path


class FeedbackTrace:
    def __init__(self, max_chunks=64):
        self.chunks=deque(maxlen=max_chunks)
        self.offset=0
        self.last_valid=None

    def receive(self, raw, received_ns):
        self.chunks.append(dict(offset=self.offset,bytes=len(raw),x5_receive_ns=received_ns,
                                raw_base64=base64.b64encode(raw).decode('ascii')))
        self.offset+=len(raw)

    def valid(self, packet, decoded, received_ns):
        self.last_valid=dict(raw_base64=base64.b64encode(packet).decode('ascii'),decoded=decoded,
                             x5_receive_ns=received_ns)

    def failure(self, parser, error, directory='diagnostics'):
        directory=Path(directory)
        directory.mkdir(parents=True,exist_ok=True)
        path=directory/f'feedback_failure_{os.getpid()}_{self.offset}.json'
        details=dict(error=str(error),total_bytes=self.offset,
                     parser_buffer_base64=base64.b64encode(parser.buffer).decode('ascii'),
                     chunks=list(self.chunks),last_valid=self.last_valid,
                     policy='Session terminated; no guessed resynchronization')
        path.write_text(json.dumps(details,allow_nan=False),encoding='utf-8')
        return dict(path=str(path),total_bytes=self.offset,retained_chunks=len(self.chunks),
                    buffer_bytes=len(parser.buffer),prefix_hex=bytes(parser.buffer[:32]).hex())
