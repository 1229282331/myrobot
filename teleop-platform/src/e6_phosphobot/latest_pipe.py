"""Bounded, latest-value JSON transport over owned local nonblocking pipes.

One partially written message can finish; only the latest replacement follows.
No sockets, leases, device commands, or input replay live in this module.
"""
import json
import os


class LatestJsonPipe:
    def __init__(self, read_fd, write_fd, limit=262144):
        self.read_fd=read_fd;self.write_fd=write_fd;self.limit=limit
        self.input=bytearray();self.pending=b'';self.offset=0;self.replacement=None
        self.replaced=0
        os.set_blocking(read_fd,False);os.set_blocking(write_fd,False)

    def offer(self,value):
        data=json.dumps(value,separators=(',',':'),allow_nan=False).encode()+b'\n'
        if len(data)>self.limit:raise ValueError('IPC message exceeds bound')
        if self.offset:
            if self.replacement is not None:self.replaced+=1
            self.replacement=data
        else:
            if self.pending:self.replaced+=1
            self.pending=data

    def flush(self):
        if not self.pending:return
        try:written=os.write(self.write_fd,self.pending[self.offset:])
        except BlockingIOError:return
        self.offset+=written
        if self.offset==len(self.pending):
            self.pending=self.replacement or b'';self.replacement=None;self.offset=0

    def receive(self):
        latest=None
        # Bound work even if a peer continuously produces data.
        for _ in range(32):
            try:data=os.read(self.read_fd,65536)
            except BlockingIOError:break
            if not data:raise EOFError('Owned IPC peer closed')
            self.input.extend(data)
            while b'\n' in self.input:
                end=self.input.index(10)
                if end>self.limit:raise ValueError('IPC line exceeds bound')
                line=bytes(self.input[:end]);del self.input[:end+1]
                latest=json.loads(line,parse_constant=lambda s:(_ for _ in ()).throw(ValueError('Nonfinite IPC value')))
                if not isinstance(latest,dict):raise ValueError('IPC object required')
            if len(self.input)>self.limit:raise ValueError('IPC buffer exceeds bound')
        return latest
