"""Read-only CDC receiver for the observed FW335/debug_flag=56 format.

Host arrival timestamps are not device sample timestamps. No serial writes,
line scanning, device reset, or inferred force units. A failed stream latches
until reconnect; old samples never become fresh again.
"""
from collections import deque
import math
import re
import threading
import time

STALE_SECONDS = .3
MAX_PENDING = 2000
ROW = re.compile(rb"[+-]?\d+\.\d{2},[+-]?\d+\.\d{2},[+-]?\d+\.\d{2}\r?")


class CsvDecoder:
    def __init__(self):
        self.pending = b""
        self.aligned = False
        self.index = 0

    def feed(self, data, timestamp):
        self.pending += data
        samples = []
        while b"\n" in self.pending:
            line, self.pending = self.pending.split(b"\n", 1)
            if not self.aligned:
                self.aligned = True
                continue  # first line can start before opening the port
            if len(line) > 100 or not ROW.fullmatch(line):
                raise ValueError("CDC 数据格式改变，请重新连接并核对 debug_flag")
            raw_force, velocity, position = map(float, line.split(b","))
            if not all(math.isfinite(x) for x in (raw_force, velocity, position)):
                raise ValueError("CDC 数据含非有限数值")
            if not -.05 <= position <= 1.05:
                raise ValueError("CDC 位置超出归一化范围：" + line.decode("ascii"))
            # Preserve raw values: do not clamp an out-of-range observation.
            self.index += 1
            samples.append(dict(monotonic=timestamp, host_sequence=self.index,
                                raw=line.decode("ascii"), position_fraction=position,
                                velocity_fraction_s=velocity, column1_unverified=raw_force))
        if len(self.pending) > 1024:
            raise ValueError("CDC 行长度异常")
        return samples


class CdcFeedback:
    simulated = False

    def __init__(self, serial_number, clock=time.perf_counter, port_factory=None, enumerate_ports=None):
        self.serial_number = serial_number
        self.clock = clock
        self.port_factory = port_factory
        self.enumerate_ports = enumerate_ports
        self.lock = threading.Lock()
        self.queue = deque()
        self.decoder = CsvDecoder()
        self.error = None
        self.port = None
        self.port_name = ""
        self.thread = None
        self.halt = threading.Event()

    def start(self):
        import serial
        from serial.tools import list_ports
        ports = (self.enumerate_ports or list_ports.comports)()
        matches = [p for p in ports if (p.vid, p.pid, p.serial_number)
                   == (0x1001, 0x03EF, self.serial_number)]
        if len(matches) != 1:
            raise RuntimeError("未找到唯一匹配的 CDC 串口，Fibre 保持只读连接")
        self.port_name = matches[0].device
        self.port = (self.port_factory or serial.Serial)(port=None, baudrate=115200, bytesize=8,
                         parity="N", stopbits=1, timeout=.05, xonxoff=False, rtscts=False, dsrdtr=False)
        try:
            self.port.dtr = False
            self.port.rts = False
            self.port.port = self.port_name
            self.port.open()
            self.thread = threading.Thread(target=self._run, name="omnipicker-cdc", daemon=True)
            self.thread.start()
        except BaseException:
            self.port.close()
            raise

    def _run(self):
        try:
            while not self.halt.is_set():
                data = self.port.read(min(max(self.port.in_waiting, 1), 512))
                if data:
                    samples = self.decoder.feed(data, self.clock())
                    with self.lock:
                        if len(self.queue) + len(samples) > MAX_PENDING:
                            raise RuntimeError("CDC 接收队列溢出，记录不完整")
                        self.queue.extend(samples)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def poll(self):
        with self.lock:
            rows = list(self.queue)
            self.queue.clear()
            return rows, self.error

    def close(self):
        self.halt.set()
        if self.thread:
            self.thread.join(timeout=1)
        if self.port:
            self.port.close()


class SimulatedFeedback:
    """Explicit mock observation, moving over time rather than echoing targets."""
    simulated = True
    port_name = "SIMULATED"

    def __init__(self, device, clock=time.perf_counter):
        self.device, self.clock = device, clock
        self.t = clock() - .01
        self.position = 0.0
        self.sequence = 0
        self.paused = False

    def start(self): pass
    def close(self): pass

    def poll(self):
        now = self.clock()
        if self.paused:
            self.t = now
            return [], None
        if now - self.t > MAX_PENDING * .01:
            return [], "模拟采样队列溢出"
        rows = []
        while now - self.t >= .01 - 1e-9:
            self.t += .01
            target = self.device.simulation_target if self.device.simulation_enabled else self.position
            delta = max(-.0018, min(.0018, target - self.position))
            self.position += delta
            self.sequence += 1
            rows.append(dict(monotonic=self.t, host_sequence=self.sequence,
                             position_fraction=round(self.position, 2), velocity_fraction_s=round(delta/.01, 2),
                             column1_unverified=0.0, raw=f"0.00,{delta/.01:.2f},{self.position:.2f}"))
        return rows, None
