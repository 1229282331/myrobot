"""Independent raw device observation. No model or consumer runs on this thread."""
from collections import deque
import base64
from dataclasses import asdict
import queue
import socket
import threading
import time
import uuid

from .dobot_transport import FeedbackStream, FeedbackFramingError


class Subscription:
    def __init__(self, capacity):
        self.queue = queue.Queue(capacity)
        self.dropped = 0


class FeedbackReader:
    def __init__(self, host, port=30004, *, allow_length_envelope=True,
                 socket_factory=socket.create_connection, reconnect_delay=.5):
        self.host, self.port = host, port
        self.allow_length_envelope = allow_length_envelope
        self.socket_factory = socket_factory
        self.reconnect_delay = reconnect_delay
        self._lock = threading.Lock()
        self._halt = threading.Event()
        self._thread = None
        self._socket = None
        self._subscriptions = []
        self.latest = None
        self.generation = None
        self.sequence = 0
        self.connected = False
        self.error = 'Not connected'
        self.events = deque(maxlen=256)
        # One consumer-side projection, keyed by the immutable frame object.
        # Never decode on the raw receiver or keep decoded history in queues.
        self._compact_cache = (None, None)

    def subscribe(self, capacity=512):
        sub = Subscription(capacity)
        with self._lock:
            self._subscriptions.append(sub)
        return sub

    def unsubscribe(self, sub):
        with self._lock:
            if sub in self._subscriptions:
                self._subscriptions.remove(sub)

    def start(self):
        if self._thread is not None:
            raise RuntimeError('Reader already started')
        self._thread = threading.Thread(target=self._run, name='e6-raw-feedback', daemon=True)
        self._thread.start()

    def _publish(self, frame):
        with self._lock:
            self.latest = frame
            self.sequence += 1
            item = (self.generation, self.sequence, frame)
            for sub in self._subscriptions:
                try:
                    sub.queue.put_nowait(item)
                except queue.Full:
                    # Overflow aborts that recorder, never the feedback receiver.
                    sub.dropped += 1

    def _run(self):
        while not self._halt.is_set():
            stream = FeedbackStream(self.allow_length_envelope)
            try:
                sock = self.socket_factory((self.host, self.port), timeout=2)
                sock.settimeout(.1)
                with self._lock:
                    self._socket = sock
                    self.generation = str(uuid.uuid4())
                    self.latest = None
                    self.sequence = 0
                    self.connected = True
                    self.error = None
                self.events.append(dict(event='connected', generation=self.generation,
                                        received_ns=time.perf_counter_ns(),
                                        reader_native_id=threading.get_native_id()))
                previous_end = None
                previous_attempt_end = None
                timeout_count = 0
                timeout_wait_ns = 0
                while not self._halt.is_set():
                    started = time.perf_counter_ns()
                    cpu = time.process_time_ns()
                    thread_cpu = time.thread_time_ns()
                    try:
                        data = sock.recv(65536)
                    except socket.timeout:
                        ended = time.perf_counter_ns()
                        timeout_count += 1
                        timeout_wait_ns += ended-started
                        # A timed-out recv is still an attempt. Without this
                        # event, its 100 ms wait looks like a scheduling gap
                        # before the next successful recv. No freshness state
                        # is updated until real feedback is received.
                        self.events.append(dict(event='receive_timeout',
                            generation=self.generation, received_ns=ended,
                            reader_native_id=threading.get_native_id(),
                            previous_publish_end_ns=previous_end,
                            previous_attempt_end_ns=previous_attempt_end,
                            recv_started_ns=started, recv_returned_ns=ended,
                            process_cpu_ns=time.process_time_ns()-cpu,
                            thread_cpu_ns=time.thread_time_ns()-thread_cpu))
                        previous_attempt_end = ended
                        continue
                    if not data:
                        stream.eof()
                        raise ConnectionError('Feedback stream closed')
                    now = time.perf_counter_ns()
                    for frame in stream.feed(data, now):
                        self._publish(frame)
                    finished = time.perf_counter_ns()
                    if (finished-started >= 50_000_000 or
                            previous_end is not None and started-previous_end >= 50_000_000):
                        # Diagnostics only: preserve the original recv timestamp
                        # and all freshness policy. Packet capture can distinguish
                        # a blocked/unscheduled recv from a source that sent late.
                        self.events.append(dict(event='receive_timing',
                            generation=self.generation, received_ns=finished,
                            reader_native_id=threading.get_native_id(),
                            previous_publish_end_ns=previous_end,
                            previous_attempt_end_ns=previous_attempt_end,
                            timeout_count_since_publish=timeout_count,
                            timeout_wait_ns_since_publish=timeout_wait_ns,
                            recv_started_ns=started, recv_returned_ns=now,
                            publish_finished_ns=finished, bytes=len(data),
                            process_cpu_ns=time.process_time_ns()-cpu,
                            thread_cpu_ns=time.thread_time_ns()-thread_cpu))
                    previous_end = finished
                    previous_attempt_end = finished
                    timeout_count = 0
                    timeout_wait_ns = 0
            except Exception as exc:
                if not self._halt.is_set():
                    with self._lock:
                        self.error = str(exc)
                    event=dict(event='disconnected', reason=str(exc),
                               generation=self.generation, received_ns=time.perf_counter_ns())
                    if isinstance(exc, FeedbackFramingError):
                        event.update(rejected_wire_b64=base64.b64encode(exc.wire).decode(),
                                     rejected_wire_length=exc.wire_length,
                                     rejected_wire_sha256=exc.wire_sha256,
                                     evidence_truncated=exc.wire_length > len(exc.wire))
                    self.events.append(event)
            finally:
                with self._lock:
                    sock, self._socket = self._socket, None
                    self.connected = False
                if sock is not None:
                    sock.close()
            self._halt.wait(self.reconnect_delay)

    def snapshot(self, *, full=False):
        with self._lock:
            frame = self.latest
            result = dict(generation=self.generation, sequence=self.sequence,
                          connected=self.connected, error=self.error)
        status = None
        if frame is not None:
            if full:
                status = asdict(frame.status)
            else:
                cached_frame, projection = self._compact_cache
                if cached_frame is not frame:
                    decoded = frame.status
                    projection = {key: getattr(decoded, key) for key in (
                        'timestamp', 'run_time', 'enable_status', 'error_status',
                        'collision_state', 'running_status', 'run_queued_cmd',
                        'current_command_id', 'user_coordinate', 'tool_coordinate',
                        'user_value', 'tool_value', 'load',
                        'load_center_x', 'load_center_y', 'load_center_z')}
                    projection['robot_mode'] = decoded.robot_mode.value
                    projection['joint_state'] = {key: getattr(decoded.joint_state, key)
                        for key in ('q_actual','q_target','qd_actual')}
                    for key in ('tool_vector_actual','tool_vector_target','actual_quaternion'):
                        projection[key] = vars(getattr(decoded, key)).copy()
                    # Publish one complete pair. Concurrent misses may decode
                    # twice; neither the raw reader nor Stop waits for a decoder
                    # lock. Each caller retains its own frame/source timestamp.
                    self._compact_cache = (frame, projection)
                status = dict(projection)
                for key in ('user_value','tool_value'):
                    status[key] = list(projection[key])
                status['joint_state'] = {key:list(value) for key,value in projection['joint_state'].items()}
                for key in ('tool_vector_actual','tool_vector_target','actual_quaternion'):
                    status[key] = dict(projection[key])
        result.update(received_ns=None if frame is None else frame.received_ns,
                      status=status)
        # Include decoding time; do not report the pre-decode age as current.
        result['age_ms'] = None if frame is None else (time.perf_counter_ns()-frame.received_ns)/1e6
        return result

    def close(self):
        self._halt.set()
        with self._lock:
            sock = self._socket
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                raise RuntimeError('Feedback reader did not release its socket')

    def drain_events(self):
        events=[]
        while self.events:
            events.append(self.events.popleft())
        return events

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()
