import socket
import time
import threading
from e6_phosphobot.feedback import FeedbackReader
from test_sdk_upstream import packet


def until(predicate, seconds=1):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.001)
    assert predicate()


def test_slow_or_failed_consumers_cannot_stop_feedback_or_cleanup():
    incoming, outgoing = socket.socketpair()
    reader = FeedbackReader('test', socket_factory=lambda *a, **k: incoming)
    slow = reader.subscribe(capacity=1)
    good = reader.subscribe(capacity=10)
    with reader:
        outgoing.sendall(packet(1) + packet(2) + packet(3))
        until(lambda: reader.snapshot()['sequence'] == 3)
        assert slow.dropped == 2
        assert good.queue.qsize() == 3
        # Independent consumer failure has no callback on the receiver thread.
        try:
            raise RuntimeError('Display model outside its domain')
        except RuntimeError:
            pass
        outgoing.sendall(packet(4))
        until(lambda: reader.snapshot()['sequence'] == 4)
        assert reader.snapshot()['status']['timestamp'] == 4
    assert not reader._thread.is_alive()
    assert not reader.snapshot()['connected']
    outgoing.close()


def test_reconnect_changes_generation_and_does_not_hide_parser_failure():
    incoming1, outgoing1 = socket.socketpair()
    incoming2, outgoing2 = socket.socketpair()
    sockets = iter([incoming1, incoming2])
    reader = FeedbackReader('test', socket_factory=lambda *a, **k: next(sockets),
                            reconnect_delay=.01)
    with reader:
        outgoing1.sendall(packet(42))
        until(lambda: reader.snapshot()['sequence'] == 1)
        generation = reader.snapshot()['generation']
        outgoing1.sendall(b'bad' + packet(43))
        until(lambda: reader.snapshot()['generation'] != generation)
        assert reader.snapshot()['status'] is None
        outgoing2.sendall(packet(1))
        until(lambda: reader.snapshot()['sequence'] == 1)
        assert reader.snapshot()['status']['timestamp'] == 1
        assert any('Invalid feedback length' in str(e) for e in reader.events)
    outgoing1.close()
    outgoing2.close()


def test_receive_diagnostics_keep_original_timestamp_after_slow_publish():
    incoming, outgoing = socket.socketpair()
    reader = FeedbackReader('test', socket_factory=lambda *a, **k: incoming)
    entered, resume = threading.Event(), threading.Event()
    original = reader._publish

    def delayed(frame):
        entered.set()
        assert resume.wait(1)
        original(frame)

    reader._publish = delayed
    try:
        with reader:
            outgoing.sendall(packet(1))
            assert entered.wait(1)
            time.sleep(.08)
            resume.set()
            until(lambda: any(e['event'] == 'receive_timing' for e in reader.events))
            event = next(e for e in reader.events if e['event'] == 'receive_timing')
            sample = reader.snapshot()
            assert event['publish_finished_ns']-event['recv_returned_ns'] >= 50_000_000
            assert sample['received_ns'] == event['recv_returned_ns']
            assert sample['age_ms'] >= 50
            assert sample['connected'] and sample['status']['timestamp'] == 1
            assert event['reader_native_id'] == reader._thread.native_id
            assert 0 <= event['thread_cpu_ns'] <= event['process_cpu_ns']
    finally:
        resume.set()
        outgoing.close()


def test_socket_timeout_is_distinguished_from_unscheduled_reader_without_refreshing_feedback():
    incoming, outgoing = socket.socketpair()
    reader = FeedbackReader('test', socket_factory=lambda *a, **k: incoming)
    try:
        with reader:
            outgoing.sendall(packet(1))
            until(lambda: reader.snapshot()['sequence'] == 1)
            first = reader.snapshot()
            until(lambda: any(e['event'] == 'receive_timeout' for e in reader.events))
            expired = reader.snapshot()
            assert expired['sequence'] == 1
            assert expired['received_ns'] == first['received_ns']
            assert expired['age_ms'] >= 90
            outgoing.sendall(packet(2))
            until(lambda: reader.snapshot()['sequence'] == 2)
            until(lambda: any(e['event'] == 'receive_timing' for e in reader.events))
            event = next(e for e in reader.events if e['event'] == 'receive_timing')
            timeouts = [e for e in reader.events if e['event'] == 'receive_timeout'
                        and e['recv_returned_ns'] <= event['recv_started_ns']]
            assert event['timeout_count_since_publish'] == len(timeouts) >= 1
            assert event['timeout_wait_ns_since_publish'] == sum(
                e['recv_returned_ns']-e['recv_started_ns'] for e in timeouts)
            assert event['previous_attempt_end_ns'] == timeouts[-1]['recv_returned_ns']
            assert event['previous_publish_end_ns'] < event['previous_attempt_end_ns']
            assert reader.snapshot()['received_ns'] == event['recv_returned_ns']
    finally:
        outgoing.close()
