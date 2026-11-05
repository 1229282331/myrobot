"""Guardian IPC must not grow with the private Dashboard history."""
from collections import deque
from multiprocessing.reduction import ForkingPickler

from e6_observer.control_guardian import recent_command_events


def test_ipc_history_budget_preserves_recent_uncertain_and_stop_evidence():
    events = deque(maxlen=256)
    for i in range(256):
        events.append(dict(command=f'ServoJ({i})', x5_sent_ns=i,
                           raw_reply='0,{' + str(i) + '},ServoJ();',
                           acknowledged=True, tcp_timing={'rtt_us': 1000}))
    events[-2].update(acknowledged=False, uncertain=True)
    events[-1].update(command='Stop()', confirmation_complete=False)
    private_before = list(events)
    published = recent_command_events(events)
    # The existing consumer's visible history and acceptance evidence are exact.
    assert published == private_before[-20:]
    assert published[-2]['uncertain']
    assert not published[-1]['confirmation_complete']
    assert list(events) == private_before
    # Production queue encoding stays bounded even after hundreds of writes.
    encoded = bytes(ForkingPickler.dumps({'command_events': published}))
    short = bytes(ForkingPickler.dumps({'command_events': private_before[-20:]}))
    full = bytes(ForkingPickler.dumps({'command_events': private_before}))
    assert encoded == short
    assert len(encoded) < len(full) / 5


def test_first_publication_retains_all_available_commands():
    assert recent_command_events(deque()) == []
    one = dict(command='RobotMode()', acknowledged=True)
    assert recent_command_events(deque([one])) == [one]
