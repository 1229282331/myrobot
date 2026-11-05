from copy import deepcopy
from types import SimpleNamespace

import pytest
from e6_phosphobot.ipc_snapshot import SnapshotPublisher, solver_wait_seconds


class Pipe:
    def __init__(self):
        self.offered = []
        self.flushes = 0

    def offer(self, value):
        self.offered.append(deepcopy(value))

    def flush(self):
        self.flushes += 1


def state():
    return dict(info=dict(ready=True, phase='active', nested=dict(error=None)),
                candidate=dict(id=1, epoch=2, valid_until_ns=100_000_000))


def test_unchanged_state_uses_heartbeat_but_partial_writes_keep_progressing():
    pipe = Pipe(); publisher = SnapshotPublisher(pipe); row = state()
    for now in (0, 2_000_000, 49_999_999, 50_000_000):
        publisher.update(row, now)
    assert len(pipe.offered) == 2 and pipe.flushes == 4
    assert pipe.offered[0] == pipe.offered[1]
    assert pipe.offered[1]['candidate']['valid_until_ns'] == 100_000_000


@pytest.mark.parametrize('change', ['release', 'stale', 'error', 'candidate', 'nested'])
def test_release_health_and_new_result_never_wait_for_heartbeat(change):
    pipe = Pipe(); publisher = SnapshotPublisher(pipe); row = state()
    publisher.update(row, 0)
    if change == 'release': row['candidate'] = None
    elif change == 'stale': row['info']['ready'] = False
    elif change == 'error': row['info'].update(phase='failed', error='worker exited')
    elif change == 'candidate': row['candidate']['id'] += 1
    else: row['info']['nested']['error'] = 'model failed'
    publisher.update(row, 1)
    assert len(pipe.offered) == 2 and pipe.offered[-1] == row


def test_wait_honors_next_step_and_does_not_spin_if_due_but_blocked():
    cycle = SimpleNamespace(pending=None, next_request_ns=10_000_000)
    assert solver_wait_seconds(cycle, 0) == .005
    assert solver_wait_seconds(cycle, 9_000_000) == .001
    assert solver_wait_seconds(cycle, 10_000_001) == .005
    cycle.pending = {'id': 1}
    assert solver_wait_seconds(cycle, 9_000_000) == .005
