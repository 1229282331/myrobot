import json
import os
import pytest
from e6_phosphobot.latest_pipe import LatestJsonPipe
from e6_phosphobot.ipc_snapshot import SnapshotPublisher

pytestmark=pytest.mark.skipif(os.name=='nt',reason='Local X5 POSIX pipe transport; exercised on Linux')


@pytest.fixture
def pipes():
    a,b=os.pipe();c,d=os.pipe()
    try:yield LatestJsonPipe(a,d),LatestJsonPipe(c,b)
    finally:
        for fd in (a,b,c,d):os.close(fd)


def test_only_latest_unsent_and_latest_received_are_used(pipes):
    a,b=pipes
    a.offer({'seq':1});a.offer({'seq':2});a.flush()
    a.offer({'seq':3});a.flush()
    assert b.receive()=={'seq':3} and b.receive() is None
    assert a.replaced==1


def test_partial_packet_finishes_before_latest_replacement(pipes):
    a,b=pipes
    a.offer({'seq':1,'payload':'x'*100000});a.flush()
    a.offer({'seq':2});a.offer({'seq':3})
    seen=[]
    for _ in range(100):
        value=b.receive()
        if value:seen.append(value['seq'])
        a.flush()
        if not a.pending:
            value=b.receive()
            if value:seen.append(value['seq'])
            break
    assert seen[-1]==3 and 2 not in seen


def test_size_and_nonfinite_rejected(pipes):
    a,b=pipes
    with pytest.raises(ValueError):a.offer({'x':'a'*300000})
    with pytest.raises(ValueError):a.offer({'x':float('nan')})
    os.write(a.write_fd,b'{"x":NaN}\n')
    with pytest.raises(ValueError):b.receive()


def test_split_read_waits_for_complete_line(pipes):
    a,b=pipes
    os.write(a.write_fd,b'{"a":');assert b.receive() is None
    os.write(a.write_fd,b'1}\n');assert b.receive()=={'a':1}


def test_dedup_publisher_finishes_partial_packet_then_latest_release(pipes):
    a,b=pipes;publisher=SnapshotPublisher(a)
    publisher.update({'candidate':1,'payload':'x'*100000},0)
    assert a.pending and a.offset
    publisher.update({'candidate':2},1)
    release={'candidate':None,'ready':False}
    publisher.update(release,2)
    seen=[]
    for now in range(3,200):
        value=b.receive()
        if value:seen.append(value)
        publisher.update(release,now)
        if not a.pending:
            value=b.receive()
            if value:seen.append(value)
            break
    assert seen[-1]==release
    assert not any(row['candidate']==2 for row in seen)
    assert publisher.publications==3
