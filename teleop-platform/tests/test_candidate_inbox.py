from e6_phosphobot.jazzy_host import CandidateInbox
from e6_phosphobot.moveit_bridge import MoveItBridge
from test_moveit_bridge import intent
import threading


class Pipe:
    replaced=0
    def __init__(self):self.row=None;self.reads=0
    def receive(self):
        self.reads+=1;row,self.row=self.row,None
        return row


def setup(api='ros'):
    now=[100_000_000];bridge=MoveItBridge('.',None,clock=lambda:now[0],api=api)
    bridge.submit(intent(),1);pipe=Pipe();inbox=CandidateInbox(bridge,pipe,'test')
    bridge.refresh_candidate=inbox.receive
    return bridge,pipe,inbox,now


def packet(bridge,ident,received,deadline):
    return dict(info=dict(ready=True),candidate=dict(id=ident,epoch=bridge.epoch,
        received_ns=received,computed_ns=received+1_000_000,valid_until_ns=deadline))


def test_tick_consumes_queued_fresh_candidate_without_waiting_for_host_poll():
    bridge,pipe,inbox,now=setup()
    bridge.latest=packet(bridge,1,60_000_000,85_000_000)['candidate']
    pipe.row=packet(bridge,2,90_000_000,115_000_000)
    assert bridge.take()['id']==2
    assert bridge.snapshot()['last_take']['sample_age_ms']==10
    assert bridge.take() is None  # Never replay once consumed.


def test_background_decode_cannot_hold_control_release_or_publish_old_epoch():
    bridge,pipe,inbox,now=setup()
    entered=threading.Event();finish=threading.Event()
    old=packet(bridge,2,now[0],now[0]+25_000_000)
    bridge.latest=packet(bridge,1,now[0],now[0]+25_000_000)['candidate']
    def decode():
        entered.set()
        assert finish.wait(2)
        return old
    pipe.receive=decode
    worker=threading.Thread(target=inbox.receive);worker.start()
    try:
        assert entered.wait(1)
        # This is the same consumer boundary called under the gateway mutex.
        # It can use an already fresh result without waiting for decoding.
        assert bridge.take()['id']==1
        bridge.stop()
        assert bridge.command is None and bridge.latest is None
    finally:
        finish.set();worker.join(2)
    assert not worker.is_alive() and bridge.latest is None
    assert not bridge.initial_candidate_event.is_set()


def test_busy_decoder_does_not_extend_candidate_deadline():
    bridge,pipe,inbox,now=setup()
    bridge.latest=packet(bridge,1,60_000_000,85_000_000)['candidate']
    with inbox.lock:
        assert bridge.take() is None
        assert bridge.snapshot()['last_take']['reason']=='sample_deadline'
        assert pipe.reads==0


def test_consumed_provenance_survives_new_publication_and_stop():
    bridge,pipe,inbox,now=setup()
    first=packet(bridge,1,now[0],now[0]+25_000_000)
    first['candidate']['trajectory_sources']=[dict(input=dict(sequence=1,intent=dict(joint_rad_s=[0]*5+[.01])))]
    pipe.row=first
    accepted=bridge.take()
    assert bridge.latest is None and bridge.take() is None
    second=packet(bridge,2,now[0],now[0]+25_000_000)
    second['candidate']['trajectory_sources']=[dict(input=dict(sequence=2,intent=dict(joint_rad_s=[0]*5+[-.01])))]
    pipe.row=second;inbox.receive()
    assert bridge.take()['trajectory_sources'][0]['input']['sequence']==2
    bridge.stop()
    assert accepted['trajectory_sources'][0]['input']==dict(sequence=1,intent=dict(joint_rad_s=[0]*5+[.01]))


def test_consumer_read_does_not_refresh_expired_or_prior_epoch_target():
    bridge,pipe,inbox,now=setup()
    pipe.row=packet(bridge,1,60_000_000,85_000_000)
    assert bridge.take() is None
    assert bridge.snapshot()['last_take']['reason']=='sample_deadline'
    old=packet(bridge,2,now[0],now[0]+25_000_000)
    bridge.stop();bridge.submit(intent(),2);pipe.row=old
    assert bridge.take() is None
    assert bridge.latest is None


def test_closed_inbox_never_touches_released_descriptor():
    bridge,pipe,inbox,now=setup();inbox.close()
    pipe.row=packet(bridge,1,now[0],now[0]+25_000_000)
    assert bridge.take() is None and pipe.reads==0


def test_first_fresh_candidate_wakes_once_and_consumption_clears_notification():
    bridge,pipe,inbox,now=setup()
    pipe.row=packet(bridge,1,now[0],now[0]+25_000_000)
    inbox.receive()
    assert bridge.initial_candidate_event.is_set()
    assert bridge.snapshot()['first_candidate']['id']==1
    assert bridge.take()['id']==1
    assert not bridge.initial_candidate_event.is_set()
    now[0]+=10_000_000
    pipe.row=packet(bridge,2,now[0],now[0]+25_000_000)
    inbox.receive()
    assert not bridge.initial_candidate_event.is_set()
    assert bridge.take()['id']==2


def test_expired_or_prior_epoch_candidate_cannot_wake_but_new_epoch_can():
    bridge,pipe,inbox,now=setup()
    pipe.row=packet(bridge,1,60_000_000,85_000_000)
    inbox.receive()
    assert not bridge.initial_candidate_event.is_set()
    old=packet(bridge,2,now[0],now[0]+25_000_000)
    bridge.stop();bridge.submit(intent(),2)
    pipe.row=old;inbox.receive()
    assert not bridge.initial_candidate_event.is_set()
    pipe.row=packet(bridge,3,now[0],now[0]+25_000_000)
    inbox.receive()
    assert bridge.initial_candidate_event.is_set()
    bridge.stop()
    assert not bridge.initial_candidate_event.is_set() and bridge.take() is None
    bridge.submit(intent(),3)
    pipe.row=packet(bridge,4,now[0],now[0]+25_000_000)
    inbox.receive()
    assert bridge.initial_candidate_event.is_set()
    assert bridge.take()['id']==4


def test_cpp_late_candidate_wakes_without_replaying_heartbeats_or_overwriting_newer_target():
    bridge,pipe,inbox,now=setup('cpp')
    pipe.row=packet(bridge,1,now[0],now[0]+100_000_000);inbox.receive()
    assert bridge.take()['id']==1
    first=dict(bridge.first_candidate)
    # The regular 20 Hz check has just found nothing; a result arrives 8 ms
    # afterwards. It must wake now, not wait for another complete 50 ms tick.
    now[0]+=58_000_000
    pipe.row=packet(bridge,2,now[0]-8_000_000,now[0]+70_000_000);inbox.receive()
    assert bridge.initial_candidate_event.is_set()
    bridge.initial_candidate_event.clear()
    original_stamp=bridge.latest['host_received_ns']
    now[0]+=1_000_000
    pipe.row=packet(bridge,2,now[0]-9_000_000,now[0]+69_000_000);inbox.receive()
    assert not bridge.initial_candidate_event.is_set()
    assert bridge.latest['host_received_ns']==original_stamp
    pipe.row=packet(bridge,1,now[0],now[0]+70_000_000);inbox.receive()
    assert bridge.latest['id']==2
    assert bridge.take()['id']==2
    assert bridge.first_candidate==first  # First-response evidence stays first.
    pipe.row=packet(bridge,2,now[0],now[0]+70_000_000);inbox.receive()
    assert bridge.latest is None and not bridge.initial_candidate_event.is_set()


def test_cpp_every_wake_still_rejects_expired_epoch_and_release():
    bridge,pipe,inbox,now=setup('cpp')
    pipe.row=packet(bridge,1,now[0],now[0]+100_000_000);inbox.receive()
    assert bridge.take()['id']==1
    now[0]+=50_000_000
    pipe.row=packet(bridge,2,now[0]-10_000_000,now[0]-1);inbox.receive()
    assert not bridge.initial_candidate_event.is_set()
    assert bridge.take() is None
    old=packet(bridge,3,now[0],now[0]+90_000_000)
    bridge.stop();bridge.submit(intent(),4);pipe.row=old;inbox.receive()
    assert bridge.latest is None and not bridge.initial_candidate_event.is_set()
    pipe.row=packet(bridge,4,now[0],now[0]+90_000_000);inbox.receive()
    assert bridge.initial_candidate_event.is_set()
    bridge.stop()
    assert bridge.take() is None and not bridge.initial_candidate_event.is_set()
