"""Safety boundaries of the feedback-driven geometry/command commitment."""
from copy import deepcopy
import math
import threading
import pytest
from e6_phosphobot.gripper_geometry import request_state,raw_position,permits_motion
from e6_phosphobot.cpp_cycle import CppCycle
from test_cpp_cycle import context,response
from test_gripper import ready

NOW=1_000_000_000

def test_uninitialized_dynamic_geometry_cannot_silently_prepare_arm_control():
 from types import SimpleNamespace
 from e6_phosphobot.gateway import NativeExecutor
 from e6_phosphobot.intent import IntentSession
 executor=NativeExecutor.__new__(NativeExecutor)
 executor.session=IntentSession();executor.episodes=None;executor.dynamic_gripper=True
 executor.gripper=SimpleNamespace(snapshot=lambda:dict(phase='uninitialized'))
 executor._receive(dict(op='prepare'))
 assert not executor.session.ready
 assert executor.session.reason=='Waiting for automatic gripper connection'
def permit():
 return dict(allowed=True,fallback=False,checked_ns=NOW,requested_raw=97,range_raw=[85,97],arm_q_rad=[0.]*6,arm_target_rad=[.0005]*6)
def grip():return dict(valid=True,phase='ready',actual=89/255,received_ns=NOW,sent_raw=93,inflight_raw=97,command_raw=85)
def arm():return dict(connected=True,received_ns=NOW,status=dict(joint_state=dict(q_actual=[0.]*6)))

def test_actual_is_never_replaced_by_a_target_and_all_commitments_are_attributed():
 assert request_state(grip(),NOW)==[89,93,97,89]
 s=grip();s['received_ns']=NOW-200_000_001
 assert request_state(s,NOW)==[-1,93,97,89]
 s['actual']=True
 assert raw_position(s,NOW)==-1

@pytest.mark.parametrize('change',[
 lambda c,s,a:c.update(checked_ns=NOW-100_000_001),
 lambda c,s,a:c.update(checked_ns=NOW+1),
 lambda c,s,a:c.update(requested_raw=True),
 lambda c,s,a:c.update(allowed=1),
 lambda c,s,a:c.update(range_raw=[0]),
 lambda c,s,a:c.update(range_raw=[None,255]),
 lambda c,s,a:c.update(arm_q_rad=[float('nan')]*6),
 lambda c,s,a:a.update(status={}),
 lambda c,s,a:a.update(received_ns=None),
 lambda c,s,a:a['status']['joint_state'].update(q_actual=[1.]*6),
 lambda c,s,a:s.update(inflight_raw=98),
 lambda c,s,a:s.update(sent_raw=84),
 lambda c,s,a:s.update(valid=False),
])
def test_uncertain_or_uncovered_motion_is_denied(change):
 c,s,a=permit(),grip(),arm();change(c,s,a)
 assert permits_motion(c,s,a,NOW,97) is False

def test_prior_arm_endpoint_must_be_within_the_checked_swept_segment():
 assert permits_motion(permit(),grip(),arm(),NOW,97,[.00025]*6)
 assert not permits_motion(permit(),grip(),arm(),NOW,97,[.01]*6)
 assert not permits_motion(permit(),grip(),arm(),NOW,97,[float('inf')]*6)

@pytest.mark.parametrize('fraction',[0.,.1,.5,.9,1.])
def test_feedback_can_progress_inside_the_fcl_checked_segment(fraction):
 c=permit();c['arm_target_rad']=[.006,-.005,.004,-.003,.002,-.001]
 a=arm();a['status']['joint_state']['q_actual']=[math.degrees(fraction*x) for x in c['arm_target_rad']]
 assert permits_motion(c,grip(),a,NOW,97,c['arm_target_rad'])

@pytest.mark.parametrize('q',[[.012,0,0,0,0,0],[-.002,0,0,0,0,0],[.005,.002,0,0,0,0]])
def test_feedback_past_before_or_sideways_from_checked_segment_is_denied(q):
 c=permit();c['arm_target_rad']=[.01,0,0,0,0,0]
 a=arm();a['status']['joint_state']['q_actual']=[math.degrees(x) for x in q]
 assert not permits_motion(c,grip(),a,NOW,97)

def test_progress_permission_still_requires_fresh_feedback_and_valid_endpoint():
 c=permit();c['arm_target_rad']=[.01,0,0,0,0,0]
 a=arm();a['status']['joint_state']['q_actual']=[math.degrees(.005),0,0,0,0,0]
 a['received_ns']=NOW-100_000_001
 assert not permits_motion(c,grip(),a,NOW,97)
 a['received_ns']=NOW;c['arm_target_rad']=[float('nan')]*6
 assert not permits_motion(c,grip(),a,NOW,97)

def dynamic_pending():
 c=CppCycle(dynamic_gripper=True);ctx=context(NOW,active=False);ctx['gripper']=grip();c.update(ctx)
 assert c.request(NOW).split()[-6:]==['G3','89','93','97','89','0']
 r=response(c);r['gripper_check']=dict(permit(),hold_state=0,actual_raw=89,sent_raw=93,inflight_raw=97,requested_raw=89,arm_safe=True,range_raw=[89,97],arm_range_raw=[89,97],arm_target_rad=[0.]*6)
 return c,r

def test_permit_uses_request_time_not_response_time():
 c,r=dynamic_pending();c.complete(r,NOW+99_000_000)
 assert c.info['gripper_check']['checked_ns']==NOW
 c,r=dynamic_pending();c.complete(r,NOW+100_000_000)
 assert not c.info['gripper_check']['allowed']

@pytest.mark.parametrize('elapsed_us',[42_000,42_296,float('nan'),float('inf')])
def test_arm_idle_does_not_allow_a_late_gripper_permit(elapsed_us):
 c,r=dynamic_pending();r['callback_us']=elapsed_us
 assert c.complete(r,NOW+49_000_000) is None
 assert not c.info['gripper_check']['allowed']

@pytest.mark.parametrize('key,value',[('inflight_raw',93),('actual_raw',True),('arm_safe',1),('arm_q_rad',[1.]*6),('arm_range_raw',[97,85])])
def test_forged_or_malformed_geometry_response_is_rejected(key,value):
 c,r=dynamic_pending();r['gripper_check'][key]=value
 with pytest.raises(ValueError):c.complete(r,NOW+1)

def test_blocked_target_holds_then_release_discards_it_and_reverse_is_usable():
 g,d=ready();g.motion_guard=lambda s,t:t<.35
 g.set_target(.40);g.cycle()
 assert d.targets[-1]==.35 and g.snapshot()['collision_blocked']
 g.hold();g.set_target(.30);g.cycle()
 assert round(d.targets[-1]*255)==85 and not g.snapshot()['collision_blocked']
 events=[e['event'] for e in g.drain()]
 assert 'gripper_collision_blocked' in events and 'gripper_collision_resumed' in events

def test_can_exchange_does_not_hold_arm_commitment_lock():
 g,d=ready();lock=threading.RLock();g.geometry_lock=lock
 entered=threading.Event();release=threading.Event();original=d.exchange
 def exchange(t):entered.set();assert release.wait(2);return original(t)
 d.exchange=exchange;g.set_target(.30)
 worker=threading.Thread(target=g.cycle);worker.start();assert entered.wait(1)
 try:
  assert lock.acquire(timeout=.05)
  lock.release()
  assert g.snapshot()['inflight_raw']==round(.30*255)
  g.hold()
 finally:release.set();worker.join(2)
 assert not worker.is_alive() and g.snapshot()['target']==.30

def test_large_accumulated_intent_is_checked_and_committed_in_bounded_batches():
 from e6_phosphobot.gripper import geometry_proposal
 g,d=ready();checked=[]
 g.motion_guard=lambda s,t:checked.append((request_state(s,1)[-1],round(t*255))) or True
 g.set_target(.6)
 for _ in range(20):g.cycle()
 raw=[round(t*255) for t in d.targets]
 assert raw[-1]==153
 assert all(a==b for a,b in checked)
 assert all(0<=b-a<=7 for a,b in zip(raw,raw[1:]))
 g.set_target(.15);g.cycle();g.hold()
 held=g.snapshot()['sent_raw']
 for _ in range(5):g.cycle()
 assert g.snapshot()['command_raw']==g.snapshot()['sent_raw']==held
 assert geometry_proposal(38,45)==38

def test_can_change_after_arm_candidate_selection_is_rechecked_at_commit():
 from e6_phosphobot.moveit_bridge import MoveItBridge
 b=MoveItBridge('.',None,clock=lambda:NOW);s=grip();b.gripper_snapshot=lambda:s
 check=dict(permit(),arm_safe=True,arm_range_raw=[89,97]);row=dict(cpp=dict(gripper_check=check))
 assert b.gripper_allows_arm(row)
 s['inflight_raw']=101
 assert not b.gripper_allows_arm(row)
 s.update(valid=False)
 assert not b.gripper_allows_arm(row)
 check['fallback']=True
 assert b.gripper_allows_arm(row)
 check['arm_safe']=False
 assert not b.gripper_allows_arm(row)
 assert not b.gripper_allows_arm({})

def test_gateway_rechecks_guard_before_servoj_and_stop_remains_available():
 from test_gateway import setup
 from test_moveit_bridge import Bridge,row
 e,d,send,step,now=setup();b=Bridge();b.geometry_lock=threading.RLock();b.last_arm_target=None
 admitted=[False];b.gripper_allows_arm=lambda r:admitted[0];e.moveit=b;e.dynamic_gripper=True
 send(deadman=True,linear_m_s=[.002,0,0]);b.row=row(now[0],e.session.sequence);step()
 assert not d.calls
 admitted[0]=True;send(deadman=True,linear_m_s=[.002,0,0]);b.row=row(now[0],e.session.sequence);step()
 assert d.calls[-1][0]=='ServoJ' and b.last_arm_target is not None
 admitted[0]=False;send()
 assert d.calls[-1][0]=='Stop'

@pytest.mark.parametrize('raw',[152,153,154])
def test_checked_neighbor_aperture_is_usable_without_changing_the_guard(raw):
 from e6_phosphobot.moveit_bridge import MoveItBridge
 b=MoveItBridge('.',None,clock=lambda:NOW)
 s=dict(valid=True,received_ns=NOW,actual=raw/255,sent_raw=153,inflight_raw=None)
 b.gripper_snapshot=lambda:s
 c=dict(permit(),arm_safe=True,arm_range_raw=[152,154],range_raw=[152,154],requested_raw=153)
 assert b.gripper_allows_arm(dict(cpp=dict(gripper_check=c)))
 assert permits_motion(c,s,arm(),NOW,153)
 s['actual']=151/255
 assert not b.gripper_allows_arm(dict(cpp=dict(gripper_check=c)))
 assert not permits_motion(c,s,arm(),NOW,153)

def test_neighbor_count_never_extends_a_stale_permit_or_allows_an_unchecked_commitment():
 from e6_phosphobot.moveit_bridge import MoveItBridge
 b=MoveItBridge('.',None,clock=lambda:NOW)
 s=dict(valid=True,received_ns=NOW,actual=153/255,sent_raw=153,inflight_raw=155)
 b.gripper_snapshot=lambda:s
 c=dict(permit(),arm_safe=True,arm_range_raw=[152,154],range_raw=[152,154],requested_raw=153)
 assert not b.gripper_allows_arm(dict(cpp=dict(gripper_check=c)))
 s['inflight_raw']=None;c['checked_ns']=NOW-100_000_001
 assert not b.gripper_allows_arm(dict(cpp=dict(gripper_check=c)))
