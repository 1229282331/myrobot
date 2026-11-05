"""One shared, feedback-only nominal pose model; no hardware access or ROS."""
import json
import math
from functools import lru_cache
from .assembly_model import ASSETS


@lru_cache(maxsize=1)
def geometry():
    path=ASSETS/'gripper-kinematics.json'
    if not path.is_file():return None
    data=json.loads(path.read_text(encoding='utf-8'))
    names=data['joint_names'];rows=data['joints_rad_by_raw']
    if (len(names)!=8 or len(set(names))!=8 or len(rows)!=256 or
        any(len(row)!=8 or not all(math.isfinite(v) for v in row) for row in rows)):
        raise ValueError('Invalid gripper geometry table')
    return data


def raw_position(snapshot,now=None):
    actual=snapshot.get('actual')
    if not snapshot.get('valid') or type(actual) not in (float,int) or not math.isfinite(actual) or not 0<=actual<=1:return -1
    stamp=snapshot.get('received_ns')
    if now is not None and (type(stamp) is not int or not 0<=now-stamp<=200_000_000):return -1
    return round(actual*255)


def pose(snapshot,now=None):
    data=geometry()
    if data is None:return None
    raw=raw_position(snapshot,now)
    return dict(valid=raw>=0,raw=raw if raw>=0 else None,received_ns=snapshot.get('received_ns'),
        source='actual_can_position',qualification=data['qualification'],
        joints_rad=dict(zip(data['joint_names'],data['joints_rad_by_raw'][raw])) if raw>=0 else None,
        fallback=raw<0)


def request_state(snapshot,now):
    from .gripper import geometry_proposal
    raw=raw_position(snapshot,now)
    def count(key):
        value=snapshot.get(key)
        return value if type(value) is int and 0<=value<=255 else -1
    # No target is ever used as actual pose.
    sent=count('sent_raw');requested=count('command_raw')
    escaping=(snapshot.get('contact') or {}).get('escape_requested',False)
    return [raw,sent,count('inflight_raw'),geometry_proposal(requested,sent) if requested>=0 and sent>=0 and not escaping else requested]


def retreat_hold(snapshot,now):
    """Fresh native stationary state, independent of CAN transaction progress.

    0 means uncertain/moving, 1 arrived/held, 2 frozen adaptive closing contact.
    This is not collision permission: C++ still proves the arm path and checks
    every committed aperture, including target error and one-count feedback.
    """
    raw,sent,inflight,requested=request_state(snapshot,now)
    if raw<0 or snapshot.get('phase')!='ready':return 0
    velocity=snapshot.get('reported_velocity_raw')
    if type(velocity) is not int or velocity!=0:return 0
    if inflight>=0 and inflight!=sent:return 0
    contact=snapshot.get('contact')
    if contact:
        return 2 if (snapshot.get('reported_state')==2 and
            snapshot.get('position_status')=='contact' and contact.get('direction')==-1 and
            not contact.get('escape_requested') and not contact.get('escape_pending') and
            sent>=0 and requested==sent) else 0
    if snapshot.get('reported_state')!=0 or snapshot.get('arrived') is not True:return 0
    if sent<0:return 1 if inflight<0 and (requested<0 or abs(raw-requested)<=1) else 0
    return 1 if requested==sent and abs(raw-sent)<=1 else 0


def permits_motion(check,snapshot,arm,now,target_raw,last_arm_target=None):
    # Malformed/partial IPC must deny a new target, never crash the CAN owner.
    try:return _permits_motion(check,snapshot,arm,now,target_raw,last_arm_target)
    except (KeyError,TypeError,ValueError,OverflowError):return False


def _permits_motion(check,snapshot,arm,now,target_raw,last_arm_target=None):
    """Consume a result only for its checked target and nearby fresh arm state.

    Per-link 5 mm geometric reserve includes at least 1 mm between path samples
    and 1 mm arm drift. Bounds below are summed link reach upper bounds, not an
    arbitrary joint-wise equality tolerance.
    """
    if not isinstance(check,dict) or check.get('allowed') is not True or type(check.get('requested_raw')) is not int or check.get('requested_raw')!=target_raw:return False
    if type(target_raw) is not int or not 0<=target_raw<=255:return False
    if type(check.get('checked_ns')) is not int or not 0<=now-check['checked_ns']<=100_000_000:return False
    if not arm.get('connected') or not arm.get('status') or type(arm.get('received_ns')) is not int or not 0<=now-arm['received_ns']<=100_000_000:return False
    q=arm['status']['joint_state']['q_actual'];seed=check.get('arm_q_rad',[])
    def vector(v):return isinstance(v,(list,tuple)) and len(v)==6 and all(type(x) in (float,int) and math.isfinite(x) for x in v)
    if not vector(seed) or not vector(q):return False
    end=check.get('arm_target_rad',[])
    if not vector(end):return False
    # The entire seed-to-end segment was checked for EVERY permitted aperture.
    # Fresh feedback may have progressed along it while FCL was computing.
    # Require proximity to that checked segment, not to its obsolete start.
    d=[b-a for a,b in zip(seed,end)];n=sum(v*v for v in d)
    def covered(position):
        t=max(0.,min(1.,sum((x-a)*v for x,a,v in zip(position,seed,d))/n)) if n else 0.
        deviation=sum(abs(x-(a+t*v))*r for x,a,v,r in zip(position,seed,d,(1.,.9,.7,.5,.4,.3)))
        return math.isfinite(deviation) and deviation<=.001
    if not covered([math.radians(a) for a in q]):return False
    # An already-sent arm endpoint may still be ahead of the feedback. It
    # must lie on/within 1 mm of the checked segment, not just near the seed.
    if last_arm_target is not None:
        if not vector(last_arm_target) or not covered(last_arm_target):return False
    raw=raw_position(snapshot,now)
    if raw<0:return check.get('fallback') is True and snapshot.get('phase')=='initializing'
    lo,hi=check.get('range_raw',[-1,-1])
    if type(lo) is not int or type(hi) is not int or not 0<=lo<=hi<=255:return False
    return lo<=raw<=hi and all(snapshot.get(k) is None or type(snapshot[k]) is int and lo<=snapshot[k]<=hi for k in ('sent_raw','inflight_raw'))
