from test_cpp_cycle import initialized,context,response
from test_gateway import setup
from test_moveit_bridge import Bridge,row


def test_official_leaving_status_and_metrics_remain_attached_to_candidate():
    c,now=initialized();c.update(context(now));c.request(now)
    reply=response(c);reply.update(status=3,singularity=dict(condition_before=60.37,condition_target=60.2,retreat_rejected=False))
    candidate=c.complete(reply,now+1)
    assert candidate['status']==3
    assert candidate['cpp']['singularity']==reply['singularity']==c.info['singularity']
    assert candidate['source_input']['intent']==c.context['command']['intent']
    # A hard block stays non-executable; neutral reset can be followed by an
    # official allowed direction without recreating the adapter.
    now+=50_000_000;c.update(context(now));c.request(now);blocked=response(c);blocked['status']=2
    assert c.complete(blocked,now+1) is None
    now+=50_000_000;c.update(context(now,active=False));c.request(now);c.complete(response(c),now+1)
    now+=50_000_000;c.update(context(now,epoch=2));c.request(now)
    leaving=response(c);leaving['status']=3
    assert c.complete(leaving,now+1)['status']==3


def test_directional_block_does_not_revoke_prepared_session_or_require_joint_mode():
    e,d,send,step,now=setup();e.moveit=Bridge();e.moveit.info['status']=2
    send(deadman=True,angular_rad_s=[.01,0,0]);step()
    assert e.session.ready and not d.calls
    assert '当前模式' in e.reason
    send();step()
    send(deadman=True,angular_rad_s=[-.01,0,0])
    e.moveit.info['status']=3;e.moveit.row=row(now[0],e.session.sequence);e.moveit.row['status']=3
    step()
    assert e.session.ready and e.reason is None
    assert [c[0] for c in d.calls]==['ServoJ']
