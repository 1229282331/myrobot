import base64
from copy import deepcopy
import json
import numpy as np
import pytest
from dobot_sdk.protocol.data_types import MyType,TEST_VALUE_MAGIC
from e6_phosphobot import omnipicker_can as can
from e6_phosphobot.native_dataset import prepare_samples,load_native,wire_joints


def source_parts():
    packet=np.zeros(1,dtype=MyType);packet['len']=1440;packet['TestValue']=TEST_VALUE_MAGIC
    packet['RobotMode']=5;packet['EnableStatus']=1
    joints=[1,2,3,4,5,6];packet['QActual'][0]=joints
    raw=[dict(generation='fixture',sequence=1,received_ns=1_030_000_000,
              raw_b64=base64.b64encode(packet.tobytes()).decode())]
    command='ServoJ(1.234567,2,3,4,5,6,t=0.050,aheadtime=50.0,gain=500.0)'
    receipt=dict(command=command,sent_ns=1_050_000_000,accepted=False,acknowledged=False)
    tx=can.encode_command(35,can.LIMITS,bounds=(15,60))
    rx=bytes([0,0,51,0,15,*tx[5:]])
    events=[dict(event='can',data=dict(kind='tx',host_perf_counter_ns=1_020_000_000,
                can_id=1,data_hex=tx.hex(),send_confirmed=True)),
            dict(event='can',data=dict(kind='rx',host_perf_counter_ns=1_022_000_000,
                flags=0,frame_hex=can.FRAME.pack(1,8,rx).hex())),
            dict(event='motion_submitted',candidate=dict(joints_rad=[99]*6),receipt=receipt,
                 pre_action_observation=dict(generation='fixture',sequence=1,received_ns=1_030_000_000,
                    status=dict(joint_state=dict(q_actual=joints)))),
            dict(event='motion_confirmed',target=dict(receipt=dict(receipt,accepted=True,acknowledged=True,reply_ns=1_052_000_000)))]
    associations=[dict(kind='motion_submitted',action_sent_ns=1_050_000_000,valid=True,
        feedback={k:raw[0][k] for k in ('generation','sequence','received_ns')},
        media=dict(rgb=dict(frame_id=7,valid=True,received_ns=1_025_000_000,
                            sdk_available_ns=1_027_000_000,capture_epoch='fixture')))]
    media=[dict(source='rgb',id=7,file='rgb/7.bin',bytes=100,sha256='fixture',
                metadata=dict(device_frame_id=7,x5_receive_monotonic_ns=1_025_000_000,
                              metadata=dict(host_perf_counter_ns=1_027_000_000,capture_epoch='fixture')))]
    return events,raw,associations,media


def test_labels_are_original_sent_bytes_with_causal_raw_observations():
    parts=source_parts();sample=prepare_samples(*parts)[0]
    np.testing.assert_allclose(sample['observation_state'],np.deg2rad([1,2,3,4,5,6]).tolist()+[51/255])
    np.testing.assert_allclose(sample['action'],np.deg2rad([1.234567,2,3,4,5,6]).tolist()+[89/255])
    assert sample['action'][0]!=99 and sample['action']!=sample['observation_state']
    assert sample['source']['gripper_target_sent_ns']<sample['source']['gripper_observed_ns']<sample['source']['arm_sent_ns']


@pytest.mark.parametrize('change,reason',[
    ('no_gripper','Missing fresh causal gripper'),('late_gripper','Missing fresh causal gripper'),
    ('echo','no confirmed response'),('no_ack','not confirmed'),('wrong_ack','not confirmed'),
    ('future_image','not available'),('invalid_image','invalid original image'),
    ('parsed_differs','differs from official SDK'),('no_raw','Missing or stale'),
    ('can_unconfirmed','Uncertain CAN'),('stale_gripper','Missing fresh causal gripper'),
])
def test_reject_missing_future_uncertain_and_inconsistent_sources(change,reason):
    events,raw,associations,media=source_parts()
    if change=='no_gripper':events=events[2:]
    elif change=='late_gripper':
        for event in events[:2]:event['data']['host_perf_counter_ns']+=50_000_000
    elif change=='echo':events[1]['data']['flags']=4
    elif change=='no_ack':events.pop()
    elif change=='wrong_ack':events[-1]['target']['receipt']['command']='ServoJ(0,0,0,0,0,0)'
    elif change=='future_image':
        media[0]['metadata']['metadata']['host_perf_counter_ns']=1_060_000_000
        associations[0]['media']['rgb']['sdk_available_ns']=1_060_000_000
    elif change=='invalid_image':associations[0]['valid']=False
    elif change=='parsed_differs':events[2]['pre_action_observation']['status']['joint_state']['q_actual'][0]=123
    elif change=='no_raw':raw=[]
    elif change=='can_unconfirmed':events[0]['data']['send_confirmed']=False
    elif change=='stale_gripper':
        for event in events[:2]:event['data']['host_perf_counter_ns']-=200_000_000
    with pytest.raises(ValueError,match=reason):prepare_samples(events,raw,associations,media)


def test_gripper_only_is_not_upgraded_with_actual_arm_state():
    parts=source_parts();parts[0][:]=parts[0][:2]
    with pytest.raises(ValueError,match='No sent arm targets'):prepare_samples(*parts)


def test_new_can_command_in_flight_cannot_reuse_previous_accepted_target():
    events,raw,associations,media=source_parts()
    tx,rx=deepcopy(events[:2])
    tx['data']['host_perf_counter_ns']=1_040_000_000
    rx['data']['host_perf_counter_ns']=1_060_000_000
    events[2:2]=[tx,rx]
    with pytest.raises(ValueError,match='unconfirmed at arm send'):
        prepare_samples(events,raw,associations,media)


@pytest.mark.parametrize('change',['duplicate_media','duplicate_association','epoch','source_id','feedback_id','magic','truncated'])
def test_original_source_identity_and_feedback_framing_are_required(change):
    events,raw,associations,media=source_parts()
    if change=='duplicate_media':media.append(deepcopy(media[0]))
    elif change=='duplicate_association':associations.append(deepcopy(associations[0]))
    elif change=='epoch':associations[0]['media']['rgb']['capture_epoch']='another'
    elif change=='source_id':media[0]['metadata']['device_frame_id']=123
    elif change=='feedback_id':associations[0]['feedback']['sequence']=2
    else:
        wire=bytearray(base64.b64decode(raw[0]['raw_b64']))
        if change=='magic':wire[48:56]=b'\0'*8
        else:wire.pop()
        raw[0]['raw_b64']=base64.b64encode(wire).decode()
    with pytest.raises(ValueError):prepare_samples(events,raw,associations,media)


def test_preflight_cannot_extract_missing_files_into_original(tmp_path):
    import hashlib,zipfile
    from e6_phosphobot.episode_transfer import verify_bundle,associate
    missing='audit/raw.jsonl';data=b'original bytes'
    archive=tmp_path/'bundle.zip'
    with zipfile.ZipFile(archive,'w') as z:
        z.writestr(missing,data)
        z.writestr('bundle_manifest.json',json.dumps([dict(file=missing,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())]))
    before=sorted(p.relative_to(tmp_path).as_posix() for p in tmp_path.rglob('*'))
    with pytest.raises(ValueError,match='Missing retained file'):
        verify_bundle(archive,tmp_path,hashlib.sha256(archive.read_bytes()).hexdigest(),read_only=True)
    with pytest.raises(ValueError,match='Missing retained action associations'):
        associate(tmp_path,[],read_only=True)
    assert before==sorted(p.relative_to(tmp_path).as_posix() for p in tmp_path.rglob('*'))


@pytest.mark.parametrize('kind,result,complete,reason',[
    ('hardware_shadow','success',True,'actual hardware'),
    ('hardware_observation','success',True,'actual hardware'),
    ('hardware_demonstration','aborted',False,'Aborted'),
    ('hardware_demonstration','success',False,'incomplete'),
])
def test_shadow_observe_aborted_and_incomplete_sources_cannot_be_exported(tmp_path,kind,result,complete,reason):
    (tmp_path/'metadata.json').write_text(json.dumps(dict(dataset_kind=kind,control_stage='live')))
    (tmp_path/'summary.json').write_text(json.dumps(dict(result=result,complete=complete)))
    with pytest.raises(ValueError,match=reason):load_native(tmp_path)


@pytest.mark.parametrize('command',['ServoP(1,2,3,4,5,6)','ServoJ(nan,0,0,0,0,0,t=.05,aheadtime=50,gain=500)',
                                   'ServoJ(1,2,3,4,5,6,t=.05)', 'ServoJ(1,2,3,4,5,6,t=.05,gain=500,aheadtime=50)'])
def test_wire_layout_and_finite_labels_are_checked(command):
    with pytest.raises(ValueError):wire_joints(command)
