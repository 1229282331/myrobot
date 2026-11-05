"""Offline qualification of native journals through the pinned LeRobot API.

Rows are confirmed ServoJ command events, not a resampled 20 Hz recording.
Each row retains the exact command, causal observations and original clocks.
The 7D action includes the last confirmed CAN target at that send time.
Missing/uncertain labels reject the entire source; no state-to-action fill.
This module has no device connection or replay path.
"""
import argparse
import base64
import bisect
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path
import re

import numpy as np
from PIL import Image
import zstandard
from .dobot_transport import FeedbackStream
from . import omnipicker_can as can
from .episode_transfer import rows,verify_bundle,safe_name

LEROBOT_COMMIT='8c920c4270460851cedd2737657584586d3dc66f'
IMAGE_KEY='observation.images.wrist'
NAMES=[f'joint_{i}_rad' for i in range(1,7)]+['gripper_open_fraction']


@dataclass
class Prepared:
    path: Path
    metadata: dict
    summary: dict
    samples: list
    bundle_sha256: str


def wire_joints(command):
    if not isinstance(command,str) or not re.fullmatch(r'ServoJ\([^()]+\)',command):
        raise ValueError('Only original ServoJ commands are supported')
    fields=command[7:-1].split(',')
    if len(fields)!=9 or [v.split('=')[0] for v in fields[6:]]!=['t','aheadtime','gain']:
        raise ValueError('Unexpected official ServoJ wire layout')
    values=np.asarray([float(v) for v in fields[:6]])
    if not np.isfinite(values).all():raise ValueError('Non-finite sent joint target')
    return np.deg2rad(values).tolist()


def gripper_observations(events):
    """Use existing protocol decoder and exactly associated TX/RX bytes."""
    pending=None;result=[];previous=0
    for event in events:
        if event.get('event')!='can':continue
        data=event['data'];kind=data.get('kind')
        if kind=='tx':
            if pending is not None:raise ValueError('CAN command has no uniquely associated response')
            if not data.get('send_confirmed'):raise ValueError('Uncertain CAN send')
            command=bytes.fromhex(data['data_hex'])
            if len(command)!=8 or command[0]!=0 or command[6:]!=b'\0\0':
                raise ValueError('Invalid position command bytes')
            if data['host_perf_counter_ns']<=previous:raise ValueError('CAN send times are not increasing')
            if result and data['host_perf_counter_ns']<result[-1]['received_ns']:
                raise ValueError('CAN transactions overlap')
            previous=data['host_perf_counter_ns']
            pending=data
        elif kind=='rx' and pending is not None:
            try:
                feedback=can.decode_feedback(bytes.fromhex(data['frame_hex']),data.get('flags',0),
                    pending['can_id'],bytes.fromhex(pending['data_hex']))
            except can.CheckFailed as exc:raise ValueError(str(exc)) from exc
            if feedback is None:continue
            received=data['host_perf_counter_ns'];sent=pending['host_perf_counter_ns']
            if not 0<=received-sent<=100_000_000:raise ValueError('Invalid CAN response timing')
            result.append(dict(received_ns=received,sent_ns=sent,actual=feedback['position_raw']/255,
                target=bytes.fromhex(pending['data_hex'])[1]/255,tx=pending,rx=data))
            pending=None
    if pending is not None:raise ValueError('Final CAN command has no confirmed response')
    return sorted(result,key=lambda row:row['received_ns'])


def prepare_samples(events,raw,associations,media):
    submitted=[e for e in events if e.get('event')=='motion_submitted']
    if not submitted:raise ValueError('No sent arm targets; gripper-only records are not 7D action episodes')
    acknowledged={}
    for event in events:
        if event.get('event')=='motion_confirmed':
            receipt=event['target']['receipt'];key=receipt['sent_ns']
            if key in acknowledged:raise ValueError('Duplicate arm confirmation')
            acknowledged[key]=receipt
    raw_index={(r['generation'],r['sequence']):r for r in raw}
    if len(raw_index)!=len(raw):raise ValueError('Duplicate raw feedback identity')
    image_index={(m['source'],m['id']):m for m in media}
    assoc_index={(a['kind'],a['action_sent_ns']):a for a in associations}
    if len(image_index)!=len(media) or len(assoc_index)!=len(associations):
        raise ValueError('Duplicate media or action association identity')
    grips=gripper_observations(events);grip_times=[g['received_ns'] for g in grips]
    grip_sends=[g['sent_ns'] for g in grips]
    result=[];previous=0
    for event in submitted:
        receipt=event['receipt'];sent=receipt['sent_ns'];ack=acknowledged.get(sent,{})
        if sent<=previous:raise ValueError('Sent command times are not increasing')
        previous=sent
        if not ack.get('accepted') or not ack.get('acknowledged') or ack.get('command')!=receipt['command']:
            raise ValueError('Arm target is not confirmed by its original command receipt')
        if ack.get('reply_ns',0)<sent:raise ValueError('Confirmation precedes send')
        observation=event['pre_action_observation'];received=observation['received_ns']
        original=raw_index.get((observation['generation'],observation['sequence']))
        if original is None or original['received_ns']!=received or not 0<=sent-received<=100_000_000:
            raise ValueError('Missing or stale pre-action arm feedback')
        wire=base64.b64decode(original['raw_b64'],validate=True)
        if len(wire)!=1440:raise ValueError('Expected original 1440-byte SDK feedback')
        stream=FeedbackStream();frames=stream.feed(wire,received);stream.eof()
        if len(frames)!=1:raise ValueError('Invalid original feedback framing')
        decoded=frames[0].status
        joints=np.asarray(decoded.joint_state.q_actual,dtype=float)
        if joints.shape!=(6,) or not np.isfinite(joints).all():raise ValueError('Invalid actual joints')
        if decoded.error_status or decoded.collision_state:raise ValueError('Faulted arm observation')
        if not np.allclose(joints,observation['status']['joint_state']['q_actual'],atol=1e-9,rtol=0):
            raise ValueError('Parsed observation differs from official SDK raw feedback')
        index=bisect.bisect_right(grip_times,sent)-1
        if index<0 or sent-grips[index]['received_ns']>100_000_000:
            raise ValueError('Missing fresh causal gripper feedback and confirmed target; no fill is allowed')
        grip=grips[index]
        if bisect.bisect_right(grip_sends,sent)-1!=index:
            raise ValueError('A newer CAN command was unconfirmed at arm send; target is uncertain')
        association=assoc_index.get(('motion_submitted',sent),{})
        image_source=association.get('media',{}).get('rgb',{})
        image=image_index.get(('rgb',image_source.get('frame_id')))
        if not association.get('valid') or image is None or not image_source.get('valid'):
            raise ValueError('Missing/invalid original image association')
        source=image['metadata'];available=source.get('metadata',{}).get('host_perf_counter_ns')
        image_received=source.get('x5_receive_monotonic_ns')
        if (source.get('device_frame_id')!=image['id'] or
            image_source.get('capture_epoch')!=source.get('metadata',{}).get('capture_epoch') or
            image_source.get('received_ns')!=image_received or image_source.get('sdk_available_ns')!=available or
            association.get('feedback')!={k:original[k] for k in ('generation','sequence','received_ns')}):
            raise ValueError('Original association/source identity differs')
        if not isinstance(available,int) or not isinstance(image_received,int) or not image_received<=available<=sent:
            raise ValueError('Image was not available before the original send')
        if sent-image_received>100_000_000:raise ValueError('Original RGB is stale')
        action=wire_joints(receipt['command'])+[grip['target']]
        result.append(dict(observation_state=np.deg2rad(joints).tolist()+[grip['actual']],action=action,
            image=image,source=dict(arm_sent_ns=sent,arm_reply_ns=ack['reply_ns'],
                feedback_generation=original['generation'],feedback_sequence=original['sequence'],
                arm_observed_ns=received,gripper_observed_ns=grip['received_ns'],gripper_target_sent_ns=grip['sent_ns'],
                gripper_tx=grip['tx'],gripper_rx=grip['rx'],rgb_id=image_source['frame_id'],
                rgb_received_ns=image_received,rgb_available_ns=available,arm_command=receipt['command'])))
    return result


def load_native(path,*,fixture=False):
    path=Path(path).resolve()
    metadata=json.loads((path/'metadata.json').read_text(encoding='utf8'))
    summary=json.loads((path/'summary.json').read_text(encoding='utf8'))
    kind='qualification_fixture' if fixture else 'hardware_demonstration'
    stage='fixture' if fixture else 'live'
    if metadata.get('dataset_kind')!=kind or metadata.get('control_stage')!=stage:
        raise ValueError('Only actual hardware demonstrations can supply action labels')
    if summary.get('result') not in ('success','failure') or not summary.get('complete'):
        raise ValueError('Aborted or incomplete source is retained but not exported')
    if metadata.get('action_association_policy')!='sdk_available_v2':
        raise ValueError('Source predates causal SDK-availability association; raw record is unchanged')
    verified=json.loads((path/'verification.json').read_text(encoding='utf8'))
    # Existing files are checked byte-for-byte; require the association file so
    # this offline read cannot create one in the original recording.
    if not (path/'associations.jsonl').is_file():raise ValueError('Source association audit is missing')
    verify_bundle(path/'bundle.zip',path,verified['bundle_sha256'],read_only=True)
    prepared=prepare_samples(rows(path/'audit/events.jsonl'),rows(path/'audit/raw.jsonl'),
                             rows(path/'associations.jsonl'),rows(path/'media/manifest.jsonl'))
    return Prepared(path,metadata,summary,prepared,verified['bundle_sha256'])


def image_array(source,row):
    entry=row['image'];file=source.path/'media'/str(safe_name(entry['file']))
    data=zstandard.ZstdDecompressor().decompress(file.read_bytes(),max_output_size=entry['bytes'])
    if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
        raise ValueError('Native image changed after preflight')
    with Image.open(io.BytesIO(data)) as image:
        if image.mode!='RGB':raise ValueError('Expected native RGB')
        return np.asarray(image).copy()


def export_prepared(sources,output,*,fixture=False):
    """Official writer/reader, with an event-index timeline and explicit clocks.

    Not a policy-ready uniform-time dataset. CAN-only intervals, pauses and
    original clocks remain in source journals and source-events.jsonl.
    """
    import importlib.metadata
    from lerobot.configs import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from torch.utils.data import DataLoader
    direct=json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'))
    if direct.get('vcs_info',{}).get('commit_id')!=LEROBOT_COMMIT:raise RuntimeError('LeRobot pin differs')
    if not sources or any(not s.samples for s in sources):raise ValueError('No eligible action rows')
    expected_kind='qualification_fixture' if fixture else 'hardware_demonstration'
    if any(s.metadata.get('dataset_kind')!=expected_kind for s in sources):
        raise ValueError('Fixture data and real data cannot be relabelled or mixed')
    output=Path(output).resolve();partial=output.with_name(output.name+'.partial')
    if output.exists() or partial.exists():raise FileExistsError('Output or unfinished export already exists')
    shape=image_array(sources[0],sources[0].samples[0]).shape
    features={'observation.state':dict(dtype='float32',shape=(7,),names=NAMES),
              'action':dict(dtype='float32',shape=(7,),names=NAMES),
              'next.success':dict(dtype='bool',shape=(1,),names=None),
              IMAGE_KEY:dict(dtype='video',shape=shape,names=['height','width','channels'])}
    writer=LeRobotDataset.create('local/e6_command_events',fps=20,root=partial,features=features,
        robot_type='qualification_fixture' if fixture else 'e6_confirmed_command_events',video_backend='pyav',
        rgb_encoder=RGBEncoderConfig(vcodec='h264',crf=18,preset='fast',g=2),encoder_threads=2,image_writer_threads=2)
    qualification=dict(dataset_kind=expected_kind,policy_ready=False,fixture=fixture,
        timestamp_semantics='event index / 20; not physical resampling',
        raw_sources=[dict(episode=s.path.name,result=s.summary['result'],bundle_sha256=s.bundle_sha256) for s in sources])
    (partial/'qualification.json').write_text(json.dumps(qualification,indent=2),encoding='utf8')
    mapping=[]
    try:
        for episode,source in enumerate(sources):
            for frame,row in enumerate(source.samples):
                state=np.asarray(row['observation_state'],dtype=np.float32);action=np.asarray(row['action'],dtype=np.float32)
                if any(v.shape!=(7,) or not np.isfinite(v).all() or not 0<=v[-1]<=1 for v in (state,action)):
                    raise ValueError('Invalid seven-dimensional vectors')
                pixels=image_array(source,row)
                if pixels.shape!=shape:raise ValueError('Native resolution changed')
                writer.add_frame({'observation.state':state,'action':action,
                    'next.success':np.asarray([source.summary['result']=='success'],dtype=bool),
                    IMAGE_KEY:pixels,'task':source.metadata['task']})
                mapping.append(dict(episode_index=episode,frame_index=frame,raw_episode=source.path.name,
                    source_bundle_sha256=source.bundle_sha256,**row['source']))
            writer.save_episode(parallel_encoding=False)
    finally:writer.finalize()
    (partial/'source-events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in mapping),encoding='utf8')
    dataset=LeRobotDataset('local/e6_command_events',root=partial,video_backend='pyav')
    expected=[row for source in sources for row in source.samples]
    np.testing.assert_array_equal(np.asarray(dataset.hf_dataset['action']),np.asarray([r['action'] for r in expected],dtype=np.float32))
    np.testing.assert_array_equal(np.asarray(dataset.hf_dataset['observation.state']),np.asarray([r['observation_state'] for r in expected],dtype=np.float32))
    # The official Arrow reader exposes shape-(1,) bool features as scalars.
    expected_success=np.asarray([s.summary['result']=='success' for s in sources for _ in s.samples],dtype=bool)
    np.testing.assert_array_equal(np.asarray(dataset.hf_dataset['next.success']),expected_success)
    assert len(dataset)==len(expected) and dataset.num_episodes==len(sources)
    batch=next(iter(DataLoader(dataset,batch_size=min(8,len(dataset)),num_workers=0,shuffle=False)))
    assert tuple(batch['action'].shape)==(min(8,len(dataset)),7)
    assert tuple(batch['observation.state'].shape)==(min(8,len(dataset)),7)
    assert tuple(batch[IMAGE_KEY].shape)==(min(8,len(dataset)),3,*shape[:2])
    offsets=[];offset=0
    for episode,source in enumerate(sources):
        table=dataset.hf_dataset.select(range(offset,offset+len(source.samples)))
        assert [int(v) for v in table['episode_index']]==[episode]*len(source.samples)
        assert [int(v) for v in table['frame_index']]==list(range(len(source.samples)))
        np.testing.assert_allclose(table['timestamp'],np.arange(len(source.samples))/20,atol=1e-4)
        errors=[]
        for index in range(len(source.samples)):
            frame=dataset[offset+index]
            assert int(frame['episode_index'])==episode and int(frame['frame_index'])==index
            decoded=frame[IMAGE_KEY].permute(1,2,0).numpy()*255
            error=float(np.abs(decoded-image_array(source,source.samples[index]).astype(float)).mean())
            assert error<12,'Decoded frame differs from original source image'
            errors.append(error)
        offsets.append(dict(episode_index=episode,raw_episode=source.path.name,result=source.summary['result'],
            frames=len(source.samples),dataset_from=offset,dataset_to=offset+len(source.samples),video_mae_max=max(errors)))
        offset+=len(source.samples)
    import av
    videos=[]
    for video in sorted(partial.glob('videos/**/*.mp4')):
        with av.open(str(video)) as container:
            count=0;last=None
            for frame in container.decode(video=0):
                assert frame.pts is not None and (last is None or frame.pts>last)
                assert (frame.height,frame.width)==shape[:2]
                last=frame.pts;count+=1
            videos.append(dict(path=video.relative_to(partial).as_posix(),frames=count,
                               codec=container.streams.video[0].codec_context.name))
    assert sum(v['frames'] for v in videos)==len(expected)
    window=LeRobotDataset('local/e6_command_events',root=partial,video_backend='pyav',
        delta_timestamps={'action':[-.05,0,.05],IMAGE_KEY:[-.05,0,.05]})
    for item in offsets:
        if item['frames']<2:continue
        for index,padding in ((item['dataset_from'],[True,False,False]),(item['dataset_to']-1,[False,False,True])):
            sample=window[index]
            assert sample['action_is_pad'].tolist()==padding and sample[IMAGE_KEY+'_is_pad'].tolist()==padding
            assert tuple(sample['action'].shape)==(3,7)
    assert rows(partial/'source-events.jsonl')==mapping
    report=dict(schema='e6-lerobot-command-events-v1',lerobot_commit=LEROBOT_COMMIT,fixture=fixture,
        policy_ready=False,uploaded=False,physical_replay_allowed=False,frames=len(dataset),episodes=offsets,
        video_backend='pyav',encoder='CPU libx264',videos=videos,all_video_frames_checked=True,
        dataloader_action_shape=list(batch['action'].shape),dataloader_image_shape=list(batch[IMAGE_KEY].shape),
        temporal_boundary_padding=True,source_mapping_roundtrip=True,
        timestamp_semantics='LeRobot timestamp=index/20 is an event-index display timeline, not physical 20Hz resampling; source-events.jsonl retains original association clocks; native journals remain authoritative.',
        action_semantics='Original confirmed ServoJ bytes plus last confirmed CAN target; no next-state substitution. CAN-only intervals and pauses remain in raw source, not fabricated as arm targets.')
    (partial/'native-validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    partial.rename(output)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,help='Explicit offline qualification export; omitted means preflight only')
    args=parser.parse_args();sources=[];report=[]
    for path in args.episodes:
        try:
            source=load_native(path);sources.append(source)
            report.append(dict(episode=path.name,eligible=True,frames=len(source.samples)))
        except (OSError,ValueError,KeyError) as exc:report.append(dict(episode=path.name,eligible=False,reason=str(exc)))
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if args.output:
        if len(sources)!=len(args.episodes):raise ValueError('Entire export refused; invalid sources are not silently omitted')
        print(json.dumps(export_prepared(sources,args.output),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
