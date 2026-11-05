"""Offline native-media and same-tick action audit. Never connects to a device."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from e6_observer.protocol import decode
from omnipicker.tools import verify_can_control as can


def require(condition,reason):
    if not condition:raise ValueError(reason)


def audit_gripper_action(row,action,clock):
    """Check the pre-action state and exact recorded CAN request/reply pair."""
    source=row['sources']['gripper']
    if source.get('valid'):
        require(source['x5_receive_monotonic_ns']<=clock,'Gripper observation was selected after the action tick')
        raw=bytes.fromhex(source['raw_hex'])
        require(len(raw)==8 and raw[2]==source['position_raw'],'Gripper observation raw position differs')
        require(abs(row['observation']['gripper']-raw[2]/255)<1e-12,'Gripper observation was filled from a target')
    else:require(row['observation'].get('gripper') is None,'Unavailable gripper observation was filled')
    if action.get('gripper') is None:return False
    require(action.get('gripper_acceptance')=='request_associated_feedback','Gripper action lacks associated feedback')
    feedback=action['gripper_feedback'];events=row['control']['can_events']
    replies=[event for event in events if event['kind']=='rx' and event['host_perf_counter_ns']==feedback['host_perf_counter_ns']]
    require(len(replies)==1,'Gripper action lacks its exact raw CAN receive')
    reply=replies[0]
    sends=[event for event in events if event['kind']=='tx' and event['host_perf_counter_ns']<reply['host_perf_counter_ns']]
    require(bool(sends),'Gripper action lacks its recorded CAN send')
    sent=max(sends,key=lambda event:event['host_perf_counter_ns']);command=bytes.fromhex(sent['data_hex'])
    require(sent['send_confirmed'] and sent['can_id']==1 and len(command)==8,'Gripper send identity/length differs')
    require(action['gateway_accepted_ns']<=sent['host_perf_counter_ns']<reply['host_perf_counter_ns'],'CAN exchange order differs')
    hard_s=(((row['control'].get('tick') or {}).get('status') or {}).get('limits') or {}).get('transaction_s',.05)
    require(hard_s in (.05,.075),'Unknown combined control/CAN budget')
    require(reply['host_perf_counter_ns']-action['transaction_started_ns']<=round(hard_s*1e9),'Combined control/CAN transaction exceeded its recorded deadline')
    require(command[1]==feedback['command_position_raw'] and abs(action['gripper']-command[1]/255)<1e-12,'Gripper action is not the transmitted target')
    parsed=can.decode_feedback(bytes.fromhex(reply['frame_hex']),reply['flags'],1,command)
    require(parsed is not None,'Gripper action used an echo or unrelated CAN frame')
    for key,value in parsed.items():require(value==feedback[key],'Gripper raw feedback differs: '+key)
    require(abs(feedback['round_trip_ms']-(reply['host_perf_counter_ns']-sent['host_perf_counter_ns'])/1e6)<1e-6,'CAN receive/send timing differs')
    return True


def audit(directory):
    directory=Path(directory).resolve()
    metadata=json.loads((directory/'metadata.json').read_text(encoding='utf-8'))
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    require(metadata['dataset_kind']=='hardware_demonstration' and metadata['control_stage']=='live','Expected a live hardware demonstration')
    require(summary['complete'] and summary['media_status']=='verified' and summary['result'] in ('success','failure'),'Episode is aborted, incomplete or awaiting media')
    previous_tick=None;previous_frame=None;previous_pc=None;previous_clock=None
    media={};actions=0;gripper_actions=0;count=0;associations=dict(rgb=0,depth=0);times=[]
    path=directory/'samples.jsonl'
    for line in path.open(encoding='utf-8'):
        row=json.loads(line);o=row['observation'];count+=1
        require(row['sample_index']==count-1 and o['valid'] and not o['is_simulated'],'Invalid sample index/observation')
        tick=o['control_tick_id'];frame=o['frame_id'];clock=o['control_tick_monotonic_ns']
        if previous_tick is not None:
            require(tick==previous_tick+1 and frame==previous_frame+1 and clock>previous_clock,'Missing, repeated or out-of-order control sample')
            require(row['pc_recorded_monotonic_ns']>previous_pc,'PC disk clock did not increase')
        previous_tick,previous_frame,previous_pc,previous_clock=tick,frame,row['pc_recorded_monotonic_ns'],clock
        times.append(clock)
        robot=row['sources']['robot']
        parsed=decode(base64.b64decode(robot['raw_base64'],validate=True),True)
        for name,value in parsed.items():
            if isinstance(value,list):np.testing.assert_allclose(value,robot['decoded'][name],atol=1e-12,rtol=0)
            else:require(value==robot['decoded'][name],'Feedback parse differs: '+name)
        action=row.get('effective_action')
        if action:
            actions+=1
            require(action['acceptance']=='controller_acknowledged_not_completion','Unconfirmed command used as effective action')
            require(robot['x5_receive_monotonic_ns']<=clock<=action['gateway_sent_ns']<=action['gateway_accepted_ns'],'Observation does not precede action')
            events=(row.get('control') or {}).get('command_events',[])
            matched=[e for e in events if action['gateway_sent_ns']<=e.get('x5_sent_ns',0)<=e.get('x5_reply_ns',0)<=action['gateway_accepted_ns'] and e.get('acknowledged') and e.get('command','').startswith('ServoJ(')]
            require(len(matched)==1,'Accepted action lacks its exact send/ACK event')
            require(int(matched[0]['raw_reply'].split('{',1)[1].split('}',1)[0])==action['command_id'],'Action command ID differs from ACK')
            target=[float(v) for v in matched[0]['command'][7:].split(')',1)[0].split(',')[:6]]
            np.testing.assert_allclose(np.radians(target),action['joints_rad'],atol=1e-8,rtol=0)
            gripper_actions+=int(audit_gripper_action(row,action,clock))
        for name in ('rgb','depth'):
            source=row['sources'][name]
            received=source.get('x5_receive_monotonic_ns')
            if received is not None:require(received<=clock,'Image was selected after the action tick')
            associations[name]+=int(row['associations'][name+'_robot']['valid'])
            if not source.get('file'):continue
            require(not source.get('media_pending'),'Media reference is unverified')
            key=(name,source['device_frame_id']);spec=(source['file'],source['sha256'],source.get('raw_uint16_sha256'))
            if key in media:require(media[key]==spec,'Repeated frame has a different reference');continue
            file=(directory/source['file']).resolve()
            require(directory in file.parents,'Media path leaves episode')
            require(hashlib.sha256(file.read_bytes()).hexdigest()==source['sha256'],'Native file checksum mismatch')
            with Image.open(file) as image:
                image.load();require(image.size==(source['width'],source['height']),'Native image dimensions mismatch')
                if name=='depth':
                    values=np.asarray(image)
                    require(values.dtype.kind in ('u','i') and values.min()>=0 and values.max()<=65535,'Expected lossless uint16 depth')
                    raw=values.astype('<u2').tobytes()
                    require(hashlib.sha256(raw).hexdigest()==source['raw_uint16_sha256'],'Native depth values changed')
            media[key]=spec
    require(count==summary['samples'] and actions>0 and actions==summary['accepted_action_samples'],'Sample/action summary mismatch')
    duration=(times[-1]-times[0])/1e9
    return dict(passed=True,episode=directory.name,samples=count,accepted_action_samples=actions,native_media_checked=len(media),
        request_associated_gripper_actions_checked=gripper_actions,
        duration_s=duration,sample_hz=(count-1)/duration,association_valid_fraction={k:v/count for k,v in associations.items()},
        action_labels='same-tick acknowledged targets, verified against sent ServoJ; no next-state substitution',
        raw_sdk_feedback_checked=count,exposure_synchronization_claimed=False,real_lerobot_export_allowed=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.directory)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
