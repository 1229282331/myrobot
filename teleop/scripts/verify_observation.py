"""Inspect actual observation rows and original media, without contacting devices."""
import argparse
import base64
import hashlib
import json
import struct
from pathlib import Path
import sys
import numpy as np
from PIL import Image
from e6_teleop.exporter import load_episode
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.protocol import decode


def verify(path,allow_aborted=False):
    path=Path(path).resolve()
    metadata=json.loads((path/'metadata.json').read_text(encoding='utf-8'))
    summary=json.loads((path/'summary.json').read_text(encoding='utf-8'))
    rows=[json.loads(s) for s in (path/'samples.jsonl').read_text(encoding='utf-8').splitlines()]
    demonstration=metadata['dataset_kind']=='hardware_demonstration'
    assert demonstration or metadata['dataset_kind']=='hardware_observation' and metadata['action_semantics'] is None
    if allow_aborted and summary['result']=='aborted':assert not summary['complete'] and summary.get('media_status','verified')=='verified'
    else:assert summary['complete'] and summary['result'] in (('success','failure') if demonstration else ('observed',))
    assert summary['samples']==len(rows) and len(rows)>0
    action_count=0
    checked=set();ids={};repeats=dict(rgb=0,depth=0);invalid=dict(rgb=0,depth=0)
    for index,row in enumerate(rows):
        o=row['observation']
        assert row['sample_index']==index and o['valid'] and not o['is_simulated']
        if not demonstration:assert row['input'] is None and row['effective_action'] is None and o['gripper'] is None
        else:
            assert row['schema_version']==3 and row['backend']=='hardware-control' and row['control_stage']==metadata['control_stage']
            before=o['control_tick_monotonic_ns']
            if index:assert o['control_tick_id']==rows[index-1]['observation']['control_tick_id']+1
            assert row['sources']['robot']['x5_receive_monotonic_ns']<=before
            for source in ('rgb','depth','gripper'):
                received=row['sources'][source].get('x5_receive_monotonic_ns')
                if received is not None:assert received<=before
            action=row.get('effective_action')
            if action:
                action_count+=1
                assert metadata['control_stage']=='live' and before<=action['gateway_sent_ns']<=action['gateway_accepted_ns']
                assert action['acceptance']=='controller_acknowledged_not_completion'
                assert len(action['joints_rad'])==6 and np.isfinite(action['joints_rad']).all()
            if metadata['control_stage']=='shadow':assert action is None
        assert len(o['joints_rad'])==6 and np.isfinite(o['joints_rad']).all()
        assert row['pc_recorded_monotonic_ns']>=row['pc_received_monotonic_ns']
        if index:
            previous=rows[index-1]['observation']
            assert o['frame_id']==previous['frame_id']+1 and o['gateway_monotonic_ns']>previous['gateway_monotonic_ns']
        decoded=decode(base64.b64decode(row['sources']['robot']['raw_base64'],validate=True),extended=demonstration)
        source=row['sources']['robot']
        if source.get('wire_prefix_base64'):
            assert base64.b64decode(source['wire_prefix_base64'],validate=True)==struct.pack('<Q',1440)
            assert source['wire_bytes']==1448 and source['transport_framing']=='explicit-duplicated-length-envelope'
        for key,value in decoded.items():
            original=row['sources']['robot']['decoded'][key]
            if key in ('joints_rad','controller_quaternion_xyzw'):
                # ARM64 and Windows libm may differ by the last float bit.
                np.testing.assert_allclose(value,original,rtol=0,atol=1e-12)
            else:assert value==original
        for name in ('rgb','depth'):
            s=row['sources'][name]
            if not s.get('file'):continue
            image=(path/s['file']).resolve();assert image.is_relative_to(path/'images')
            repeats[name]+=s['repeated'];identity=(name,s['device_frame_id'])
            assert s['repeated']==(identity in ids)
            ids[identity]=s['sha256']
            if image not in checked:
                assert hashlib.sha256(image.read_bytes()).hexdigest()==s['sha256']
                with Image.open(image) as im:
                    im.load()
                    assert im.size==((1280,720) if name=='rgb' else (640,480))
                    if name=='rgb':assert im.mode=='RGB'
                    else:
                        data=np.asarray(im,dtype='<u2').tobytes()
                        assert hashlib.sha256(data).hexdigest()==s['raw_uint16_sha256']
                checked.add(image)
            delta=(s['x5_receive_monotonic_ns']-row['sources']['robot']['x5_receive_monotonic_ns'])/1e6
            a=row['associations'][name+'_robot']
            assert a['receive_delta_ms']==delta
            if a['valid']:assert abs(delta)<=100
            else:invalid[name]+=1
        if metadata.get('control_stage')!='live':assert row['sources']['gripper']['opening'] is None
    if demonstration:assert summary['accepted_action_samples']==action_count
    try:load_episode(path)
    except ValueError:pass
    else:raise AssertionError('Observation exported as action data')
    elapsed=(rows[-1]['observation']['gateway_monotonic_ns']-rows[0]['observation']['gateway_monotonic_ns'])/1e9
    return dict(raw_episode=str(path),result=summary['result'],complete=summary['complete'],dataset_kind=metadata['dataset_kind'],control_stage=metadata.get('control_stage'),is_real_demonstration=metadata.get('is_real_demonstration',False),accepted_action_samples=action_count,samples=len(rows),duration_s=elapsed,actual_sample_hz=(len(rows)-1)/elapsed if elapsed else None,
                unique_images=len(checked),repeated_images=repeats,invalid_associations=invalid,
                association_valid_fraction={n:1-invalid[n]/len(rows) for n in invalid},
                all_media_hashes_checked=True,depth_uint16_lossless=True,export_rejected=True,
                robot_mode_values=sorted({r['sources']['robot']['decoded']['robot_mode'] for r in rows}),
                enable_values=sorted({r['sources']['robot']['decoded']['enable_status'] for r in rows}),
                error_values=sorted({r['sources']['robot']['decoded']['error_status'] for r in rows}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path);p.add_argument('--allow-aborted',action='store_true',help='Verify the preserved prefix; never marks it complete');a=p.parse_args()
    report=verify(a.episode,a.allow_aborted)
    if a.output:a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
