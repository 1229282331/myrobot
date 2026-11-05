"""Local-only official LeRobot qualification; all generated data are synthetic.

Run with the pinned teleop Python 3.12 dataset environment. Never opens devices.
Reuses the native protocol fixture and the production importer/writer/reader.
"""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'upstream/dobot-sdk'),str(ROOT/'tests')]
import numpy as np
from PIL import Image,ImageDraw
import zstandard
from dobot_sdk.protocol.data_types import MyType
from e6_phosphobot.episode_transfer import associate,digest_file
from e6_phosphobot.native_dataset import load_native,export_prepared
from test_native_dataset import source_parts


def write_json(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2),encoding='utf8')


def write_rows(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in data),encoding='utf8')


def fixture(path,result,count):
    path.mkdir(parents=True,exist_ok=False)
    metadata=dict(dataset_kind='qualification_fixture',control_stage='fixture',
        task='Synthetic native format and official LeRobot I/O qualification; not real robot data',
        action_association_policy='sdk_available_v2')
    write_json(path/'metadata.json',metadata)
    write_json(path/'summary.json',dict(result=result,complete=True,synthetic=True))
    all_events=[];all_raw=[];all_media=[]
    for index in range(count):
        events,raw,_,media=source_parts()
        # Intentionally irregular physical clocks; the official table timeline
        # must be described as event indices and retain these original times.
        offset=index*60_000_000+(index%3)*1_000_000
        for event in events:
            if event['event']=='can':event['data']['host_perf_counter_ns']+=offset
            else:
                receipt=event.get('receipt') or event['target']['receipt']
                for key in ('sent_ns','reply_ns'):
                    if key in receipt:receipt[key]+=offset
        raw[0]['sequence']=index+1;raw[0]['received_ns']+=offset
        packet=np.frombuffer(base64.b64decode(raw[0]['raw_b64']),dtype=MyType).copy()
        packet['QActual'][0][0]+=index*.01
        raw[0]['raw_b64']=base64.b64encode(packet.tobytes()).decode()
        observation=events[2]['pre_action_observation']
        observation.update({k:raw[0][k] for k in ('generation','sequence','received_ns')})
        observation['status']['joint_state']['q_actual']=packet['QActual'][0].tolist()
        item=media[0];item.update(id=index,file=f'rgb_{index:04d}.bin')
        item['metadata']['device_frame_id']=index
        item['metadata']['x5_receive_monotonic_ns']+=offset
        meta=item['metadata']['metadata'];meta['host_perf_counter_ns']+=offset
        meta.update(width=1280,height=720)
        pixels=Image.new('RGB',(1280,720),((index*47)%256,32 if result=='success' else 210,(index*31)%256))
        draw=ImageDraw.Draw(pixels)
        draw.rectangle((index*10,100,index*10+150,400),fill='white')
        draw.text((20,20),f'SYNTHETIC {result} {index}',fill='black')
        buffer=io.BytesIO();pixels.save(buffer,format='JPEG',quality=95);data=buffer.getvalue()
        item.update(bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),storage_codec='zstd')
        destination=path/'media'/item['file'];destination.parent.mkdir(exist_ok=True)
        destination.write_bytes(zstandard.ZstdCompressor().compress(data))
        all_events+=events;all_raw+=raw;all_media+=media
    write_rows(path/'audit/events.jsonl',all_events)
    write_rows(path/'audit/raw.jsonl',all_raw)
    write_rows(path/'media/manifest.jsonl',all_media)
    write_rows(path/'media/camera_samples.jsonl',[])
    associate(path,all_media)
    files=sorted(p for p in path.rglob('*') if p.is_file() and p.name!='associations.jsonl')
    manifest=[dict(file=p.relative_to(path).as_posix(),bytes=p.stat().st_size,sha256=digest_file(p)) for p in files]
    with zipfile.ZipFile(path/'bundle.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,p.relative_to(path).as_posix())
        archive.writestr('bundle_manifest.json',json.dumps(manifest))
    write_json(path/'verification.json',dict(bundle_sha256=digest_file(path/'bundle.zip')))
    return load_native(path,fixture=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    sources=[fixture(args.output/'fixture-success','success',24),fixture(args.output/'fixture-failure','failure',28)]
    report=export_prepared(sources,args.output/'lerobot',fixture=True)
    write_json(args.output/'qualification-report.json',report)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
