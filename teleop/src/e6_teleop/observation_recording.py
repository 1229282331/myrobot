"""Raw v2 observations / v3 demonstrations; neither uses the simulation exporter."""
import base64
import hashlib
from io import BytesIO
import json
from pathlib import Path
import time
import numpy as np
from PIL import Image
from .recording import EpisodeRecorder


class ObservationRecorder(EpisodeRecorder):
    def __init__(self, root, dataset_kind='hardware_observation'):
        super().__init__(root)
        if dataset_kind not in ('hardware_observation','hardware_demonstration'):raise ValueError('Unknown hardware dataset kind')
        self.dataset_kind=dataset_kind
        self.pending=False;self.phase='idle';self.transfer_error=None

    def start(self, task, metadata):
        if self.pending:raise ValueError('原始媒体仍在传输与校验，不能开始新片段')
        name = super().start(task, {})
        demonstration=self.dataset_kind=='hardware_demonstration'
        metadata = dict(metadata, schema_version=3 if demonstration else 2, dataset_kind=self.dataset_kind, is_real_demonstration=demonstration and metadata.get('control_stage')=='live',
                        fps=20, task=task.strip(), action_semantics='controller acknowledged absolute targets; completion separately observed' if demonstration else None, quaternion_order="xyzw",
                        clock_note="Association uses X5 receive perf_counter_ns only; PC and device clocks remain separate")
        (self.directory/"metadata.json").write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding="utf-8")
        self.instance = None
        self.ids = {}
        self.repeated = dict(rgb=0,depth=0)
        self.invalid_associations = dict(rgb=0,depth=0)
        self.sampled_source_skipped = dict(robot=0,rgb=0,depth=0)
        self.previous_source_seq={}
        self.first_ns = None
        self.control_stage=metadata.get('control_stage');self.accepted_actions=0
        self.last_control_tick=None
        self.deferred=metadata.get('media_storage')=='x5-spool-then-verified-pc'
        self.specs={};self.verified={};self.phase='capturing';self.transfer_error=None
        return name

    def sample(self, frame):
        if not self.active: return
        expected=3 if self.dataset_kind=='hardware_demonstration' else 2
        backend='hardware-control' if expected==3 else 'hardware-observe'
        if frame.get('schema_version')!=expected or frame.get('backend')!=backend or expected==2 and (frame.get('input') is not None or frame.get('effective_action') is not None):
            self.incomplete=True; raise ValueError(f"Expected {backend} protocol v{expected}")
        if expected==3:
            if (frame.get('status') or {}).get('faulted'):
                self.retain_failure(frame,'Control fault before sample acceptance')
                self.incomplete=True;raise ValueError('Control fault aborted the hardware recording; explicit new preparation required')
            tick=frame['observation'].get('control_tick_id')
            if type(tick) is not int or self.last_control_tick is not None and tick!=self.last_control_tick+1:
                self.incomplete=True;raise ValueError('Control tick duplicate or gap; no action resampling')
            a=frame.get('effective_action')
            if a:self.accepted_actions+=1
            before=frame['observation'].get('control_tick_monotonic_ns')
            if a and (frame.get('control_stage')!='live' or before is None or a['gateway_sent_ns']<before or a['gateway_accepted_ns']<a['gateway_sent_ns']):
                self.incomplete=True;raise ValueError('Invalid hardware action association')
            for name in ('rgb','depth'):
                received=frame['sources'][name].get('x5_receive_monotonic_ns')
                if received is not None and before is not None and received>before:
                    self.incomplete=True;raise ValueError('Image selected after action observation')
            received=frame['sources']['gripper'].get('x5_receive_monotonic_ns')
            if received is not None and before is not None and received>before:
                self.incomplete=True;raise ValueError('Gripper observation selected after action')
        o=frame['observation']
        if self.deferred and frame.get('archive_session')!=self.directory.name:
            self.incomplete=True;raise ValueError('Wrong X5 archive session')
        if not o['valid'] or o['is_simulated']:
            self.incomplete=True; raise ValueError("Required E6/RGB observation unavailable or stale")
        if self.instance is not None and self.instance!=frame['instance']:
            self.incomplete=True; raise ValueError("X5 gateway restarted; explicit new recording required")
        if self.last_id is not None and o['frame_id']!=self.last_id+1:
            self.incomplete=True; raise ValueError("Observation sampling gap/overflow")
        if self.first_ns is None: self.first_ns=o['gateway_monotonic_ns']
        self.instance=frame['instance']
        row={k:v for k,v in frame.items() if k not in ('images','preview')}
        row['sources']={n:{k:v for k,v in s.items() if k not in ('jpeg','uint16_raw','preview_jpeg')} for n,s in frame['sources'].items()}
        for name in ('robot','rgb','depth'):
            seq=frame['sources'][name].get('seq')
            previous=self.previous_source_seq.get(name)
            if seq is not None:
                if previous is not None:self.sampled_source_skipped[name]+=max(0,seq-previous-1)
                self.previous_source_seq[name]=seq
        for name,blob_key,extension in (('rgb','jpeg','.jpg'),('depth','uint16_raw','.png')):
            source=frame['sources'][name]
            if not source.get('available'):continue
            if self.deferred:
                if not source.get('original_sha256'):continue
            elif not source.get(blob_key):continue
            source_id=source['device_frame_id']
            key=(name,source_id)
            repeated=key in self.ids
            if repeated:
                filename,digest,raw_digest=self.ids[key];self.repeated[name]+=1
            else:
                filename=f'images/{name}_{source_id:010d}{extension}'
                if self.deferred:
                    digest=source['original_sha256'] if name=='rgb' else None
                    raw_digest=source['original_sha256'] if name=='depth' else None
                    self.specs[key]=dict(source=name,id=source_id,sha256=source['original_sha256'],
                                         bytes=source['original_bytes'],width=source['width'],height=source['height'],file=filename)
                    self.ids[key]=(filename,digest,raw_digest)
                    row['sources'][name].update(file=filename,sha256=digest,repeated=False,media_pending=True)
                    if raw_digest:row['sources'][name]['raw_uint16_sha256']=raw_digest
                    if not row['associations'][name+'_robot']['valid']:self.invalid_associations[name]+=1
                    continue
                data=base64.b64decode(source[blob_key],validate=True)
                raw_digest=None
                if name=='rgb':
                    with Image.open(BytesIO(data)) as im:
                        im.load()
                        if im.size!=(source['width'],source['height']):raise ValueError('RGB dimensions mismatch')
                else:
                    if len(data)!=source['width']*source['height']*2:raise ValueError('Expected raw little-endian uint16 depth')
                    raw_digest=hashlib.sha256(data).hexdigest()
                    buf=BytesIO();Image.frombytes('I;16',(source['width'],source['height']),data).save(buf,'PNG',compress_level=1)
                    data=buf.getvalue()
                filename=f'images/{name}_{source_id:010d}{extension}'
                (self.directory/filename).write_bytes(data)
                digest=hashlib.sha256(data).hexdigest();self.ids[key]=(filename,digest,raw_digest)
            row['sources'][name].update(file=filename,sha256=digest,repeated=repeated)
            if self.deferred:row['sources'][name]['media_pending']=True
            if raw_digest:row['sources'][name]['raw_uint16_sha256']=raw_digest
            if not row['associations'][name+'_robot']['valid']:self.invalid_associations[name]+=1
        row.update(sample_index=self.samples,pc_recorded_monotonic_ns=time.perf_counter_ns(),
                   image=row['sources']['rgb']['file'],elapsed_s=(o['gateway_monotonic_ns']-self.first_ns)/1e9)
        self.stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        self.samples+=1;self.last_id=o['frame_id']
        if expected==3:self.last_control_tick=frame['observation']['control_tick_id']

    def retain_failure(self,frame,reason):
        """Diagnostic evidence is separate from accepted training samples."""
        if not self.active or not isinstance(frame,dict):return
        row={k:v for k,v in frame.items() if k not in ('images','preview')}
        row['sources']={name:{k:v for k,v in source.items() if k not in ('jpeg','uint16_raw','preview_jpeg')} for name,source in frame.get('sources',{}).items()}
        snapshot=dict(schema_version=frame.get('schema_version'),reason=str(reason),accepted_sample=False,
                      pc_failure_recorded_monotonic_ns=time.perf_counter_ns(),frame=row)
        target=self.directory/'failure_frame.json'
        # Preserve the first cause and exact command uncertainty. Later Stop
        # observations cannot replace the triggering frame.
        if not target.exists():
            with target.open('x',encoding='utf-8') as stream:json.dump(snapshot,stream,ensure_ascii=False,allow_nan=False)

    def finish(self, result, reason=None):
        allowed=('observed','aborted') if self.dataset_kind=='hardware_observation' else ('success','failure','aborted')
        if result not in allowed:raise ValueError('Invalid hardware recording result')
        if result=='aborted':self.incomplete=True
        name=super().finish('aborted' if result=='aborted' else 'success',reason)
        path=self.directory/'summary.json';summary=json.loads(path.read_text(encoding='utf-8'))
        summary.update(result=result,dataset_kind=self.dataset_kind,repeated_images=self.repeated,
                       invalid_associations=self.invalid_associations,action_labels_available=self.accepted_actions>0,accepted_action_samples=self.accepted_actions,is_real_demonstration=self.dataset_kind=='hardware_demonstration' and self.control_stage=='live',control_stage=self.control_stage,export_allowed=False)
        summary['sampled_source_skipped']=self.sampled_source_skipped
        if self.deferred:
            summary.update(complete=False,media_status='pending',expected_media=len(self.specs),verified_media=0)
            self.pending=True;self.phase='transferring'
        else:self.phase='complete' if summary['complete'] else 'aborted'
        path.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        return name

    def import_media(self, spec, data, x5_entry):
        """Validate native bytes before writing; preview pixels never enter raw."""
        if len(data)!=spec['bytes'] or hashlib.sha256(data).hexdigest()!=spec['sha256']:
            raise ValueError('Native media length/checksum mismatch')
        if x5_entry['sha256']!=spec['sha256'] or x5_entry['id']!=spec['id'] or x5_entry['source']!=spec['source']:
            raise ValueError('Archive media identity mismatch')
        received=time.perf_counter_ns()
        if spec['source']=='rgb':
            with Image.open(BytesIO(data)) as im:
                im.load()
                if im.size!=(spec['width'],spec['height']):raise ValueError('Native RGB dimensions mismatch')
        else:
            if len(data)!=spec['width']*spec['height']*2:raise ValueError('Native uint16 dimensions mismatch')
            buf=BytesIO();Image.frombytes('I;16',(spec['width'],spec['height']),data).save(buf,'PNG',compress_level=1)
            data=buf.getvalue()
        path=self.directory/spec['file'];temp=path.with_suffix(path.suffix+'.part')
        temp.write_bytes(data);temp.replace(path)
        entry=dict(sha256=hashlib.sha256(data).hexdigest(),pc_media_received_monotonic_ns=received,
                   pc_media_written_monotonic_ns=time.perf_counter_ns(),x5_media_written_monotonic_ns=x5_entry['x5_written_monotonic_ns'])
        self.verified[(spec['source'],spec['id'])]=entry
        with (self.directory/'media_transfer.jsonl').open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(dict(source=spec['source'],id=spec['id'],**entry))+'\n')

    def finalize_media(self, error=None):
        """Only complete after every referenced native file has passed validation."""
        if not self.pending:return
        self.transfer_error=error
        temp=self.directory/'samples.verified.jsonl'
        with (self.directory/'samples.jsonl').open(encoding='utf-8') as src,temp.open('w',encoding='utf-8') as dst:
            for line in src:
                row=json.loads(line)
                for name in ('rgb','depth'):
                    source=row['sources'][name];key=(name,source.get('device_frame_id'))
                    if key in self.verified:source.update(self.verified[key],media_pending=False)
                dst.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        temp.replace(self.directory/'samples.jsonl')
        path=self.directory/'summary.json';summary=json.loads(path.read_text(encoding='utf-8'))
        ok=not error and len(self.verified)==len(self.specs)
        summary.update(complete=ok and not self.incomplete and self.samples>0,
                       verified_media=len(self.verified),media_status='verified' if ok else 'failed')
        if not ok:summary.update(result='aborted',reason=error or 'Native media missing')
        path.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        self.pending=False;self.phase='complete' if summary['complete'] else 'aborted'

    def state(self):
        return dict(super().state(),phase=self.phase,pending=self.pending,
                    expected_media=len(getattr(self,'specs',{})),verified_media=len(getattr(self,'verified',{})),
                    transfer_error=self.transfer_error)
