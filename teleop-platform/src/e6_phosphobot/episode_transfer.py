"""Verified PC transfer and browsing of immutable native episode bundles."""
import bisect
from copy import deepcopy
import hashlib
import io
import json
import logging
from pathlib import Path,PurePosixPath
import threading
import time
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import zipfile
import zstandard
from PIL import Image
from .camera_archive import atomic_json,read_atomic_json
from .episodes import checked_id
logger=logging.getLogger('e6.pc.transfer')


def digest_file(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def safe_name(name):
    path=PurePosixPath(name)
    if path.is_absolute() or not path.parts or any(p in ('..','.') for p in path.parts) or '\\' in name or ':' in name:
        raise ValueError('Unsafe bundle path')
    return path


def rows(path):
    if not path.is_file():return []
    with path.open(encoding='utf-8') as stream:return [json.loads(line) for line in stream if line.strip()]


def verify_bundle(bundle,directory,expected,*,read_only=False):
    if digest_file(bundle)!=expected:raise ValueError('Bundle SHA256 mismatch')
    with zipfile.ZipFile(bundle) as archive:
        names=archive.namelist()
        if len(names)!=len(set(names)):raise ValueError('Duplicate ZIP entry')
        if sum(info.file_size for info in archive.infolist())>5*1024**3:raise ValueError('Bundle exceeds local budget')
        for name in names:safe_name(name)
        manifest=json.loads(archive.read('bundle_manifest.json'))
        entries={row['file']:row for row in manifest}
        if len(entries)!=len(manifest) or set(entries)!=(set(names)-{'bundle_manifest.json'}):
            raise ValueError('Bundle manifest coverage mismatch')
        for name,item in entries.items():
            target=directory.joinpath(*safe_name(name).parts)
            # Retries compare immutable bytes rather than overwrite earlier evidence.
            if target.exists():
                if target.stat().st_size!=item['bytes'] or digest_file(target)!=item['sha256']:
                    raise ValueError('Existing retained file differs: '+name)
                continue
            if read_only:raise ValueError('Missing retained file: '+name)
            target.parent.mkdir(parents=True,exist_ok=True)
            sha=hashlib.sha256();size=0
            partial=target.with_name(target.name+'.extract.part')
            if partial.exists():partial.rename(target.with_name(target.name+'.failed-'+str(time.time_ns())))
            with archive.open(name) as source,partial.open('xb') as dest:
                for chunk in iter(lambda:source.read(1024*1024),b''):
                    sha.update(chunk);size+=len(chunk);dest.write(chunk)
            if size!=item['bytes'] or sha.hexdigest()!=item['sha256']:raise ValueError('Entry hash mismatch: '+name)
            partial.rename(target)
    media=rows(directory/'media/manifest.jsonl')
    for item in media:
        stored=directory/'media'/str(safe_name(item['file']))
        if not 0<item['bytes']<=20*1024**2:raise ValueError('Invalid native image size')
        data=zstandard.ZstdDecompressor().decompress(stored.read_bytes(),max_output_size=item['bytes'])
        if len(data)!=item['bytes'] or hashlib.sha256(data).hexdigest()!=item['sha256']:
            raise ValueError('Native media SHA mismatch')
        meta=item['metadata']['metadata']
        if item['source']=='rgb':
            with Image.open(io.BytesIO(data)) as image:
                image.load()
                if image.size!=(meta['width'],meta['height']):raise ValueError('RGB dimensions mismatch')
        elif item['source']=='depth':
            if len(data)!=meta['width']*meta['height']*2:raise ValueError('uint16 depth length mismatch')
        else:raise ValueError('Unknown native source')
    association=associate(directory,media,read_only=read_only)
    return dict(bundle_sha256=expected,files=len(entries),native_files=len(media),**association,
                pc_verified_ns=time.perf_counter_ns(),exposure_synchronized=False,export_allowed=False)


def association_policy(directory):
    metadata_path=directory/'metadata.json'
    metadata=json.loads(metadata_path.read_text(encoding='utf-8')) if metadata_path.exists() else {}
    policy=metadata.get('action_association_policy','sample_before_action_v1')
    if policy not in ('sample_before_action_v1','sdk_available_v2'):
        raise ValueError('Unknown action association policy: '+str(policy))
    return policy


class AvailableCameraFrames:
    """Index retained frames by original SDK availability, not later disk writes.

    Shared by immutable action verification and derived cycle browsing. Kernel
    receipt alone cannot establish availability when the SDK buffered a frame.
    """
    def __init__(self,media):
        self.entries={name:[] for name in ('rgb','depth')}
        for item in media:
            src=item.get('metadata',{});meta=src.get('metadata',{})
            when=meta.get('host_perf_counter_ns');received=src.get('x5_receive_monotonic_ns')
            if when is None or received is None:continue
            if any(type(n) is not int or n<=0 for n in (when,received)) or received>when:
                raise ValueError('Invalid camera source receipt/SDK availability')
            if src.get('device_frame_id')!=item['id']:
                raise ValueError('Camera manifest/source identity differs')
            self.entries[item['source']].append((when,item['id'],src))
        for entries in self.entries.values():entries.sort(key=lambda r:(r[0],r[1]))
        self.times={name:[r[0] for r in entries] for name,entries in self.entries.items()}

    def select(self,name,cutoff):
        index=bisect.bisect_right(self.times[name],cutoff)-1
        if index<0:return None,{}
        when,_,src=self.entries[name][index]
        return when,src


def associate(directory,media,*,read_only=False):
    """Select causal observations; keep legacy episode policy reproducible."""
    policy=association_policy(directory)
    events=rows(directory/'audit/events.jsonl')
    raw=sorted(rows(directory/'audit/raw.jsonl'),key=lambda r:r['received_ns'])
    raw_times=[r['received_ns'] for r in raw]
    identities={(r['generation'],r['sequence']):r for r in raw}
    samples=sorted(rows(directory/'media/camera_samples.jsonl'),key=lambda r:r['x5_sample_ns'])
    sample_times=[r['x5_sample_ns'] for r in samples]
    files={(r['source'],r['id']):r for r in media}
    available=AvailableCameraFrames(media) if policy=='sdk_available_v2' else None
    commands=[];previous={};invalid=0
    for event in events:
        kind=event.get('event');receipt=event.get('receipt',{})
        if kind=='motion_submitted':
            cutoff=receipt.get('sent_ns');obs=event.get('pre_action_observation')
        elif kind=='can' and event['data'].get('kind')=='tx':
            cutoff=event['data']['host_perf_counter_ns'];obs=None
        else:continue
        if cutoff is None:raise ValueError('Action has no original send timestamp')
        if obs:
            arm=identities.get((obs['generation'],obs['sequence']))
            if arm and arm['received_ns']!=obs['received_ns']:raise ValueError('Action/raw observation identity differs')
        else:
            index=bisect.bisect_right(raw_times,cutoff)-1
            arm=raw[index] if index>=0 else None
        arm_valid=arm is not None and 0<=cutoff-arm['received_ns']<=100_000_000
        index=bisect.bisect_right(sample_times,cutoff)-1
        selected=samples[index] if index>=0 else None
        sources={}
        for name in ('rgb','depth'):
            src=selected['sources'].get(name,{}) if selected else {}
            sdk_available=None
            if available is not None:
                sdk_available,src=available.select(name,cutoff)
            fid=src.get('device_frame_id');received=src.get('x5_receive_monotonic_ns')
            valid=(received is not None and (name,fid) in files and 0<=cutoff-received<=100_000_000)
            sources[name]=dict(frame_id=fid,received_ns=received,valid=valid,reused=fid is not None and previous.get(name)==fid,
                               age_ms=None if received is None else (cutoff-received)/1e6)
            if policy=='sdk_available_v2':
                sources[name].update(sdk_available_ns=sdk_available,
                    capture_epoch=src.get('metadata',{}).get('capture_epoch'),
                    selection_basis=policy)
            previous[name]=fid
        valid=arm_valid and sources['rgb']['valid']
        invalid+=not valid
        commands.append(dict(kind=kind,action_sent_ns=cutoff,action_event_ns=event.get('at_ns'),
            feedback=None if arm is None else {k:arm[k] for k in ('generation','sequence','received_ns')},
            feedback_valid=arm_valid,media=sources,valid=valid,
            feedback_rgb_delta_ms=None if arm is None or sources['rgb']['received_ns'] is None else
                (arm['received_ns']-sources['rgb']['received_ns'])/1e6))
    content=''.join(json.dumps(row)+'\n' for row in commands)
    target=directory/'associations.jsonl'
    if target.exists():
        if target.read_text(encoding='utf-8')!=content:
            raise ValueError('Existing retained action associations differ')
    elif read_only:raise ValueError('Missing retained action associations')
    else:target.write_text(content,encoding='utf-8')
    freshness={name:sum(not r['sources'].get(name,{}).get('fresh',False) for r in samples) for name in ('rgb','depth')}
    return dict(association_policy=policy,actions=len(commands),invalid_action_associations=int(invalid),raw_frames=len(raw),
                invalid_camera_sample_associations=freshness,
                camera_samples=len(samples),cycle_samples=sum(e.get('event')=='cycle' for e in events))


class TransferWorker:
    def __init__(self,url,root):
        self.url=url.rstrip('/');self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True)
        self.halt=threading.Event();self.attempted=set();self.error=None;self.current=None
        self.thread=None;self.catalogue_errors=[]
        self.status_lock=threading.Lock();self.statuses={}
    def snapshot(self):
        with self.status_lock:
            return dict(episodes=deepcopy(list(self.statuses.values())),transferring=self.current,error=self.error,
                        catalogue_errors=deepcopy(self.catalogue_errors))
    def publish_progress(self,status,stage,received,total):
        # Progress is advisory and memory-only: do not add per-block disk IO or
        # report verification before the immutable bundle and receipt succeed.
        status['progress']=dict(stage=stage,received_bytes=received,total_bytes=total)
        with self.status_lock:self.statuses[status['episode']]=deepcopy(status)
    @staticmethod
    def failure_text(exc):
        if isinstance(exc,HTTPError):
            try:
                with exc:body=json.loads(exc.read(4096))
                detail=body.get('error') if isinstance(body,dict) else None
                if isinstance(detail,str):return f'HTTP {exc.code}: {detail[:2000]}'
            except Exception:pass  # A broken error body must not hide the HTTP failure.
        return str(exc)
    def get(self,path):
        with urlopen(self.url+path,timeout=3) as response:return json.load(response)
    def one(self,item):
        episode=checked_id(item['episode']);digest=item['bundle_sha256'];directory=self.root/episode
        logger.info('Receiving episode %s',episode)
        status=dict(episode=episode,phase='transferring',pc_receive_started_ns=time.perf_counter_ns(),
                    bundle_sha256=digest,status_saved=False,attempt_errors=[])
        with self.status_lock:
            previous=deepcopy(self.statuses.get(episode))
            self.current=episode;self.statuses[episode]=dict(status)
        step='local_status'
        try:
            directory.mkdir(exist_ok=True)
            saved=directory/'pc_transfer.json'
            if previous is None and saved.exists():previous=read_atomic_json(saved)
            if previous:
                status['attempt_errors']=list(previous.get('attempt_errors',[]))
                if previous.get('error') and not status['attempt_errors']:
                    status['attempt_errors'].append({k:previous.get(k) for k in ('pc_receive_started_ns','failed_step','error')})
            atomic_json(saved,dict(status,status_saved=True));status['status_saved']=True
            step='download'
            bundle=directory/'bundle.zip';part=directory/'bundle.zip.part'
            if not bundle.exists():
                # Failed transfers remain identifiable; an explicit retry preserves them.
                if part.exists():part.rename(directory/('bundle.failed-'+str(time.time_ns())+'.part'))
                with urlopen(self.url+'/episodes/'+episode+'/bundle',timeout=10) as src,part.open('xb') as dst:
                    if src.headers.get('X-Bundle-SHA256')!=digest:raise ValueError('Server bundle changed')
                    length=src.headers.get('Content-Length')
                    total=int(length) if length is not None else None
                    received=0;published=time.monotonic()
                    self.publish_progress(status,'receiving',received,total)
                    for block in iter(lambda:src.read(1024*1024),b''):
                        if self.halt.is_set():raise RuntimeError('PC transfer interrupted')
                        dst.write(block);received+=len(block)
                        now=time.monotonic()
                        if now-published>=.5:
                            self.publish_progress(status,'receiving',received,total);published=now
                self.publish_progress(status,'checksum',received,total)
                if digest_file(part)!=digest:raise ValueError('Transferred ZIP checksum mismatch')
                part.rename(bundle)
            status.update(pc_received_ns=time.perf_counter_ns())
            step='verification'
            size=bundle.stat().st_size
            self.publish_progress(status,'verification',size,size)
            report=verify_bundle(bundle,directory,digest)
            atomic_json(directory/'verification.json',report)
            payload=json.dumps(dict(episode=episode,bundle_sha256=digest)).encode()
            step='receipt'
            self.publish_progress(status,'receipt',size,size)
            with urlopen(Request(self.url+'/verified',data=payload,headers={'Content-Type':'application/json'}),timeout=3) as response:
                json.load(response)
            status.update(phase='verified',pc_written_ns=time.perf_counter_ns(),verification=report,remote_verified=True)
            logger.info('Episode %s verified; %s bytes; remote receipt confirmed',episode,size)
            status['progress']['stage']='complete'
            step='final_status'
            atomic_json(saved,dict(status,status_saved=True))
        except Exception as exc:
            logger.exception('Episode %s transfer failed during %s',episode,step)
            status.update(phase='failed',error=self.failure_text(exc),failed_step=step,status_saved=False)
            status['attempt_errors'].append({k:status[k] for k in ('pc_receive_started_ns','failed_step','error')})
            # If storage is unavailable, retain the failure in memory for the
            # formal page and the next explicit retry. Never strand activity
            # cleanup behind a second failing status write.
            try:
                atomic_json(directory/'pc_transfer.json',dict(status,status_saved=True))
                status['status_saved']=True
            except Exception as write_error:status['status_write_error']=self.failure_text(write_error)
        finally:
            with self.status_lock:
                self.statuses[episode]=deepcopy(status)
                self.error=status.get('error');self.current=None
        return status
    def retry(self,episode):
        checked_id(episode);self.attempted.discard(episode)
    def _run(self):
        while not self.halt.is_set():
            try:
                listing=self.get('/episodes')
                with self.status_lock:self.catalogue_errors=listing.get('catalogue_errors',[])
                for item in listing['episodes']:
                    if self.halt.is_set():break
                    if item['bundle_sha256'] and item['episode'] not in self.attempted:
                        local=self.root/item['episode']/'pc_transfer.json'
                        self.attempted.add(item['episode'])
                        if local.exists():
                            saved=read_atomic_json(local)
                            if saved['phase']=='verified' and saved.get('bundle_sha256')==item['bundle_sha256']:continue
                        self.one(item)
            except Exception as exc:self.error=str(exc)
            self.halt.wait(1)
    def start(self):
        self.thread=threading.Thread(target=self._run,name='episode-pc-transfer',daemon=True);self.thread.start()
    def close(self):
        self.halt.set()
        if self.thread:self.thread.join(15)
        if self.thread and self.thread.is_alive():raise RuntimeError('PC transfer worker did not exit')
