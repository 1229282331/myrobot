import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import zipfile
import pytest
from e6_phosphobot import episode_bundle as module


def sources(tmp_path):
    directory=tmp_path/'episode';media=tmp_path/'camera/media/episode'
    directory.mkdir();media.mkdir(parents=True)
    (directory/'metadata.json').write_text('{"source":"test"}',encoding='utf8')
    (directory/'audit').mkdir();(directory/'audit/raw.jsonl').write_bytes(b'retained raw\n'*1000)
    (media/'depth.bin').write_bytes(bytes(range(256))*1000)
    (media/'rgb.jpg').write_bytes(b'encoded jpeg fixture'*100)
    return directory,media


def test_subprocess_bundle_retains_entries_compression_and_all_manifest_hashes(tmp_path):
    directory,media=sources(tmp_path)
    original={p:p.read_bytes() for base in (directory,media) for p in base.rglob('*') if p.is_file()}
    direct=module.bundle_files(directory,media)
    with zipfile.ZipFile(directory/'bundle.zip') as z:
        expected={name:(z.read(name),z.getinfo(name).compress_type) for name in z.namelist()}
    result=module.run_bundle(directory,media,shutdown_deadline=lambda:float('inf'))
    assert len(direct)==64
    assert result==hashlib.sha256((directory/'bundle.zip').read_bytes()).hexdigest()
    assert all(p.read_bytes()==data for p,data in original.items())
    with zipfile.ZipFile(directory/'bundle.zip') as z:
        # The generated manifest entry has the current ZIP timestamp, just as
        # the original implementation did. Source payloads/manifest are exact.
        assert {name:(z.read(name),z.getinfo(name).compress_type) for name in z.namelist()}==expected
        for row in json.loads(z.read('bundle_manifest.json')):
            data=z.read(row['file'])
            assert hashlib.sha256(data).hexdigest()==row['sha256'] and len(data)==row['bytes']
        assert z.getinfo('media/depth.bin').compress_type==zipfile.ZIP_STORED
        assert z.getinfo('audit/raw.jsonl').compress_type==zipfile.ZIP_DEFLATED


def test_existing_partial_bundle_is_preserved_and_failure_is_reported(tmp_path):
    directory,media=sources(tmp_path);partial=directory/'bundle.zip.part'
    partial.write_bytes(b'prior partial')
    with pytest.raises(RuntimeError,match='FileExistsError'):
        module.run_bundle(directory,media,shutdown_deadline=lambda:float('inf'))
    assert partial.read_bytes()==b'prior partial'
    assert not (directory/'bundle.zip').exists()
    assert (directory/'metadata.json').exists()


def test_expired_shutdown_does_not_launch_a_worker(tmp_path,monkeypatch):
    directory,media=sources(tmp_path)
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**k:pytest.fail('Must not spawn'))
    with pytest.raises(RuntimeError,match='退出期间'):
        module.run_bundle(directory,media,shutdown_deadline=lambda:0)


@pytest.mark.parametrize('failure',['deadline','crash','malformed'])
def test_worker_failure_or_stop_is_reaped_and_keeps_raw_and_partial_files(tmp_path,monkeypatch,failure):
    directory,media=sources(tmp_path);original=(directory/'metadata.json').read_bytes()
    worker=tmp_path/'substitute_worker.py';marker=tmp_path/'started'
    worker.write_text('import os,time\nfrom pathlib import Path\n'+
        f'Path({str(directory / "bundle.zip.part")!r}).write_bytes(b"partial")\n'+
        f'Path({str(marker)!r}).write_text(str(os.getpid()))\n'+
        ('time.sleep(30)\n' if failure=='deadline' else 'raise SystemExit(7)\n' if failure=='crash' else 'print("not-a-digest")\n'),encoding='utf8')
    popen=subprocess.Popen;children=[];release=threading.Event();errors=[];deadline=[float('inf')]
    def factory(command,**kwargs):
        child=popen([sys.executable,'-I','-S',str(worker)],**kwargs);children.append(child);return child
    monkeypatch.setattr(module.subprocess,'Popen',factory)
    def run():
        try:module.run_bundle(directory,media,shutdown_deadline=lambda:deadline[0])
        except BaseException as exc:errors.append(exc)
        finally:release.set()
    thread=threading.Thread(target=run);thread.start()
    end=time.monotonic()+5
    while not marker.exists() and time.monotonic()<end:time.sleep(.01)
    assert marker.exists()
    if failure=='deadline':deadline[0]=time.monotonic()-1
    assert release.wait(5);thread.join()
    assert len(errors)==1 and isinstance(errors[0],RuntimeError)
    assert children and all(p.poll() is not None and p.stdout.closed and p.stderr.closed for p in children)
    assert (directory/'bundle.zip.part').read_bytes()==b'partial'
    assert (directory/'metadata.json').read_bytes()==original
    assert not (directory/'bundle.zip').exists()
