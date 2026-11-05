"""Episode ZIP/SHA work in a separate interpreter, with no device imports.

The original file order, compression, manifest and hash semantics are retained.
The caller owns the subprocess and bounds shutdown; raw and partial files are
preserved on failure. No source file is changed by this worker.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time
import zipfile


def bundle_files(directory,media):
    directory=Path(directory);media=Path(media);entries=[]
    for prefix,folder in [('',directory),('media',media)]:
        if not folder.is_dir():continue
        for file in sorted(folder.rglob('*')):
            if not file.is_file() or file.name in ('bundle.zip','bundle.zip.part'):continue
            name=(Path(prefix)/file.relative_to(folder)).as_posix()
            entries.append((name,file))
    manifest=[];temp=directory/'bundle.zip.part'
    with zipfile.ZipFile(temp,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as bundle:
        for name,file in entries:
            digest=hashlib.sha256()
            with file.open('rb') as src:
                for chunk in iter(lambda:src.read(1024*1024),b''):digest.update(chunk)
            manifest.append(dict(file=name,bytes=file.stat().st_size,sha256=digest.hexdigest()))
            bundle.write(file,name,compress_type=zipfile.ZIP_STORED if name.endswith('.bin') else zipfile.ZIP_DEFLATED)
        bundle.writestr('bundle_manifest.json',json.dumps(manifest))
    target=directory/'bundle.zip';temp.replace(target)
    digest=hashlib.sha256()
    with target.open('rb') as src:
        for chunk in iter(lambda:src.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def run_bundle(directory,media,*,shutdown_deadline):
    # Absolute script entrypoint also works in an unpacked source deployment or
    # Windows pytest environment. The child needs only the standard library.
    if time.monotonic()>shutdown_deadline():
        raise RuntimeError('退出期间打包未完成；保留原始文件与部分包')
    command=[sys.executable,'-I','-S',str(Path(__file__).resolve()),'--directory',str(directory),'--media',str(media)]
    process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        while process.poll() is None:
            if time.monotonic()>shutdown_deadline():
                raise RuntimeError('退出期间打包未完成；保留原始文件与部分包')
            time.sleep(.02)
        stdout,stderr=process.communicate()
        if process.returncode:
            raise RuntimeError('封存子进程失败: '+stderr.decode('utf8',errors='replace').strip()[:2000])
        digest=stdout.decode('ascii').strip()
        if not re.fullmatch('[0-9a-f]{64}',digest):raise RuntimeError('封存子进程未返回有效校验值')
        return digest
    finally:
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=2)
        if process.stdout:process.stdout.close()
        if process.stderr:process.stderr.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--media',type=Path,required=True);args=p.parse_args()
    try:print(bundle_files(args.directory,args.media),flush=True)
    except Exception as exc:
        # Bounded error output cannot fill the pipe while the parent polls.
        sys.stderr.buffer.write((type(exc).__name__+': '+str(exc)).encode('utf8')[:2048])
        sys.stderr.buffer.flush()
        return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
