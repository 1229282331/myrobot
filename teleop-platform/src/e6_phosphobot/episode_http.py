"""Loopback media-only server; use a separate SSH connection from intents."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import threading
import time
from urllib.parse import urlsplit,parse_qs
from .episodes import checked_id


def copy_limited(source,destination,bytes_per_second,*,clock=time.monotonic,wait=time.sleep):
    """Pace bulk media locally before SSH; never accumulate catch-up credit."""
    if bytes_per_second<=0:raise ValueError('Positive media bandwidth required')
    deadline=clock()
    while True:
        block=source.read(16384)
        if not block:return
        delay=deadline-clock()
        if delay>0:wait(delay)
        destination.write(block)
        deadline=max(deadline,clock())+len(block)/bytes_per_second


def start_server(episodes,port,*,bytes_per_second=1024*1024,logs=None):
    episodes.root.mkdir(parents=True,exist_ok=True)
    transfer_lock=threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,code=200):
            data=json.dumps(value).encode()
            self.send_response(code);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def do_GET(self):
            try:
                url=urlsplit(self.path)
                log_parts=url.path.strip('/').split('/')
                if logs is not None and log_parts[0]=='logs':
                    if log_parts==['logs']:
                        self.reply(logs.listing());return
                    if len(log_parts)!=3 or log_parts[2] not in ('stream','download'):
                        raise ValueError('Unknown log path')
                    if log_parts[2]=='stream':
                        data,headers=logs.read(log_parts[1],parse_qs(url.query).get('cursor',[''])[0])
                        self.send_response(200);self.send_header('Content-Type','text/plain; charset=utf-8')
                        self.send_header('Content-Length',str(len(data)))
                        for name,value in headers.items():self.send_header(name,value)
                        self.end_headers();self.wfile.write(data);return
                    stream,size=logs.download(log_parts[1])
                    with stream:
                        self.send_response(200);self.send_header('Content-Type','text/plain; charset=utf-8')
                        self.send_header('Content-Length',str(size));self.send_header('Cache-Control','no-store')
                        self.end_headers()
                        # Snapshot size, not a tail that follows writes indefinitely.
                        remaining=size
                        while remaining:
                            block=stream.read(min(65536,remaining))
                            if not block:break
                            self.wfile.write(block);remaining-=len(block)
                    return
                parts=self.path.strip('/').split('/')
                if parts==['episodes']:
                    if hasattr(episodes,'listing'):
                        self.reply(episodes.listing());return
                    items=[]
                    for file in sorted(episodes.root.glob('*/transfer.json')):
                        item=json.loads(file.read_text(encoding='utf-8'))
                        item['summary']=json.loads((file.parent/'summary.json').read_text(encoding='utf-8'))
                        items.append(item)
                    self.reply(dict(current=episodes.state(),episodes=items));return
                if len(parts)!=3 or parts[0]!='episodes' or parts[2]!='bundle':raise ValueError('Unknown media path')
                directory=(episodes.directory(parts[1]) if hasattr(episodes,'listing')
                           else episodes.root/checked_id(parts[1]));path=directory/'bundle.zip'
                transfer=json.loads((directory/'transfer.json').read_text(encoding='utf-8'))
                if not transfer['bundle_sha256']:raise ValueError('Bundle is not sealed')
                if not transfer_lock.acquire(blocking=False):
                    self.reply(dict(error='A bulk media transfer is already active'),409);return
                try:
                    self.send_response(200);self.send_header('Content-Type','application/zip')
                    self.send_header('Content-Length',str(path.stat().st_size))
                    self.send_header('X-Media-Bytes-Per-Second',str(bytes_per_second))
                    self.send_header('X-Bundle-SHA256',transfer['bundle_sha256']);self.end_headers()
                    with path.open('rb') as stream:copy_limited(stream,self.wfile,bytes_per_second)
                finally:transfer_lock.release()
            except (BrokenPipeError,ConnectionResetError):return
            except (OSError,ValueError,KeyError) as exc:self.reply(dict(error=str(exc)),404)
        def do_POST(self):
            try:
                if self.headers.get('Origin'):raise ValueError('Use the local transfer worker')
                if self.path!='/verified':raise ValueError('Unknown media operation')
                length=int(self.headers.get('Content-Length','0'))
                if not 1<=length<=4096:raise ValueError('Invalid receipt size')
                data=json.loads(self.rfile.read(length))
                episode=checked_id(data['episode'])
                # A checksum receipt cannot initialize a control session.
                result=episodes.transferred(episode,data['bundle_sha256'])
                self.reply(result)
            except (ValueError,OSError,KeyError) as exc:self.reply(dict(error=str(exc)),409)
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.daemon_threads=True
    threading.Thread(target=server.serve_forever,name='episode-media-http',daemon=True).start()
    return server


def main():
    """Resume a stopped session's media transfer without opening any devices."""
    import argparse
    from pathlib import Path
    from types import SimpleNamespace
    import signal
    from .episodes import Episodes
    parser=argparse.ArgumentParser(description='Serve retained, sealed episodes only; no device connections')
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--root',type=Path)
    source.add_argument('--sessions-root',type=Path)
    parser.add_argument('--camera-dir',type=Path)
    parser.add_argument('--managed-stdin',action='store_true')
    parser.add_argument('--recovery-id')
    parser.add_argument('--episode')
    parser.add_argument('--port',type=int,default=8777)
    args=parser.parse_args()
    if args.sessions_root:
        from .retained_episodes import RetainedEpisodes
        episodes=RetainedEpisodes(args.sessions_root,recovery_id=args.recovery_id,episode=args.episode)
    else:
        if args.camera_dir is None:parser.error('--root requires --camera-dir')
        episodes=Episodes(args.root,args.camera_dir,SimpleNamespace(),'shadow')
    server=start_server(episodes,args.port);halt=threading.Event()
    if args.managed_stdin:
        import sys
        def parent():
            try:
                for line in sys.stdin:
                    if line.strip()=='stop':break
            finally:halt.set()
        threading.Thread(target=parent,name='media-parent',daemon=True).start()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:halt.set())
    try:halt.wait()
    finally:server.shutdown();server.server_close();episodes.close()


if __name__=='__main__':main()
