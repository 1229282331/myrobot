"""Separate Orbbec capture and FFmpeg preview process. No robot/CAN imports.

Original SDK metadata/UVC PTS are retained. Preview drops its own stale frames;
encoding or an absent viewer cannot block the acquisition health producer.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import glob
import json
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from .camera_outputs import CameraOutputs,atomic
from .camera_recovery import RecoveringCapture


def latency_marker_payload(meta):
    """Identify both the real timestamp basis and restart epoch in the pixels."""
    basis={'uvc_kernel_receipt':'E6K','sdk_wait_return':'E6S'}[meta['source_receipt_basis']]
    return f"{basis}:{meta['capture_epoch']}:{meta['index']}:{meta['source_received_perf_ns']//1000000}"


def render_preview(meta,data,decode_color,cv2,qr_encoder=None,marker_pool=None):
    # Only one frame and one QR job are in flight. The future belongs to this
    # exact metadata object; never reuse a cached marker on a newer image.
    marker=marker_pool.submit(qr_encoder.encode,latency_marker_payload(meta)) if marker_pool else None
    frame=decode_color(data,meta['width'],meta['height'],meta['format_id'])
    frame=cv2.resize(frame,(640,360),interpolation=cv2.INTER_AREA)
    stamp=meta.get('source_received_perf_ns',meta['host_perf_counter_ns'])
    cv2.putText(frame,f"{meta['index']} X5:{stamp//1000000}",
        (8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
    if marker:
        code=marker.result()
        code=cv2.copyMakeBorder(code,4,4,4,4,cv2.BORDER_CONSTANT,value=255)
        code=cv2.resize(code,(132,132),interpolation=cv2.INTER_NEAREST)
        frame[0:132,508:640]=cv2.cvtColor(code,cv2.COLOR_GRAY2BGR)
    return frame


def prepare_output(path, resume=False):
    if not resume:
        path.mkdir(parents=True,exist_ok=False)
    elif path.is_symlink() or not path.is_dir():
        raise ValueError('Camera restart requires its existing owned output directory')
    elif any((path/name).exists() for name in ('recording.request.json','final.json')):
        raise ValueError('Camera restart requires archived prior request and final status')


def preview_command(url, codec='h264'):
    """Startup-fixed FFmpeg/MediaMTX codec; acquisition/native archives unchanged."""
    common=['ffmpeg','-hide_banner','-loglevel','warning',
        '-f','rawvideo','-pix_fmt','bgr24','-s','640x360','-r','20','-i','pipe:0','-an']
    if codec=='h264':
        options=['-c:v','libx264','-threads','2','-preset','ultrafast','-tune','zerolatency',
            '-pix_fmt','yuv420p','-profile:v','baseline','-b:v','900k','-maxrate','900k',
            '-bufsize','90k','-g','20','-bf','0']
    elif codec=='vp8':
        options=['-c:v','libvpx','-threads','2','-deadline','realtime','-cpu-used','8',
            '-lag-in-frames','0','-auto-alt-ref','0','-error-resilient','1',
            '-pix_fmt','yuv420p','-b:v','900k','-maxrate','900k','-bufsize','90k','-g','20']
    else:raise ValueError('Unknown preview codec')
    return common+options+['-f','rtsp','-rtsp_transport','tcp',url]


def run(args):
    codec=getattr(args,'preview_codec','h264')
    encoder_command=preview_command(args.rtsp_url,codec)
    sys.path.insert(0,str(args.camera_tools))
    import cv2
    import numpy as np
    from camera import Camera,decode_color,decode_depth
    cv2.setNumThreads(1)
    prepare_output(args.output,getattr(args,'resume_output',False))
    from .operator_logs import start_logging
    logger,human_sink=start_logging(args.output/'human.log','e6.camera')
    logger.info('Camera starting; SN=AY4A1520111; RGB=1280x720; depth=640x480; requested=30 fps; UVC PTS enabled; preview=%s',codec)
    nodes=glob.glob('/dev/video*')
    if nodes and subprocess.run(['fuser',*nodes],capture_output=True).returncode==0:
        raise RuntimeError('Camera is already owned; refusing takeover')
    stopped=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.set())
    preview=queue.Queue(1)
    stats=dict(serial='AY4A1520111',rgb_received_ns=None,rgb_frames=0,depth_frames=0,
               preview_dropped=0,preview_written=0,encoder_error=None,started_ns=time.perf_counter_ns(),
               latency_marker=args.latency_marker,preview_codec=codec)
    encoder=None
    log=None
    if args.rtsp_url:
        log=(args.output/'ffmpeg.log').open('ab')
        encoder=subprocess.Popen(encoder_command,
            stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
    def encode():
        marker_pool=None
        try:
            qr_encoder=cv2.QRCodeEncoder_create() if args.latency_marker else None
            if qr_encoder:marker_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='preview-qr')
            while not stopped.is_set():
                try:meta,data=preview.get(timeout=.1)
                except queue.Empty:continue
                frame=render_preview(meta,data,decode_color,cv2,qr_encoder,marker_pool)
                encoder.stdin.write(frame.tobytes());encoder.stdin.flush()
                stats['preview_written']+=1
        except Exception as exc:
            logger.exception('Preview encoder stopped')
            key='encoder_shutdown_note' if stopped.is_set() else 'encoder_error'
            stats[key]=str(exc)
            stats[key+'_ns']=time.perf_counter_ns()
        finally:
            if marker_pool:marker_pool.shutdown(wait=True)
    worker=threading.Thread(target=encode,name='preview-encoder',daemon=True) if encoder else None
    camera=None
    from .camera_archive import CameraArchive
    archive=CameraArchive(args.output,status_path=args.health_file.with_name('recording.status.json') if args.health_file else None)
    archive.start()
    def depth_valid(meta,data):
        raw,_=decode_depth(data,meta['width'],meta['height'],meta['format_id'],meta['depth_scale_mm'])
        return float(np.mean(raw>0))
    outputs=CameraOutputs(args.output,args.health_file or args.output/'health.json',depth_valid=depth_valid,
        snapshot_root=args.health_file.parent/'snapshots' if args.health_file else None)
    outputs.start()
    try:
        camera=Camera('both',serial='AY4A1520111',profiles={
            'color':(1280,720,5,30),'depth':(640,480,12,30)},align_mode=0,uvc_pts=True)
        stats.update(identity=camera.info,profiles=camera.selected)
        logger.info('Camera connected; selected profiles: %s',camera.selected)
        def capture_event(event):
            # Sparse discontinuity evidence, not per-frame synchronous I/O.
            print(json.dumps(event),flush=True)
            logger.warning('Camera capture: %s',event)
        capture=RecoveringCapture(camera,report=capture_event)
        if worker:worker.start()
        next_preview=0
        while not stopped.is_set():
            frames=capture.read()
            stats.update(capture.stats)
            outputs.status(stats)
            for meta,data in frames:
                meta={k:v for k,v in meta.items() if k!='_original_bytes'}
                archive.offer(meta,data)
                now=time.perf_counter_ns()
                if meta['stream']=='color':
                    stats.update(rgb_received_ns=meta['source_received_perf_ns'],frame_id=meta['index'],rgb_metadata=meta)
                    stats['rgb_frames']+=1
                    if worker and now>=next_preview:
                        next_preview=max(next_preview+50_000_000,now)
                        if preview.full():
                            try:preview.get_nowait();stats['preview_dropped']+=1
                            except queue.Empty:pass
                        preview.put_nowait((meta,data))
                elif meta['stream']=='depth':
                    stats['depth_frames']+=1
                    stats.update(depth_received_ns=meta['source_received_perf_ns'],depth_frame_id=meta['index'],depth_metadata=meta)
                elapsed=(now-stats['started_ns'])/1e9
                stats.update(rgb_fps=stats['rgb_frames']/elapsed,depth_fps=stats['depth_frames']/elapsed)
                stats['recording']=archive.public_status
                outputs.offer(meta,data,stats)
    except Exception as exc:
        logger.exception('Camera capture failed')
        stats['error']=str(exc)
        raise
    finally:
        stopped.set()
        cleanup_errors=[]
        try:
            if camera:cleanup_errors.extend('camera: '+str(error) for error in camera.close())
        except Exception as exc:cleanup_errors.append('camera: '+str(exc))
        try:archive.close()
        except Exception as exc:cleanup_errors.append('archive: '+str(exc))
        if encoder:
            encoder.terminate()
            try:encoder.wait(timeout=3)
            except subprocess.TimeoutExpired:encoder.kill();encoder.wait()
            if worker.ident is not None:worker.join(2)
            encoder.stdin.close();log.close()
        stats['closed_ns']=time.perf_counter_ns()
        stats['cleanup_errors']=cleanup_errors
        cleanup_errors.extend(outputs.close(stats))
        atomic(args.output/'final.json',json.dumps(stats).encode())
        logger.info('Camera stopped; RGB=%.2f fps; depth=%.2f fps; cleanup=%s',stats.get('rgb_fps',0),stats.get('depth_fps',0),cleanup_errors)
        logger.removeHandler(human_sink);human_sink.close()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--camera-tools',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume-output',action='store_true',help='Resource supervisor only: reuse this session output after archiving prior state')
    p.add_argument('--health-file',type=Path,help='Session-scoped tmpfs health path; native archives remain in --output')
    p.add_argument('--rtsp-url')
    p.add_argument('--preview-codec',choices=('h264','vp8'),default='h264')
    p.add_argument('--latency-marker',action='store_true',help='QR timestamp only on diagnostic preview; native RGB unchanged')
    run(p.parse_args())


if __name__=='__main__':main()
