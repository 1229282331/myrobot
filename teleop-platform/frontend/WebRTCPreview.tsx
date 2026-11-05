import {Button} from '@/components/ui/button';
import {memo,useEffect,useRef,useState} from 'react';
import {MediaMTXWebRTCReader} from './vendor/mediamtx-reader.js';
import jsQR from 'jsqr';
import {clockBound,frameLatency} from './videoTiming';
import type {ClockBound,ClockSample} from './videoTiming';

// WHEP/signalling/reconnection remains the pinned official MediaMTX reader.
// This wrapper measures browser presentation, not source exposure time.
export const WebRTCPreview=memo(function WebRTCPreview({url}:{url:string}) {
  const video=useRef<HTMLVideoElement>(null);
  const [error,setError]=useState('Connecting video…');
  const [stats,setStats]=useState({fps:0,frames:0,dropped:0});
  const [stalled,setStalled]=useState(false);
  const [connection,setConnection]=useState(0);
  const [timing,setTiming]=useState<{samples:number;p95Lower:number;p95Upper:number}|null>(null);
  const [pixelMeasurement,setPixelMeasurement]=useState(true);
  const [reportFile,setReportFile]=useState<{url:string;name:string;sha256:string}|null>(null);
  const [reportSaving,setReportSaving]=useState(false);
  const [reportError,setReportError]=useState('');
  const pixelMeasurementRef=useRef(true);
  const measurement=useRef({schema:2,method:'X5 source receipt (basis/epoch in each QR) → browser expectedDisplayTime; OpenCV QR / jsQR; four-timestamp offset interval; source quantization 1 ms; relative drift budget 1000 ppm including clock exchange; not exposure synchronization or panel photon timing',
    segmentation:'video track or capture epoch; legacy connection field identifies a measurement segment',
    attempts:0,decodeFailures:0,unsynchronized:0,invalidBounds:0,truncated:0,
    receiverStats:[] as {atMs:number;connection:number;values:Record<string,number>;decoder:Record<string,string|boolean>}[],
    receiverAvailable:false,receiverChecks:0,receiverFailures:0,receiverError:'',receiverTypes:[] as string[],
    measurementChanges:[] as {atMs:number;enabled:boolean}[],
    segmentChanges:[] as {atMs:number;connection:number;reason:string;captureEpoch?:string|null}[],
    callbackCosts:[] as {atMs:number;qrMs:number;callbackDelayMs:number;decoderMs:number|null}[],
    connections:0,presentationWindows:[] as {connection:number;startMs:number;endMs:number;frames:number;fps:number;dropped:number}[],
    samples:[] as (NonNullable<ReturnType<typeof frameLatency>>&{connection:number})[]});
  useEffect(()=>{
    const element=video.current!;
    setStalled(false);setError('Connecting video…');setStats({fps:0,frames:0,dropped:0});
    setTiming(null);
    const measure=new URL(url).searchParams.get('measure')==='1';
    const endpoint=new URL(url);endpoint.pathname=endpoint.pathname.replace(/\/$/,'')+'/whep';endpoint.search='';
    let receiver:RTCRtpReceiver|undefined,alive=true,readingStats=false;
    let callback=0,frames=0,lastFrames=0,last=performance.now(),lastPresented=0;
    let frameBase:number|null=0,dropBase=0,track:MediaStreamTrack|undefined;
    let segment=++measurement.current.connections;
    let offset:ClockBound|null=null,lastMeasured=0,captureEpoch:string|null|undefined;
    const resetSegment=(reason:string,epoch?:string|null)=>{
      segment=++measurement.current.connections;
      captureEpoch=epoch;offset=null;lastPresented=0;lastMeasured=0;
      frames=0;lastFrames=0;frameBase=null;last=performance.now();
      dropBase=element.getVideoPlaybackQuality().droppedVideoFrames;
      setTiming(null);setStats({fps:0,frames:0,dropped:0});
      if(measurement.current.segmentChanges.length<1800)
        measurement.current.segmentChanges.push({atMs:last,connection:segment,reason,captureEpoch:epoch});
    };
    const reader=new MediaMTXWebRTCReader({url:endpoint.toString(),onError:setError,
      onTrack:event=>{
        if(!alive||event.track.kind!=='video')return;
        // The official reader reconnects without remounting this React component.
        // A new track must not inherit the previous peer's timing or FPS.
        if(track&&track!==event.track){
          resetSegment('video_track_replaced');
          window.dispatchEvent(new CustomEvent('e6-video-health',{detail:{fresh:false}}));
        }
        track=event.track;receiver=event.receiver;
        measurement.current.receiverAvailable=!!receiver;
        element.srcObject=event.streams[0];setError('');}});
    // Standard read-only WebRTC counters distinguish network jitter, decoder
    // load and display scheduling. No vendor reader patch or extra transport.
    const diagnosticTimer=measure?setInterval(async()=>{
      if(!receiver||readingStats||!alive)return;
      readingStats=true;
      measurement.current.receiverChecks++;
      const requestedSegment=segment,requestedReceiver=receiver;
      try {
        const report=await receiver.getStats();
        if(!alive||requestedSegment!==segment||requestedReceiver!==receiver)return;
        report.forEach(item=>{
          if(!measurement.current.receiverTypes.includes(item.type))measurement.current.receiverTypes.push(item.type);
          if(item.type!=='inbound-rtp'||(item.kind!=='video'&&item.mediaType!=='video'))return;
          const values:Record<string,number>={};
          for(const key of ['framesDecoded','framesReceived','framesDropped','packetsReceived','packetsLost',
            'bytesReceived','jitter','jitterBufferDelay','jitterBufferEmittedCount','totalDecodeTime',
            'totalProcessingDelay','totalInterFrameDelay','freezeCount','totalFreezesDuration','keyFramesDecoded','nackCount','pliCount']) {
            if(typeof item[key]==='number'&&Number.isFinite(item[key]))values[key]=item[key];
          }
          const decoder:Record<string,string|boolean>={};
          if(typeof item.decoderImplementation==='string')decoder.implementation=item.decoderImplementation;
          if(typeof item.powerEfficientDecoder==='boolean')decoder.powerEfficient=item.powerEfficientDecoder;
          const codec=report.get(item.codecId);
          if(typeof codec?.mimeType==='string')decoder.codec=codec.mimeType;
          if(typeof codec?.sdpFmtpLine==='string')decoder.format=codec.sdpFmtpLine;
          if(measurement.current.receiverStats.length<1800)
            measurement.current.receiverStats.push({atMs:performance.now(),connection:segment,values,decoder});
        });
      } catch(error) {measurement.current.receiverFailures++;measurement.current.receiverError=String(error);}
      finally {readingStats=false;}
    },1000):null;
    const canvas=document.createElement('canvas');canvas.width=132;canvas.height=132;
    const pixels=canvas.getContext('2d',{willReadFrequently:true})!;
    const clock=(event:Event)=>{offset=clockBound((event as CustomEvent<ClockSample>).detail);};
    window.addEventListener('e6-clock-sample',clock);
    const paused=()=>{
      setStalled(true);setError('预览已暂停，点击Resume video');
      window.dispatchEvent(new CustomEvent('e6-video-health',{detail:{fresh:false}}));
    };
    element.addEventListener('pause',paused);
    const percentile=(values:number[])=>values.sort((a,b)=>a-b)[Math.min(values.length-1,Math.ceil(values.length*.95)-1)];
    const freshness=setInterval(()=>{
      const fresh=!element.paused&&lastPresented>0&&performance.now()-lastPresented<=500;
      if(lastPresented>0&&!fresh)setStalled(true);
      // Repeat current health so a control page mounted after the first video
      // frame still receives it. This is display health, not exposure age.
      window.dispatchEvent(new CustomEvent('e6-video-health',{detail:{fresh}}));
    },100);
    const presented=(now:number,metadata:VideoFrameCallbackMetadata)=>{
      lastPresented=now;
      setStalled(false);setError('');
      if(frameBase===null)frameBase=metadata.presentedFrames-1;
      frames=metadata.presentedFrames-frameBase;
      if(measure&&pixelMeasurementRef.current&&now-lastMeasured>=180&&element.videoWidth===640){
        lastMeasured=now;
        const qrStarted=performance.now();
        const report=measurement.current;report.attempts++;
        pixels.drawImage(element,508,0,132,132,0,0,132,132);
        const rgba=pixels.getImageData(0,0,132,132);
        const qr=jsQR(rgba.data,132,132,{inversionAttempts:'dontInvert'});
        if(!qr)report.decodeFailures++;
        else {
          const sample=frameLatency(qr.data,metadata.expectedDisplayTime,offset);
          if(!sample)report.unsynchronized++;
          else if(captureEpoch!==undefined&&captureEpoch!==sample.captureEpoch){
            // A publisher can restart within the same WebRTC track. Keep its
            // old samples, but require a new clock exchange before timing the
            // new epoch, which may also have a different monotonic clock.
            resetSegment('capture_epoch_changed',sample.captureEpoch);
            report.unsynchronized++;
          }
          else if(sample.upperMs<0){report.invalidBounds++;}
          else if(report.samples.length<9000){captureEpoch=sample.captureEpoch;
            report.samples.push({...sample,connection:segment});}
          else report.truncated++;
        }
        if(report.callbackCosts.length<9000)report.callbackCosts.push({atMs:now,
          qrMs:performance.now()-qrStarted,callbackDelayMs:now-metadata.expectedDisplayTime,
          decoderMs:metadata.processingDuration===undefined?null:metadata.processingDuration*1000});
      }
      if(now-last>=2000) {
        const current={fps:(frames-lastFrames)*1000/(now-last),frames,
                  dropped:Math.max(0,element.getVideoPlaybackQuality().droppedVideoFrames-dropBase)};
        setStats(current);
        if(measure&&measurement.current.presentationWindows.length<1800)
          measurement.current.presentationWindows.push({connection:segment,startMs:last,endMs:now,
            frames:frames-lastFrames,fps:current.fps,dropped:current.dropped});
        last=now;lastFrames=frames;
        const samples=measurement.current.samples.filter(sample=>sample.connection===segment);
        if(measure&&samples.length){
          const summary={samples:samples.length,p95Lower:percentile(samples.map(s=>s.lowerMs)),
            p95Upper:percentile(samples.map(s=>s.upperMs)),connection:segment};
          setTiming(summary);
        }
      }
      callback=element.requestVideoFrameCallback(presented);
    };
    callback=element.requestVideoFrameCallback(presented);
    return()=>{alive=false;if(diagnosticTimer!==null)clearInterval(diagnosticTimer);
      clearInterval(freshness);window.removeEventListener('e6-clock-sample',clock);
      element.removeEventListener('pause',paused);
      element.cancelVideoFrameCallback(callback);reader.close();element.srcObject=null;
      window.dispatchEvent(new CustomEvent('e6-video-health',{detail:{fresh:false}}));};
  },[url,connection]);
  const recover=async()=>{
    const element=video.current;
    if(element?.paused&&element.readyState>=2){
      try {await element.play();} catch {setError('Playback unavailable. Retry video.');}
    } else setConnection(n=>n+1); // Reuse the official reader's connection lifecycle.
    // Playback success alone is not a fresh frame and never prepares control.
  };
  return <div className="space-y-2">
    <video ref={video} autoPlay playsInline muted className="w-full aspect-video"/>
    <div className="flex flex-wrap items-center justify-between gap-2">
    <p data-testid="e6-video-stats" className="text-xs">
      {stalled?(error||'Video paused · control paused'):error||`Display ${stats.fps.toFixed(1)} fps · ${stats.frames} frames · dropped ${stats.dropped}`}
    </p>
    {stalled?<Button variant="outline" size="sm" type="button" onClick={recover}>Resume video</Button>:
      <Button variant="outline" size="sm" type="button" onClick={()=>video.current?.pause()}>Pause video & control</Button>}
    </div>
    {timing&&<details className="text-xs">
      <summary>视频测量：{timing.samples}样本 · P95 {timing.p95Lower.toFixed(0)}–{timing.p95Upper.toFixed(0)} ms</summary>
      <p>源接收至浏览器呈现估计；每帧记录UVC内核或SDK接收起点。区间包含往返测时误差，非曝光同步。</p>
      <label><input type="checkbox" checked={pixelMeasurement} onChange={event=>{
        const enabled=event.target.checked;pixelMeasurementRef.current=enabled;setPixelMeasurement(enabled);
        measurement.current.measurementChanges.push({atMs:performance.now(),enabled});
      }}/>解码像素测时标记（取消仅暂停测时，继续统计呈现帧率）</label>
      <p>报告包含全部连接、失败及原始时钟；点击后保存到本PC诊断目录，再提供下载链接。</p>
      <Button variant="outline" size="sm" disabled={reportSaving} onClick={async()=>{
        if(reportSaving)return;
        setReportSaving(true);setReportError('');
        try {
        const history=measurement.current;
        const samples=history.samples.filter(s=>s.connection===history.connections);
        const p95=(values:number[])=>values.sort((a,b)=>a-b)[Math.ceil(values.length*.95)-1];
        const summary={samples:samples.length,p95Lower:p95(samples.map(s=>s.lowerMs)),
          p95Upper:p95(samples.map(s=>s.upperMs)),connection:history.connections};
        const reportText=JSON.stringify({...history,summary});
        const response=await fetch('/e6/diagnostics/video',{method:'POST',headers:{'Content-Type':'application/json'},body:reportText});
        if(!response.ok)throw new Error(`保存失败（HTTP ${response.status}），可稍后重试`);
        const saved=await response.json();
        if(!/^video-[0-9a-f]{32}\.json$/.test(saved.name)||saved.url!==`/e6/diagnostics/video/${saved.name}`
          ||!/^[0-9a-f]{64}$/.test(saved.sha256))throw new Error('保存回执无效，请检查本机服务');
        setReportFile(saved);
        } catch(error) {setReportError(error instanceof Error?error.message:'报告未保存');}
        finally {setReportSaving(false);}
      }}>{reportSaving?'保存报告中…':'保存视频测量报告'}</Button>
      {reportError&&<p role="alert">{reportError}</p>}
      {reportFile&&<p><span>已保存到本PC：{reportFile.name} · </span>
        <a className="underline" href={reportFile.url} download={reportFile.name}>下载已保存报告</a>
        <span> · 生成时的完整快照；再次保存会创建新文件。</span></p>}
    </details>}
  </div>;
});
