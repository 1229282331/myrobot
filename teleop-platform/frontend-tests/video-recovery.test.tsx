import React from 'react';
import {act,cleanup,fireEvent,render,screen} from '@testing-library/react';
import {afterEach,beforeEach,expect,it,vi} from 'vitest';
import {WebRTCPreview} from '../frontend/WebRTCPreview';
import jsQR from 'jsqr';

let readerConfig:any;
vi.mock('../frontend/vendor/mediamtx-reader.js',()=>({MediaMTXWebRTCReader:class {
 constructor(config:any){readerConfig=config;} close(){}
}}));
vi.mock('jsqr',()=>({default:vi.fn()}));
let presented:(now:number,metadata:any)=>void;
let play:ReturnType<typeof vi.fn>;
let health:boolean[];
let downloaded:string[];
async function downloadReport(){
 await act(async()=>fireEvent.click(screen.getByRole('button',{name:'保存视频测量报告'})));
 return JSON.parse(downloaded.at(-1)!);
}
const onHealth=(e:Event)=>health.push((e as CustomEvent).detail.fresh);
beforeEach(()=>{
 vi.useFakeTimers();health=[];downloaded=[];
 vi.stubGlobal('fetch',vi.fn(async(_url,options)=>{
  downloaded.push(options.body);
  return {ok:true,json:async()=>({url:'/e6/diagnostics/video/video-'+'a'.repeat(32)+'.json',
    name:'video-'+'a'.repeat(32)+'.json',sha256:'b'.repeat(64)})};
 }));
 URL.createObjectURL=vi.fn(()=> 'blob:test');URL.revokeObjectURL=vi.fn();
 vi.spyOn(HTMLAnchorElement.prototype,'click').mockImplementation(()=>{});
 vi.spyOn(HTMLCanvasElement.prototype,'getContext').mockReturnValue({} as any);
 play=vi.fn().mockResolvedValue(undefined);
 Object.defineProperty(HTMLMediaElement.prototype,'play',{configurable:true,value:play});
 Object.defineProperty(HTMLVideoElement.prototype,'requestVideoFrameCallback',{configurable:true,value:(f:any)=>{presented=f;return 1;}});
 Object.defineProperty(HTMLVideoElement.prototype,'cancelVideoFrameCallback',{configurable:true,value:vi.fn()});
 Object.defineProperty(HTMLVideoElement.prototype,'getVideoPlaybackQuality',{configurable:true,value:()=>({droppedVideoFrames:0})});
 window.addEventListener('e6-video-health',onHealth);
});
afterEach(()=>{cleanup();window.removeEventListener('e6-video-health',onHealth);vi.useRealTimers();vi.unstubAllGlobals();});
it('a paused preview reports stale and offers explicit playback recovery without claiming freshness',async()=>{
 const view=render(<WebRTCPreview url="http://127.0.0.1:8889/gemini"/>);
 const video=view.container.querySelector('video')!;
 Object.defineProperty(video,'readyState',{configurable:true,value:4});
 act(()=>fireEvent.pause(video));
 expect(health.at(-1)).toBe(false);
 await act(async()=>fireEvent.click(screen.getByRole('button',{name:'Resume video'})));
 expect(play).toHaveBeenCalledTimes(1);
 expect(health.at(-1)).toBe(false); // Only a new presented frame can restore health.
 view.unmount();
 act(()=>fireEvent.pause(video));
 expect(play).toHaveBeenCalledTimes(1);
});
it('a presentation stall replaces the old FPS display instead of showing a frozen 20 fps',()=>{
 const view=render(<WebRTCPreview url="http://127.0.0.1:8889/gemini"/>);
 Object.defineProperty(view.container.querySelector('video')!,'paused',{configurable:true,value:false});
 act(()=>{vi.advanceTimersByTime(2100);presented(performance.now(),{presentedFrames:42});});
 act(()=>vi.advanceTimersByTime(700));
 expect(screen.getByTestId('e6-video-stats').textContent).toContain('Video paused');
 expect(health.at(-1)).toBe(false);
});
it('measurement exports pixel identity, clock bounds and actual presentation windows together',async()=>{
 const pixels={drawImage:vi.fn(),getImageData:()=>({data:new Uint8ClampedArray(132*132*4)})};
 vi.spyOn(HTMLCanvasElement.prototype,'getContext').mockReturnValue(pixels as any);
 const epoch='1234567890abcdef'.repeat(2);
 vi.mocked(jsQR).mockReturnValue({data:`E6K:${epoch}:17:3000`} as any);
 const view=render(<WebRTCPreview url="http://127.0.0.1:8889/gemini?measure=1"/>);
 const video=view.container.querySelector('video')!;
 Object.defineProperty(video,'videoWidth',{configurable:true,value:640});
 act(()=>{
  window.dispatchEvent(new CustomEvent('e6-clock-sample',{detail:{client_sent_ms:900,client_received_ms:950,
    x5_received_ns:1920e6,x5_sent_ns:1921e6}}));
  vi.advanceTimersByTime(2100);presented(performance.now(),{presentedFrames:42,expectedDisplayTime:2100});
 });
 expect(downloaded).toHaveLength(0);
 expect(screen.queryByTestId('e6-video-report')).toBeNull();
 const report=await downloadReport();
 const download=screen.getByRole('link',{name:'下载已保存报告'});
 expect(download.getAttribute('href')).toBe('/e6/diagnostics/video/video-'+'a'.repeat(32)+'.json');
 expect(download.getAttribute('download')).toMatch(/^video-[0-9a-f]{32}\.json$/);
 expect(HTMLAnchorElement.prototype.click).not.toHaveBeenCalled();
 act(()=>vi.advanceTimersByTime(1100));
 expect(URL.revokeObjectURL).not.toHaveBeenCalled(); // Link remains usable beyond one second.
 expect(report.schema).toBe(2);expect(report.samples).toHaveLength(1);
 expect(report.samples[0]).toMatchObject({captureEpoch:epoch,frameId:17,basis:'uvc_kernel_receipt',
   sourceMs:3000,presented:2100,clock:{startedAt:900,at:950,lower:971,upper:1020}});
 expect(report.presentationWindows).toHaveLength(1);
 expect(report.presentationWindows[0]).toMatchObject({frames:42,endMs:2100,connection:1});
 expect(report.summary.samples).toBe(report.samples.length);
 expect(report.attempts).toBe(1);expect(report.decodeFailures).toBe(0);
 expect(report.callbackCosts).toHaveLength(1);
 expect(report.callbackCosts[0]).toMatchObject({atMs:2100,qrMs:0,callbackDelayMs:0,decoderMs:null});
 act(()=>fireEvent.click(screen.getByLabelText('解码像素测时标记（取消仅暂停测时，继续统计呈现帧率）')));
 act(()=>{vi.advanceTimersByTime(2100);presented(performance.now(),{presentedFrames:84,expectedDisplayTime:4200});});
 expect(downloaded).toHaveLength(1); // Presentation never serializes the growing history.
 const pausedReport=await downloadReport();
 expect(pausedReport.attempts).toBe(1);
 expect(pausedReport.presentationWindows).toHaveLength(2);
 expect(pausedReport.measurementChanges).toEqual([{atMs:3200,enabled:false}]);
 expect(URL.createObjectURL).not.toHaveBeenCalled(); // HTTP files do not depend on temporary blobs.
 vi.mocked(fetch).mockResolvedValueOnce({ok:false,status:507} as Response);
 await act(async()=>fireEvent.click(screen.getByRole('button',{name:'保存视频测量报告'})));
 expect(screen.getByRole('alert').textContent).toContain('507');
 expect(screen.getByRole('link',{name:'下载已保存报告'})).toBeTruthy(); // Earlier file remains available.
 // A gateway replacement may remove the optional marker. Old measurements
 // remain in the exported history, but cannot label a new live stream.
 view.rerender(<WebRTCPreview url="http://127.0.0.1:8889/gemini"/>);
 act(()=>{vi.advanceTimersByTime(2100);presented(performance.now(),{presentedFrames:42,expectedDisplayTime:6300});});
 expect(screen.queryByTestId('e6-video-report')).toBeNull();
 expect(screen.queryByText(/视频测量：/)).toBeNull();
});

it('diagnostic receiver counters are read-only, bounded to video fields and stop on unmount',async()=>{
 const pixels={drawImage:vi.fn(),getImageData:()=>({data:new Uint8ClampedArray(132*132*4)})};
 vi.spyOn(HTMLCanvasElement.prototype,'getContext').mockReturnValue(pixels as any);
 vi.mocked(jsQR).mockReturnValue({data:`E6K:${'1234567890abcdef'.repeat(2)}:17:3000`} as any);
 const getStats=vi.fn().mockResolvedValue(new Map([['video',{type:'inbound-rtp',kind:'video',
   framesDecoded:40,packetsLost:2,totalDecodeTime:.02,jitter:Infinity,unrelatedAddress:'do not retain',
   codecId:'codec',decoderImplementation:'test decoder',powerEfficientDecoder:false}],
   ['codec',{type:'codec',mimeType:'video/H264',sdpFmtpLine:'profile-level-id=42e01f'}],
   ['audio',{type:'inbound-rtp',kind:'audio',framesDecoded:99}]]));
 const view=render(<WebRTCPreview url="http://127.0.0.1:8889/gemini?measure=1"/>);
 Object.defineProperty(view.container.querySelector('video')!,'videoWidth',{configurable:true,value:640});
 act(()=>readerConfig.onTrack({streams:[{}],track:{kind:'video'},receiver:{getStats}}));
 const audioStats=vi.fn().mockResolvedValue(new Map());
 act(()=>readerConfig.onTrack({streams:[{}],track:{kind:'audio'},receiver:{getStats:audioStats}}));
 await act(async()=>vi.advanceTimersByTimeAsync(1000));
 expect(getStats).toHaveBeenCalledTimes(1);
 expect(audioStats).not.toHaveBeenCalled();
 // A presented marker is needed for the existing report output; counters alone
 // must not claim a fresh frame or generate a video timing sample.
 expect(health.every(value=>!value)).toBe(true);
 act(()=>window.dispatchEvent(new CustomEvent('e6-clock-sample',{detail:{client_sent_ms:900,client_received_ms:950,
   x5_received_ns:1920e6,x5_sent_ns:1921e6}})));
 await act(async()=>vi.advanceTimersByTimeAsync(1100));
 act(()=>presented(performance.now(),{presentedFrames:42,expectedDisplayTime:2100,processingDuration:.0005}));
 const report=await downloadReport();
 expect(report.receiverStats).toHaveLength(2);
 expect(report.receiverStats[0].values).toEqual({framesDecoded:40,packetsLost:2,totalDecodeTime:.02});
 expect(report.receiverStats[0].decoder).toEqual({implementation:'test decoder',powerEfficient:false,
   codec:'video/H264',format:'profile-level-id=42e01f'});
 expect(report.callbackCosts[0].decoderMs).toBe(.5);
 view.unmount();
 await act(async()=>vi.advanceTimersByTimeAsync(3000));
 expect(getStats).toHaveBeenCalledTimes(2);
 const ordinary=render(<WebRTCPreview url="http://127.0.0.1:8889/gemini"/>);
 act(()=>readerConfig.onTrack({streams:[{}],track:{kind:'video'},receiver:{getStats}}));
 await act(async()=>vi.advanceTimersByTimeAsync(3000));
 expect(getStats).toHaveBeenCalledTimes(2);ordinary.unmount();
});

it.each(['track','source epoch'] as const)('a new %s starts a fresh timing segment without deleting old evidence',async change=>{
 const pixels={drawImage:vi.fn(),getImageData:()=>({data:new Uint8ClampedArray(132*132*4)})};
 vi.spyOn(HTMLCanvasElement.prototype,'getContext').mockReturnValue(pixels as any);
 const first='a'.repeat(32),second='b'.repeat(32);
 vi.mocked(jsQR).mockReturnValue({data:`E6K:${first}:17:3000`} as any);
 const view=render(<WebRTCPreview url="http://127.0.0.1:8889/gemini?measure=1"/>);
 const video=view.container.querySelector('video')!;
 Object.defineProperty(video,'videoWidth',{configurable:true,value:640});
 Object.defineProperty(video,'paused',{configurable:true,value:false});
 let dropped=7;
 vi.spyOn(video,'getVideoPlaybackQuality').mockImplementation(()=>({droppedVideoFrames:dropped} as VideoPlaybackQuality));
 const receiver={getStats:vi.fn().mockResolvedValue(new Map())};
 const clock=(at:number)=>window.dispatchEvent(new CustomEvent('e6-clock-sample',{detail:{
   client_sent_ms:at-50,client_received_ms:at,x5_received_ns:(at+970)*1e6,x5_sent_ns:(at+971)*1e6}}));
 act(()=>{readerConfig.onTrack({streams:[{}],track:{kind:'video'},receiver});clock(950);
   vi.advanceTimersByTime(2100);presented(performance.now(),{presentedFrames:42,expectedDisplayTime:2100});});
 const before=await downloadReport();expect(before.summary.samples).toBe(1);
 act(()=>{
   if(change==='track')readerConfig.onTrack({streams:[{}],track:{kind:'video'},receiver});
   else {vi.mocked(jsQR).mockReturnValue({data:`E6K:${second}:1:5200`} as any);
     clock(4000);vi.advanceTimersByTime(2200);
     presented(performance.now(),{presentedFrames:86,expectedDisplayTime:4300});}
 });
 expect(screen.queryByText(/视频测量：/)).toBeNull();
 dropped=10;
 const next=performance.now()+2200;
 act(()=>{clock(next-1000);vi.mocked(jsQR).mockReturnValue({data:`E6K:${second}:2:${next+900}`} as any);
   vi.advanceTimersByTime(2200);presented(performance.now(),{presentedFrames:130,expectedDisplayTime:next});});
 const after=await downloadReport();
 expect(after.summary.samples).toBe(1);expect(after.summary.connection).toBe(2);
 expect(after.samples).toHaveLength(2);
 expect(after.samples.map((s:any)=>s.captureEpoch)).toEqual([first,second]);
 expect(after.samples.map((s:any)=>s.connection)).toEqual([1,2]);
 expect(after.segmentChanges).toHaveLength(1);
 expect(after.segmentChanges[0].reason).toBe(change==='track'?'video_track_replaced':'capture_epoch_changed');
 expect(after.presentationWindows.at(-1).dropped).toBe(3);
});
