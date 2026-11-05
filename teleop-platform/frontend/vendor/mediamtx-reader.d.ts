export class MediaMTXWebRTCReader {
  constructor(config:{url:string;onError:(message:string)=>void;onTrack:(event:RTCTrackEvent)=>void});
  close():void;
}
