// Bounds from a four-timestamp exchange; no cross-host clock subtraction
// without an offset interval. The payload identifies the displayed frame.
export type ClockSample={client_sent_ms:number;client_received_ms:number;
  x5_received_ns:number;x5_sent_ns:number};
export type ClockBound={lower:number;upper:number;at:number;rtt:number;startedAt?:number};
export function clockBound(sample:ClockSample):ClockBound|null {
  const {client_sent_ms:a,client_received_ms:d,x5_received_ns:b,x5_sent_ns:c}=sample;
  if(![a,b,c,d].every(Number.isFinite)||d<a||c<b)return null;
  const lower=c/1e6-d,upper=b/1e6-a;
  if(lower>upper)return null;
  return {lower,upper,at:d,rtt:d-a-(c-b)/1e6,startedAt:a};
}
export function frameLatency(marker:string,presented:number,bound:ClockBound|null) {
  if(!bound||!Number.isFinite(presented)||presented<bound.at||presented-bound.at>3000)return null;
  const fields=marker.split(':');
  const legacy=fields.length===3&&fields[0]==='E6';
  if(!legacy&&(fields.length!==4||!['E6K','E6S'].includes(fields[0])||!/^[a-f0-9]{32}$/.test(fields[1])))return null;
  const captureEpoch=legacy?null:fields[1];
  const basis=legacy?'legacy_unspecified':fields[0]==='E6K'?'uvc_kernel_receipt':'sdk_wait_return';
  const rawId=fields[legacy?1:2],rawTime=fields[legacy?2:3];
  if(!/^\d+$/.test(rawId)||!/^\d+$/.test(rawTime))return null;
  const frameId=Number(rawId),sourceMs=Number(rawTime);
  if(!Number.isSafeInteger(frameId)||!Number.isSafeInteger(sourceMs))return null;
  // 1 ms source quantization, plus an explicit 1000 ppm relative drift budget.
  // Include the whole clock exchange, not only time since its reply.
  // This is a documented engineering bound, not exposure/hardware sync.
  const error=1+(presented-(bound.startedAt??bound.at))*.001;
  return {frameId,sourceMs,presented,captureEpoch,basis,clock:{...bound},driftAndQuantizationMs:error,
    lowerMs:presented+bound.lower-sourceMs-error,
    upperMs:presented+bound.upper-sourceMs+error,clockWidthMs:bound.upper-bound.lower+2*error};
}
