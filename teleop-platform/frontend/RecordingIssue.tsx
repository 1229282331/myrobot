/** Presentation only: preserve the recorder's outcome and original reason. */
export function RecordingIssue({reason,transferPhase,outcome=false}:{reason:string;transferPhase?:string;outcome?:boolean}) {
  const diagnostic=reason.replace(/^相机记录异常: /,'');
  const capacityReached=diagnostic==='Native image payload budget exceeded';
  const bufferFull=diagnostic==='Native archive writer payload buffer exceeded'
    ||diagnostic==='Native archive writer queue overflow';
  if(!capacityReached&&!bufferFull)return outcome?<p>End reason: {reason}</p>
    :<p className="text-sm text-destructive" role="alert">{reason}</p>;
  return <div className="rounded-md border border-amber-500/40 bg-amber-500/5 p-3 text-sm space-y-2" role="alert">
    <p className="font-medium">{capacityReached?'Image capacity reached · recording aborted':'Image buffer full · recording aborted'}</p>
    <p className="text-muted-foreground">{bufferFull&&'Image storage fell behind capture. '}Existing data retained as aborted.</p>
    <p>{transferPhase==='verified'
      ?'Files verified. Prepare again to record.'
      :transferPhase==='failed'
        ?'Transfer incomplete. Check the episode, then retry.'
        :'Wait for verification, then prepare again.'}</p>
    <details className="text-xs text-muted-foreground"><summary>Diagnostics</summary><p className="mt-1 break-words">{reason}</p></details>
  </div>;
}
