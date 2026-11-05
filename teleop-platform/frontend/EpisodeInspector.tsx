import {Card,CardContent} from '@/components/ui/card';
type Pose={position_m?:number[];quaternion_xyzw?:number[];joints_rad?:number[];gripper?:number|null;input_sequence?:number;receipt?:Receipt;
  pose_frame?:string;flange_pose?:{position_m:number[]}};
type Receipt={command?:string;accepted?:boolean;acknowledged?:boolean;uncertain?:boolean;sent_ns?:number;reply_ns?:number;reply?:string;error?:string};
type ImageSource={device_frame_id?:number|null;x5_receive_monotonic_ns?:number;fresh?:boolean;reused?:boolean;age_ms?:number;
  sdk_available_ns?:number|null;cycle_valid?:boolean;cycle_reused?:boolean;
  metadata?:{host_perf_counter_ns?:number}};
type Association={kind?:string;valid?:boolean;action_sent_ns?:number;feedback_valid?:boolean;
  media?:Record<string,{frame_id?:number;valid?:boolean;reused?:boolean;age_ms?:number}>};
export type SamplePage={total:number;cycles:{at_ns:number;state:{actual?:Pose;candidate?:Pose;accepted?:Pose;receipt?:Receipt;
  pending?:Receipt;feedback?:{age_ms?:number;sequence?:number};stage?:string;reason?:string}}[];
  media:{sources:{rgb?:ImageSource;depth?:ImageSource};association_sample_ns?:number;selection_basis?:string;recorded_sample?:unknown}[];
  received_inputs?:({at_ns:number;connection_id?:number;data:{seq?:number;source?:string;intent?:{
    deadman:boolean;frame:string;linear_m_s:number[];angular_rad_s:number[];joint_rad_s:number[];gripper_rate_s:number}}}|null)[];
  events?:{at_ns:number;event:string;receipt?:Receipt;target?:Pose;data?:{kind?:string;data_hex?:string}}[];
  associations:Association[]};
const number=(n:unknown,scale=1,digits=2)=>typeof n==='number'&&Number.isFinite(n)?(n*scale).toFixed(digits):'Unknown';
const vector=(values?:number[],scale=1,digits=2)=>values?.length?values.map(n=>number(n,scale,digits)).join(' / '):'Not recorded';
const verdict=(value?:boolean)=>value===true?'Valid':value===false?'Invalid':'Not recorded';
const receiptText=(receipt?:Receipt)=>!receipt?'No receipt':receipt.uncertain===true||receipt.acknowledged!==true?'Unconfirmed':
  receipt.accepted===true?'Accepted':receipt.accepted===false?'Rejected':'Replied; acceptance unknown';
function PoseCard({title,pose}:{title:string;pose?:Pose}) {
  return <Card><CardContent className="space-y-2 pt-6"><h4>{title}</h4>{pose?<dl>
    <dt>{pose.pose_frame==='grasp_tcp'?'Grasp TCP XYZ · mm':'Flange XYZ · mm'}</dt><dd>{vector(pose.position_m,1000)}</dd>
    {pose.flange_pose&&<><dt>Original flange XYZ · mm</dt><dd>{vector(pose.flange_pose.position_m,1000)}</dd></>}
    <dt>J1–J6 · °</dt><dd>{vector(pose.joints_rad,180/Math.PI)}</dd>
    <dt>Quaternion · xyzw</dt><dd>{vector(pose.quaternion_xyzw,1,5)}</dd>
    {pose.input_sequence!==undefined&&<><dt>Input sequence</dt><dd>{pose.input_sequence}</dd></>}
    {title==='Actual feedback'&&<><dt>Gripper · native CAN</dt><dd>{pose.gripper==null?'Unknown':`${number(pose.gripper,100)} %`}</dd></>}
    {pose.receipt&&<><dt>Receipt</dt><dd>{receiptText(pose.receipt)}</dd></>}
  </dl>:<p>No {title.toLowerCase()} recorded</p>}</CardContent></Card>;
}
export function EpisodeInspector({page,episode}:{page:SamplePage;episode:string}) {
  const row=page.cycles[0];const input=page.received_inputs?.[0];const intent=input?.data.intent;
  const source=page.media[0]?.sources;const frame=source?.rgb?.device_frame_id;
  const sdkSelection=page.media[0]?.selection_basis==='sdk_available_v2';
  if(!row)return <p role="status">No cycle samples. Original files are retained.</p>;
  const receivedAge=input?(row.at_ns-input.at_ns)/1e6:undefined;
  return <div className="space-y-4 [&_h4]:font-semibold [&_dt]:text-muted-foreground [&_dd]:break-words [&_dl]:space-y-1">
    <div className="space-y-3">
      {frame!=null?<img className="w-full max-h-[45vh] object-contain rounded-md border bg-muted" key={`${episode}/${frame}`} src={`/e6/episodes/${episode}/rgb/${frame}`} alt="Saved original RGB"/>:
        <p role="status">No RGB image available before this sample.</p>}
      <details><summary className="cursor-pointer text-muted-foreground">Image associations</summary><div className="space-y-2 pt-2">{(['rgb','depth'] as const).map(name=>{
        const src=source?.[name];const age=src?.x5_receive_monotonic_ns===undefined?undefined:(row.at_ns-src.x5_receive_monotonic_ns)/1e6;
        return <p key={name}>{name==='rgb'?'RGB':'Depth'} #{src?.device_frame_id??'Unknown'} · Cycle age {number(age)} ms<br/>
          {age!==undefined&&<><span className={age>100||age<0?'text-amber-700 dark:text-amber-400':'text-muted-foreground'}>
            {age>100?'Stale: over 100 ms':age<0?'Received after cycle; invalid':'Within 100 ms'}</span><br/></>}
          {sdkSelection?<>Cycle association {verdict(src?.cycle_valid)} · Reused across cycles {src?.cycle_reused===true?'Yes':src?.cycle_reused===false?'No':'Not recorded'}<br/>
            SDK available before cycle {src?.sdk_available_ns==null?'Unknown':`${number((row.at_ns-src.sdk_available_ns)/1e6)} ms`}</>:
            <>Capture freshness {verdict(src?.fresh)} · Capture reused {src?.reused===true?'Yes':src?.reused===false?'No':'Not recorded'}</>}</p>;
      })}<p className="text-muted-foreground">{sdkSelection?
        'Selected from frames already available to the X5 SDK. Original associations are retained. Exposures are not synchronized.':
        'Selected before this cycle in X5 time. Exposures are not synchronized.'}</p></div></details>
    </div>
    <div className="grid gap-4 md:grid-cols-2">
      <Card><CardContent className="space-y-2 pt-6"><h4>Received input</h4>{intent?<dl>
        <dt>Source / sequence</dt><dd>{input?.data.source||'Unknown'} / {input?.data.seq??'Unknown'}</dd>
        <dt>Cycle age</dt><dd>{number(receivedAge)} ms</dd>
        <dt>Deadman / frame</dt><dd>{intent.deadman?'Held':'Released'} / {intent.frame==='base'?'Base':intent.frame==='tool'?'Tool':intent.frame}</dd>
        <dt>Linear velocity · mm/s</dt><dd>{vector(intent.linear_m_s,1000)}</dd>
        <dt>Angular velocity · °/s</dt><dd>{vector(intent.angular_rad_s,180/Math.PI)}</dd>
        <dt>Joint velocity · °/s</dt><dd>{vector(intent.joint_rad_s,180/Math.PI)}</dd>
        <dt>Gripper rate · %/s</dt><dd>{number(intent.gripper_rate_s,100)}</dd>
      </dl>:<p>No preceding input</p>}<p className="text-muted-foreground">Input is not an executed action.</p></CardContent></Card>
      <PoseCard title="Candidate target" pose={row.state.candidate}/>
      <PoseCard title="Accepted target" pose={row.state.accepted}/>
      <PoseCard title="Actual feedback" pose={row.state.actual}/>
    </div>
    <Card><CardContent className="space-y-2 pt-6"><h4>Execution state</h4>
      <p>Latest receipt：{receiptText(row.state.receipt)} · Pending command：{row.state.pending?'Unconfirmed':'None recorded'}</p>
      <p>Feedback #{row.state.feedback?.sequence??'Unknown'} · Age {number(row.state.feedback?.age_ms)} ms</p>
      {row.state.reason&&<p>{row.state.reason}</p>}
      <h4 className="mt-3">Send / acknowledgement events until next sample</h4>
      {page.events?.length?<ul>{page.events.map((event,i)=><li key={i}>
        {event.event} · +{number((event.at_ns-row.at_ns)/1e6)} ms · {event.event==='can'?`CAN ${event.data?.kind||''} ${event.data?.data_hex||''}`:
          receiptText(event.receipt||event.target?.receipt)}</li>)}</ul>:<p>No action events in this window.</p>}
      <p className="text-muted-foreground">Accepted targets are historical snapshots; actual feedback is separate.</p>
    </CardContent></Card>
    <details className="rounded-md border p-4"><summary className="cursor-pointer">Action associations · {page.associations.length}</summary>
      {page.associations.length?<div className="overflow-x-auto"><table className="w-full text-left text-xs [&_td]:border-t [&_td]:p-2 [&_th]:p-2"><thead><tr><th>Sent at · X5 ns</th><th>Overall</th><th>Feedback</th><th>RGB / Depth</th></tr></thead>
        <tbody>{page.associations.map((a,i)=><tr key={i}><td>{a.action_sent_ns??'Not recorded'}</td><td>{verdict(a.valid)}</td><td>{verdict(a.feedback_valid)}</td><td>
          {(['rgb','depth'] as const).map(name=><p key={name}>{name} #{a.media?.[name]?.frame_id??'Unknown'} · {verdict(a.media?.[name]?.valid)} · {number(a.media?.[name]?.age_ms)} ms · {a.media?.[name]?.reused===true?'Reused':a.media?.[name]?.reused===false?'New frame':'Reuse unknown'}</p>)}
        </td></tr>)}</tbody></table></div>:<p>No sent action associations in this window.</p>}
    </details>
    <details className="rounded-md border p-4"><summary>Raw inputs, state, events & associations</summary><pre className="mt-3 max-h-96 overflow-auto rounded-md bg-muted p-3 text-xs">{JSON.stringify(page,null,2)}</pre></details>
  </div>;
}
