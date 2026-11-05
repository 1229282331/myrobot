import {useEffect,useRef,useState} from 'react';
import useSWR from 'swr';
import {Button} from '@/components/ui/button';
import {Dialog,DialogContent,DialogDescription,DialogHeader,DialogTitle,DialogTrigger} from '@/components/ui/dialog';
import {announceEpisodeBrowser} from './episodeBrowserState';
import {Badge} from '@/components/ui/badge';
import {Slider} from '@/components/ui/slider';
import {Select,SelectTrigger,SelectValue,SelectContent,SelectItem} from '@/components/ui/select';
import {Table,TableBody,TableCell,TableHead,TableHeader,TableRow} from '@/components/ui/table';
import {Input} from '@/components/ui/input';
import {EpisodeInspector,SamplePage} from './EpisodeInspector';
import {ResearchContext} from './recordingContext';
import {RecordingIssue} from './RecordingIssue';

const read=async(url:string)=>{const response=await fetch(url);if(!response.ok)throw new Error('Could not read recordings');return response.json();};
type Recording={phase:string;episode?:string;error?:string;samples?:number;result?:string;
  capacity?:{limit_bytes:number;used_bytes:number;remaining_bytes:number;warning:boolean;accounting:string}|null};
export function useNativeRecording() {
  const {data:caps}=useSWR('/e6/capabilities',read);
  const [state,setState]=useState<Recording>({phase:'disconnected'});
  const {data:transfers,mutate:refreshTransfers}=useSWR<{episodes:Episode[]}>('/e6/episodes',read,{
    refreshInterval:state.phase==='transfer_pending'?2000:0});
  useEffect(()=>{
    if(state.episode&&['transfer_pending','complete','aborted','failed'].includes(state.phase))
      void refreshTransfers().catch(()=>{});
  },[state.episode,state.phase,refreshTransfers]);
  useEffect(()=>{const listener=(event:Event)=>setState((event as CustomEvent<Recording>).detail||{phase:'unavailable'});
    window.addEventListener('e6-recording-state',listener);return()=>window.removeEventListener('e6-recording-state',listener);},[]);
  const request=(op:string,result?:string,task?:string)=>window.dispatchEvent(new CustomEvent('e6-recording-request',{detail:{op,result,task}}));
  return {enabled:!!caps?.original_recording,state,request,
    transfer:transfers?.episodes.find(item=>item.episode===state.episode),
    active:['starting','capturing'].includes(state.phase),busy:['starting','sealing','transfer_pending'].includes(state.phase),
    canStart:['idle','complete','aborted'].includes(state.phase)};
}

type Episode={episode:string;phase:string;error?:string;status_saved?:boolean;summary?:{result:string;samples:number;reason?:string};metadata?:{task:string;dataset_kind:string;created_at?:string;research_context?:ResearchContext};
  progress?:{stage:string;received_bytes:number;total_bytes:number|null};
  verification?:{actions:number;invalid_action_associations:number;native_files:number;camera_samples?:number;invalid_camera_sample_associations?:{rgb:number;depth:number}}};
const outcomeNames:Record<string,string>={success:'Success',failure:'Failure',aborted:'Aborted'};
const kindNames:Record<string,string>={hardware_demonstration:'Hardware demonstration',hardware_shadow:'Hardware shadow',hardware_observation:'Observation'};
export const recordingPhaseNames:Record<string,string>={idle:'Idle',starting:'Starting',capturing:'Recording',sealing:'Sealing',
  transfer_pending:'Transferring',complete:'Saved',aborted:'Aborted · retained',failed:'Save failed',disconnected:'Select input',unavailable:'Unavailable'};

export function EpisodeTransferProgress({item}:{item?:Episode}) {
  const progress=item?.progress;
  if(!progress||!['transferring','failed'].includes(item!.phase))return null;
  const received=progress.received_bytes,total=progress.total_bytes;
  if(!Number.isFinite(received)||received<0)return null;
  const known=typeof total==='number'&&Number.isFinite(total)&&total>0;
  const bytes=(value:number)=>`${(value/1024**2).toFixed(1)} MiB`;
  const stages:Record<string,string>={receiving:'Receiving originals',checksum:'Checking package',verification:'Verifying files and associations',receipt:'Confirming receipt'};
  return <div aria-label="Transfer progress" className="rounded-md border p-3 text-xs space-y-1">
    <div className="flex justify-between gap-2"><span>{item?.phase==='failed'?'Transfer paused':stages[progress.stage]||'Saving'}</span>
      <span className="tabular-nums">{bytes(received)}{known?` / ${bytes(total)} · ${Math.min(100,100*received/total).toFixed(0)}%`:''}</span></div>
    {known&&<progress aria-label="Transfer completion" className="w-full" value={Math.min(received,total)} max={total}/>}
    <p className="text-muted-foreground">Background transfer · control available. Next recording after verification.</p>
  </div>;
}

function EpisodeDetail({item,onRetry}:{item:Episode;onRetry:()=>void}) {
  const [offset,setOffset]=useState(0);const [retryError,setRetryError]=useState('');
  const ready=item.phase==='verified';
  const {data:page,isValidating,error}=useSWR<SamplePage>(ready?`/e6/episodes/${item.episode}/samples?offset=${offset}&limit=1`:null,read,{keepPreviousData:true});
  return <div className="min-w-0 space-y-4 text-sm">
    <div className="flex flex-wrap gap-2"><Badge variant="outline">{outcomeNames[item.summary?.result||'']||'Incomplete / unknown'}</Badge>
      <Badge variant="secondary">{kindNames[item.metadata?.dataset_kind||'']||'Unknown source'}</Badge>
      <Badge variant="outline">{item.phase==='verified'?'Files verified':item.phase}</Badge></div>
    <p className="break-all text-xs text-muted-foreground">{item.episode}</p>
    <details><summary className="cursor-pointer text-muted-foreground">Metadata & verification</summary><div className="space-y-2 pt-2">
      <p>Scene: {item.metadata?.research_context?.scene||'—'} · Operator: {item.metadata?.research_context?.operator||'—'}</p>
      {item.verification&&<><p>{item.verification.actions} sent actions · {item.verification.native_files} original images · {item.verification.invalid_action_associations} invalid action associations</p>
        {item.verification.invalid_camera_sample_associations&&<p>Camera samples: {item.verification.camera_samples} · Invalid RGB: {item.verification.invalid_camera_sample_associations.rgb} · Invalid depth: {item.verification.invalid_camera_sample_associations.depth}</p>}
        <p className="text-muted-foreground">File verification does not certify task success or association validity.</p></>}
    </div></details>
    {item.summary?.reason&&<RecordingIssue reason={item.summary.reason} transferPhase={item.phase} outcome/>}
    <EpisodeTransferProgress item={item}/>
    {item.error&&<div role="alert" className="space-y-2"><p className="break-words text-destructive">{item.error}</p>
      <Button size="sm" variant="outline" onClick={async()=>{try {
        const response=await fetch(`/e6/episodes/${item.episode}/retry`,{method:'POST'});
        if(!response.ok)throw new Error('Transfer retry failed');setRetryError('');onRetry();
      }catch(error){setRetryError(String(error));}}}>Retry transfer</Button></div>}
    {retryError&&<p role="alert">{retryError}</p>}
    {item.phase==='failed'&&item.status_saved===false&&<p role="alert" className="text-destructive">Local status could not be saved. Recover storage, then retry transfer.</p>}
    {!ready&&<p role="status">Waiting for file verification.</p>}
    {ready&&<>
      <div className="flex flex-wrap items-center gap-3">
        <span className="tabular-nums">Sample {offset+1} / {page?.total||0}</span>
        <Slider aria-label="Episode sample" className="min-w-32 flex-1" min={0} max={Math.max(0,(page?.total||1)-1)} step={1} value={[offset]} onValueChange={value=>setOffset(value[0])}/>
        <Button size="sm" variant="outline" disabled={offset===0} onClick={()=>setOffset(n=>n-1)}>Previous</Button>
        <Button size="sm" variant="outline" disabled={offset+1>=(page?.total||0)} onClick={()=>setOffset(n=>n+1)}>Next</Button>
      </div>
      {isValidating&&<p role="status">Loading sample…</p>}
      {error&&<p role="alert">Could not read this sample.</p>}
      {!isValidating&&!error&&page&&<EpisodeInspector page={page} episode={item.episode}/>}
    </>}
  </div>;
}

function EpisodeTable() {
  const {data,mutate,error}=useSWR<{episodes:Episode[]}>('/e6/episodes',read,{refreshInterval:2000});
  const [selected,setSelected]=useState('');const [query,setQuery]=useState('');
  const [outcome,setOutcome]=useState('all');const [kind,setKind]=useState('all');
  const visible=(data?.episodes||[]).filter(item=>
    (outcome==='all'||item.summary?.result===outcome||(outcome==='unfinished'&&!item.summary?.result))&&
    (kind==='all'||item.metadata?.dataset_kind===kind)&&
    [item.episode,item.metadata?.task,item.metadata?.research_context?.scene,item.metadata?.research_context?.operator]
      .some(value=>value?.toLocaleLowerCase().includes(query.trim().toLocaleLowerCase())));
  const item=data?.episodes.find(entry=>entry.episode===selected);
  return <div className="space-y-4">
    {error&&<p role="alert">Could not refresh recordings. Displayed entries may be outdated.</p>}
    <div className="flex flex-wrap gap-3">
      <Input className="min-w-48 flex-1" aria-label="Search recordings" value={query} onChange={e=>setQuery(e.target.value)} placeholder="Search task, scene, operator or ID"/>
      <Select value={outcome} onValueChange={setOutcome}><SelectTrigger aria-label="Episode result" className="w-44"><SelectValue/></SelectTrigger><SelectContent>
        <SelectItem value="all">All results</SelectItem>{Object.entries(outcomeNames).map(([key,label])=><SelectItem key={key} value={key}>{label}</SelectItem>)}<SelectItem value="unfinished">Incomplete / unknown</SelectItem>
      </SelectContent></Select>
      <Select value={kind} onValueChange={setKind}><SelectTrigger aria-label="Episode source" className="w-44"><SelectValue/></SelectTrigger><SelectContent>
        <SelectItem value="all">All sources</SelectItem>{Object.entries(kindNames).map(([key,label])=><SelectItem key={key} value={key}>{label}</SelectItem>)}
      </SelectContent></Select>
    </div>
    <div className="rounded-md border"><Table><TableHeader><TableRow>
      <TableHead>Task</TableHead><TableHead>Result</TableHead><TableHead>Source</TableHead><TableHead>Samples</TableHead><TableHead>Status</TableHead><TableHead/>
    </TableRow></TableHeader><TableBody>
      {visible.map(entry=><TableRow key={entry.episode}>
        <TableCell><div className="max-w-80 break-words font-medium">{entry.metadata?.task||'Untitled'}</div><div className="text-xs text-muted-foreground">{entry.episode}</div></TableCell>
        <TableCell><Badge variant="outline">{outcomeNames[entry.summary?.result||'']||'Incomplete / unknown'}</Badge></TableCell>
        <TableCell>{kindNames[entry.metadata?.dataset_kind||'']||entry.metadata?.dataset_kind||'Unknown'}</TableCell>
        <TableCell>{entry.summary?.samples??'—'}</TableCell>
        <TableCell><Badge variant={entry.error?'destructive':'secondary'}>{entry.phase==='verified'?'Verified':recordingPhaseNames[entry.phase]||entry.phase}</Badge></TableCell>
        <TableCell><Button size="sm" variant="outline" aria-label={entry.episode} onClick={()=>setSelected(entry.episode)}>View</Button></TableCell>
      </TableRow>)}
      {!visible.length&&<TableRow><TableCell colSpan={6} className="h-24 text-center">{data?'No recordings found.':'Loading recordings…'}</TableCell></TableRow>}
    </TableBody></Table></div>
    <p className="text-sm text-muted-foreground">{visible.length} / {data?.episodes.length||0} recordings</p>
    <Dialog open={!!selected} onOpenChange={value=>{if(!value)setSelected('');}}><DialogContent className="max-h-[90dvh] overflow-y-auto sm:max-w-6xl" data-testid="e6-episode-browser">
      <DialogHeader><DialogTitle>{item?.metadata?.task||'Episode inspection'}</DialogTitle><DialogDescription>Saved inputs, execution and observations. Read only.</DialogDescription></DialogHeader>
      {item&&<EpisodeDetail key={item.episode} item={item} onRetry={()=>mutate()}/>}
    </DialogContent></Dialog>
  </div>;
}

export function EpisodeLibrary() {
  useEffect(()=>{announceEpisodeBrowser(true);return()=>announceEpisodeBrowser(false);},[]);
  return <EpisodeTable/>;
}

// Kept for recovery mode and existing recorder integrations.
export function EpisodeBrowser({state,compact=false,recoveryOnly=false}:{state:Recording;compact?:boolean;recoveryOnly?:boolean}) {
  const [open,setOpen]=useState(false);const opened=useRef(false);
  const changeOpen=(value:boolean)=>{opened.current=value;announceEpisodeBrowser(value);setOpen(value);};
  useEffect(()=>{const show=()=>changeOpen(true);window.addEventListener('e6-open-episodes',show);
    if(new URLSearchParams(window.location.search).get('panel')==='episodes')show();
    return()=>{window.removeEventListener('e6-open-episodes',show);if(opened.current)announceEpisodeBrowser(false);};},[]);
  return <div className="space-y-2 text-sm">
    {!compact&&<p role="status">{recordingPhaseNames[state.phase]||state.phase} · {state.samples||0} samples</p>}
    <Dialog open={open} onOpenChange={changeOpen}><DialogTrigger asChild><Button variant="outline">Browse recordings</Button></DialogTrigger>
      <DialogContent className="max-h-[90dvh] overflow-y-auto sm:max-w-6xl"><DialogHeader><DialogTitle>Local recordings</DialogTitle>
        <DialogDescription>{recoveryOnly?'Recovery mode · devices disconnected.':'Control paused. Prepare explicitly after closing.'}</DialogDescription></DialogHeader><EpisodeTable/>
      </DialogContent>
    </Dialog>
  </div>;
}
