import {Circle,CircleCheck,CircleX,FolderOpen,Settings2} from 'lucide-react';
import {Button} from '@/components/ui/button';
import {Input} from '@/components/ui/input';
import {Label} from '@/components/ui/label';
import {Badge} from '@/components/ui/badge';
import {Popover,PopoverContent,PopoverTrigger} from '@/components/ui/popover';
import {useState} from 'react';
import {useNavigate} from 'react-router-dom';
import {contextStorageKey,readRecordingContext,ResearchContext} from './recordingContext';
import {EpisodeTransferProgress,recordingPhaseNames,useNativeRecording} from './EpisodeBrowser';
import {RecordingIssue} from './RecordingIssue';

export function RecordingPanel({native,task,onTask,onStart,onSuccess,onFailure}: {
  native:ReturnType<typeof useNativeRecording>;task:string;onTask:(value:string)=>void;
  onStart:()=>void;onSuccess:()=>void;onFailure:()=>void;
}) {
  const navigate=useNavigate();
  const [context,setContext]=useState(readRecordingContext);
  const capacity=native.state.capacity;
  const byteLabel=(bytes:number)=>bytes>=1024**3?`${(bytes/1024**3).toFixed(2)} GiB`:`${(bytes/1024**2).toFixed(1)} MiB`;
  const updateContext=(key:keyof ResearchContext,value:string)=>{
    const next={...context,[key]:value};setContext(next);
    localStorage.setItem(contextStorageKey,JSON.stringify(next));
  };
  return <section className="flex min-w-0 flex-wrap items-center gap-2" aria-label="Recorder">
    <Popover><PopoverTrigger asChild><Button variant="outline" className="max-w-52" aria-label="Recording settings">
      <Settings2 className="size-4 shrink-0"/><span className="truncate">{task&&task!=='None'?task:'Task'}</span>
    </Button></PopoverTrigger><PopoverContent align="end" className="w-80 space-y-3">
      <div className="space-y-1"><Label htmlFor="e6-task">Task</Label>
        <Input id="e6-task" value={task==='None'?'':task} maxLength={500} disabled={native.active||native.busy}
          onChange={event=>onTask(event.target.value)}/></div>
      <div className="space-y-1"><Label htmlFor="e6-scene">Scene / notes</Label>
        <Input id="e6-scene" value={context.scene||''} maxLength={500} disabled={native.active||native.busy}
          onChange={e=>updateContext('scene',e.target.value)}/></div>
      <div className="space-y-1"><Label htmlFor="e6-operator">Operator</Label>
        <Input id="e6-operator" value={context.operator||''} maxLength={80} disabled={native.active||native.busy}
          onChange={e=>updateContext('operator',e.target.value)}/></div>
      {capacity&&<div className="space-y-1 text-xs" aria-label="Episode capacity">
        <p>Raw images · {byteLabel(capacity.used_bytes)} / {byteLabel(capacity.limit_bytes)}</p>
        <progress className="w-full" aria-label="Raw image capacity" value={capacity.used_bytes} max={capacity.limit_bytes}/>
      </div>}
    </PopoverContent></Popover>
    <Button onClick={onStart} disabled={native.active||native.busy||!native.canStart}><Circle className="size-3 fill-current"/>Record</Button>
    {native.active&&<>
      <Button onClick={onSuccess} variant="outline"><CircleCheck className="size-4"/>Success</Button>
      <Button onClick={onFailure} variant="outline"><CircleX className="size-4"/>Failure</Button>
      <Button onClick={()=>native.request('record_finish','aborted')} variant="ghost">Abort</Button>
    </>}
    <Badge variant="outline" className="gap-2" role="status">
      {native.active&&<span className="size-2 rounded-full bg-red-500"/>}
      {recordingPhaseNames[native.state.phase]||native.state.phase} · {native.state.samples||0}
    </Badge>
    <Button variant="ghost" size="icon" aria-label="Browse recordings" onClick={()=>navigate("/browse")}><FolderOpen className="size-4"/></Button>
    {capacity?.warning&&native.active&&<p role="alert" className="w-full text-xs text-amber-700 dark:text-amber-400">
      Capacity nearly full · {byteLabel(capacity.remaining_bytes)} left. Finish to save; reaching the limit retains an aborted episode.
    </p>}
    <EpisodeTransferProgress item={native.transfer}/>
    {native.state.error&&<RecordingIssue reason={native.state.error} transferPhase={native.transfer?.phase}/>}
  </section>;
}
