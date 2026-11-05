import {useState} from 'react';
import {LogStream} from '@/components/custom/LogsStream';
import {Button} from '@/components/ui/button';
import {Select,SelectContent,SelectItem,SelectTrigger,SelectValue} from '@/components/ui/select';
import {TerminalSquare,ChevronDown,ChevronUp} from 'lucide-react';
import {ProblemReport} from './ProblemReport';

// One reader beneath the input tabs; never owns a control hook or WebSocket.
export function LogConsole(){
  const [open,setOpen]=useState(false);
  const [source,setSource]=useState('control');
  return <section className="mt-6 min-w-0" aria-label="Program logs" data-e6-input-isolation>
    <Button variant="ghost" onClick={()=>setOpen(value=>!value)} aria-expanded={open} aria-controls="program-logs">
      <TerminalSquare className="size-4"/>Logs{open?<ChevronUp className="size-4"/>:<ChevronDown className="size-4"/>}
    </Button>
    {open&&<div id="program-logs" className="mt-2"><LogStream logFile={null} isLoading={false} title="Program logs"
      streamUrl={`/e6/logs/${source}/stream`} downloadUrl={`/e6/logs/${source}/download`}
      tools={<><Select value={source} onValueChange={setSource}><SelectTrigger aria-label="Log source" className="w-28"><SelectValue/></SelectTrigger>
        <SelectContent>{['Control','PC','Camera','Servo'].map(label=><SelectItem key={label} value={label.toLowerCase()}>{label}</SelectItem>)}</SelectContent>
      </Select><ProblemReport/></>}/></div>}
  </section>;
}
