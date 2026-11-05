import {useEffect,useRef,useState} from 'react';
import {Card,CardContent,CardHeader,CardTitle} from '@/components/ui/card';
import {Button} from '@/components/ui/button';
import {Badge} from '@/components/ui/badge';
import {Box,Expand,RotateCcw,RefreshCw,Eye,EyeOff} from 'lucide-react';
import type {WorkspaceScene,GripperPose} from './workspaceRenderer';

type Props={joints?:number[];fresh:boolean;mode:string;reason?:string;assemblyId?:string|null;observation?:number;gripper?:GripperPose|null};
type Model={model_id:string;robot_url:string;map_url:string;map_available:boolean;gripper_collision?:{display_union?:{file:string}}};

export function WorkspaceView({joints,fresh,mode,reason,assemblyId,observation,gripper}:Props){
  const host=useRef<HTMLDivElement>(null),scene=useRef<WorkspaceScene|null>(null);
  const selectedLayers=useRef({points:true,collision:false});
  const [model,setModel]=useState<Model|null>(null),[error,setError]=useState('');
  const [paused,setPaused]=useState(false);
  const [reload,setReload]=useState(0),[loading,setLoading]=useState(true);
  const receipt=useRef({token:observation??joints,at:performance.now()});
  if(receipt.current.token!==(observation??joints))receipt.current={token:observation??joints,at:performance.now()};
  const [now,setNow]=useState(()=>performance.now());
  useEffect(()=>{const timer=setInterval(()=>setNow(performance.now()),100);return()=>clearInterval(timer);},[]);
  const feedbackFresh=fresh&&now-receipt.current.at<=500;
  const matches=mode!=='model'||!!(model&&assemblyId===model.model_id);
  const valid=feedbackFresh&&matches;
  const gripReceipt=useRef({stamp:gripper?.received_ns,at:performance.now()});
  if(gripReceipt.current.stamp!==gripper?.received_ns)gripReceipt.current={stamp:gripper?.received_ns,at:performance.now()};
  const gripFresh=valid&&!!gripper?.valid&&now-gripReceipt.current.at<=200;
  const gripPose:GripperPose=gripFresh?gripper!:{valid:false,fallback:true,joints_rad:null};
  const latest=useRef({joints,fresh:valid,gripper:gripPose});latest.current={joints,fresh:valid,gripper:gripPose};
  const [points,setPoints]=useState(true),[collision,setCollision]=useState(false),[expanded,setExpanded]=useState(false);
  const slot=useRef<HTMLDivElement>(null);const [reservedHeight,setReservedHeight]=useState<number>();
  const toggleExpanded=()=>{if(!expanded)setReservedHeight(slot.current?.getBoundingClientRect().height);setExpanded(!expanded);};
  useEffect(()=>{
    const abort=new AbortController();let disposed=false,timedOut=false;
    setModel(null);setError('');setLoading(true);
    const timer=setTimeout(()=>{timedOut=true;abort.abort();},5000);
    fetch('/e6/workspace',{signal:abort.signal,cache:'no-store'}).then(async r=>{
      if(!r.ok)throw new Error('Workspace model unavailable');return r.json();
    }).then(v=>{if(!v.robot_url)throw new Error('Workspace model unavailable');if(!disposed)setModel(v);})
      .catch(e=>{if(!disposed)setError(timedOut?'Workspace request timed out':String(e));})
      .finally(()=>{clearTimeout(timer);if(!disposed)setLoading(false);});
    return()=>{disposed=true;clearTimeout(timer);abort.abort();};
  },[assemblyId,reload]);
  useEffect(()=>{
    if(!model||!host.current||paused)return;
    let disposed=false;
    import('./workspaceRenderer').then(({createWorkspaceScene})=>{if(disposed||!host.current)return null;const union=model.gripper_collision?.display_union?.file;return createWorkspaceScene(host.current,model.robot_url,model.map_url,model.model_id,message=>{if(!disposed)setError(message);},union?new URL(encodeURIComponent(union),new URL(model.robot_url,window.location.href)).href:undefined);}).then(value=>{
      if(!value)return;
      if(disposed){value.dispose();return;}scene.current=value;
      value.layers(selectedLayers.current.points,selectedLayers.current.collision);
      if(latest.current.fresh&&latest.current.joints)value.setJoints(latest.current.joints);
      value.setGripper?.(latest.current.gripper);
    }).catch(e=>{if(!disposed)setError('3D view unavailable: '+String(e));});
    return()=>{disposed=true;scene.current?.dispose();scene.current=null;};
  },[model,paused]);
  useEffect(()=>{if(valid&&joints)scene.current?.setJoints(joints);},[joints,valid]);
  useEffect(()=>{scene.current?.setGripper?.(gripPose);},[gripper,gripFresh]);
  useEffect(()=>{selectedLayers.current={points,collision};scene.current?.layers(points,collision);},[points,collision]);
  useEffect(()=>{if(!expanded)return;const escape=(e:KeyboardEvent)=>{if(e.key==='Escape')setExpanded(false);};window.addEventListener('keydown',escape);return()=>window.removeEventListener('keydown',escape);},[expanded]);
  return <div ref={slot} style={expanded?{minHeight:reservedHeight}:undefined} className="e6-workspace-slot"><Card data-e6-input-isolation data-testid="workspace-view" className={expanded?'e6-workspace-card e6-workspace-expanded':'e6-workspace-card'}>
    <CardHeader className="flex flex-row items-center justify-between gap-2 pb-3"><CardTitle className="flex items-center gap-2 text-2xl"><Box className="size-5"/>Workspace</CardTitle>
      <div className="flex gap-1"><Button size="icon" variant="ghost" aria-label={paused?'Resume 3D view':'Pause 3D view'} onClick={()=>setPaused(!paused)}>{paused?<EyeOff className="size-4"/>:<Eye className="size-4"/>}</Button><Button size="icon" variant="ghost" aria-label="Reset workspace view" onClick={()=>scene.current?.reset()}><RotateCcw className="size-4"/></Button>
      <Button size="icon" variant="ghost" aria-label="Reload workspace" disabled={loading} onClick={()=>setReload(n=>n+1)}><RefreshCw className="size-4"/></Button>
      <Button size="icon" variant="ghost" aria-label={expanded?'Collapse workspace':'Expand workspace'} onClick={toggleExpanded}><Expand className="size-4"/></Button></div></CardHeader>
    <CardContent className={expanded?'flex min-h-0 flex-1 flex-col gap-3':'space-y-3'}>
      <div ref={host} tabIndex={0} onPointerDown={event=>event.currentTarget.focus()} aria-label="Robot workspace model" className={expanded?'e6-workspace-canvas min-h-0 flex-1 overflow-hidden rounded-md bg-muted/40':'e6-workspace-canvas h-[320px] overflow-hidden rounded-md bg-muted/40'}>{paused&&<div className="flex h-full items-center justify-center text-sm text-muted-foreground">3D paused</div>}</div>
      <div className="flex flex-wrap gap-2"><Badge variant="outline">{mode==='model'?'Model range':'Local range · 50 mm'}</Badge><Badge variant="outline">{!matches?'Model mismatch':feedbackFresh?'Live feedback':'Feedback unavailable'}</Badge></div>
      <div className="flex flex-wrap gap-2"><Button size="sm" variant={points?'secondary':'ghost'} aria-pressed={points} onClick={()=>setPoints(!points)}>Reachability</Button>
      <Button size="sm" variant={collision?'secondary':'ghost'} aria-pressed={collision} onClick={()=>setCollision(!collision)}>Collision bounds</Button>
      <span className={'flex items-center gap-3 text-xs '+(collision?'':'invisible')} aria-hidden={!collision}><span className="text-sky-400">Rigid</span><span className="text-amber-400">{gripFresh?'Moving fingers':'Unknown · Full stroke'}</span></span></div>
      <div className="h-10 text-xs text-muted-foreground leading-5"><p>Environment not modeled · Nominal attachment geometry</p><p className="truncate" title={error||reason}>{error||reason||(gripFresh?'Gripper follows measured aperture · Nominal linkage':'Gripper feedback unavailable · Conservative fallback')}</p></div>
    </CardContent>
  </Card></div>;
}
