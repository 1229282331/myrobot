import { useE6Control } from './useE6Control';
import { Card,CardContent } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { useEffect, useRef, useState } from 'react';
import {CameraPreview} from './PlatformPages';
import {Badge} from '@/components/ui/badge';
import {Tabs,TabsList,TabsTrigger} from '@/components/ui/tabs';
import {Select,SelectTrigger,SelectValue,SelectContent,SelectItem} from '@/components/ui/select';
import {Popover,PopoverTrigger,PopoverContent} from '@/components/ui/popover';
import {CircleHelp} from 'lucide-react';
import { GamepadAcceptance } from './GamepadAcceptance';
import {WorkspaceView} from './WorkspaceView';
import {revealAcceptanceViews} from './acceptanceViewport';
import './workbench.css';

export function ControlStatus({control:c}: {control:ReturnType<typeof useE6Control>}) {
  const [testing,setTesting]=useState(false);
  const [testKey,setTestKey]=useState('d');
  const [testMs,setTestMs]=useState(1000);
  const [testFine,setTestFine]=useState(false);
  const [testCentered,setTestCentered]=useState(false);
  const gripperTest=testKey==='f'||testKey==='r';
  const includesGripper=/[fr]/.test(testKey);
  const testDuration=Math.min(testMs,3000);
  // Discovery is cached across a server restart; only the current control
  // connection can attest the stage and limits shown to the operator.
  const connected=c.state.state!=='未连接'&&c.state.echo_ns>0;
  const backendLabel=connected?c.caps?.backend:'E6 offline';
  const gripperOpening=c.state.gripper?.stroke?c.state.gripper.opening:c.state.actual?.gripper;
  const gripperRange=c.state.gripper?.stroke||c.state.gripper?.limits;
  const reasonLabels:Record<string,string>={operator_stop:'Stopped',
    recording_finished:'Recording ended · prepare to continue',
    input_source_changed:'Input changed · prepare again',
    input_stale:'Input paused · release / center to resume',
    solver_stream_stale:'Control paused · release / center to resume',
    control_policy_changed:'Control mode changed · release / center to resume',
    browsing_recordings:'Browsing records',
    page_blur:'Window unfocused',
    page_hidden:'Page hidden',
    gamepad_disconnected:'Gamepad disconnected',
    preview_stale:'Video paused'};
  const notices:Record<string,string>={
    '正在初始化录制，完成后可准备控制。':'Recording starting · wait to prepare',
    '正在封存原始记录，封存后可重新准备。':'Saving recording · wait to prepare',
    '松开允许动作并回中后切换':'Release and center to switch',
    '连接已恢复，请回中后重新准备':'Connected · center and prepare',
    '浏览结束后回中并重新准备':'Center and prepare after browsing',
    '先关闭片段浏览再准备':'Close the episode browser to prepare',
    '等待实时画面恢复':'Waiting for live video',
    '松开允许动作，摇杆回中后准备':'Release and center to prepare',
    '正在重新连接':'Reconnecting…',
    '释放操作并回中后开始录制':'Release and center to record',
    '释放允许动作并回中后初始化夹爪':'Release and center to initialize gripper',
    '释放允许动作并回中后使能':'Release and center to enable',
    '当前设备状态不可使能':'Robot cannot be enabled',
    '释放允许动作并回中后重连相机':'Release and center to reconnect camera',
    '请结束录制并等待设备静止后重连相机':'Finish recording and wait for rest',
  };
  const pending=useRef<{timer:ReturnType<typeof setTimeout>|null;keys:string[]}|null>(null);
  const dispatch=(type:string,key:string)=>window.dispatchEvent(new KeyboardEvent(type,{key,bubbles:true}));
  const releaseTest=()=>{
    const active=pending.current;
    if(!active)return;
    pending.current=null;
    if(active.timer!==null)clearTimeout(active.timer);
    // All releases share a task; the hook samples the final neutral vector.
    for(const key of active.keys)dispatch('keyup',key);
    setTesting(false);
  };
  useEffect(()=>{
    const hidden=()=>{if(document.hidden)releaseTest();};
    const escape=(event:KeyboardEvent)=>{if(event.key==='Escape')releaseTest();};
    window.addEventListener('blur',releaseTest);
    window.addEventListener('keydown',escape);
    document.addEventListener('visibilitychange',hidden);
    return ()=>{releaseTest();window.removeEventListener('blur',releaseTest);window.removeEventListener('keydown',escape);
      document.removeEventListener('visibilitychange',hidden);};
  },[]);
  useEffect(()=>{
    if(!c.enabled||!c.state.ready||c.state.reason||!c.caps?.synthetic_keys||c.source!=='keyboard')releaseTest();
  },[c.enabled,c.state.ready,c.state.reason,c.caps?.synthetic_keys,c.source]);
  const testKeys=(continuous=false)=>{
    if(pending.current||!c.state.ready||!c.caps?.synthetic_keys||c.source!=='keyboard')return;
    if(continuous&&(includesGripper||!testKey||testDuration<500))return;
    const keys=[...(testFine?['Shift']:[]),...testKey.split('')];
    const active={timer:null as ReturnType<typeof setTimeout>|null,keys};
    pending.current=active;setTesting(true);
    try {
      // Exercise the installed keyboard handler, not a target or HTTP endpoint.
      // The hook coalesces the complete vector at this task's microtask.
      for(const key of keys)dispatch('keydown',key);
      if(!continuous)active.timer=setTimeout(releaseTest,testDuration);
      else {
        // A bounded input fixture, not a robot trajectory. Every axis
        // event goes through the real mapper, with no permission modifier.
        const opposite:Record<string,string>={a:'d',d:'a',s:'w',w:'s',q:'e',e:'q'};
        const forward=testKey.split(''),backward=forward.map(key=>opposite[key]);
        const phaseMs=testDuration;
        // Center the same six full periods around the initial pose: half a
        // forward phase, eleven full alternating phases, half a forward phase.
        // This changes only the fixture waveform, never speed or device limits.
        const phaseCount=testCentered?13:12;
        let phase=0;
        const next=()=>{
          if(pending.current!==active)return;
          if(++phase>=phaseCount){releaseTest();return;}
          const from=phase%2?forward:backward,to=phase%2?backward:forward;
          for(const key of from)dispatch('keyup',key);
          for(const key of to)dispatch('keydown',key);
          active.keys=[...(testFine?['Shift']:[]),...to];
          active.timer=setTimeout(next,testCentered&&phase===12?phaseMs/2:phaseMs);
        };
        active.timer=setTimeout(next,testCentered?phaseMs/2:phaseMs);
      }
    } catch(error) {releaseTest();throw error;}
  };
  if (!c.enabled) return null;
  const statusLabels:Record<string,string>={'未连接':'Offline','可准备':'Standby','可控制':'Ready','暂挂':'Paused','设备故障':'Device fault'};
  const offline=c.state.state==='未连接';
  const statusMessages=Array.from(new Set([
    c.state.reason&&(offline?'Control service disconnected':reasonLabels[c.state.reason]||c.state.reason),
    c.notice&&c.notice!==c.state.reason&&(notices[c.notice]||c.notice),
    c.prepareBlockedReason&&(notices[c.prepareBlockedReason]||c.prepareBlockedReason),
    c.state.ready&&c.state.model?.status===4&&'Model clearance · reduced speed',
  ].filter((message):message is string=>!!message)));
  return <Card><CardContent className="space-y-4 pt-6" data-testid="e6-status">
    <div className="flex items-center justify-between gap-3"><strong>{statusLabels[c.state.state]||c.state.state}</strong>
      <Badge variant="outline" data-testid="e6-backend">{connected?c.caps?.backend:'E6 offline'}</Badge></div>
    {/* Reserve two lines even when empty so status updates never move controls.
        Full pause reasons remain available to assistive tech and on hover. */}
    <div className="h-10 min-w-0 text-sm leading-5 text-muted-foreground" role="status" aria-live="polite" aria-atomic="true" data-testid="e6-status-messages">
      <p className="line-clamp-2 break-words" title={statusMessages.join(' · ')}>{statusMessages.join(' · ')}</p>
    </div>
    {c.state.stage==='live'&&c.state.arm&&<div className="flex flex-wrap gap-2 items-center text-sm">
      <Button variant="outline" disabled={!c.state.arm.can_enable} onClick={c.enableArm}>Enable Robot</Button>
      <Badge variant="secondary">{c.state.arm.enabled===null?'Unknown':c.state.arm.enabled?'Enabled':'Disabled'}</Badge>
      {c.state.arm.enable_phase==='observing'&&<span>Stabilizing · {(c.state.arm.stable_ms/1000).toFixed(1)} / 5 s</span>}
    </div>}
    <div className="flex flex-wrap gap-3">
      <Tabs value={c.mode} onValueChange={v=>c.chooseMode(v as typeof c.mode)}><TabsList aria-label="Control mode">
        <TabsTrigger value="translation">Translation</TabsTrigger>
        {c.caps?.orientation_assist?.version===1&&<TabsTrigger value="assisted">Position assist</TabsTrigger>}
        <TabsTrigger value="rotation">Rotation</TabsTrigger><TabsTrigger value="joint">Joint</TabsTrigger>
      </TabsList></Tabs>
      <Select value={c.frame} onValueChange={v=>c.chooseFrame(v as typeof c.frame)}><SelectTrigger aria-label="Reference frame" className="w-36"><SelectValue/></SelectTrigger>
        <SelectContent><SelectItem value="base">Base frame</SelectItem><SelectItem value="tool">Tool frame</SelectItem></SelectContent></Select>
      {c.mode==='joint'&&<Select value={String(c.joint)} onValueChange={v=>c.chooseJoint(Number(v))}>
        <SelectTrigger aria-label="Joint" className="w-24"><SelectValue/></SelectTrigger><SelectContent>
          {[0,1,2,3,4,5].map(i=><SelectItem key={i} value={String(i)}>J{i+1}</SelectItem>)}
        </SelectContent></Select>}
    </div>
    <div className="flex flex-wrap gap-2 text-sm"><Badge variant="secondary">{c.source==='keyboard'?'Direction keys':'Sticks / triggers'} · Move after Prepare</Badge>
      <Badge variant="outline">{c.source==='keyboard'?'Shift':'RB'} · Fine 25%</Badge></div>
    <div className="min-h-10 text-sm text-muted-foreground" data-testid="e6-orientation-policy">
      {c.mode==='assisted'?<>Position assist · automatic rotation up to 15° per movement, at most {Math.min(3,(c.caps?.angular_rad_s||0)*180/Math.PI*(c.speed??1)).toFixed(1)}°/s.
        {c.frame==='tool'?' Tool axes stay fixed during each movement.':' Release to hold the new orientation.'}</>:
        c.mode==='translation'?'Translation · hold the current orientation.':null}
    </div>
    <p className="text-sm text-muted-foreground" data-testid="e6-limits">{connected?<>Selected speed · {((c.caps?.linear_m_s||0)*1000*(c.speed??1)).toFixed(1)} mm/s · {((c.caps?.angular_rad_s||0)*180/Math.PI*(c.speed??1)).toFixed(1)}°/s</>:'Speed unavailable'}</p>
    {c.state.actual&&<div data-testid="e6-actual" className="space-y-2">
      <div className="grid grid-cols-3 gap-3">{c.state.actual.position_m.map((v,i)=><div key={i} className="rounded-md bg-muted p-3">
        <p className="text-xs text-muted-foreground">{['X','Y','Z'][i]} · mm</p><p className="font-mono">{(v*1000).toFixed(2)}</p></div>)}</div>
      <p className="text-sm text-muted-foreground">{c.state.actual.pose_frame==='grasp_tcp'?'Grasp TCP':'Flange'} · Gripper {gripperOpening==null?'Unknown':`${(gripperOpening*100).toFixed(1)}%`}</p>
      {c.mode==='joint'&&<div className="grid grid-cols-3 gap-2 text-sm font-mono" aria-label="Actual joint angles">
        {c.state.actual.joints_rad.map((v,i)=><span key={i}>J{i+1} · {(v*180/Math.PI).toFixed(2)}°</span>)}
      </div>}
    </div>}
    {c.state.stage==='live'&&<div className="space-y-2 text-sm">
      <div className="flex flex-wrap gap-2 items-center"><span>Gripper{gripperRange&&` · ${Math.round(gripperRange.minimum*100)}–${Math.round(gripperRange.maximum*100)}%`}</span><Badge variant="outline" title={c.state.gripper?.position_status==='dropped'?'Release input, then open to recover. No automatic regrasp.':undefined}>{c.state.gripper?.position_status==='dropped'?'Dropped':c.state.gripper?.position_status==='contact'?'Contact':c.state.gripper?.position_status==='obstructed'?'Blocked':c.state.gripper?.phase||'Unknown'}</Badge>
        {c.state.gripper?.phase==='uninitialized'&&<span className="text-muted-foreground">Waiting for device readiness</span>}</div>
    </div>}
    <Popover><PopoverTrigger asChild><Button size="sm" variant="ghost"><CircleHelp className="size-4"/>Controls reference</Button></PopoverTrigger>
      <PopoverContent data-e6-input-isolation className="text-sm space-y-2"><p>{c.source==='keyboard'?'A/D · X, S/W · Y, Q/E · Z; rotation uses the same axes.':'Left stick · X/Y, right stick vertical · Z; rotation uses the same axes.'}</p>
        {c.caps?.orientation_assist?.version===1&&<p>Position assist lets the wrist turn gradually while you translate. Release holds its new orientation; each new movement starts a fresh 15° allowance. Use Translation when the object must stay level.</p>}
        <p>{c.source==='keyboard'?'Tab':'X'} switches between Rotation and your last selected position mode.</p>
        <p>{c.source==='keyboard'?'F/R · Open/close, Tab · Mode, C · Frame, Esc · Stop':'LT/RT · Open/close, X · Mode, Y · Frame, Back · Stop'}</p>
    </PopoverContent></Popover>
    {c.caps?.synthetic_keys && c.source==='gamepad' && <GamepadAcceptance control={c}/>}
    {c.caps?.synthetic_keys && c.source==='keyboard' && <details className="e6-acceptance" onToggle={revealAcceptanceViews}><summary>Synthetic input diagnostics</summary>
      <select aria-label="验收方向" value={testKey} disabled={testing} onChange={e=>{setTestKey(e.target.value);setTestMs(ms=>Math.min(ms,1000));}}>
        {[['','回中（无动作）'],['d','X+'],['a','X−'],['w','Y+'],['s','Y−'],['e','Z+'],['q','Z−'],
            ['dw','XY+'],['as','XY−'],['dwe','XYZ+'],['asq','XYZ−'],['f','夹爪张开'],['r','夹爪闭合'],['wf','Y+ / open'],['sr','Y− / close']].map(([k,n])=><option key={k} value={k}>{n}</option>)}
      </select>
      <label>持续时间 <select aria-label="验收持续时间" value={testDuration} disabled={testing} onChange={e=>setTestMs(Number(e.target.value))}>
        <option value={100}>0.1秒（单次）</option><option value={200}>0.2秒（单次）</option>
        <option value={500}>0.5秒</option><option value={750}>0.75秒</option><option value={1000}>1秒</option>
        <option value={3000}>3秒</option></select></label>
        <label><input type="checkbox" checked={testFine} disabled={testing} onChange={e=>setTestFine(e.target.checked)}/>验收精细25%</label>
        <label><input type="checkbox" checked={testCentered} disabled={testing||gripperTest} onChange={e=>setTestCentered(e.target.checked)}/>对称往返（首尾半周期）</label>
      <Button disabled={!c.state.ready||testing} onClick={()=>testKeys()}>合成键盘：{testDuration/1000}秒后释放</Button>
        <Button disabled={!c.state.ready||testing||includesGripper||!testKey||testDuration<500} onClick={()=>testKeys(true)}>连续往返：6轮</Button>
      {testing&&<Button variant="outline" onClick={releaseTest}>结束验收输入</Button>}
        <p className="text-sm">0.1／0.2秒仅用于单次输入。连续往返保持允许动作；每向使用上方持续时间（{testDuration/1000}秒）{testCentered?'，首尾各半时长，中间11段保持完整时长':''}，总计六轮后自动释放。仅用于已确认包络内的空载验收。</p>
      <p className="text-sm">通过本页键盘事件与正式意图连接。当前后端：{backendLabel}。</p>
    </details>}
  </CardContent></Card>;
}

// Shared state only: neither view creates another control owner.
export function ControlCamera({control:c}: {control:ReturnType<typeof useE6Control>}) {
  const connected=c.state.state!=='未连接'&&c.state.echo_ns>0;
  const view=connected?c.state:c.observation;
  return c.enabled?<div className="e6-observation-row">
    {!connected&&view&&<p role="status" className="text-sm text-muted-foreground">Read-only view · Prepare to reconnect control</p>}
    <div className="e6-observation-grid">
    <WorkspaceView joints={view?.actual?.joints_rad}
      gripper={view?.gripper?.geometry}
      observation={view?.echo_ns}
      fresh={view?.feedback?.connected===true&&(view.feedback.age_ms??Infinity)>=0&&(view.feedback.age_ms??Infinity)<=100}
      mode={c.caps?.test_workspace?.mode||'local'} assemblyId={c.caps?.workspace?.assembly_id} reason={view?.reason}/>
    <CameraPreview url={c.caps?.video_url} footer={c.state.camera_restart?.available&&<div className="flex flex-wrap items-center gap-2 text-sm">
      <Button size="sm" variant="outline" onClick={c.restartCamera} disabled={!c.state.camera_restart.can_restart}>Reconnect Camera</Button>
      <span className="text-muted-foreground">{c.state.camera_restart.phase}</span>
    </div>}/>
  </div></div>:null;
}
