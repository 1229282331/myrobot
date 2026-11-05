import { useEffect, useRef, useState } from 'react';
import useSWR from 'swr';
import { extractAnalogValues } from '@/pages/control/GamepadUtils';
import {isEpisodeBrowserOpen} from './episodeBrowserState';
import {readRecordingContext} from './recordingContext';
import {acceptanceGamepad} from './acceptanceGamepad';
import {useDeviceObservation} from './WorkbenchChrome';

export type ControlMode = 'translation' | 'assisted' | 'rotation' | 'joint';
export type Frame = 'base' | 'tool';
const nextMode=(mode:ControlMode,positionMode:'translation'|'assisted'):ControlMode=>
  mode==='rotation'||mode==='joint'?positionMode:'rotation';
type Source = 'keyboard' | 'gamepad';
export type Intent = {
  linear_m_s: number[]; angular_rad_s: number[]; joint_rad_s: number[];
  gripper_rate_s: number; deadman: boolean; frame: Frame;
  orientation_assist?:boolean; assist_angular_rad_s?:number;
};
type Capabilities = { enabled: boolean; backend: string; linear_m_s: number;
  orientation_assist?:{version:number;max_angle_deg:number;max_rate_rad_s:number}|null;
  tcp?: {profile:string;frame:string;label:string;translation_m:number[];calibration:string;sha256:string};
  angular_rad_s: number; joint_rad_s: number; gripper_rate_s: number; synthetic_keys?: boolean; video_url?: string; original_recording?:boolean;
  test_workspace?: {mode?:'local'|'model';position_m:number;rotation_deg:number|null;joint_deg:number|null};
  workspace?: {mode:'local'|'model';assembly_id:string|null;restriction?:string};
  joint_range?: {margin_deg:number;lower_deg:number[];upper_deg:number[]} | null };
type DeviceState = { state: string; ready: boolean; reason?: string; echo_ns: number;
  model?:{status?:number;phase?:string;ready?:boolean;error?:string|null;
    orientation_assist?:{enabled:boolean;deviation_deg:number;reason:string}}|null;
  feedback?: {connected:boolean;age_ms?:number};
  arm?: { enabled:boolean|null;mode:number|null;can_enable:boolean;enable_phase:string;enable_reason?:string|null;stable_ms:number };
  capabilities?: Capabilities;
  gripper?:{phase:string;valid:boolean;reason?:string;target:number|null;last_sent:number|null;
    opening?:number|null;stroke?:{revision:string;minimum:number;maximum:number;closed_raw:number;open_raw:number};
    geometry?:import('./workspaceRenderer').GripperPose|null;
    limits?:{minimum:number;maximum:number;rate_s:number};
    effective_target?:number|null;arrived?:boolean|null;position_status?:string;position_error_counts?:number|null};
  stage?: string; candidate?: {position_m:number[]} | null; accepted?: {position_m:number[]} | null;
  recording?: {phase:string;episode?:string;error?:string;samples?:number;result?:string};
  camera?: {valid:boolean;age_ms?:number;frame_id?:number}; stationary?:boolean; stop_acknowledged?:boolean;
  camera_restart?: {available:boolean;phase:string;can_restart:boolean;error?:string};
  actual?: { joints_rad: number[]; position_m: number[]; gripper: number | null; pose_frame?:string;
    flange_pose?:{position_m:number[];quaternion_xyzw:number[]} } };
export const neutral = (frame: Frame = 'base'): Intent => ({linear_m_s:[0,0,0],
  angular_rad_s:[0,0,0],joint_rad_s:[0,0,0,0,0,0],gripper_rate_s:0,deadman:false,frame});
export function radial(x: number, y: number) {
  const norm = Math.hypot(x,y);
  const scale = norm <= .05 ? 0 : (Math.min(norm,1)-.05)/(.95*norm);
  return [x*scale,y*scale];
}
const axis = (value: number) => Math.abs(value)<=.05 ? 0 : Math.sign(value)*(Math.min(Math.abs(value),1)-.05)/.95;
const editable = (target: EventTarget | null) => target instanceof HTMLElement &&
  !!target.closest('input,textarea,select,[contenteditable="true"],[role="textbox"],[role="combobox"],[role="listbox"],[data-e6-input-isolation]');
const recordingPrepareBlock = (phase?:string):string|null => {
  if(phase==='starting')return '正在初始化录制，完成后可准备控制。';
  if(phase==='sealing')return '正在封存原始记录，封存后可重新准备。';
  return null;
};
const preparationBlock = (state:DeviceState):string|null =>
  recordingPrepareBlock(state.recording?.phase)||
  (state.gripper?.geometry&&state.gripper.phase==='uninitialized'
    ?'Waiting for automatic gripper connection':null)||
  (state.gripper?.phase==='connecting'||state.gripper?.phase==='initializing'?'Wait for gripper connection':null);

export function mappedIntent(channels: number[], gripper: number, deadman: boolean,
  fine: boolean, mode: ControlMode, frame: Frame, joint: number, scale: number, caps: Capabilities): Intent {
  const result = neutral(frame);
  result.deadman = deadman;
  // Vector magnitude is bounded; diagonals do not increase selected speed.
  const norm = Math.max(1,Math.hypot(...channels));
  const speed = Math.max(0,Math.min(1,scale)) * (fine ? .25 : 1);
  if (mode==='translation'||mode==='assisted') result.linear_m_s=channels.map(x=>x/norm*caps.linear_m_s*speed);
  if(mode==='assisted'){
    if(caps.orientation_assist?.version!==1||speed<=0)return neutral(frame);
    result.orientation_assist=true;
    result.assist_angular_rad_s=Math.min(caps.orientation_assist.max_rate_rad_s,caps.angular_rad_s*speed);
  }
  if (mode==='rotation') result.angular_rad_s=channels.map(x=>x/norm*caps.angular_rad_s*speed);
  if (mode==='joint') result.joint_rad_s[joint]=channels[0]*caps.joint_rad_s*speed;
  result.gripper_rate_s=gripper*caps.gripper_rate_s*speed;
  return result;
}

export function useE6Control(source: Source, speed: number, gamepadIndex: number | null = null) {
  const {data:discoveredCaps,mutate:syncCapabilities} = useSWR<Capabilities>('/e6/capabilities', async (url:string) => {
    const response=await fetch(url);
    if (!response.ok) throw new Error('设备网关尚未连接');
    try { return await response.json(); } catch { return {enabled:false}; }
  });
  const [state,setState]=useState<DeviceState>({state:'未连接',ready:false,echo_ns:0});
  // The shared HTTP observer restores display only. Its ready/echo values
  // never enter the control session or authorize a new WebSocket owner.
  const {value:observation}=useDeviceObservation<DeviceState>();
  const controlConnected=state.state!=='未连接'&&state.echo_ns>0;
  // A late discovery request must not replace this owner's current limits.
  const caps=controlConnected&&state.capabilities?state.capabilities:discoveredCaps;
  const enabled=!!caps?.enabled;
  const [mode,setMode]=useState<ControlMode>('translation');
  const preferredPositionMode=useRef<'translation'|'assisted'>('translation');
  const [frame,setFrame]=useState<Frame>('base');
  const [joint,setJoint]=useState(0);
  const [notice,setNotice]=useState('');
  const [inputFocusBlocked,setInputFocusBlocked]=useState(false);
  const [activeKeys,setActiveKeys]=useState<string[]>([]);
  const [connectionEpoch,setConnectionEpoch]=useState(0);
  const prepareOnConnect=useRef(false);
  const preparePadIdentity=useRef<string|null>(null);
  const previewFresh=useRef(false);
  const keys=useRef(new Set<string>());
  const blockedKeys=useRef(new Set<string>());
  const ws=useRef<WebSocket | null>(null);
  const last=useRef<Intent>(neutral());
  const current=useRef({source,speed,gamepadIndex,mode,frame,joint,caps,state});
  current.current={source,speed,gamepadIndex,mode,frame,joint,caps,state};
  const sequence=useRef(0);
  const sample=useRef<() => void>(()=>{});
  const keyReleaseGeneration=useRef(0);
  const browsingEpisodes=useRef(isEpisodeBrowserOpen());
  const prepareAfterBrowsing=useRef(false);
  const testPad=useRef<{selected:boolean;pad:Gamepad|null}>({selected:false,pad:null});
  const gamepadNeedsPrepare=useRef(false);
  const [testPadState,setTestPadState]=useState({selected:false,connected:false});
  const clearTestPad=()=>{if(testPad.current.pad)testPad.current.pad=acceptanceGamepad();};
  const selectedGamepad=()=>{
    const c=current.current;
    const pad=testPad.current.selected ? (c.caps?.synthetic_keys ? testPad.current.pad : null) :
      c.gamepadIndex===null?null:navigator.getGamepads()[c.gamepadIndex];
    return pad&&pad.connected!==false?pad:null;
  };
  const gamepadIdentity=(pad=selectedGamepad())=>{
    return pad?`${testPad.current.selected?'test':current.current.gamepadIndex}:${pad.id}`:null;
  };

  const send=(op:string, body:object={}) => {
    const socket=ws.current;
    if (socket?.readyState !== WebSocket.OPEN) return;
    if (socket.bufferedAmount>4096) { socket.close(); return; }
    socket.send(JSON.stringify({op,seq:++sequence.current,echo_ns:current.current.state.echo_ns,
      source,client_sample_ms:performance.now(),...body}));
  };
  const stop=(reason='operator_stop') => {
    prepareOnConnect.current=false;
    clearTestPad();
    keyReleaseGeneration.current++;
    keys.current.clear(); last.current=neutral(current.current.frame);
    send('stop',{reason}); setActiveKeys([]);
  };
  const recording=(op:'record_start'|'record_finish',result?:string,task?:string)=>{
    if(browsingEpisodes.current||prepareAfterBrowsing.current)return;
    if(op==='record_start') {
      if(!centered()){setNotice('释放操作并回中后开始录制');return;}
      send(op,{task:task||localStorage.getItem('e6-recording-task')||'空载遥操作示教',
        research_context:readRecordingContext()});
    } else {
      keys.current.clear();last.current=neutral(current.current.frame);setActiveKeys([]);
      send(op,{result:result||'success'});
    }
  };
  const centered=() => !(source==='keyboard'&&[...keys.current].some(k=>['a','d','s','w','q','e','f','r'].includes(k))) && !last.current.deadman && ![
    ...last.current.linear_m_s,...last.current.angular_rad_s,...last.current.joint_rad_s,
    last.current.gripper_rate_s].some(x=>Math.abs(x)>1e-9);
  const chooseMode=(value:ControlMode) => {
    if (!centered()) {setNotice('松开允许动作并回中后切换'); return;}
    if(value==='assisted'&&current.current.caps?.orientation_assist?.version!==1){setNotice('姿态辅助尚未就绪');return;}
    if(value==='translation'||value==='assisted')preferredPositionMode.current=value;
    setMode(value);setNotice('');
  };
  const chooseFrame=(value:Frame) => {
    if (!centered()) {setNotice('松开允许动作并回中后切换'); return;}
    setFrame(value);setNotice('');
  };
  const chooseJoint=(value:number) => {
    if (centered() && value>=0 && value<6) setJoint(value);
  };
  const sampleKeys=()=>{
    const generation=++keyReleaseGeneration.current;
    // An atomic multi-axis change is sampled once; a physical release still
    // reaches the transport at this event's microtask, without a timer delay.
    queueMicrotask(()=>{
      if(keyReleaseGeneration.current===generation)sample.current();
    });
  };
  const keyDown=(key:string) => {
    const normalized=key.toLowerCase();
    if(!['a','d','s','w','q','e','f','r','shift'].includes(normalized)||blockedKeys.current.has(normalized))return;
    keys.current.add(normalized);sampleKeys();
  };
  const keyUp=(key:string) => {
    const normalized=key.toLowerCase();
    blockedKeys.current.delete(normalized);
    if(keys.current.delete(normalized))sampleKeys();
  };

  useEffect(()=>{
    if(controlConnected||!enabled||!observation?.capabilities?.enabled)return;
    const observedCaps=observation.capabilities;
    if(JSON.stringify(observedCaps)!==JSON.stringify(current.current.caps)){
      current.current.caps=observedCaps;
      void syncCapabilities(observedCaps,{revalidate:false});
    }
  },[observation,controlConnected,enabled,syncCapabilities]);

  useEffect(()=>{
    if (!enabled) return;
    const socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/e6/control`);
    ws.current=socket;
    socket.onmessage=event=>{
      // A replaced socket may deliver queued events after the new source is
      // connected. Only the current connection owns input/state updates.
      if(ws.current!==socket||socket.readyState!==WebSocket.OPEN)return;
      const message=JSON.parse(event.data);
      if(message.kind==='clock_probe'){
        const measurement={...message,client_received_ms:performance.now()};
        window.dispatchEvent(new CustomEvent('e6-clock-sample',{detail:measurement}));
        send('clock_sample',measurement);
        return;
      }
      if (message.kind==='state') {
        // The active connection is authoritative after a gateway replacement.
        // Updating only when values change keeps the input timer continuous.
        if(message.capabilities && JSON.stringify(message.capabilities)!==JSON.stringify(current.current.caps)) {
          current.current.caps=message.capabilities;
          void syncCapabilities(message.capabilities,{revalidate:false});
        }
        current.current.state=message;
        setState(message);
        window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:message.recording}));
        if (prepareOnConnect.current && message.echo_ns && !browsingEpisodes.current) {
          prepareOnConnect.current=false;
          // The connection has recovered even when a fresh preview or a
          // recording transition prevents this one preparation attempt.
          setNotice('连接已恢复，请回中后重新准备');
          sample.current();
          if (!preparationBlock(message)&&centered()&&(source!=='gamepad'||
              (selectedGamepad()&&gamepadIdentity()===preparePadIdentity.current&&!gamepadNeedsPrepare.current))&&
              (!current.current.caps?.video_url||previewFresh.current)) {
            gamepadNeedsPrepare.current=false;prepareAfterBrowsing.current=false;send('prepare');setNotice('');}
        }
      }
    };
    socket.onclose=()=>{ if(ws.current!==socket)return;
      // Keep the server's refusal/diagnosis after it closes the connection.
      // Actual device values are deliberately dropped until a new state arrives.
      const reason=current.current.state.reason||'控制连接已关闭，请回中后重新准备';
      clearTestPad();
      prepareOnConnect.current=false;
      setNotice('');
      keys.current.clear();last.current=neutral(current.current.frame);setActiveKeys([]);
      const disconnected={state:'未连接',ready:false,echo_ns:0,reason};
      current.current.state=disconnected;setState(disconnected);
      window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:{phase:'disconnected'}})); };
    const clockTimer=setInterval(()=>send('clock_probe',{client_sent_ms:performance.now()}),1000);
    // Reconnect is explicit through the preparation button, never replay old input.
    return ()=>{ clearInterval(clockTimer);socket.close(); ws.current=null; keys.current.clear(); };
  },[enabled,source,connectionEpoch]);

  useEffect(()=>{
    if (!enabled || !caps) return;
    let buttons: boolean[]=[];
    let padFocusBlocked=false;
    let previousPadIdentity=source==='gamepad'?gamepadIdentity():null;
    const tick=()=>{
      const c=current.current;
      if (!c.caps) return;
      let xyz=[0,0,0],gripper=0,fine=false;
      if (source==='keyboard') {
        const k=keys.current;
        xyz=[Number(k.has('d'))-Number(k.has('a')),Number(k.has('w'))-Number(k.has('s')),
             Number(k.has('e'))-Number(k.has('q'))];
        gripper=Number(k.has('f'))-Number(k.has('r'));
        fine=k.has('shift');
      } else {
        const pad=selectedGamepad();
        const identity=gamepadIdentity(pad);
        const identityChanged=identity!==previousPadIdentity;
        if(identityChanged) {
          const hadPad=previousPadIdentity!==null;
          previousPadIdentity=identity;
          gamepadNeedsPrepare.current=true;
          prepareOnConnect.current=false;
          buttons=pad?.buttons.map(button=>button.pressed)||[];
          if(hadPad||c.state.ready)stop(pad?'input_source_changed':'gamepad_disconnected');
        }
        if (!pad) {
          if (!identityChanged&&(last.current.deadman || c.state.ready)) stop('gamepad_disconnected');
          last.current=neutral(c.frame);send('intent',{intent:last.current});return;
        }
        // Upstream defaults to a 15% per-axis deadzone. E6 applies its own
        // 5% radial/scalar remap below, so request the unfiltered stick values.
        const raw=extractAnalogValues(pad,0);
        const xy=radial(raw.leftStickX,-raw.leftStickY);
        xyz=[...xy,axis(-raw.rightStickY)];
        gripper=axis(raw.leftTrigger-raw.rightTrigger);
        fine=!!pad.buttons[5]?.pressed;
        // Gamepads have no DOM event target. Apply the same editor/viewer
        // isolation as keyboard input, then require physical centering before
        // resuming. Keep last as the raw intent so Prepare cannot bypass it.
        if(editable(document.activeElement))padFocusBlocked=true;
        else if(xyz.every(x=>x===0)&&gripper===0)padFocusBlocked=false;
        setInputFocusBlocked(padFocusBlocked);
        const rising=(i:number)=>!browsingEpisodes.current&&!prepareAfterBrowsing.current&&
          (i===8||!padFocusBlocked)&&pad.buttons[i]?.pressed&&!buttons[i];
        if (rising(8)) {
          buttons=pad.buttons.map(b=>b.pressed);
          stop('operator_stop');
          // Back preempts this raw sample, including a simultaneously deflected stick.
          // Do not publish an active intent immediately after the Stop request.
          return;
        }
        if (rising(9) && xyz.every(x=>x===0) && gripper===0) recording('record_start');
        if (c.state.recording?.phase==='capturing') {
          if (rising(0)) recording('record_finish','success');
          if (rising(1)) recording('record_finish','failure');
        }
        if (xyz.every(x=>x===0) && gripper===0) {
          if (rising(2)) setMode(nextMode(c.mode,c.caps.orientation_assist?.version===1?preferredPositionMode.current:'translation'));
          if (rising(3)) setFrame(c.frame==='base'?'tool':'base');
        }
        buttons=pad.buttons.map(b=>b.pressed);
      }
      // Wire protocol keeps its active-input flag; no physical permission
      // modifier is needed. Only mapped motion (including gripper) activates it.
      last.current=mappedIntent(xyz,gripper,true,fine,c.mode,c.frame,c.joint,c.speed,c.caps);
      last.current.deadman=[...last.current.linear_m_s,...last.current.angular_rad_s,
        ...last.current.joint_rad_s,last.current.gripper_rate_s].some(v=>Math.abs(v)>1e-9);
      // Keep sampling the physical state and button edges while browsing so a
      // held input cannot become a new action on close. Never transmit it.
      send('intent',{intent:browsingEpisodes.current||prepareAfterBrowsing.current||padFocusBlocked||
        (source==='gamepad'&&gamepadNeedsPrepare.current)?neutral(c.frame):last.current});
      setActiveKeys([...keys.current]);
    };
    sample.current=tick;
    const timer=setInterval(tick,50);
    const down=(event:KeyboardEvent)=>{
      if(browsingEpisodes.current||prepareAfterBrowsing.current){
        // Track held keys for the preparation centering check without stealing
        // Tab, Space or text editing from the modal.
        if(source==='keyboard'&&['a','d','s','w','q','e','f','r','shift'].includes(event.key.toLowerCase()))keyDown(event.key);
        return;
      }
      if (source!=='keyboard') return;
      if(editable(event.target)){blockedKeys.current.add(event.key.toLowerCase());return;}
      const key=event.key.toLowerCase();
      if (key==='escape') {event.preventDefault();stop();return;}
      if (key==='tab' || key==='c') {
        event.preventDefault(); if(event.repeat) return;
        if (key==='tab') chooseMode(nextMode(current.current.mode,current.current.caps?.orientation_assist?.version===1?preferredPositionMode.current:'translation'));
        else chooseFrame(current.current.frame==='base'?'tool':'base');
        return;
      }
      if (['a','d','s','w','q','e','f','r','shift'].includes(key)) {
        event.preventDefault(); if(!event.repeat||keys.current.has(key))keyDown(key);
      }
    };
    const up=(event:KeyboardEvent)=>keyUp(event.key);
    const blur=()=>stop('page_blur');
    const padDisconnected=(event:GamepadEvent)=>{
      if(source!=='gamepad'||testPad.current.selected||event.gamepad.index!==current.current.gamepadIndex)return;
      // A disconnect/reconnect can reuse the same slot and ID between ticks.
      previousPadIdentity=null;buttons=[];gamepadNeedsPrepare.current=true;
      stop('gamepad_disconnected');
    };
    const hidden=()=>{if(document.hidden)stop('page_hidden');};
    const videoHealth=(event:Event)=>{
      const wasFresh=previewFresh.current;
      previewFresh.current=!!(event as CustomEvent<{fresh:boolean}>).detail.fresh;
      if(wasFresh&&!previewFresh.current&&current.current.state.ready&&current.current.caps?.video_url)stop('preview_stale');
    };
    const focus=(event:FocusEvent)=>{
      if (editable(event.target)) {for(const key of keys.current)blockedKeys.current.add(key);keys.current.clear();keyReleaseGeneration.current++;tick();}
    };
    const recordRequest=(event:Event)=>{
      const request=(event as CustomEvent<{op:'record_start'|'record_finish';result?:string;task?:string}>).detail;
      recording(request.op,request.result,request.task);
    };
    const browse=(event:Event)=>{
      const open=!!(event as CustomEvent<{open:boolean}>).detail.open;
      browsingEpisodes.current=open;
      if(open){prepareOnConnect.current=false;prepareAfterBrowsing.current=true;
        const held=[...keys.current];
        stop('browsing_recordings');
        // Stop clears the active command, but centering must still see keys
        // physically held when the dialog opened, until their keyup arrives.
        for(const key of held)keys.current.add(key);
        setNotice('浏览结束后回中并重新准备');}
    };
    window.addEventListener('keydown',down);window.addEventListener('keyup',up);
    window.addEventListener('blur',blur);document.addEventListener('visibilitychange',hidden);
    window.addEventListener('gamepaddisconnected',padDisconnected);
    document.addEventListener('focusin',focus);
    window.addEventListener('e6-video-health',videoHealth);
    window.addEventListener('e6-recording-request',recordRequest);
    window.addEventListener('e6-episode-browser',browse);
    if(isEpisodeBrowserOpen())browse(new CustomEvent('e6-episode-browser',{detail:{open:true}}));
    return ()=>{clearInterval(timer);keyReleaseGeneration.current++;sample.current=()=>{};window.removeEventListener('keydown',down);
      window.removeEventListener('keyup',up);window.removeEventListener('blur',blur);
      window.removeEventListener('gamepaddisconnected',padDisconnected);
      document.removeEventListener('visibilitychange',hidden);document.removeEventListener('focusin',focus);
      window.removeEventListener('e6-video-health',videoHealth);
      window.removeEventListener('e6-recording-request',recordRequest);
      window.removeEventListener('e6-episode-browser',browse);};
  // Capability values (including the current workspace restriction) change
  // during motion. tick reads current.current; replacing these listeners would
  // leave a passive-effect gap in which a child's cancellation keyup is lost.
  // Keep the input lifetime tied only to its owner and enabled state.
  },[enabled,source]);

  useEffect(()=>{
    // Changing page source cannot leave synthetic input armed in memory.
    setInputFocusBlocked(false);
    testPad.current={selected:false,pad:null};
    setTestPadState({selected:false,connected:false});
  },[source]);
  useEffect(()=>{
    if(!caps?.synthetic_keys&&testPad.current.selected){
      stop('input_source_changed');
      // Stay disconnected: withdrawing test capability must never select a
      // physical pad whose stick might already be deflected.
      testPad.current.pad=null;setTestPadState({selected:true,connected:false});
    }
  },[caps?.synthetic_keys]);

  const selectTestPad=(choice:'connected'|'disconnected'|'physical')=>{
    if(source!=='gamepad'||!current.current.caps?.synthetic_keys)return;
    stop(choice==='disconnected'?'gamepad_disconnected':'input_source_changed');
    testPad.current={selected:choice!=='physical',pad:choice==='connected'?acceptanceGamepad():null};
    setTestPadState({selected:choice!=='physical',connected:choice==='connected'});
    sample.current();
  };
  const setTestPadInput=(axes:number[],buttons:number[])=>{
    const c=current.current;
    if(source!=='gamepad'||!c.caps?.synthetic_keys||!testPad.current.selected||!testPad.current.pad)return false;
    const pad=acceptanceGamepad(axes,buttons);
    if((axes.some(v=>Math.abs(v)>.05)||pad.buttons[6].value>0||pad.buttons[7].value>0)&&
        (!c.state.ready||browsingEpisodes.current||prepareAfterBrowsing.current))return false;
    testPad.current.pad=pad;
    sample.current();
    return true;
  };

  const prepare=async()=>{
    if(browsingEpisodes.current){setNotice('先关闭片段浏览再准备');return;}
    // Match the existing gateway phase rule; completion never queues a prepare.
    if(preparationBlock(current.current.state))return;
    if(source==='gamepad'&&!selectedGamepad()) {
      setNotice('Selected gamepad unavailable · reconnect or select a controller');return;
    }
    sample.current();
    if (!centered()) {setNotice('松开允许动作，摇杆回中后准备');return;}
    if (ws.current?.readyState!==WebSocket.OPEN) {
      preparePadIdentity.current=gamepadIdentity();gamepadNeedsPrepare.current=false;
      prepareOnConnect.current=true;setConnectionEpoch(n=>n+1);setNotice('正在重新连接');return;
    }
    if(current.current.caps?.video_url&&!previewFresh.current){setNotice('等待实时画面恢复');return;}
    gamepadNeedsPrepare.current=false;prepareAfterBrowsing.current=false;send('prepare');setNotice('');
  };
  const initializeGripper=()=>{if(browsingEpisodes.current)return;sample.current();if(centered())send('gripper_initialize');
    else setNotice('释放允许动作并回中后初始化夹爪');};
  const enableArm=()=>{
    if(browsingEpisodes.current)return;
    sample.current();
    if(!centered()){setNotice('释放允许动作并回中后使能');return;}
    if(!current.current.state.arm?.can_enable){setNotice('当前设备状态不可使能');return;}
    send('arm_enable');setNotice('');
  };
  const restartCamera=()=>{
    if(browsingEpisodes.current)return;
    sample.current();
    if(!centered()){setNotice('释放允许动作并回中后重连相机');return;}
    if(!current.current.state.camera_restart?.can_restart){setNotice('请结束录制并等待设备静止后重连相机');return;}
    send('camera_restart');setNotice('');
  };
  const prepareBlockedReason=preparationBlock(state);
  return {enabled,caps,state,observation,source,speed,mode,frame,joint,
    notice:inputFocusBlocked?'Input focused · click outside and center to resume':notice,
    activeKeys,prepare,stop,prepareBlockedReason,
    chooseMode,chooseFrame,chooseJoint,keyDown,keyUp,initializeGripper,enableArm,restartCamera,
    testPadState,selectTestPad,setTestPadInput};
}
