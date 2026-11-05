import React from 'react';
import { act,cleanup,fireEvent,render,renderHook } from '@testing-library/react';
import { afterEach,beforeEach,describe,expect,it,vi } from 'vitest';
import { SWRConfig } from 'swr';
import { useE6Control,mappedIntent } from '../frontend/useE6Control';
import { ControlStatus } from '../frontend/ControlStatus';
import {LogConsole} from '../frontend/LogConsole';
import {announceEpisodeBrowser} from '../frontend/episodeBrowserState';
vi.mock('@/components/common/camera-stream-card',()=>({CameraStreamCard:()=>null}));

const caps={enabled:true,backend:'DOM test fixture, no device connection',linear_m_s:.03,
  angular_rad_s:Math.PI/18,joint_rad_s:Math.PI/18,gripper_rate_s:.025};
class Socket {
  static OPEN=1;
  static all:Socket[]=[];
  readyState=1;bufferedAmount=0;
  sent:any[]=[];
  onmessage:((event:{data:string})=>void)|null=null;
  onclose:(()=>void)|null=null;
  constructor(public url:string){Socket.all.push(this);}
  send(data:string){this.sent.push(JSON.parse(data));}
  close(){this.readyState=3;}
  state(extra:object={}) {this.onmessage?.({data:JSON.stringify({kind:'state',state:'可准备',
    ready:false,echo_ns:123,capabilities:caps,...extra})});}
  closed(){this.close();this.onclose?.();}
}
let pads:(Gamepad|null)[]=[];
const pad=()=>({axes:[0,0,0,0],buttons:Array.from({length:17},()=>({pressed:false,value:0})),
  connected:true,index:0,id:'fixture',mapping:'standard',timestamp:0} as unknown as Gamepad);
function boot(source:'keyboard'|'gamepad'='keyboard') {
  const cache=new Map();
  const hook=renderHook(({source})=>useE6Control(source,1,source==='gamepad'?0:null),{
    initialProps:{source},wrapper:({children})=><SWRConfig value={{provider:()=>cache,
      fallback:{'/e6/capabilities':caps},revalidateOnMount:false,revalidateOnFocus:false}}>{children}</SWRConfig>});
  const socket=Socket.all.at(-1)!;
  act(()=>socket.state());
  return {...hook,socket};
}
const intents=(s:Socket)=>s.sent.filter(m=>m.op==='intent');

it.each(['keyboard','gamepad'] as const)('%s position assist is explicit, translation-only and releases normally',async source=>{
  if(source==='gamepad')pads=[pad()];
  const {socket,result}=boot(source);
  act(()=>result.current.chooseMode('assisted'));
  expect(result.current.mode).toBe('translation');
  const assistedCaps={...caps,orientation_assist:{version:1,max_angle_deg:15,max_rate_rad_s:Math.PI/60}};
  act(()=>socket.state({ready:true,capabilities:assistedCaps}));
  act(()=>result.current.chooseMode('assisted'));
  expect(result.current.mode).toBe('assisted');
  await act(async()=>{
    if(source==='keyboard')fireEvent.keyDown(window,{key:'d'});else pads[0]!.axes[0]=1;
    await vi.advanceTimersByTimeAsync(50);
  });
  expect(intents(socket).at(-1).intent).toMatchObject({deadman:true,orientation_assist:true,
    angular_rad_s:[0,0,0],linear_m_s:[.03,0,0],assist_angular_rad_s:Math.PI/60});
  act(()=>result.current.chooseMode('rotation'));
  expect(result.current.mode).toBe('assisted');
  await act(async()=>{
    if(source==='keyboard')fireEvent.keyUp(window,{key:'d'});else pads[0]!.axes[0]=0;
    await vi.advanceTimersByTimeAsync(50);
  });
  expect(intents(socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0],angular_rad_s:[0,0,0]});
  expect(socket.sent.some(m=>m.op==='prepare'||m.op==='arm_enable')).toBe(false);
  expect(Socket.all).toHaveLength(1);
});

it('assist respects fine speed and cannot be sent to an older backend',()=>{
  expect(mappedIntent([1,0,0],0,true,false,'assisted','base',0,1,caps).deadman).toBe(false);
  const enabled={...caps,orientation_assist:{version:1,max_angle_deg:15,max_rate_rad_s:Math.PI/60}};
  const fine=mappedIntent([1,1,1],0,true,true,'assisted','tool',0,.2,enabled);
  expect(Math.hypot(...fine.linear_m_s)).toBeCloseTo(.0015);
  expect(fine.assist_angular_rad_s).toBeCloseTo(caps.angular_rad_s*.05);
  expect(fine.angular_rad_s).toEqual([0,0,0]);
});

it.each(['keyboard','gamepad'] as const)('%s mode shortcut keeps the existing rotation toggle and remembers an explicit assist choice',async source=>{
  if(source==='gamepad')pads=[pad()];
  const {socket,result}=boot(source);
  act(()=>socket.state({capabilities:{...caps,orientation_assist:{version:1,max_angle_deg:15,max_rate_rad_s:Math.PI/60}}}));
  const toggle=async()=>{
    await act(async()=>{
      if(source==='keyboard')fireEvent.keyDown(window,{key:'Tab'});else pads[0]!.buttons[2].pressed=true;
      await vi.advanceTimersByTimeAsync(50);
    });
    await act(async()=>{
      if(source==='keyboard')fireEvent.keyUp(window,{key:'Tab'});else pads[0]!.buttons[2].pressed=false;
      await vi.advanceTimersByTimeAsync(50);
    });
  };
  await toggle();expect(result.current.mode).toBe('rotation');
  await toggle();expect(result.current.mode).toBe('translation');
  act(()=>result.current.chooseMode('assisted'));
  await toggle();expect(result.current.mode).toBe('rotation');
  await toggle();expect(result.current.mode).toBe('assisted');
  expect(intents(socket).some(m=>m.intent.deadman)).toBe(false);
});

it.each(['keyboard','gamepad'] as const)('%s waits for controller-owned gripper connection without auto-preparing',async source=>{
  if(source==='gamepad')pads=[pad()];
  const {socket,result}=boot(source);
  act(()=>socket.state({gripper:{phase:'uninitialized',geometry:{fallback:true,valid:false}}}));
  expect(result.current.prepareBlockedReason).toBe('Waiting for automatic gripper connection');
  await act(async()=>result.current.prepare());
  expect(socket.sent.some(m=>m.op==='prepare'||m.op==='gripper_initialize')).toBe(false);
  act(()=>socket.state({gripper:{phase:'connecting',geometry:{fallback:true,valid:false}}}));
  expect(result.current.prepareBlockedReason).toBe('Wait for gripper connection');
  expect(socket.sent.filter(m=>m.op==='gripper_initialize')).toHaveLength(0);
  act(()=>socket.state({gripper:{phase:'ready',valid:true,geometry:{fallback:false,valid:true}}}));
  expect(result.current.prepareBlockedReason).toBeNull();
  expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
  await act(async()=>result.current.prepare());
  expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
});

it.each([['contact','Contact'],['obstructed','Blocked'],['dropped','Dropped']])('shows %s without fault, auto-initialization or motion', (position_status,label)=>{
  const {socket,result}=boot();
  act(()=>socket.state({stage:'live',ready:false,gripper:{phase:'ready',valid:true,position_status,
    actual:101/255,opening:89/243,geometry:{valid:true,fallback:false}}}));
  const view=render(<ControlStatus control={result.current}/>);
  expect(view.getByText(label)).toBeTruthy();
  if(position_status==='dropped')expect(view.getByText(label).title).toContain('Release input, then open');
  expect(result.current.prepareBlockedReason).toBeNull();
  expect(socket.sent.some(m=>m.op==='gripper_initialize'||m.op==='prepare')).toBe(false);
  expect(intents(socket).some(m=>m.intent.deadman)).toBe(false);
});

it('shows proximity slowdown and input expiry in the existing fixed status area',()=>{
  const {socket,result}=boot();
  act(()=>socket.state({ready:true,model:{status:4}}));
  const view=render(<ControlStatus control={result.current}/>);
  const status=view.getByTestId('e6-status-messages');
  expect(status.textContent).toContain('Model clearance · reduced speed');
  expect(status.className).toContain('h-10');
  act(()=>socket.state({ready:true,reason:'input_stale',model:{status:0}}));
  view.rerender(<ControlStatus control={result.current}/>);
  expect(status.textContent).toContain('Input paused · release / center to resume');
  expect(status.textContent).not.toContain('Model clearance');
  expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
});
beforeEach(()=>{
  vi.useFakeTimers();Socket.all=[];pads=[];
  vi.stubGlobal('WebSocket',Socket);
  vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(new Error('No network allowed in DOM fixture'))));
  Object.defineProperty(navigator,'getGamepads',{configurable:true,value:()=>pads});
});
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();
  localStorage.clear();
  announceEpisodeBrowser(false);
  delete (navigator as any).getGamepads;document.body.innerHTML='';});

it('log console opens, refreshes and accepts keyboard focus without a second controller or motion',async()=>{
  const {socket}=boot();act(()=>socket.state({ready:true}));
  vi.stubGlobal('fetch',vi.fn(async()=>new Response('native line\n',{headers:{'X-Log-Cursor':'cursor'}})));
  const view=render(<LogConsole/>);const count=Socket.all.length;
  fireEvent.click(view.getByRole('button',{name:'Logs'}));
  await act(async()=>{await vi.advanceTimersByTimeAsync(600);});
  const output=view.getByLabelText('Program output');act(()=>output.focus());
  fireEvent.keyDown(output,{key:' '});fireEvent.keyDown(output,{key:'d'});
  await act(async()=>{await vi.advanceTimersByTimeAsync(100);});
  fireEvent.keyUp(output,{key:'d'});fireEvent.keyUp(output,{key:' '});
  fireEvent.click(view.getByRole('button',{name:'Pause follow'}));
  fireEvent.click(view.getByLabelText('Clear display'));
  fireEvent.click(view.getByRole('button',{name:'Logs'}));
  expect(Socket.all).toHaveLength(count);
  expect(intents(socket).every(m=>!m.intent.deadman)).toBe(true);
  expect(socket.sent.some(m=>['prepare','enable','gripper_init'].includes(m.op))).toBe(false);
});

describe('formal input hook with DOM-only transport fixture',()=>{
  it.each(['keyboard','gamepad'] as const)('%s preparation waits for recording work and never resumes automatically',async source=>{
    if(source==='gamepad')pads=[pad()];
    const {result,socket}=boot(source);
    for(const phase of ['starting','sealing']) {
      act(()=>socket.state({recording:{phase}}));
      expect(result.current.prepareBlockedReason).toBeTruthy();
      await act(async()=>result.current.prepare());
    }
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    act(()=>{socket.state({recording:{phase:'transfer_pending'}});vi.advanceTimersByTime(100);});
    expect(result.current.prepareBlockedReason).toBeNull();
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    await act(async()=>result.current.prepare());
    expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
    act(()=>socket.state({recording:{phase:'capturing'}}));
    expect(result.current.prepareBlockedReason).toBeNull();
  });
  it('a reconnect during sealing waits for explicit preparation once transfer begins',async()=>{
    const {result,socket}=boot();
    act(()=>socket.closed());
    await act(async()=>result.current.prepare());
    const reconnected=Socket.all.at(-1)!;
    expect(reconnected).not.toBe(socket);
    act(()=>reconnected.state({recording:{phase:'sealing'}}));
    const view=render(<ControlStatus control={result.current}/>);
    expect(result.current.prepareBlockedReason).toBeTruthy();
    expect(view.getByTestId('e6-status-messages').textContent).toContain('Saving recording · wait to prepare');
    expect(reconnected.sent.some(m=>m.op==='prepare')).toBe(false);
    act(()=>reconnected.state({recording:{phase:'transfer_pending',error:'Transfer retry pending'}}));
    view.rerender(<ControlStatus control={result.current}/>);
    expect(result.current.prepareBlockedReason).toBeNull();
    expect(view.getByTestId('e6-status-messages').textContent).not.toContain('wait to prepare');
    expect(reconnected.sent.some(m=>m.op==='prepare')).toBe(false);
    await act(async()=>result.current.prepare());
    expect(reconnected.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
  });
  it.each(['keyboard','gamepad'] as const)('%s recording takes an immutable annotation snapshot through the formal socket',async source=>{
    if(source==='gamepad')pads=[pad()];
    const {socket}=boot(source);
    localStorage.setItem('e6-recording-task','local test');
    localStorage.setItem('e6-recording-context',JSON.stringify({scene:'table',operator:'auto'}));
    act(()=>{
      if(source==='keyboard')window.dispatchEvent(new CustomEvent('e6-recording-request',{detail:{op:'record_start'}}));
      else {pads[0]!.buttons[9].pressed=true;vi.advanceTimersByTime(50);}
    });
    const starts=socket.sent.filter(m=>m.op==='record_start');
    expect(starts).toHaveLength(1);
    expect(starts[0]).toMatchObject({task:'local test',research_context:{scene:'table',operator:'auto'}});
    localStorage.setItem('e6-recording-context',JSON.stringify({scene:'later'}));
    expect(starts[0].research_context.scene).toBe('table');
  });
  it.each(['keyboard','gamepad'] as const)('a fragment browser opened before %s connects blocks preparation and held input',async source=>{
    announceEpisodeBrowser(true);
    if(source==='gamepad')pads=[pad()];
    const {result,socket}=boot(source);
    await act(async()=>result.current.prepare());
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    act(()=>{
      if(source==='keyboard'){fireEvent.keyDown(window,{key:' '});fireEvent.keyDown(window,{key:'d'});}
      else {pads[0]!.buttons[4].pressed=true;pads[0]!.axes[0]=1;}
      vi.advanceTimersByTime(100);
    });
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
    act(()=>announceEpisodeBrowser(false));
    await act(async()=>result.current.prepare());
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    act(()=>{
      if(source==='keyboard'){fireEvent.keyUp(window,{key:' '});fireEvent.keyUp(window,{key:'d'});}
      else {pads[0]!.buttons[4].pressed=false;pads[0]!.axes[0]=0;}
      vi.advanceTimersByTime(50);
    });
    await act(async()=>result.current.prepare());
    expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
  });
  it.each(['keyboard','gamepad'] as const)('episode browsing stops %s and consumes held input until explicit centered preparation',async source=>{
    if(source==='gamepad')pads=[pad()];
    const {result,socket}=boot(source);act(()=>socket.state({ready:true,recording:{phase:'capturing'}}));
    const browse=(open:boolean)=>window.dispatchEvent(new CustomEvent('e6-episode-browser',{detail:{open}}));
    act(()=>browse(true));
    expect(socket.sent.at(-1)).toMatchObject({op:'stop',reason:'browsing_recordings'});
    const start=socket.sent.length;
    act(()=>{
      if(source==='keyboard'){fireEvent.keyDown(window,{key:' '});fireEvent.keyDown(window,{key:'d'});}
      else {pads[0]!.axes[0]=1;pads[0]!.buttons[4].pressed=true;pads[0]!.buttons[0].pressed=true;}
      vi.advanceTimersByTime(150);
    });
    await act(async()=>result.current.prepare());
    act(()=>{browse(false);vi.advanceTimersByTime(150);});
    await act(async()=>result.current.prepare()); // still held, no prepare
    expect(socket.sent.slice(start).every(m=>m.op==='intent'&&!m.intent.deadman)).toBe(true);
    act(()=>{
      if(source==='keyboard'){fireEvent.keyUp(window,{key:' '});fireEvent.keyUp(window,{key:'d'});}
      else {pads[0]!.axes[0]=0;pads[0]!.buttons[4].pressed=false;}
      vi.advanceTimersByTime(50);
    });
    await act(async()=>result.current.prepare());
    expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
    act(()=>vi.advanceTimersByTime(100));
    expect(socket.sent.some(m=>m.op==='record_finish')).toBe(false); // A held across modal close is consumed.
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
    act(()=>{
      if(source==='keyboard'){fireEvent.keyDown(window,{key:'d'});fireEvent.keyDown(window,{key:' '});}
      else {pads[0]!.axes[0]=1;pads[0]!.buttons[4].pressed=true;}
      vi.advanceTimersByTime(50);
    });
    expect(intents(socket).at(-1).intent).toMatchObject({deadman:true,linear_m_s:[.03,0,0]});
  });
  it('opening the browser during a held keyboard command cannot erase the centering requirement',async()=>{
    const {result,socket}=boot();act(()=>socket.state({ready:true}));
    act(()=>{result.current.keyDown('d');result.current.keyDown(' ');});
    act(()=>{
      window.dispatchEvent(new CustomEvent('e6-episode-browser',{detail:{open:true}}));
      window.dispatchEvent(new CustomEvent('e6-episode-browser',{detail:{open:false}}));
    });
    await act(async()=>result.current.prepare());
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
    act(()=>{result.current.keyUp(' ');result.current.keyUp('d');});
    await act(async()=>result.current.prepare());
    expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
  });
  it('does not present cached stage or speed as current after disconnect',async()=>{
    const {result,socket}=boot();
    act(()=>socket.state({stage:'shadow',capabilities:{...caps,backend:'E6 shadow'}}));
    const view=render(<ControlStatus control={result.current}/>);
    expect(view.getByTestId('e6-backend').textContent).toBe('E6 shadow');
    expect(view.getByTestId('e6-limits').textContent).toContain('30.0 mm/s');
    act(()=>socket.closed());
    view.rerender(<ControlStatus control={result.current}/>);
    expect(result.current.enabled).toBe(true); // Keep E6 routing, never upstream fallback.
    expect(view.getByTestId('e6-backend').textContent).toBe('E6 offline');
    expect(view.getByTestId('e6-limits').textContent).not.toContain('30.0');
    expect(view.getByTestId('e6-limits').textContent).toContain('Speed unavailable');
    // A new authoritative handshake updates the label; it is not preparation.
    socket.readyState=1;
    await act(async()=>socket.state({stage:'live',capabilities:{...caps,backend:'E6 live',linear_m_s:.005}}));
    view.rerender(<ControlStatus control={result.current}/>);
    expect(view.getByTestId('e6-backend').textContent).toBe('E6 live');
    expect(view.getByTestId('e6-limits').textContent).toContain('5.0 mm/s');
    expect(result.current.state.ready).toBe(false);
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
  });
  it.each(['translation','rotation'])('samples simultaneous held %s keys as one complete vector',async mode=>{
    const {result,socket}=boot();
    act(()=>result.current.chooseMode(mode as 'translation'|'rotation'));
    act(()=>{fireEvent.keyDown(window,{key:' '});});
    const start=socket.sent.length;
    await act(async()=>{
      for(const key of ['d','w','e'])fireEvent.keyDown(window,{key});
      await Promise.resolve();
    });
    const initial=socket.sent.slice(start).filter(m=>m.op==='intent');
    expect(initial).toHaveLength(1);
    const channel=mode==='translation'?'linear_m_s':'angular_rad_s';
    const scale=mode==='translation'?caps.linear_m_s:caps.angular_rad_s;
    initial[0].intent[channel].forEach((x:number)=>expect(x).toBeCloseTo(scale/Math.sqrt(3)));
    const begin=socket.sent.length;
    await act(async()=>{
      for(const key of ['d','w','e'])fireEvent.keyUp(window,{key});
      for(const key of ['a','s','q'])fireEvent.keyDown(window,{key});
      await Promise.resolve();
    });
    const reversed=socket.sent.slice(begin).filter(m=>m.op==='intent');
    expect(reversed).toHaveLength(1);
    reversed[0].intent[channel].forEach((x:number)=>expect(x).toBeCloseTo(-scale/Math.sqrt(3)));
    const end=socket.sent.length;
    await act(async()=>{
      for(const key of ['a','s','q'])fireEvent.keyUp(window,{key});
      await Promise.resolve();
      expect(socket.sent.at(-1).intent.deadman).toBe(false);
    });
    expect(socket.sent.slice(end).filter(m=>m.op==='intent')).toHaveLength(1);
  });
  it('coalesces same-task reversal without a zero intent; lone axis release is still prompt',async()=>{
    const {result,socket}=boot();act(()=>socket.state({ready:true}));
    act(()=>{result.current.keyDown('d');result.current.keyDown(' ');});
    const begin=socket.sent.length;
    await act(async()=>{
      result.current.keyUp('d');result.current.keyDown('a');await Promise.resolve();
    });
    expect(socket.sent.slice(begin).filter(m=>m.op==='intent').map(m=>m.intent.linear_m_s)).toEqual([[-.03,0,0]]);
    const release=socket.sent.length;
    await act(async()=>{result.current.keyUp('a');await Promise.resolve();});
    expect(socket.sent.slice(release).filter(m=>m.op==='intent').map(m=>m.intent)).toEqual([
      expect.objectContaining({deadman:false,linear_m_s:[0,0,0]})]);
  });
  it.each(['escape','blur','unmount'])('does not replay a deferred axis after %s',async reason=>{
    const {result,socket,unmount}=boot();act(()=>socket.state({ready:true}));
    act(()=>{result.current.keyDown('d');result.current.keyDown(' ');});
    let after=0;
    await act(async()=>{
      result.current.keyUp('d');
      result.current.keyDown('a');
      if(reason==='escape')fireEvent.keyDown(window,{key:'Escape'});
      if(reason==='blur')fireEvent.blur(window);
      if(reason==='unmount')unmount();
      after=socket.sent.length;
      await Promise.resolve();
    });
    expect(socket.sent).toHaveLength(after);
  });
  it('camera restart requires neutral input and never prepares after completion',()=>{
    const {result,socket}=boot();
    act(()=>socket.state({camera_restart:{available:true,phase:'idle',can_restart:false}}));
    act(()=>result.current.restartCamera());
    expect(socket.sent.some(m=>m.op==='camera_restart')).toBe(false);
    act(()=>socket.state({camera_restart:{available:true,phase:'idle',can_restart:true}}));
    act(()=>{result.current.keyDown('d');vi.advanceTimersByTime(50);});
    act(()=>result.current.restartCamera());
    expect(socket.sent.some(m=>m.op==='camera_restart')).toBe(false);
    act(()=>{result.current.keyUp('d');vi.advanceTimersByTime(50);});
    act(()=>result.current.restartCamera());
    expect(socket.sent.filter(m=>m.op==='camera_restart')).toHaveLength(1);
    act(()=>socket.state({camera_restart:{available:true,phase:'complete',can_restart:true},ready:false}));
    act(()=>vi.advanceTimersByTime(200));
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    expect(result.current.state.ready).toBe(false);
  });
  it('returns four clock timestamps without changing input, readiness or source',()=>{
    const {result,socket}=boot();act(()=>socket.state({ready:true}));
    act(()=>{result.current.keyDown(' ');result.current.keyDown('d');});
    const before=intents(socket).length;
    const probe={kind:'clock_probe',client_sent_ms:10,x5_received_ns:1110000000,x5_sent_ns:1112000000};
    act(()=>socket.onmessage?.({data:JSON.stringify(probe)}));
    expect(socket.sent.at(-1)).toMatchObject({...probe,op:'clock_sample'});
    expect(Number.isFinite(socket.sent.at(-1).client_received_ms)).toBe(true);
    expect(intents(socket)).toHaveLength(before);
    expect(result.current.state.ready).toBe(true);
    act(()=>vi.advanceTimersByTime(50));
    expect(intents(socket).at(-1).intent).toMatchObject({deadman:true,linear_m_s:[.03,0,0]});
  });
  it('100 direct start/release cycles preserve preparation without Space',async()=>{
    const {result,socket}=boot();act(()=>socket.state({ready:true}));
    for(let i=0;i<100;i++) {
      act(()=>{result.current.keyDown('d');vi.advanceTimersByTime(50);});
      expect(intents(socket).at(-1).intent).toMatchObject({deadman:true,linear_m_s:[.03,0,0]});
      await act(async()=>{result.current.keyUp('d');await Promise.resolve();});
      expect(intents(socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
      expect(result.current.state.ready).toBe(true);
    }
    expect(socket.sent.filter(m=>m.op==='stop')).toHaveLength(0);
    expect(Socket.all).toHaveLength(1);
  });

  it('20 released mode/frame switches work and a held input cannot switch',()=>{
    const {result,socket}=boot();
    for(let i=0;i<20;i++) {
      const mode=i%2?'translation':'rotation',frame=i%2?'base':'tool';
      act(()=>{result.current.chooseMode(mode);result.current.chooseFrame(frame);});
      expect(result.current.mode).toBe(mode);expect(result.current.frame).toBe(frame);
    }
    act(()=>{result.current.keyDown('d');vi.advanceTimersByTime(50);});
    act(()=>{result.current.chooseMode('joint');result.current.chooseFrame('tool');});
    expect(result.current.mode).toBe('translation');expect(result.current.frame).toBe('base');
    expect(socket.sent.filter(m=>m.op==='stop')).toHaveLength(0);
  });

  it('10 closed connections require explicit reconnect preparation and no stale axes',()=>{
    const {result}=boot();
    for(let i=0;i<10;i++) {
      const old=Socket.all.at(-1)!;
      act(()=>{old.state({ready:true});result.current.keyDown(' ');result.current.keyDown('d');});
      act(()=>old.closed());expect(result.current.state.ready).toBe(false);
      expect(Socket.all).toHaveLength(i+1);
      act(()=>{void result.current.prepare();});
      const next=Socket.all.at(-1)!;expect(next).not.toBe(old);
      expect(next.sent).toHaveLength(0);
      act(()=>next.state());
      expect(next.sent.map(m=>m.op)).toEqual(['intent','prepare']);
      expect(intents(next).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
      expect(result.current.state.ready).toBe(false);
    }
  });

  it('text/select focus releases keyboard input and ignores control shortcuts until focus leaves',async()=>{
    const {result,socket}=boot();
    for(const tag of ['input','textarea','select']) {
      const field=document.createElement(tag);document.body.appendChild(field);
      act(()=>{result.current.keyDown(' ');result.current.keyDown('d');});
      act(()=>{field.focus();fireEvent.keyDown(field,{key:' '});fireEvent.keyDown(field,{key:'d'});});
      expect(intents(socket).at(-1).intent.deadman).toBe(false);
      await act(async()=>{field.blur();fireEvent.keyUp(window,{key:'d'});fireEvent.keyDown(window,{key:'d'});await Promise.resolve();});
      expect(intents(socket).at(-1).intent.linear_m_s[0]).toBe(.03);
      act(()=>{fireEvent.keyUp(window,{key:'d'});fireEvent.keyUp(window,{key:' '});});
      field.remove();
    }
  });

  it.each(['input','select','workspace','logs'])('isolates held gamepad input and shortcuts while %s has focus',async kind=>{
    pads=[pad()];const {result,socket}=boot('gamepad');
    act(()=>socket.state({ready:true,recording:{phase:'capturing'}}));
    pads[0]!.buttons[4].pressed=true;pads[0]!.axes[0]=1;
    act(()=>vi.advanceTimersByTime(50));
    expect(intents(socket).at(-1).intent.deadman).toBe(true);
    const field=document.createElement(['input','select'].includes(kind)?kind:'div');
    if(kind==='workspace'||kind==='logs'){field.setAttribute('data-e6-input-isolation','');field.tabIndex=0;}
    document.body.append(field);
    act(()=>field.focus());
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
    const start=socket.sent.length;
    act(()=>{pads[0]!.buttons[4].pressed=false;pads[0]!.axes[0]=0;
      for(const i of [0,2,3,9])pads[0]!.buttons[i].pressed=true;
      vi.advanceTimersByTime(100);});
    expect(socket.sent.slice(start).some(m=>m.op.startsWith('record_'))).toBe(false);
    expect(result.current.mode).toBe('translation');expect(result.current.frame).toBe('base');
    act(()=>{pads[0]!.buttons[4].pressed=true;pads[0]!.axes[0]=1;field.blur();vi.advanceTimersByTime(100);});
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
    await act(async()=>result.current.prepare());
    expect(socket.sent.slice(start).some(m=>m.op==='prepare')).toBe(false);
    act(()=>{pads[0]!.buttons[4].pressed=false;pads[0]!.axes[0]=0;vi.advanceTimersByTime(50);});
    expect(socket.sent.slice(start).some(m=>m.op.startsWith('record_'))).toBe(false);
    act(()=>{pads[0]!.buttons[4].pressed=true;pads[0]!.axes[0]=1;vi.advanceTimersByTime(50);});
    expect(intents(socket).at(-1).intent.linear_m_s[0]).toBe(.03);
    expect(Socket.all).toHaveLength(1);
  });

  it('gamepad disconnect stops and reconnect does not prepare automatically',()=>{
    pads=[pad()];const {result,socket}=boot('gamepad');
    act(()=>socket.state({ready:true}));
    (pads[0]!.buttons[4] as any).pressed=true;(pads[0]!.axes as number[])[0]=1;
    act(()=>vi.advanceTimersByTime(50));
    expect(intents(socket).at(-1).intent.linear_m_s[0]).toBe(.03);
    pads=[];act(()=>vi.advanceTimersByTime(50));
    expect(socket.sent.find(m=>m.op==='stop')?.reason).toBe('gamepad_disconnected');
    expect(socket.sent.filter(m=>m.op==='stop')).toHaveLength(1);
    act(()=>socket.state({ready:false}));pads=[pad()];act(()=>vi.advanceTimersByTime(100));
    expect(result.current.state.ready).toBe(false);
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
  });

  it('does not prepare a missing or explicitly disconnected selected gamepad',async()=>{
    pads=[null,pad()];const {result,socket}=boot('gamepad');
    await act(async()=>result.current.prepare());
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    expect(result.current.notice).toContain('gamepad');
    pads=[{...pad(),connected:false}];
    await act(async()=>result.current.prepare());
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
  });

  it('device replacement in the same slot stops and needs a new centered Prepare',async()=>{
    pads=[pad()];const {result,socket}=boot('gamepad');
    act(()=>socket.state({ready:true}));act(()=>vi.advanceTimersByTime(50));
    const start=socket.sent.length;
    pads=[{...pad(),id:'replacement'}];pads[0]!.axes[0]=1;
    act(()=>vi.advanceTimersByTime(100));
    expect(socket.sent.slice(start).some(m=>m.op==='stop'&&m.reason==='input_source_changed')).toBe(true);
    expect(intents(socket).slice(-2).every(m=>!m.intent.deadman)).toBe(true);
    await act(async()=>result.current.prepare());
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    pads[0]!.axes[0]=0;act(()=>vi.advanceTimersByTime(100));
    await act(async()=>result.current.prepare());
    expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
    expect(Socket.all).toHaveLength(1);
  });

  it('handles same-slot reconnect between polls and releases its disconnect listener on unmount',async()=>{
    pads=[pad()];const {result,socket,unmount}=boot('gamepad');
    act(()=>socket.state({ready:true}));
    const event=new Event('gamepaddisconnected');Object.assign(event,{gamepad:pad()});
    act(()=>window.dispatchEvent(event));
    expect(socket.sent.at(-1)).toMatchObject({op:'stop',reason:'gamepad_disconnected'});
    pads=[pad()];pads[0]!.axes[0]=1;act(()=>vi.advanceTimersByTime(50));
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
    pads[0]!.axes[0]=0;await act(async()=>result.current.prepare());
    expect(socket.sent.filter(m=>m.op==='prepare')).toHaveLength(1);
    pads[0]!.axes[0]=1;act(()=>vi.advanceTimersByTime(50));
    expect(intents(socket).at(-1).intent.linear_m_s[0]).toBe(.03);
    unmount();const count=socket.sent.length;act(()=>window.dispatchEvent(event));
    expect(socket.sent).toHaveLength(count);
  });

  it.each(['missing','replacement','blur'])('cancels pending socket preparation after %s',async change=>{
    pads=[pad()];const {result,socket}=boot('gamepad');
    act(()=>socket.closed());await act(async()=>result.current.prepare());
    const next=Socket.all.at(-1)!;expect(next).not.toBe(socket);
    if(change==='missing')pads=[];
    if(change==='replacement')pads=[{...pad(),id:'replacement'}];
    if(change==='blur')act(()=>fireEvent.blur(window));
    act(()=>next.state());
    expect(next.sent.some(m=>m.op==='prepare')).toBe(false);
    act(()=>vi.advanceTimersByTime(100));
    expect(intents(next).every(m=>!m.intent.deadman)).toBe(true);
  });

  it('late close/message from a replaced source cannot erase the new session',()=>{
    const {result,rerender}=boot('keyboard');pads=[pad()];
    for(let i=0;i<20;i++) {
      const old=Socket.all.at(-1)!;const source=i%2?'keyboard':'gamepad';
      rerender({source});const next=Socket.all.at(-1)!;
      act(()=>next.state({ready:true}));
      if(source==='keyboard')act(()=>{result.current.keyDown(' ');result.current.keyDown('d');});
      act(()=>old.closed());
      expect(result.current.state.ready).toBe(true);
      act(()=>old.state({ready:false,state:'过期旧连接'}));
      expect(result.current.state.ready).toBe(true);
      expect(result.current.state.state).not.toBe('过期旧连接');
      if(source==='keyboard') {
        act(()=>vi.advanceTimersByTime(50));
        expect(intents(next).at(-1).intent).toMatchObject({deadman:true,linear_m_s:[.03,0,0]});
      }
    }
  });

  it('page blur sends stop and clears held keyboard axes',()=>{
    const {result,socket}=boot();
    act(()=>{socket.state({ready:true});result.current.keyDown(' ');result.current.keyDown('d');});
    act(()=>fireEvent.blur(window));
    expect(socket.sent.at(-1)).toMatchObject({op:'stop',reason:'page_blur'});
    act(()=>vi.advanceTimersByTime(50));
    expect(intents(socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
  });

  it('a rejected second tab retains its reason after close and clears it on explicit recovery',()=>{
    const {result,socket}=boot();
    act(()=>socket.state({ready:false,state:'暂挂',reason:'已有控制者'}));
    act(()=>socket.closed());
    expect(result.current.state.reason).toBe('已有控制者');
    expect(result.current.state.ready).toBe(false);
    act(()=>{void result.current.prepare();});
    const next=Socket.all.at(-1)!;
    act(()=>next.state());
    expect(result.current.state.reason).toBeUndefined();
    expect(next.sent.map(m=>m.op)).toEqual(['intent','prepare']);
  });
});

describe('bounded acceptance inputs through the formal keyboard handler',()=>{
  function panel(ready=true,synthetic=true) {
    const testCaps={...caps,synthetic_keys:synthetic};const cache=new Map();
    function Panel(){const control=useE6Control('keyboard',1);return <ControlStatus control={control}/>;}
    const view=render(<SWRConfig value={{provider:()=>cache,fallback:{'/e6/capabilities':testCaps},
      revalidateOnMount:false,revalidateOnFocus:false}}><Panel/></SWRConfig>);
    const socket=Socket.all.at(-1)!;
    act(()=>socket.state({ready,capabilities:testCaps}));
    return {...view,socket,testCaps};
  }
  it('releases a fixture when refreshed capabilities and a boundary arrive together',async()=>{
    const v=panel();
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'w'}});
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：1秒后释放'}));await Promise.resolve();});
    await act(async()=>{await vi.advanceTimersByTimeAsync(400);});
    await act(async()=>v.socket.state({ready:true,reason:'模型碰撞边界：当前方向已停止',
      capabilities:{...v.testCaps,workspace:{mode:'model',assembly_id:'refreshed'}}}));
    await act(async()=>{await vi.advanceTimersByTimeAsync(2000);});
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
  });
    it.each([['wf',1],['sr',-1]] as const)('bounds combined arm/gripper %s input and releases both',async(key,sign)=>{
      const v=panel();fireEvent.change(v.getByLabelText('验收方向'),{target:{value:key}});
      expect(v.queryByRole('option',{name:'3秒（仅夹爪）'})).toBeNull();
      expect((v.getByRole('button',{name:'连续往返：6轮'}) as HTMLButtonElement).disabled).toBe(true);
      await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：1秒后释放'}));await Promise.resolve();});
      await act(async()=>{await vi.advanceTimersByTimeAsync(100);});
      const input=intents(v.socket).at(-1).intent;
      expect(input.deadman).toBe(true);expect(input.linear_m_s).toEqual([0,sign*.03,0]);
      expect(input.gripper_rate_s).toBe(sign*.025);
      await act(async()=>{await vi.advanceTimersByTimeAsync(900);});
      expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0],gripper_rate_s:0});
    });
    it.each(['f','r'])('cancels a three-second %s gripper fixture on video stop and can restart centered',async key=>{
    const v=panel();
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:key}});
    fireEvent.change(v.getByLabelText('验收持续时间'),{target:{value:'3000'}});
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：3秒后释放'}));await Promise.resolve();});
    await act(async()=>{await vi.advanceTimersByTimeAsync(150);});
    expect(intents(v.socket).at(-1).intent.deadman).toBe(true);
    await act(async()=>v.socket.state({ready:false,reason:'preview_stale',capabilities:v.testCaps}));
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,gripper_rate_s:0});
    const offset=v.socket.sent.length;
    await act(async()=>{await vi.advanceTimersByTimeAsync(4000);});
    expect(v.socket.sent.slice(offset).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
    await act(async()=>v.socket.state({ready:true,reason:null,capabilities:v.testCaps}));
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：3秒后释放'}));await Promise.resolve();});
    expect(intents(v.socket).at(-1).intent.deadman).toBe(true);
    await act(async()=>{await vi.advanceTimersByTimeAsync(3000);});
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,gripper_rate_s:0});
  });
  it('continuous XYZ reversal never emits partial-axis intermediate intents',async()=>{
    const v=panel();
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'dwe'}});
    fireEvent.click(v.getByLabelText('对称往返（首尾半周期）'));
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'连续往返：6轮'}));await Promise.resolve();});
    for(let phase=0;phase<13;phase++) {
      await act(async()=>{await vi.advanceTimersByTimeAsync(phase===0||phase===12?500:1000);});
    }
    const active=intents(v.socket).filter(m=>m.intent.deadman);
    for(const {intent} of active) {
      expect(intent.linear_m_s[0]).not.toBe(0);
      expect(intent.linear_m_s[1]).toBe(intent.linear_m_s[0]);
      expect(intent.linear_m_s[2]).toBe(intent.linear_m_s[0]);
      expect(Math.hypot(...intent.linear_m_s)).toBeCloseTo(.03);
    }
    const phases=active.filter((m,i)=>!i||m.intent.linear_m_s[0]!==active[i-1].intent.linear_m_s[0]);
    expect(phases).toHaveLength(13);
    expect(intents(v.socket).at(-1).intent.deadman).toBe(false);
  });
  it.each([100,200,500])('sets a complete fine diagonal before deadman and releases it at %i ms',async duration=>{
    const v=panel();
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'dwe'}});
    fireEvent.change(v.getByLabelText('验收持续时间'),{target:{value:String(duration)}});
    fireEvent.click(v.getByLabelText('验收精细25%'));
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:`合成键盘：${duration/1000}秒后释放`}));await Promise.resolve();});
    const active=intents(v.socket).filter(m=>m.intent.deadman);
    expect(active).toHaveLength(1);
    for(const x of active[0].intent.linear_m_s)expect(x).toBeCloseTo(.0075/Math.sqrt(3));
    await act(async()=>{await vi.advanceTimersByTimeAsync(duration-1);});
    expect(intents(v.socket).at(-1).intent.deadman).toBe(true);
    const before=v.socket.sent.length;
    await act(async()=>{await vi.advanceTimersByTimeAsync(1);});
    // The regular 20Hz tick can run first at this same timestamp. Once the
    // timeout releases deadman, removing axes must never reactivate motion.
    const tail=v.socket.sent.slice(before).filter(m=>m.op==='intent');
    const released=tail.findIndex(m=>!m.intent.deadman);expect(released).toBeGreaterThanOrEqual(0);
    expect(tail.slice(released).every(m=>!m.intent.deadman)).toBe(true);
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
    expect(v.socket.sent.some(m=>m.op==='stop')).toBe(false);
    expect((v.getByRole('button',{name:'连续往返：6轮'}) as HTMLButtonElement).disabled).toBe(duration<500);
  });
  it('keeps deadman across twelve continuous phases and releases without delayed keys',async()=>{
    const v=panel();await act(async()=>{fireEvent.click(v.getByRole('button',{name:'连续往返：6轮'}));await Promise.resolve();});
    const begin=v.socket.sent.length;
    for(let phase=1;phase<12;phase++){
      await act(async()=>{await vi.advanceTimersByTimeAsync(1000);});
      expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:true,linear_m_s:[phase%2?-.03:.03,0,0]});
    }
    expect(v.socket.sent.slice(begin).filter(m=>m.op==='intent').every(m=>m.intent.deadman)).toBe(true);
    expect(v.socket.sent.slice(begin).filter(m=>m.op==='intent').every(m=>Math.abs(m.intent.linear_m_s[0])===.03)).toBe(true);
    await act(async()=>{await vi.advanceTimersByTimeAsync(1000);});
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
    const end=v.socket.sent.length;await act(async()=>{await vi.advanceTimersByTimeAsync(5000);});
    expect(v.socket.sent.slice(end).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
  });
  it.each(['blur','unready','boundary','cancel','escape'])('cancels all continuous reversals on %s',async reason=>{
    const v=panel();await act(async()=>{fireEvent.click(v.getByRole('button',{name:'连续往返：6轮'}));await Promise.resolve();});
    await act(async()=>{await vi.advanceTimersByTimeAsync(1200);});
    if(reason==='blur')act(()=>fireEvent.blur(window));
    if(reason==='unready')await act(async()=>v.socket.state({ready:false,capabilities:v.testCaps}));
    if(reason==='boundary')await act(async()=>v.socket.state({ready:true,reason:'当前测试边界：可向范围内退回',capabilities:v.testCaps}));
    if(reason==='cancel')fireEvent.click(v.getByRole('button',{name:'结束验收输入'}));
    if(reason==='escape')fireEvent.keyDown(window,{key:'Escape'});
    const end=v.socket.sent.length;await act(async()=>{await vi.advanceTimersByTimeAsync(15000);});
    expect(v.socket.sent.slice(end).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
    expect(intents(v.socket).at(-1).intent.linear_m_s).toEqual([0,0,0]);
  });
  it.each([500,1000])('uses the selected %i ms for every rotation phase and excludes gripper sequences',async phaseMs=>{
    const v=panel();fireEvent.mouseDown(v.getByRole('tab',{name:'Rotation',exact:true}),{button:0,ctrlKey:false});
    fireEvent.change(v.getByLabelText('验收持续时间'),{target:{value:String(phaseMs)}});
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'连续往返：6轮'}));await Promise.resolve();});
    await act(async()=>{await vi.advanceTimersByTimeAsync(phaseMs);});
    expect(intents(v.socket).at(-1).intent.angular_rad_s[0]).toBeCloseTo(-caps.angular_rad_s);
    await act(async()=>{await vi.advanceTimersByTimeAsync(phaseMs*11);});
    expect(intents(v.socket).at(-1).intent.deadman).toBe(false);
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'r'}});
    expect((v.getByRole('button',{name:'连续往返：6轮'}) as HTMLButtonElement).disabled).toBe(true);
  });
  it.each([500,1000])('centered rotation preserves full speed, total duration and zero input integral at %i ms',async phaseMs=>{
    const v=panel();fireEvent.mouseDown(v.getByRole('tab',{name:'Rotation',exact:true}),{button:0,ctrlKey:false});
    fireEvent.change(v.getByLabelText('验收持续时间'),{target:{value:String(phaseMs)}});
    fireEvent.click(v.getByLabelText('对称往返（首尾半周期）'));
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'连续往返：6轮'}));await Promise.resolve();});
    let integral=0,total=0;
    for(let phase=0;phase<13;phase++) {
      const intent=intents(v.socket).at(-1).intent;
      expect(intent.deadman).toBe(true);
      expect(intent.angular_rad_s[0]).toBeCloseTo((phase%2?-1:1)*caps.angular_rad_s);
      const duration=(phase===0||phase===12)?phaseMs/2:phaseMs;
      integral+=intent.angular_rad_s[0]*duration;total+=duration;
      await act(async()=>{await vi.advanceTimersByTimeAsync(duration);});
    }
    expect(total).toBe(12*phaseMs);expect(integral).toBeCloseTo(0);
    expect(intents(v.socket).at(-1).intent.deadman).toBe(false);
    const end=v.socket.sent.length;await act(async()=>{await vi.advanceTimersByTimeAsync(5000);});
    expect(v.socket.sent.slice(end).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
  });
  it('uses the selected rotation and tool frame and never rearms after blur',async()=>{
    const v=panel();fireEvent.mouseDown(v.getByRole('tab',{name:'Rotation',exact:true}),{button:0,ctrlKey:false});
    fireEvent.click(v.getByRole('combobox',{name:'Reference frame'}));fireEvent.click(v.getByRole('option',{name:'Tool frame'}));
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'as'}});
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：1秒后释放'}));await Promise.resolve();});
    const intent=intents(v.socket).at(-1).intent;
    expect(intent.frame).toBe('tool');expect(intent.linear_m_s).toEqual([0,0,0]);
    expect(intent.angular_rad_s[0]).toBeCloseTo(-caps.angular_rad_s/Math.sqrt(2));
    expect(intent.angular_rad_s[1]).toBeCloseTo(-caps.angular_rad_s/Math.sqrt(2));
    act(()=>fireEvent.blur(window));const after=v.socket.sent.length;
    await act(async()=>{await vi.advanceTimersByTimeAsync(1500);});
    expect(v.socket.sent.some(m=>m.op==='stop'&&m.reason==='page_blur')).toBe(true);
    expect(v.socket.sent.slice(after).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
  });
  it('cancels on lost readiness and unmount, without leaving scheduled key events',async()=>{
    const v=panel();await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：1秒后释放'}));await Promise.resolve();});
    await act(async()=>v.socket.state({ready:false,capabilities:v.testCaps}));
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
    await act(async()=>v.socket.state({ready:true,capabilities:v.testCaps}));
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：1秒后释放'}));await Promise.resolve();});
    v.unmount();const after=v.socket.sent.length;
    await act(async()=>{await vi.advanceTimersByTimeAsync(1500);});expect(v.socket.sent).toHaveLength(after);
  });
  it('requires explicit acceptance capability and a ready session',async()=>{
    const v=panel(false);expect((v.getByRole('button',{name:'合成键盘：1秒后释放'}) as HTMLButtonElement).disabled).toBe(true);
    v.unmount();const disabled=panel(true,false);
    expect(disabled.queryByText('Synthetic input diagnostics')).toBeNull();
  });
  it('permits a three-second gripper hold but never carries it into an arm input',async()=>{
    const v=panel();
    expect(v.queryByRole('option',{name:'3秒（仅夹爪）'})).toBeNull();
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'r'}});
    fireEvent.change(v.getByLabelText('验收持续时间'),{target:{value:'3000'}});
    await act(async()=>{fireEvent.click(v.getByRole('button',{name:'合成键盘：3秒后释放'}));await Promise.resolve();});
    await act(async()=>{await vi.advanceTimersByTimeAsync(2500);});
    expect(intents(v.socket).at(-1).intent).toMatchObject({deadman:true,gripper_rate_s:-.025,linear_m_s:[0,0,0],angular_rad_s:[0,0,0]});
    await act(async()=>{await vi.advanceTimersByTimeAsync(500);});
    expect(intents(v.socket).at(-1).intent.deadman).toBe(false);
    fireEvent.change(v.getByLabelText('验收方向'),{target:{value:'d'}});
    expect(v.queryByRole('option',{name:'3秒（仅夹爪）'})).toBeNull();
    expect(v.getByRole('button',{name:'合成键盘：1秒后释放'})).toBeTruthy();
  });
  it('keeps actual gripper opening and hides target diagnostics',async()=>{
    const v=panel();
    const actual={position_m:[.1,.2,.3],joints_rad:[0,0,0,0,0,0],gripper:.325};
    await act(async()=>v.socket.state({stage:'live',actual,gripper:{phase:'ready',valid:true,target:.35,effective_target:.34},capabilities:v.testCaps}));
    expect(v.getByTestId('e6-actual').textContent).toContain('32.5%');
    expect(v.queryByTestId('e6-gripper-position')).toBeNull();
    expect(v.queryByText('Device details')).toBeNull();
    await act(async()=>v.socket.state({stage:'live',actual:{...actual,gripper:null},capabilities:v.testCaps}));
    expect(v.getByTestId('e6-actual').textContent).toContain('Unknown');
  });
  it('shows installed usable stroke while retaining native feedback separately',async()=>{
    const v=panel();
    const actual={position_m:[.1,.2,.3],joints_rad:[0,0,0,0,0,0],gripper:12/255};
    const gripper={phase:'ready',valid:true,opening:0,stroke:{revision:'test',minimum:0,maximum:1,closed_raw:12,open_raw:255}};
    await act(async()=>v.socket.state({stage:'live',actual,gripper,capabilities:v.testCaps}));
    expect(v.getByTestId('e6-actual').textContent).toContain('Gripper 0.0%');
    expect(v.getByText('Gripper · 0–100%')).toBeTruthy();
    await act(async()=>v.socket.state({stage:'live',actual:{...actual,gripper:null},gripper:{...gripper,valid:false,opening:null},capabilities:v.testCaps}));
    expect(v.getByTestId('e6-actual').textContent).toContain('Unknown');
  });
});

it('shows joint selection only in joint mode and preserves mode/frame callbacks',()=>{
  const {result}=boot();const view=render(<ControlStatus control={result.current}/>);
  expect(view.queryByRole('combobox',{name:'Joint',exact:true})).toBeNull();
  fireEvent.mouseDown(view.getByRole('tab',{name:'Joint',exact:true}),{button:0,ctrlKey:false});
  view.rerender(<ControlStatus control={result.current}/>);
  expect(result.current.mode).toBe('joint');expect(view.getByRole('combobox',{name:'Joint',exact:true})).toBeTruthy();
  fireEvent.mouseDown(view.getByRole('tab',{name:'Rotation',exact:true}),{button:0,ctrlKey:false});
  view.rerender(<ControlStatus control={result.current}/>);
  expect(result.current.mode).toBe('rotation');expect(view.queryByRole('combobox',{name:'Joint',exact:true})).toBeNull();
});


it.each(['keyboard','gamepad'] as const)('%s moves directly without Space or LB and releases to neutral',async source=>{
  if(source==='gamepad')pads=[pad()];
  const {socket}=boot(source);act(()=>socket.state({ready:true}));
  await act(async()=>{
    if(source==='keyboard')fireEvent.keyDown(window,{key:'d'});
    else pads[0]!.axes[0]=1;
    await vi.advanceTimersByTimeAsync(100);
  });
  expect(intents(socket).at(-1).intent).toMatchObject({deadman:true,linear_m_s:[.03,0,0]});
  if(source==='gamepad')expect(pads[0]!.buttons[4].pressed).toBe(false);
  await act(async()=>{
    if(source==='keyboard')fireEvent.keyUp(window,{key:'d'});
    else pads[0]!.axes[0]=0;
    await vi.advanceTimersByTimeAsync(50);
  });
  expect(intents(socket).at(-1).intent).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
  expect(socket.sent.filter(m=>m.op==='stop'||m.op==='prepare')).toHaveLength(0);
});

it('Space and LB alone are not active motion and keyboard repeat cannot escape focused editing',async()=>{
  const {socket}=boot();act(()=>socket.state({ready:true}));
  const input=document.createElement('input');document.body.append(input);
  await act(async()=>{fireEvent.keyDown(window,{key:' '});await vi.advanceTimersByTimeAsync(50);});
  expect(intents(socket).at(-1).intent.deadman).toBe(false);
  await act(async()=>{input.focus();fireEvent.keyDown(input,{key:'d'});input.blur();
    fireEvent.keyDown(window,{key:'d',repeat:true});await vi.advanceTimersByTimeAsync(100);});
  expect(intents(socket).at(-1).intent.deadman).toBe(false);
  await act(async()=>{fireEvent.keyUp(window,{key:'d'});fireEvent.keyDown(window,{key:'d'});await Promise.resolve();});
  expect(intents(socket).at(-1).intent.deadman).toBe(true);
});


it('direct gamepad triggers reject resting noise and preserve proportional full-stroke commands',async()=>{
  pads=[pad()];const {socket}=boot('gamepad');act(()=>socket.state({ready:true}));
  for(const value of [.001,.04,.05]){
    pads[0]!.buttons[6].value=value;
    await act(async()=>{await vi.advanceTimersByTimeAsync(50);});
    expect(intents(socket).at(-1).intent.deadman).toBe(false);
  }
  pads[0]!.buttons[6].value=.525;
  await act(async()=>{await vi.advanceTimersByTimeAsync(50);});
  expect(intents(socket).at(-1).intent.gripper_rate_s).toBeCloseTo(caps.gripper_rate_s*.5);
  pads[0]!.buttons[6].value=0;pads[0]!.buttons[7].value=1;
  await act(async()=>{await vi.advanceTimersByTimeAsync(50);});
  expect(intents(socket).at(-1).intent.gripper_rate_s).toBe(-caps.gripper_rate_s);
  expect(pads[0]!.buttons[4].pressed).toBe(false);
});
