import React from 'react';
import {act,cleanup,render,screen} from '@testing-library/react';
import {afterEach,beforeEach,expect,it,vi} from 'vitest';
import {SWRConfig} from 'swr';
import {useE6Control} from '../frontend/useE6Control';
import {ControlCamera} from '../frontend/ControlStatus';

vi.mock('@/components/common/camera-stream-card',()=>({
  CameraStreamCard:({streamPath}:{streamPath:string})=><div data-testid="camera">{streamPath}</div>,
}));
vi.mock('../frontend/WorkspaceView',()=>({
  WorkspaceView:(props:unknown)=><div data-testid="workspace">{JSON.stringify(props)}</div>,
}));
const caps={enabled:true,backend:'test gateway',linear_m_s:.03,angular_rad_s:.1,joint_rad_s:.1,gripper_rate_s:.1,
  video_url:'http://127.0.0.1:8889/old/'};
const observed=(extra:object={})=>({kind:'state',state:'可控制',ready:true,echo_ns:999,
  feedback:{connected:true,age_ms:5},actual:{joints_rad:[1,2,3,4,5,6],position_m:[.1,.2,.3],gripper:.5},
  capabilities:{...caps,video_url:'http://127.0.0.1:8889/recovered/'},...extra});
const reply=(value:unknown)=>({ok:true,json:async()=>value});
class Socket {
  static OPEN=1;static all:Socket[]=[];
  readyState=1;bufferedAmount=0;sent:any[]=[];
  onmessage:((event:{data:string})=>void)|null=null;onclose:(()=>void)|null=null;
  constructor(public url:string){Socket.all.push(this);}
  send(data:string){this.sent.push(JSON.parse(data));}
  close(){this.readyState=3;}
  closed(){this.close();this.onclose?.();}
  state(extra:object={}){this.onmessage?.({data:JSON.stringify({kind:'state',state:'可准备',ready:false,
    echo_ns:123,capabilities:caps,...extra})});}
}
let control:ReturnType<typeof useE6Control>;
const flush=async()=>{await act(async()=>{await Promise.resolve();});};
function boot(){
  const cache=new Map();
  function Panel(){control=useE6Control('keyboard',.2);return <ControlCamera control={control}/>;}
  const view=render(<SWRConfig value={{provider:()=>cache,fallback:{'/e6/capabilities':caps},
    revalidateOnFocus:false}}><Panel/></SWRConfig>);
  return {...view,socket:Socket.all.at(-1)!};
}
beforeEach(()=>{
  vi.useFakeTimers();Socket.all=[];
  vi.stubGlobal('WebSocket',Socket);
  Object.defineProperty(navigator,'getGamepads',{configurable:true,value:()=>[]});
});
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();localStorage.clear();
  delete (navigator as any).getGamepads;});

it('recovers camera and 3D through HTTP without claiming control or importing another owner readiness',async()=>{
  let online=false;
  const fetch=vi.fn((url:string)=>url==='/e6/capabilities'?Promise.resolve(reply(caps)):
    online?Promise.resolve(reply(observed())):Promise.reject(new Error('offline')));
  vi.stubGlobal('fetch',fetch);
  const {socket}=boot();await flush();
  act(()=>{socket.state({ready:true});socket.closed();});
  online=true;
  await act(async()=>{await vi.advanceTimersByTimeAsync(1200);});
  expect(control.state).toMatchObject({state:'未连接',ready:false,echo_ns:0});
  expect(control.observation?.echo_ns).toBe(999);
  expect(control.caps?.video_url).toContain('/recovered/');
  expect(screen.getByTestId('camera').textContent).toContain('/recovered/');
  expect(JSON.parse(screen.getByTestId('workspace').textContent!)).toMatchObject({fresh:true,joints:[1,2,3,4,5,6]});
  expect(screen.getByRole('status').textContent).toContain('Read-only view');
  expect(Socket.all).toHaveLength(1);
  expect(socket.sent.some(x=>['prepare','arm_enable','gripper_initialize'].includes(x.op))).toBe(false);
  expect(fetch.mock.calls.every(([url])=>url==='/e6/state'||url==='/e6/capabilities')).toBe(true);
});

it('explicit reconnect works before video returns, but preparation still requires new video and another click',async()=>{
  vi.stubGlobal('fetch',vi.fn((url:string)=>Promise.resolve(reply(url==='/e6/capabilities'?caps:observed()))));
  const {socket}=boot();await flush();
  act(()=>{socket.state();socket.closed();});await flush();
  await act(async()=>control.prepare());
  const next=Socket.all.at(-1)!;
  expect(next).not.toBe(socket);
  act(()=>next.state());
  expect(next.sent.some(x=>x.op==='prepare')).toBe(false);
  act(()=>window.dispatchEvent(new CustomEvent('e6-video-health',{detail:{fresh:true}})));
  await act(async()=>{await vi.advanceTimersByTimeAsync(100);});
  expect(next.sent.some(x=>x.op==='prepare')).toBe(false);
  await act(async()=>control.prepare());
  expect(next.sent.filter(x=>x.op==='prepare')).toHaveLength(1);
  expect(next.sent.filter(x=>x.op==='intent').every(x=>!x.intent.deadman)).toBe(true);
});

it('late HTTP observations cannot replace the restored control state or its capabilities',async()=>{
  vi.stubGlobal('fetch',vi.fn((url:string)=>Promise.resolve(reply(url==='/e6/capabilities'?caps:observed()))));
  const {socket}=boot();await flush();
  const currentCaps={...caps,backend:'current control owner',linear_m_s:.007,video_url:'http://127.0.0.1:8889/current/'};
  act(()=>socket.state({ready:true,echo_ns:321,capabilities:currentCaps,
    feedback:{connected:true,age_ms:2},actual:{joints_rad:[0,0,0,0,0,0],position_m:[0,0,0],gripper:null}}));
  await act(async()=>{await vi.advanceTimersByTimeAsync(1100);});
  expect(control.observation?.echo_ns).toBe(999);
  expect(control.state.echo_ns).toBe(321);
  expect(control.caps?.backend).toBe('current control owner');
  await act(async()=>{control.keyDown('d');await Promise.resolve();});
  expect(socket.sent.filter(x=>x.op==='intent').at(-1).intent.linear_m_s[0]).toBeCloseTo(.0014);
  expect(JSON.parse(screen.getByTestId('workspace').textContent!)).toMatchObject({joints:[0,0,0,0,0,0]});
  expect(screen.queryByRole('status')).toBeNull();
  expect(Socket.all).toHaveLength(1);
});

it('clears a recovered-connection notice when that connection closes again',async()=>{
  vi.stubGlobal('fetch',vi.fn((url:string)=>Promise.resolve(reply(url==='/e6/capabilities'?caps:observed()))));
  const {socket}=boot();await flush();
  act(()=>{socket.state();socket.closed();});
  await act(async()=>control.prepare());
  const next=Socket.all.at(-1)!;
  act(()=>next.state());
  expect(control.notice).toContain('连接已恢复');
  act(()=>next.closed());
  expect(control.notice).toBe('');
  expect(control.state.state).toBe('未连接');
  expect(control.state.ready).toBe(false);
  expect(next.sent.some(x=>x.op==='prepare')).toBe(false);
});

it('an unavailable read-only snapshot freezes the display without reconnecting a rejected second tab',async()=>{
  let online=true;
  vi.stubGlobal('fetch',vi.fn((url:string)=>url==='/e6/capabilities'?Promise.resolve(reply(caps)):
    online?Promise.resolve(reply(observed())):Promise.reject(new Error('offline'))));
  const {socket}=boot();await flush();
  act(()=>{socket.state({state:'暂挂',ready:false,reason:'已有控制者'});socket.closed();});await flush();
  expect(control.state.reason).toBe('已有控制者');
  online=false;
  await act(async()=>{await vi.advanceTimersByTimeAsync(1200);});
  expect(control.observation).toBeUndefined();
  expect(JSON.parse(screen.getByTestId('workspace').textContent!).fresh).toBe(false);
  expect(control.state.reason).toBe('已有控制者');
  expect(control.state.ready).toBe(false);
  expect(Socket.all).toHaveLength(1);
});
