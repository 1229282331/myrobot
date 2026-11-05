import React from 'react';
import {act,cleanup,fireEvent,render,renderHook} from '@testing-library/react';
import {afterEach,beforeEach,describe,expect,it,vi} from 'vitest';
import {SWRConfig} from 'swr';
import {useE6Control} from '../frontend/useE6Control';
import {GamepadAcceptance} from '../frontend/GamepadAcceptance';
import {acceptanceGamepad} from '../frontend/acceptanceGamepad';
import {extractAnalogValues} from '@/pages/control/GamepadUtils';

const caps={enabled:true,backend:'DOM fixture, no hardware',synthetic_keys:true,linear_m_s:.03,
  angular_rad_s:Math.PI/18,joint_rad_s:Math.PI/18,gripper_rate_s:.1};
class Socket {
  static OPEN=1;static all:Socket[]=[];
  readyState=1;bufferedAmount=0;sent:any[]=[];
  onmessage:((e:{data:string})=>void)|null=null;onclose:(()=>void)|null=null;
  constructor(){Socket.all.push(this);}
  send(s:string){this.sent.push(JSON.parse(s));}
  close(){this.readyState=3;}
  state(extra:object={}){this.onmessage?.({data:JSON.stringify({kind:'state',echo_ns:100,state:'可控制',
    ready:true,capabilities:caps,...extra})});}
}
let pads:(Gamepad|null)[]=[];
const values=(entries:Record<number,number>)=>Array.from({length:17},(_,i)=>entries[i]||0);
const last=(s:Socket)=>s.sent.filter(m=>m.op==='intent').at(-1).intent;
function prepareFixture(control:ReturnType<typeof useE6Control>,socket:Socket){
  // Source changes require explicit preparation; ready feedback alone is not
  // a substitute. Retain assertions about no automatic prepares during a run.
  act(()=>socket.state({ready:false}));
  act(()=>{void control.prepare();});
  expect(socket.sent.at(-1)?.op).toBe('prepare');
  act(()=>socket.state());socket.sent=[];
}
function setup(synthetic=true){
  const cache=new Map();const testCaps={...caps,synthetic_keys:synthetic};
  const wrapper=({children}:{children:React.ReactNode})=><SWRConfig value={{provider:()=>cache,
    fallback:{'/e6/capabilities':testCaps},revalidateOnMount:false,revalidateOnFocus:false}}>{children}</SWRConfig>;
  const hook=renderHook(()=>useE6Control('gamepad',1,0),{wrapper});
  const socket=Socket.all.at(-1)!;act(()=>socket.state({capabilities:testCaps}));
  return {...hook,socket,testCaps};
}
beforeEach(()=>{vi.useFakeTimers();Socket.all=[];pads=[];vi.stubGlobal('WebSocket',Socket);
  vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(Error('No network allowed'))));
  Object.defineProperty(navigator,'getGamepads',{configurable:true,value:()=>pads});});
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();delete (navigator as any).getGamepads;});

describe('upstream analog mapping and the E6 5% deadzone',()=>{
  it('preserves upstream default behavior while allowing raw axes for E6 radial mapping',()=>{
    const p=acceptanceGamepad([.1,.1,0,.1]);
    expect(extractAnalogValues(p).leftStickX).toBe(0);
    expect(extractAnalogValues(p,0).leftStickX).toBe(.1);
  });
  it.each([.04,.05,.06,.1,.15,.5,1,-.06,-.1])('maps a physical %s stick input continuously without a second 15% deadzone',amount=>{
    pads=[acceptanceGamepad([amount,0,0,0],values({4:1}))];
    const {socket}=setup();act(()=>vi.advanceTimersByTime(50));
    const expected=Math.abs(amount)<=.05?0:Math.sign(amount)*(Math.abs(amount)-.05)/.95*.03;
    expect(last(socket).linear_m_s[0]).toBeCloseTo(expected,12);
  });
  it('uses radial XY deadzone even when each axis is below 5%',()=>{
    pads=[acceptanceGamepad([.04,-.04,0,0],values({4:1}))];
    const {socket}=setup();act(()=>vi.advanceTimersByTime(50));
    expect(Math.hypot(...last(socket).linear_m_s)).toBeCloseTo((Math.hypot(.04,.04)-.05)/.95*.03);
    expect(last(socket).linear_m_s[0]).toBeGreaterThan(0);
  });
});

describe('synthetic raw Gamepad at the formal device-read boundary',()=>{
  it.each([
    ['x+', [1,0,0,0]],['x-',[-1,0,0,0]],['y+',[0,-1,0,0]],['y-',[0,1,0,0]],
    ['z+',[0,0,0,-1]],['z-',[0,0,0,1]],['xyz',[1,-1,0,-1]],['partial',[.1,-.08,0,.06]]
  ])('matches the physical reader mapping for %s, fine, and both frames/modes',(_name,axes)=>{
    const a=axes as number[];const {result,socket}=setup();
    for(const mode of ['translation','rotation','joint'] as const)for(const frame of ['base','tool'] as const) {
      act(()=>{pads=[acceptanceGamepad()];result.current.selectTestPad('physical');vi.advanceTimersByTime(50);});
      prepareFixture(result.current,socket);
      act(()=>{result.current.chooseMode(mode);result.current.chooseFrame(frame);result.current.chooseJoint(5);});
      act(()=>{pads=[acceptanceGamepad(a,values({4:1,5:1,6:.3,7:.1}))];vi.advanceTimersByTime(50);});
      const physical=last(socket);
      expect(physical.deadman).toBe(true);
      act(()=>result.current.selectTestPad('connected'));
      prepareFixture(result.current,socket);
      act(()=>result.current.setTestPadInput(a,values({4:1,5:1,6:.3,7:.1})));
      expect(last(socket)).toEqual(physical);
      expect(socket.sent.at(-1).source).toBe('gamepad');
      act(()=>result.current.setTestPadInput([0,0,0,0],[]));
    }
  });
  it('blocks selection and synthetic samples without the explicit commissioning capability',()=>{
    const {result,socket}=setup(false);
    act(()=>result.current.selectTestPad('connected'));
    expect(result.current.testPadState.selected).toBe(false);
    expect(result.current.setTestPadInput([1,0,0,0],values({4:1}))).toBe(false);
    expect(socket.sent).toHaveLength(0);
  });
  it('cannot fall back to a held physical pad on synthetic disconnect, and reconnect never prepares',()=>{
    pads=[acceptanceGamepad([1,0,0,0],values({4:1}))];
    const {result,socket}=setup();
    act(()=>result.current.selectTestPad('connected'));
    act(()=>result.current.selectTestPad('disconnected'));
    act(()=>{socket.state({ready:false});vi.advanceTimersByTime(500);});
    expect(last(socket).deadman).toBe(false);
    act(()=>result.current.selectTestPad('connected'));
    expect(socket.sent.some(m=>m.op==='prepare')).toBe(false);
    expect(last(socket).deadman).toBe(false);
  });
  it('requires release/centering for X/Y, consumes held button edges, and supports signed triggers',()=>{
    const {result,socket}=setup();act(()=>result.current.selectTestPad('connected'));
    prepareFixture(result.current,socket);
    act(()=>result.current.setTestPadInput([1,0,0,0],values({4:1,2:1,3:1})));
    expect(result.current.mode).toBe('translation');expect(result.current.frame).toBe('base');
    act(()=>result.current.setTestPadInput([0,0,0,0],values({2:1,3:1})));
    expect(result.current.mode).toBe('translation');
    act(()=>result.current.setTestPadInput([0,0,0,0],[]));
    act(()=>result.current.setTestPadInput([0,0,0,0],values({2:1,3:1})));
    expect(result.current.mode).toBe('rotation');expect(result.current.frame).toBe('tool');
    act(()=>vi.advanceTimersByTime(300));expect(result.current.mode).toBe('rotation');
    for(const [button,sign] of [[6,1],[7,-1]]) {
      act(()=>result.current.setTestPadInput([0,0,0,0],values({4:1,[button]:.5})));
      expect(last(socket).gripper_rate_s).toBeCloseTo(sign*.05);
    }
    act(()=>result.current.setTestPadInput([0,0,0,0],[]));
    expect(last(socket)).toMatchObject({deadman:false,gripper_rate_s:0});
  });
  it('rejects invalid raw samples before publishing',()=>{
    const {result,socket}=setup();act(()=>result.current.selectTestPad('connected'));const count=socket.sent.length;
    expect(()=>result.current.setTestPadInput([NaN,0,0,0],[])).toThrow();
    expect(()=>result.current.setTestPadInput([2,0,0,0],[])).toThrow();
    expect(()=>result.current.setTestPadInput([0,0,0,0],[-1])).toThrow();
    expect(socket.sent).toHaveLength(count);
  });
  it('Back with a held physical LB/axis preempts that sample',()=>{
    pads=[acceptanceGamepad([1,0,0,0],values({4:1,8:1}))];
    const {socket}=setup();act(()=>vi.advanceTimersByTime(50));
    expect(socket.sent.at(-1)).toMatchObject({op:'stop',reason:'operator_stop'});
    expect(socket.sent.some(m=>m.op==='intent'&&m.intent.deadman)).toBe(false);
  });
  it('withdrawing the capability disconnects the test pad without selecting a held physical pad',async()=>{
    pads=[acceptanceGamepad([1,0,0,0],values({4:1}))];
    const {result,socket}=setup();act(()=>result.current.selectTestPad('connected'));
    act(()=>result.current.setTestPadInput([0,0,0,-1],values({4:1})));
    await act(async()=>socket.state({capabilities:{...caps,synthetic_keys:false}}));
    const begin=socket.sent.length;act(()=>vi.advanceTimersByTime(500));
    expect(result.current.testPadState).toEqual({selected:true,connected:false});
    expect(socket.sent.slice(begin).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
  });
});

describe('bounded gamepad commissioning panel',()=>{
  function panel(){
    const cache=new Map();let control:ReturnType<typeof useE6Control>;
    function Panel(){control=useE6Control('gamepad',1);return <GamepadAcceptance control={control}/>;}
    const view=render(<SWRConfig value={{provider:()=>cache,fallback:{'/e6/capabilities':caps},
      revalidateOnMount:false,revalidateOnFocus:false}}><Panel/></SWRConfig>);
    const socket=Socket.all.at(-1)!;act(()=>socket.state());
    fireEvent.click(view.getByRole('button',{name:'连接合成手柄'}));
    prepareFixture(control!,socket);return {...view,socket,control:()=>control!};
  }
  it.each(['timer','blur','hidden','escape','disconnect','fault','unmount'])('releases all controls on %s and never replays after recovery',reason=>{
    const v=panel();fireEvent.click(v.getByRole('button',{name:'合成手柄：0.2秒后释放'}));
    expect(last(v.socket)).toMatchObject({deadman:true,linear_m_s:[0,0,.03]});
    if(reason==='timer')act(()=>vi.advanceTimersByTime(200));
    if(reason==='blur')fireEvent.blur(window);
    if(reason==='hidden'){Object.defineProperty(document,'hidden',{configurable:true,value:true});fireEvent(document,new Event('visibilitychange'));delete (document as any).hidden;}
    if(reason==='escape')fireEvent.keyDown(window,{key:'Escape'});
    if(reason==='disconnect')fireEvent.click(v.getByRole('button',{name:'断开合成手柄'}));
    if(reason==='fault')act(()=>v.socket.state({ready:false,reason:'feedback_stale'}));
    if(reason==='unmount')v.unmount();
    const start=v.socket.sent.length;
    if(reason!=='unmount')act(()=>v.socket.state());
    act(()=>vi.advanceTimersByTime(1000));
    expect(v.socket.sent.slice(start).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
    expect(v.socket.sent.some(m=>m.op==='prepare')).toBe(false);
  });
  it('supports Start/A/B/Back through the formal handler without directly calling a recording endpoint',()=>{
    const v=panel();fireEvent.click(v.getByRole('button',{name:'Start录制'}));
    expect(v.socket.sent.some(m=>m.op==='record_start')).toBe(true);
    act(()=>{vi.advanceTimersByTime(100);v.socket.state({recording:{phase:'capturing'}});});
    fireEvent.click(v.getByRole('button',{name:'A成功'}));
    expect(v.socket.sent.some(m=>m.op==='record_finish'&&m.result==='success')).toBe(true);
    act(()=>vi.advanceTimersByTime(100));fireEvent.click(v.getByRole('button',{name:'B失败'}));
    expect(v.socket.sent.some(m=>m.op==='record_finish'&&m.result==='failure')).toBe(true);
    act(()=>vi.advanceTimersByTime(100));fireEvent.click(v.getByRole('button',{name:'Back停止'}));
    expect(v.socket.sent.some(m=>m.op==='stop'&&m.reason==='operator_stop')).toBe(true);
  });
  it.each([500,750])('holds LB through six %s ms raw-axis reversals and releases once',duration=>{
    const v=panel();
    fireEvent.change(v.getByLabelText('手柄验收时长'),{target:{value:String(duration)}});
    fireEvent.click(v.getByRole('checkbox',{name:'手柄验收RB精细25%'}));
    const begin=v.socket.sent.length;
    fireEvent.click(v.getByRole('button',{name:'手柄连续往返：6轮'}));
    expect(last(v.socket)).toMatchObject({deadman:true,linear_m_s:[0,0,.0075]});
    act(()=>vi.advanceTimersByTime(duration/2));
    expect(last(v.socket)).toMatchObject({deadman:true,linear_m_s:[0,0,-.0075]});
    for(let phase=2;phase<=12;phase++) {
      act(()=>vi.advanceTimersByTime(duration));
      expect(last(v.socket).linear_m_s[2]).toBeCloseTo(phase%2?-.0075:.0075);
    }
    const beforeRelease=v.socket.sent.slice(begin).filter(m=>m.op==='intent');
    expect(beforeRelease.every(m=>m.source==='gamepad'&&m.intent.deadman&&Math.abs(m.intent.linear_m_s[2])===.0075)).toBe(true);
    expect(v.socket.sent.slice(begin).some(m=>m.op==='stop')).toBe(false);
    act(()=>vi.advanceTimersByTime(duration/2));
    expect(last(v.socket)).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
    expect(v.getByRole('button',{name:'手柄连续往返：6轮'}).hasAttribute('disabled')).toBe(false);
  });
  it.each(['blur','hidden','escape','disconnect','fault','unmount'])('cancels remaining continuous phases on %s without resuming',reason=>{
    const v=panel();fireEvent.change(v.getByLabelText('手柄验收时长'),{target:{value:'500'}});
    fireEvent.click(v.getByRole('button',{name:'手柄连续往返：6轮'}));
    act(()=>vi.advanceTimersByTime(300));
    if(reason==='blur')fireEvent.blur(window);
    if(reason==='hidden'){Object.defineProperty(document,'hidden',{configurable:true,value:true});fireEvent(document,new Event('visibilitychange'));delete (document as any).hidden;}
    if(reason==='escape')fireEvent.keyDown(window,{key:'Escape'});
    if(reason==='disconnect')fireEvent.click(v.getByRole('button',{name:'断开合成手柄'}));
    if(reason==='fault')act(()=>v.socket.state({ready:false,reason:'feedback_stale'}));
    if(reason==='unmount')v.unmount();
    const after=v.socket.sent.length;
    if(reason!=='unmount')act(()=>v.socket.state());
    act(()=>vi.advanceTimersByTime(10000));
    expect(v.socket.sent.slice(after).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
    expect(v.socket.sent.some(m=>m.op==='prepare')).toBe(false);
  });
  it.each([30,60])('finishes %s rounds at the stated duration with no intermediate release',rounds=>{
    const v=panel();fireEvent.change(v.getByLabelText('手柄验收时长'),{target:{value:'500'}});
    fireEvent.change(v.getByLabelText('手柄连续轮数'),{target:{value:String(rounds)}});
    const begin=v.socket.sent.length;
    fireEvent.click(v.getByRole('button',{name:`手柄连续往返：${rounds}轮`}));
    expect(v.getByLabelText('手柄连续轮数').hasAttribute('disabled')).toBe(true);
    act(()=>vi.advanceTimersByTime(rounds*1000-1));
    expect(last(v.socket).deadman).toBe(true);
    const active=v.socket.sent.slice(begin).filter(m=>m.op==='intent');
    expect(active.every(m=>m.source==='gamepad'&&m.intent.deadman)).toBe(true);
    const changes=active.map(m=>Math.sign(m.intent.linear_m_s[2])).filter((v,i,a)=>i===0||v!==a[i-1]);
    expect(changes).toHaveLength(2*rounds+1);
    expect(changes[0]).toBe(1);expect(changes.at(-1)).toBe(1);
    expect(v.socket.sent.slice(begin).some(m=>m.op==='stop')).toBe(false);
    act(()=>vi.advanceTimersByTime(1));
    expect(last(v.socket)).toMatchObject({deadman:false,linear_m_s:[0,0,0]});
    expect(v.getByLabelText('手柄连续轮数').hasAttribute('disabled')).toBe(false);
  });
  it('cancels a long run after 20 seconds and never replays remaining rounds after recovery',()=>{
    const v=panel();fireEvent.change(v.getByLabelText('手柄验收时长'),{target:{value:'500'}});
    fireEvent.change(v.getByLabelText('手柄连续轮数'),{target:{value:'60'}});
    fireEvent.click(v.getByRole('button',{name:'手柄连续往返：60轮'}));
    act(()=>vi.advanceTimersByTime(20200));
    act(()=>v.socket.state({ready:false,reason:'feedback_stale'}));
    const begin=v.socket.sent.length;
    act(()=>{v.socket.state();vi.advanceTimersByTime(70000);});
    expect(v.socket.sent.slice(begin).filter(m=>m.op==='intent').every(m=>!m.intent.deadman)).toBe(true);
    expect(v.socket.sent.some(m=>m.op==='prepare')).toBe(false);
  });
});
