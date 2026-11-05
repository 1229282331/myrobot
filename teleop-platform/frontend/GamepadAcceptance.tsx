import {useEffect,useRef,useState} from 'react';
import {Button} from '@/components/ui/button';
import type {useE6Control} from './useE6Control';
import {revealAcceptanceViews} from './acceptanceViewport';

export function GamepadAcceptance({control:c}:{control:ReturnType<typeof useE6Control>}) {
  const [direction,setDirection]=useState('z+');
  const [duration,setDuration]=useState(200);
  const [amplitude,setAmplitude]=useState(1);
  const [fine,setFine]=useState(false);
  const [rounds,setRounds]=useState(6);
  const [running,setRunning]=useState(false);
  const armDirection=/^(x|y|z|xy|xyz)[+-]$/.test(direction);
  const timer=useRef<ReturnType<typeof setTimeout>|null>(null);
  const current=useRef(c);current.current=c;
  const release=()=>{
    if(timer.current===null)return;
    clearTimeout(timer.current);timer.current=null;
    current.current.setTestPadInput([0,0,0,0],[]);setRunning(false);
  };
  useEffect(()=>{
    const hidden=()=>{if(document.hidden)release();};
    const escape=(e:KeyboardEvent)=>{if(e.key==='Escape'){release();current.current.stop();}};
    window.addEventListener('blur',release);window.addEventListener('keydown',escape);
    document.addEventListener('visibilitychange',hidden);
    return ()=>{release();window.removeEventListener('blur',release);window.removeEventListener('keydown',escape);
      document.removeEventListener('visibilitychange',hidden);};
  },[]);
  useEffect(()=>{
    if(!c.state.ready||c.state.reason||!c.testPadState.connected)release();
  },[c.state.ready,c.state.reason,c.testPadState.connected]);
  const start=(button?:number,continuous=false)=>{
    if(timer.current!==null||!c.testPadState.connected||!c.caps?.synthetic_keys)return;
    if(button===undefined&&!c.state.ready)return;
    if(continuous&&(button!==undefined||!armDirection||duration<500))return;
    const axes=[0,0,0,0],buttons=Array(17).fill(0);
    if(button!==undefined)buttons[button]=1;
    else {
      const a=amplitude;
      const inputs:Record<string,number[]>={'x+':[a,0,0,0],'x-':[-a,0,0,0],
        'y+':[0,-a,0,0],'y-':[0,a,0,0],'z+':[0,0,0,-a],'z-':[0,0,0,a],
        'xy+':[a,-a,0,0],'xy-':[-a,a,0,0],'xyz+':[a,-a,0,-a],'xyz-':[-a,a,0,a]};
      axes.splice(0,4,...(inputs[direction]||[0,0,0,0]));
      buttons[5]=fine?1:0;
      if(direction==='open')buttons[6]=a;
      if(direction==='close')buttons[7]=a;
    }
    if(!c.setTestPadInput(axes,buttons))return;
    setRunning(true);
    if(!continuous)timer.current=setTimeout(release,button===undefined?duration:100);
    else {
      // Half forward, alternating full phases, half forward. Longer runs
      // repeat the same input waveform; they do not enlarge its excursion.
      // Only raw input axes change;
      // LB remains released; the real mapper/transport produce commands.
      let phase=0;
      const lastPhase=2*rounds;
      const next=()=>{
        if(timer.current===null)return;
        if(++phase>lastPhase){release();return;}
        const sample=axes.map(v=>phase%2?-v:v);
        if(!current.current.setTestPadInput(sample,buttons)){release();return;}
        timer.current=setTimeout(next,phase===lastPhase?duration/2:duration);
      };
      timer.current=setTimeout(next,duration/2);
    }
  };
  return <details className="e6-acceptance" onToggle={revealAcceptanceViews}><summary>自动手柄输入验收</summary>
    <p>标准手柄原始轴与按钮样本 → 上游映射 → 正式控制连接。属于合成输入，不代表实体接收器验收；只在已验证路线内使用。</p>
    <p data-testid="e6-test-pad-state">{!c.testPadState.selected?'使用实体手柄':c.testPadState.connected?(c.state.ready?'合成手柄已准备':'合成手柄已连接（须准备）'):'合成手柄已断开'}</p>
    <Button variant="outline" disabled={running||c.testPadState.connected} onClick={()=>c.selectTestPad('connected')}>连接合成手柄</Button>
    <Button variant="outline" disabled={!c.testPadState.connected} onClick={()=>{release();c.selectTestPad('disconnected');}}>断开合成手柄</Button>
    <Button variant="outline" disabled={running||!c.testPadState.selected} onClick={()=>c.selectTestPad('physical')}>恢复实体手柄</Button>
    <div>
      <label>输入 <select aria-label="手柄验收方向" value={direction} disabled={running} onChange={e=>setDirection(e.target.value)}>
        {[['hold','回中'],['x+','X+'],['x-','X−'],['y+','Y+'],['y-','Y−'],['z+','Z+'],['z-','Z−'],
          ['xy+','XY+'],['xy-','XY−'],['xyz+','XYZ+'],['xyz-','XYZ−'],['open','LT张开'],['close','RT闭合']].map(([v,n])=><option key={v} value={v}>{n}</option>)}
      </select></label>
      <label>幅度 <select aria-label="手柄验收幅度" value={amplitude} disabled={running} onChange={e=>setAmplitude(Number(e.target.value))}>
        {[.04,.06,.1,.5,1].map(v=><option key={v} value={v}>{v*100}%</option>)}</select></label>
      <label>时长 <select aria-label="手柄验收时长" value={duration} disabled={running} onChange={e=>setDuration(Number(e.target.value))}>
        {[100,200,500,750,1000,3000].map(v=><option key={v} value={v}>{v/1000}秒</option>)}</select></label>
      <label><input type="checkbox" checked={fine} disabled={running} onChange={e=>setFine(e.target.checked)}/>手柄验收RB精细25%</label>
      <label>连续轮数 <select aria-label="手柄连续轮数" value={rounds} disabled={running} onChange={e=>setRounds(Number(e.target.value))}>
        {[6,30,60].map(v=><option key={v} value={v}>{v}轮</option>)}</select></label>
    </div>
    <Button disabled={!c.state.ready||!c.testPadState.connected||running} onClick={()=>start()}>合成手柄：{duration/1000}秒后释放</Button>
    <Button disabled={!c.state.ready||!c.testPadState.connected||running||!armDirection||duration<500}
      onClick={()=>start(undefined,true)}>手柄连续往返：{rounds}轮</Button>
    <p>连续用例无需LB，首尾各半周期，其余每向使用上方时长；{rounds}轮共{2*rounds*duration/1000}秒后回中释放。增加轮数不保证实际位置不会漂移，仍须观察反馈并保持在已验证路线内。不是轨迹回放。</p>
    {running&&<Button variant="outline" onClick={release}>释放合成手柄</Button>}
    <div>{[[2,'X切模式'],[3,'Y切坐标'],[8,'Back停止'],[9,'Start录制'],[0,'A成功'],[1,'B失败']].map(([i,n])=>
      <Button key={i} variant="outline" disabled={!c.testPadState.connected||running} onClick={()=>start(Number(i))}>{n}</Button>)}</div>
  </details>;
}
