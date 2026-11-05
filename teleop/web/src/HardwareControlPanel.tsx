import { useEffect, useRef, useState } from "react";
import { BrowserInputGate, blocksControlInput, restrictGripper } from './hardwareCapabilities';
import {
  command,
  fromGamepad,
  fromKeyboard,
  getAvailableGamepads,
  type Mode,
  type Frame,
  type Source,
  type Input,
} from "./input";
type Limits = {
  translation_m_s: number;
  rotation_rad_s: number;
  gripper_s: number;
  radius_m: number;
  orientation_rad: number;
};
type Status = {
  armed: boolean;
  reason: string;
  limits: Limits;
  lease?: string;
  session?: string;
  input_delivery?: string;
  status_revision_ns?: number;
  transaction_budget?: {target_ms:number;hard_deadline_ms:number;tolerated_jitter_count:number};
  stop_acknowledged?: boolean;
  stop_confirmation?: { send_confirmed?: boolean; confirmation_complete?: boolean; send_duration_ms?: number; confirmation_duration_ms?: number; error?: string } | null;
  stopped_feedback?: boolean;
  enabled_once?: boolean;
  gripper_initialized?: boolean;
  gripper_faulted?: boolean;
  gripper_command_profile?: {name:string;enabled?:boolean;velocity:number;force:number;acceleration:number;deceleration:number};
  commissioning?: {state:string;operation:string} | null;
  first_fault?: { reason: string; x5_monotonic_ns: number } | null;
  recovery?: { policy: string; state: string; reason?: string; attempts_in_window: number; max_attempts: number };
  kinematics?: { kind: string; prepared?: boolean; connection_ready?: boolean; actual_residual_m?: number; actual_residual_rad?: number; profile_sha256?: string };
};
export default function HardwareControlPanel({robotMode,enabled}:{robotMode?:number;enabled?:number}) {
  const record = async (result?:'success'|'failure') => {
    if(recordPending.current)return;
    recordPending.current=true;
    try {
      const response=await fetch(result?'/api/record/finish':'/api/record/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(result?{result}:{task:'实机手柄/键盘空载示教'})});
      if(!response.ok){const d=await response.json();setNotice(d.detail??'录制请求失败')}
    } catch {setNotice('录制连接失败，请查看片段状态');}
    finally {recordPending.current=false;}
  };
  const [state, setState] = useState<{
    control_stage?: string;
    capabilities?: { operator_control: boolean };
    status?: Status;
  }>({});
  const [source, setSource] = useState<Source>("keyboard"),
    [mode, setMode] = useState<Mode>("translation"),
    [frame, setFrame] = useState<Frame>("base"),
    [notice, setNotice] = useState("");
  const [input,setInput]=useState<Input>(fromKeyboard(new Set()));
  const [connection,setConnection]=useState('未连接');
  const [reconnect,setReconnect]=useState(0);
  const [inputHint,setInputHint]=useState('');
  const inputGate=useRef(new BrowserInputGate());
  const pendingOperation=useRef('');
  const recordPending=useRef(false);
  const cfg = useRef({ source, mode, frame });
  cfg.current = { source, mode, frame };
  const keys = useRef(new Set<string>()),
    socket = useRef<WebSocket | null>(null),
    lease = useRef(""),
    seq = useRef(0),
    pending = useRef<number | null>(null),
    requestId = useRef(0),
    outstanding = useRef(new Set<number>()),
    currentNeutral = useRef(true),
    deadman = useRef(false);
  const status = useRef<Status | undefined>(undefined),
    sentAt = useRef(0);
  const rpc = (operation: string) => {
    if (socket.current?.readyState === WebSocket.OPEN && (!pending.current || operation === "stop")) {
      pending.current = ++requestId.current;
      pendingOperation.current = operation;
      if(operation==='stop') inputGate.current.stop(pending.current);
      sentAt.current = performance.now();
      outstanding.current.add(pending.current);
      socket.current.send(JSON.stringify({ operation, request_id: pending.current }));
    }
  };
  useEffect(() => {
    let active = true;
    const poll = () =>
      fetch("/api/state")
        .then((r) => r.json())
        .then((d) => {
          if (active) {
            setState(d);
            if ((d.status?.status_revision_ns ?? 0) >= (status.current?.status_revision_ns ?? 0)) {
              if (d.status?.session === status.current?.session && d.status?.lease) lease.current = d.status.lease;
              status.current = d.status;
            }
          }
        });
    poll();
    const timer = setInterval(poll, 500);
    return () => {
      active = false;
      clearInterval(timer);
    };
  }, []);
  useEffect(() => {
    const allowed=!!state.capabilities?.operator_control;
    const ws = allowed?new WebSocket(`ws://${location.host}/ws/control`):null;
    inputGate.current.reset();
    socket.current = ws;
    if(ws) {
    setConnection('连接中');
    ws.onmessage = (e) => {
      const r = JSON.parse(e.data);
      outstanding.current.delete(r.request_id);
      inputGate.current.acknowledge(r.request_id,!!r.ok);
      if (r.request_id === pending.current) pending.current = null;
      if (r.status && (r.status.status_revision_ns ?? 0) >= (status.current?.status_revision_ns ?? 0)) {
        lease.current = r.status.lease;
        status.current = r.status;
        setConnection('已连接');
      }
      if (!r.ok) setNotice(r.error);
    };
    ws.onclose = () => {
      lease.current = "";
      pending.current = null;
      outstanding.current.clear();
      inputGate.current.reset();
      setConnection('已断开');
      setNotice("控制断开，请刷新并重新准备");
    };
    }
    const down = (e: KeyboardEvent) => {
      if (
        (e.target as HTMLElement)?.closest(
          'input,textarea,select,[contenteditable="true"]',
        )
      ) {
        keys.current.clear();
        rpc("stop");
        return;
      }
      if (e.code === "Space" || e.code === "Tab") e.preventDefault();
      keys.current.add(e.code);
      if (e.repeat) return;
      if (e.code === "Escape") rpc("stop");
      if(allowed) {
        if(e.code==='Enter')void record();
        if(e.code==='Digit1')void record('success');
        if(e.code==='Digit2')void record('failure');
      }
      if (!deadman.current && currentNeutral.current) {
        if (e.code === "Tab")
          setMode((m) => (m === "translation" ? "rotation" : "translation"));
        if (e.code === "KeyC")
          setFrame((f) => (f === "base" ? "tool" : "base"));
      }
    };
    const up = (e: KeyboardEvent) => keys.current.delete(e.code);
    let inputBlocked=false;
    const blur = () => {
      keys.current.clear();
      rpc("stop");
      inputBlocked=true;
    };
    let previous: boolean[] = [];
    const timer = setInterval(() => {
      const c = cfg.current;
      const element=document.activeElement as HTMLElement|null;
      const editing=blocksControlInput(c.source,element?.tagName??'',
        !!element?.closest('[contenteditable="true"]'));
      const physicalInput =
        c.source === "gamepad"
          ? fromGamepad(getAvailableGamepads()[0] ?? null)
          : fromKeyboard(keys.current, editing);
      const i=restrictGripper(physicalInput,status.current?.gripper_command_profile?.enabled!==false);
      currentNeutral.current = i.neutral;
      deadman.current = i.deadman;
      setInput(i);
      if (c.source === "gamepad") {
        if (i.buttons[8] && !previous[8]) rpc("stop");
        if(allowed&&i.buttons[9]&&!previous[9])void record();
        if(allowed&&i.buttons[0]&&!previous[0])void record('success');
        if(allowed&&i.buttons[1]&&!previous[1])void record('failure');
        if (!i.deadman && i.neutral) {
          if (i.buttons[2] && !previous[2])
            setMode((m) => (m === "translation" ? "rotation" : "translation"));
          if (i.buttons[3] && !previous[3])
            setFrame((f) => (f === "base" ? "tool" : "base"));
        }
        previous = i.buttons;
      }
      if (pending.current && performance.now() - sentAt.current > 250) {
        ws?.close();
        return;
      }
      if (
        !ws || ws.readyState !== WebSocket.OPEN ||
        !lease.current ||
        (pending.current && status.current?.input_delivery !== "streaming") ||
        !status.current?.limits
      )
        return;
      const focused=document.hasFocus()&&!document.hidden;
      if(!focused||editing) {
        if(!inputBlocked) { keys.current.clear();rpc('stop'); }
        inputBlocked=true;
        setInputHint(!focused?'页面未获得焦点：点击此控制区后回中并准备':'正在编辑：结束编辑后回中并准备');
        return;
      }
      inputBlocked=false;
      const identityKey=JSON.stringify([c.source,i.identity]);
      const decision=inputGate.current.decision(identityKey,i.neutral,i.deadman);
      if(decision==='wait'||decision==='center') {
        setInputHint(decision==='center'?'切换输入后请松开允许动作键并回中':'等待停止回包或新输入握手');
        return;
      }
      if(decision==='handshake'&&outstanding.current.size) return;
      if(pending.current&&pendingOperation.current==='arm'&&(!i.neutral||i.deadman)) return;
      setInputHint('');
      const input = command(
        i,
        c.mode,
        c.frame,
        focused,
        status.current.limits,
      );
      if (ws.bufferedAmount > 4096 || outstanding.current.size >= 32) {
        ws.close();
        setNotice("控制发送或确认积压，已断开；回中后重新准备");
        return;
      }
      const identity = ++requestId.current;
      outstanding.current.add(identity);
      if (decision==='handshake'||status.current.input_delivery !== "streaming") {
        pending.current = identity;
        pendingOperation.current = 'input';
        sentAt.current = performance.now();
      }
      if(decision==='handshake')inputGate.current.sentHandshake(identity,identityKey);
      ws.send(
        JSON.stringify({
          ...input,
          source: c.source,
          seq: ++seq.current,
          lease: lease.current,
          request_id: identity,
        }),
      );
    }, 50);
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    window.addEventListener("blur", blur);
    document.addEventListener("visibilitychange", blur);
    return () => {
      clearInterval(timer);
      ws?.close();
      socket.current=null;
      lease.current = "";
      pending.current = null;
      outstanding.current.clear();
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", blur);
      document.removeEventListener("visibilitychange", blur);
    };
  }, [state.capabilities?.operator_control,reconnect]);
  const allowed = !!state.capabilities?.operator_control;
  const gripperDisabled=state.status?.gripper_command_profile?.enabled===false;
  return (
    <section className="card">
      <div className="card-header">
        实机控制 ·{" "}
        {state.control_stage === "shadow" ? "影子验证，不执行动作" : "受限实机"}
      </div>
      <div className="record-body">
        <p>
          {allowed
            ? "实体输入已开放"
            : "自动信号验收中 · 实体手柄与键盘控制未开放"}
        </p>
        <p>{state.status?.reason ?? "等待控制守护进程"}</p>
        {state.status?.recovery?.policy === 'bounded' && <p>自动恢复：{state.status.recovery.reason ?? '就绪'}
          {' '}· 最近60秒 {state.status.recovery.attempts_in_window}/{state.status.recovery.max_attempts} 次
          {' '}· 异常后松开并回中，恢复完成后重新按住允许动作键。</p>}
        {state.status?.kinematics && <p>运动学：{state.status.kinematics.kind === 'controller-local' ? '控制器采样局部模型' : '控制器原生 IK/FK'}
          {state.status.kinematics.kind === 'controller-local' && <> · 原生准备复核{state.status.kinematics.prepared ? '已通过' : '待验证'}
            {state.status.kinematics.actual_residual_m != null && <> · 实际位姿偏差 {(1000*state.status.kinematics.actual_residual_m).toFixed(3)} mm / {((state.status.kinematics.actual_residual_rad ?? 0)*180/Math.PI).toFixed(3)}°</>}</>}
        </p>}
        {state.status?.first_fault && <p>首个故障：{state.status.first_fault.reason} · 重新准备成功后清除</p>}
        {allowed&&state.control_stage==='live'&&<p><button disabled={state.status?.armed||enabled===1||robotMode!==4||state.status?.enabled_once||state.status?.commissioning?.state==='running'} onClick={()=>rpc('enable')}>{enabled===1?'机械臂已使能':'独立使能（一次）'}</button><button disabled={gripperDisabled||state.status?.armed||state.status?.gripper_initialized||state.status?.gripper_faulted||state.status?.commissioning?.state==='running'||robotMode!==5} onClick={()=>rpc('initialize-gripper')}>{gripperDisabled?'夹爪暂未开放':state.status?.gripper_initialized?'夹爪已初始化':'独立初始化夹爪15%'}</button>使能后须稳定至少5秒，再准备控制。</p>}
        <p>
          最近Stop回包：
          {state.status?.stop_acknowledged === true
            ? "已确认"
            : state.status?.stop_acknowledged === false
              ? "未确认"
              : "尚未请求"}{" "}
          · 静止回读：{state.status?.stopped_feedback ? "已确认" : "尚未确认"}
        </p>
        {state.status?.stop_confirmation && <p>停止请求：{state.status.stop_confirmation.send_confirmed ? '已发送' : '未确认发送'}
          {state.status.stop_confirmation.send_duration_ms != null && <> · 发送 {state.status.stop_confirmation.send_duration_ms.toFixed(1)} ms</>}
          {state.status.stop_confirmation.confirmation_duration_ms != null && <> · 回包 {state.status.stop_confirmation.confirmation_duration_ms.toFixed(1)} ms</>}
          {!state.status.stop_confirmation.confirmation_complete && <> · 等待回包，禁止新目标</>}
          {state.status.stop_confirmation.error && <> · {state.status.stop_confirmation.error}</>}
        </p>}
        <p>
          运动上限{" "}
          {(1000 * (state.status?.limits?.translation_m_s ?? 0)).toFixed(1)}{" "}
          mm/s ·{" "}
          {(
            ((state.status?.limits?.rotation_rad_s ?? 0) * 180) /
            Math.PI
          ).toFixed(1)}{" "}
          °/s · 工作区半径{" "}
          {(1000 * (state.status?.limits?.radius_m ?? 0)).toFixed(0)} mm
        </p>
        {gripperDisabled?<p>当前开放机械臂控制与示教录制。夹爪未通过连续控制验收，CAN配置、发送和夹爪输入均禁用。</p>:<p>夹爪目标速率 {(100*(state.status?.limits?.gripper_s??0)).toFixed(1)} 个百分点/秒 · Force{state.status?.gripper_command_profile?.force??'未知'} / Vel{state.status?.gripper_command_profile?.velocity??'未知'}（协议值）</p>}
        {state.status?.transaction_budget&&<p>控制周期 {state.status.transaction_budget.target_ms} ms · 事务余量至 {state.status.transaction_budget.hard_deadline_ms} ms · 已容纳短时延迟 {state.status.transaction_budget.tolerated_jitter_count} 次</p>}
        <select
          value={source}
          onChange={(e) => {
            rpc("stop");
            keys.current.clear();
            setSource(e.target.value as Source);
            e.target.blur();
          }}
        >
          <option value="keyboard">键盘</option>
          <option value="gamepad">手柄</option>
        </select>
        <button
          disabled={!allowed || !input.connected || input.deadman || !input.neutral || connection!=='已连接' || state.status?.kinematics?.connection_ready === false || state.status?.stop_acknowledged === false}
          onClick={() => rpc("arm")}
        >
          准备控制
        </button>
        <button disabled={!allowed} onClick={() => rpc("stop")}>
          软件停止
        </button>
        {allowed&&connection==='已断开'&&<button onClick={()=>{keys.current.clear();setReconnect(n=>n+1);}}>重新连接 · 回中后准备</button>}
        <button disabled={input.deadman||!input.neutral} onClick={()=>setMode(m=>m==='translation'?'rotation':'translation')}>{mode==='translation'?'切换旋转':'切换平移'}</button>
        <button disabled={input.deadman||!input.neutral} onClick={()=>setFrame(f=>f==='base'?'tool':'base')}>{frame==='base'?'切换工具坐标':'切换基座坐标'}</button>
        <p>本地输入：{input.connected?(source==='gamepad'?input.identity:'键盘已就绪'):'未检测到标准布局手柄'} · {input.deadman?'允许动作键按下':'允许动作键松开'} · {input.fine?'25%精细':'普通速度'} · {input.neutral?'回中':'有轴输入'}{allowed?` · 控制${connection}`:' · 仅预览，未发送'}</p>
        {allowed&&inputHint&&<p role="status">{inputHint}</p>}
        <div className="input-monitor">{[...input.axes,input.grip].map((v,n)=><div key={n}><label>{n===3?'张开 / 闭合':(mode==='translation'?['X','Y','Z']:['Rx','Ry','Rz'])[n]} <strong>{v.toFixed(2)}</strong></label><div className="input-track"><span style={{left:`${50+50*v}%`}} /></div></div>)}</div>
        <p>
          {mode === "translation" ? "平移" : "旋转"} ·{" "}
          {frame === "base" ? "基座坐标" : "工具坐标"} · LB / Space允许动作，RB
          / Shift精细操作
        </p>
        <p className="input-help">键盘：A/D、S/W、Q/E 三轴 · Tab 平移/旋转 · C 基座/工具 · F/R 夹爪 · Enter 录制 · 1 成功 / 2 失败 · Esc 停止。手柄：X/Y 切换 · LT/RT 夹爪 · Start 录制 · A/B 结束 · Back 停止。切换模式前须松开并回中，文本编辑时屏蔽控制。</p>
        {notice && <p>{notice}</p>}
      </div>
    </section>
  );
}
