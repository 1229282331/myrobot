import { useEffect, useRef, useState } from "react";
import {
  Activity,
  ArrowUpRight,
  CircleDot,
  Gamepad2,
  Keyboard,
  Layers3,
  Pause,
  Play,
  Radio,
  Rotate3d,
  Save,
  Square,
  Target,
  Video,
  Download,
  Check,
  X,
  ChevronRight,
  Crosshair,
} from "lucide-react";
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
type Row = {
  sample_index: number;
  observation: {
    joints_rad: number[];
    gripper: number;
    tcp_position_m: number[];
  };
  effective_action: { joints_rad: number[] };
  image: string;
};
type Episode = {
  id: string;
  task: string;
  result: string;
  samples: number;
  complete: boolean;
};
type Dataset = {
  id: string;
  frames: number;
  episodes: { result: string; frames: number }[];
  validated: boolean;
};
type DatasetFrame = {
  index: number;
  total: number;
  episode_index: number;
  frame_index: number;
  timestamp_s: number;
  task: string;
  state: number[];
  action: number[];
  image_jpeg: string;
  reader: string;
};
type Live = {
  observation?: {
    joints_rad: number[];
    tcp_position_m: number[];
    tcp_quaternion_xyzw: number[];
    gripper: number;
    frame_id: number;
  };
  status?: { armed: boolean; reason: string };
  images?: { overview_jpeg: string; wrist_png: string };
  recording?: { active: boolean; samples: number; episode: string };
  error?: string;
};
const resultText: Record<string, string> = {
  success: "成功",
  failure: "失败",
  aborted: "中止",
};
const wsUrl = (path: string) =>
  `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}${path}`;
export default function App() {
  const [live, setLive] = useState<Live>({}),
    [status, setStatus] = useState({ armed: false, reason: "连接模拟网关…" });
  const [source, setSource] = useState<Source>("gamepad"),
    [mode, setMode] = useState<Mode>("translation"),
    [frame, setFrame] = useState<Frame>("base");
  const [input, setInput] = useState<Input>(fromGamepad(null)),
    [task, setTask] = useState("移动末端到黄色标记附近");
  const [episodes, setEpisodes] = useState<Episode[]>([]),
    [selected, setSelected] = useState<Episode | null>(null),
    [rows, setRows] = useState<Row[]>([]),
    [cursor, setCursor] = useState(0);
  const [tab, setTab] = useState("control"),
    [notice, setNotice] = useState(""),
    [exportState, setExportState] = useState("idle"),
    [exportLog, setExportLog] = useState("");
  const [datasets, setDatasets] = useState<Dataset[]>([]),
    [dataset, setDataset] = useState<Dataset | null>(null),
    [datasetFrame, setDatasetFrame] = useState<DatasetFrame | null>(null),
    [datasetCursor, setDatasetCursor] = useState(0),
    [loadingDataset, setLoadingDataset] = useState(false);
  const keys = useRef(new Set<string>()),
    ctrl = useRef<WebSocket | null>(null),
    config = useRef({ source, mode, frame, task, tab });
  const lease = useRef(""),
    seq = useRef(0),
    pending = useRef(false),
    ops = useRef<Record<string, unknown>[]>([]),
    previousButtons = useRef<boolean[]>([]);
  const current = useRef(input),
    latest = useRef(live);
  config.current = { source, mode, frame, task, tab };
  latest.current = live;
  current.current = input;
  const send = (data: Record<string, unknown>) => {
    ops.current.push(data);
  };
  const refreshEpisodes = () =>
    fetch("/api/episodes")
      .then((r) => r.json())
      .then(setEpisodes);
  const changeMode = () => {
    if (current.current.neutral && !current.current.deadman)
      setMode((m) => (m === "translation" ? "rotation" : "translation"));
    else setNotice("请松开允许动作键并回中后切换模式");
  };
  const changeFrame = () => {
    if (current.current.neutral && !current.current.deadman)
      setFrame((f) => (f === "base" ? "tool" : "base"));
    else setNotice("请松开允许动作键并回中后切换坐标系");
  };
  const actions = useRef({ changeMode, changeFrame });
  actions.current = { changeMode, changeFrame };
  useEffect(() => {
    let active = true;
    let retry: ReturnType<typeof setTimeout>,
      observerRetry: ReturnType<typeof setTimeout>;
    let observations: WebSocket | null = null,
      pendingSince = 0;
    const connect = () => {
      if (!active) return;
      const ws = new WebSocket(wsUrl("/ws/control"));
      ctrl.current = ws;
      ws.onmessage = (e) => {
        if (!active || ctrl.current !== ws) return;
        const reply = JSON.parse(e.data);
        pending.current = false;
        if (reply.status) {
          lease.current = reply.status.lease;
          setStatus(reply.status);
        }
        if (!reply.ok) setNotice(reply.error ?? "命令未被接受");
        if (reply.recording && !reply.recording.active) refreshEpisodes();
      };
      ws.onclose = () => {
        if (!active || ctrl.current !== ws) return;
        lease.current = "";
        pending.current = false;
        ops.current = [];
        setStatus({ armed: false, reason: "控制连接断开，需要重新准备" });
        retry = setTimeout(connect, 1000);
      };
    };
    connect();
    const observe = () => {
      if (!active) return;
      const ws = new WebSocket(wsUrl("/ws/state"));
      observations = ws;
      ws.onmessage = (e) => {
        if (!active) return;
        const data = JSON.parse(e.data);
        setLive(data);
        if (data.status && lease.current) setStatus(data.status);
      };
      ws.onclose = () => {
        if (!active) return;
        setLive({});
        ops.current = [];
        ops.current.push({ operation: "stop" });
        setNotice("观测连接断开：已停止，重连后需要重新准备");
        observerRetry = setTimeout(observe, 1000);
      };
    };
    observe();
    const keydown = (e: KeyboardEvent) => {
      const editing =
        e.target instanceof HTMLElement &&
        !!e.target.closest('input,textarea,select,[contenteditable="true"]');
      if (editing) {
        keys.current.clear();
        return;
      }
      if (["Space", "Tab", "ArrowUp", "ArrowDown"].includes(e.code))
        e.preventDefault();
      keys.current.add(e.code);
      if (
        e.repeat ||
        config.current.source !== "keyboard" ||
        config.current.tab !== "control"
      )
        return;
      if (e.code === "Tab") actions.current.changeMode();
      if (e.code === "KeyC") actions.current.changeFrame();
      if (e.code === "Escape") ops.current.unshift({ operation: "stop" });
      if (e.code === "Enter" && !latest.current.recording?.active)
        send({ operation: "record/start", task: config.current.task });
      if (e.code === "Digit1" && latest.current.recording?.active)
        send({ operation: "record/finish", result: "success" });
      if (e.code === "Digit2" && latest.current.recording?.active)
        send({ operation: "record/finish", result: "failure" });
    };
    const keyup = (e: KeyboardEvent) => keys.current.delete(e.code);
    const blur = () => {
      keys.current.clear();
      ops.current = [{ operation: "stop" }];
    };
    window.addEventListener("keydown", keydown);
    window.addEventListener("keyup", keyup);
    window.addEventListener("blur", blur);
    document.addEventListener("visibilitychange", blur);
    const timer = setInterval(() => {
      const c = config.current;
      const editing =
        document.activeElement instanceof HTMLElement &&
        !!document.activeElement.closest(
          'input,textarea,select,[contenteditable="true"]',
        );
      const pad = getAvailableGamepads()[0] ?? null;
      const i =
        c.source === "gamepad"
          ? fromGamepad(pad)
          : fromKeyboard(keys.current, editing);
      current.current = i;
      setInput(i);
      if (c.source === "gamepad" && c.tab === "control") {
        const edge = (b: number) => i.buttons[b] && !previousButtons.current[b];
        if (edge(2)) actions.current.changeMode();
        if (edge(3)) actions.current.changeFrame();
        if (edge(8)) ops.current.unshift({ operation: "stop" });
        if (edge(9) && !latest.current.recording?.active)
          send({ operation: "record/start", task: c.task });
        if (edge(0) && latest.current.recording?.active)
          send({ operation: "record/finish", result: "success" });
        if (edge(1) && latest.current.recording?.active)
          send({ operation: "record/finish", result: "failure" });
        previousButtons.current = i.buttons;
      }
      if (pending.current && performance.now() - pendingSince > 750) {
        ctrl.current?.close();
        return;
      }
      if (
        ctrl.current?.readyState !== WebSocket.OPEN ||
        !lease.current ||
        pending.current
      )
        return;
      const next = ops.current.shift() ?? {
        ...command(
          i,
          c.mode,
          c.frame,
          document.hasFocus() &&
            !document.hidden &&
            !editing &&
            c.tab === "control",
        ),
        source: c.source,
        lease: lease.current,
        seq: ++seq.current,
      };
      pending.current = true;
      pendingSince = performance.now();
      ctrl.current.send(JSON.stringify(next));
    }, 50);
    return () => {
      active = false;
      clearTimeout(retry);
      clearTimeout(observerRetry);
      clearInterval(timer);
      ctrl.current?.close();
      observations?.close();
      lease.current = "";
      pending.current = false;
      ops.current = [];
      window.removeEventListener("keydown", keydown);
      window.removeEventListener("keyup", keyup);
      window.removeEventListener("blur", blur);
      document.removeEventListener("visibilitychange", blur);
    };
  }, []);
  useEffect(() => {
    refreshEpisodes();
    const t = setInterval(
      () =>
        fetch("/api/state")
          .then((r) => r.json())
          .then((d) => {
            setExportState(d.export.state);
            setExportLog(d.export.log ?? "");
          }),
      1500,
    );
    return () => clearInterval(t);
  }, []);
  const selectEpisode = async (e: Episode) => {
    setSelected(e);
    setCursor(0);
    const d = await fetch(`/api/episodes/${e.id}`).then((r) => r.json());
    setRows(d.rows);
  };
  const exportEpisode = async () => {
    if (!selected) return;
    const r = await fetch(`/api/export/${selected.id}`, { method: "POST" });
    const d = await r.json();
    if (!r.ok) setNotice(d.detail);
    else setExportState("running");
  };
  useEffect(() => {
    if (tab !== "datasets" || !dataset) return;
    const controller = new AbortController();
    setLoadingDataset(true);
    const timer = setTimeout(
      () =>
        fetch(`/api/datasets/${dataset.id}/frames/${datasetCursor}`, {
          signal: controller.signal,
        })
          .then(async (r) => {
            if (!r.ok) throw new Error((await r.json()).detail);
            return r.json();
          })
          .then((d) => {
            setDatasetFrame(d);
            setLoadingDataset(false);
          })
          .catch((e) => {
            if (e.name !== "AbortError") {
              setNotice(String(e));
              setLoadingDataset(false);
            }
          }),
      150,
    );
    return () => {
      clearTimeout(timer);
      controller.abort();
    };
  }, [tab, dataset, datasetCursor]);
  const navigate = (target: string) => {
    ops.current = [{ operation: "stop" }];
    keys.current.clear();
    setTab(target);
    if (target === "episodes") refreshEpisodes();
    if (target === "datasets")
      fetch("/api/datasets")
        .then((r) => r.json())
        .then(setDatasets);
  };
  const obs = live.observation,
    row = rows[cursor];
  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-icon">
            <Layers3 size={22} />
          </div>
          <div>
            <strong>E6 WORKSPACE</strong>
            <span>遥操作与示教采集</span>
          </div>
        </div>
        <div className="section-label">工作空间</div>
        <button
          className={`nav-item ${tab === "control" ? "selected" : ""}`}
          onClick={() => navigate("control")}
        >
          <Gamepad2 size={19} />
          遥操作台
          <ChevronRight size={15} />
        </button>
        <button
          className={`nav-item ${tab === "episodes" ? "selected" : ""}`}
          onClick={() => navigate("episodes")}
        >
          <Video size={19} />
          示教数据<span className="count">{episodes.length}</span>
        </button>
        <button
          className={`nav-item ${tab === "datasets" ? "selected" : ""}`}
          onClick={() => navigate("datasets")}
        >
          <Layers3 size={19} />
          LeRobot 数据
          <ChevronRight size={15} />
        </button>
        <div className="sidebar-bottom">
          <div className="sim-label">
            <CircleDot size={14} /> SIMULATION
          </div>
          <p>
            模型随动与数据链路验证
            <br />
            当前无硬件连接
          </p>
          <div className="version">
            E6 / 6 AXES <span>v0.1</span>
          </div>
        </div>
      </aside>
      <main>
        <header>
          <div className="breadcrumb">
            工作空间 <ChevronRight size={14} />{" "}
            {tab === "control"
              ? "遥操作台"
              : tab === "datasets"
                ? "LeRobot 数据"
                : "示教数据"}
          </div>
          <div className="header-right">
            <span className={`dot ${obs ? "online" : ""}`} />
            {obs ? "模拟网关在线" : "连接中"}
            <span className="separator" />
            <span>本地会话</span>
          </div>
        </header>
        <div className="page-title">
          <div>
            <div className="eyebrow">TELEOPERATION / E6</div>
            <h1>
              {tab === "control"
                ? "末端遥操作"
                : tab === "datasets"
                  ? "LeRobot 数据读回"
                  : "示教数据"}
            </h1>
            <p>
              {tab === "control"
                ? "调整末端位置与姿态，记录可追溯的示教片段。"
                : "检查图像、状态和动作，导出用于学习的模拟数据。"}
            </p>
          </div>
          <div className="title-badge">
            <Radio size={16} />
            模拟环境
          </div>
        </div>
        {notice && (
          <div className="notice">
            {notice}
            <button onClick={() => setNotice("")}>
              <X size={15} />
            </button>
          </div>
        )}
        {tab === "control" ? (
          <>
            <div className="work-grid">
              <section className="viewport card">
                <div className="card-header">
                  <span>
                    <Layers3 size={17} />
                    机械臂全景
                  </span>
                  <span className="tiny-tag">E6 · 运动学模型</span>
                </div>
                <div className="view-image">
                  {live.images ? (
                    <img
                      src={`data:image/jpeg;base64,${live.images.overview_jpeg}`}
                      alt="E6机械臂全景"
                    />
                  ) : (
                    <div className="empty">
                      <Layers3 size={35} />
                      <p>正在加载机械臂模型</p>
                    </div>
                  )}
                  <div className="view-overlay">
                    <span>
                      <span className="dot online" /> LIVE
                    </span>
                    <span>
                      <span className="axis red">X</span>
                      <span className="axis green">Y</span>
                      <span className="axis blue">Z</span>末端坐标轴
                    </span>
                  </div>
                </div>
                <div className="view-footer">
                  <Target size={15} />
                  黄色标记是模拟目标<span>TCP：Link6原点</span>
                </div>
              </section>
              <section className="wrist card">
                <div className="card-header">
                  <span>
                    <Video size={17} />
                    虚拟腕部相机
                  </span>
                  <span className="tiny-tag">640 × 360</span>
                </div>
                {live.images ? (
                  <img
                    className="wrist-image"
                    src={`data:image/png;base64,${live.images.wrist_png}`}
                    alt="虚拟腕部相机画面"
                  />
                ) : (
                  <div className="empty small">等待画面</div>
                )}
                <div className="camera-meta">
                  <span className="dot online" />
                  模拟图像<span>帧 {obs?.frame_id ?? "—"}</span>
                </div>
                <div className="pose-block">
                  <div className="section-label">末端位置 / MM</div>
                  <div className="position-grid">
                    {["X", "Y", "Z"].map((label, i) => (
                      <div key={label}>
                        <label>{label}</label>
                        <strong>
                          {obs
                            ? (obs.tcp_position_m[i] * 1000).toFixed(1)
                            : "—"}
                        </strong>
                      </div>
                    ))}
                  </div>
                  <div className="section-label quaternion">姿态 / XYZW</div>
                  <div className="quat-values">
                    {obs?.tcp_quaternion_xyzw
                      .map((v) => v.toFixed(3))
                      .join(" · ") ?? "—"}
                  </div>
                </div>
              </section>
            </div>
            <div className="bottom-grid">
              <section className="input-card card">
                <div className="card-header">
                  <span>
                    <Gamepad2 size={17} />
                    输入与随动
                  </span>
                  <span
                    className={`input-indicator ${input.connected ? "good" : ""}`}
                  >
                    {input.connected ? "已连接" : "未检测到手柄"}
                  </span>
                </div>
                <div className="controls-body">
                  <div className="segmented">
                    {(["gamepad", "keyboard"] as Source[]).map((s) => (
                      <button
                        key={s}
                        className={source === s ? "active" : ""}
                        onClick={() => {
                          send({ operation: "stop" });
                          setSource(s);
                        }}
                      >
                        {s === "gamepad" ? (
                          <Gamepad2 size={16} />
                        ) : (
                          <Keyboard size={16} />
                        )}{" "}
                        {s === "gamepad" ? "游戏手柄" : "键盘"}
                      </button>
                    ))}
                  </div>
                  <div className="device-name">{input.identity}</div>
                  <div className="control-switches">
                    <button onClick={changeMode}>
                      <Rotate3d size={16} />
                      {mode === "translation" ? "平移 XYZ" : "旋转 RxRyRz"}
                      <span>{source === "gamepad" ? "X" : "Tab"}</span>
                    </button>
                    <button onClick={changeFrame}>
                      <Crosshair size={16} />
                      {frame === "base" ? "基座坐标" : "工具坐标"}
                      <span>{source === "gamepad" ? "Y" : "C"}</span>
                    </button>
                  </div>
                  <div className="motion-bars">
                    {["X / Rx", "Y / Ry", "Z / Rz"].map((label, i) => (
                      <div key={label}>
                        <label>{label}</label>
                        <div className="axis-track">
                          <i style={{ left: `${50 + input.axes[i] * 43}%` }} />
                        </div>
                        <span>{input.axes[i].toFixed(2)}</span>
                      </div>
                    ))}
                  </div>
                  <div className="arm-row">
                    <button
                      className="primary"
                      disabled={
                        !input.connected ||
                        input.deadman ||
                        !input.neutral ||
                        status.armed
                      }
                      onClick={() => send({ operation: "arm" })}
                    >
                      <Play size={15} />
                      {status.armed ? "已准备" : "准备控制"}
                    </button>
                    <div
                      className={`hold-state ${input.deadman && status.armed ? "holding" : ""}`}
                    >
                      <CircleDot size={14} />
                      {input.deadman && status.armed
                        ? "随动中"
                        : `按住 ${source === "gamepad" ? "LB" : "Space"} 随动`}
                    </div>
                    <button
                      className="stop-button"
                      onClick={() => ops.current.unshift({ operation: "stop" })}
                    >
                      <Square size={14} />
                      停止
                    </button>
                  </div>
                  <p className="status-reason">{live.error ?? status.reason}</p>
                </div>
              </section>
              <section className="joints-card card">
                <div className="card-header">
                  <span>
                    <Activity size={17} />
                    模型状态
                  </span>
                  <span className="tiny-tag">RAD → DEG</span>
                </div>
                <div className="joint-grid">
                  {Array.from({ length: 6 }, (_, i) => (
                    <div key={i}>
                      <label>J{i + 1}</label>
                      <strong>
                        {obs
                          ? ((obs.joints_rad[i] * 180) / Math.PI).toFixed(2)
                          : "—"}
                        <small>°</small>
                      </strong>
                    </div>
                  ))}
                </div>
                <div className="gripper-block">
                  <div>
                    <span>夹爪开度</span>
                    <strong>{obs ? Math.round(obs.gripper * 100) : 0}%</strong>
                  </div>
                  <div className="progress">
                    <i style={{ width: `${(obs?.gripper ?? 0) * 100}%` }} />
                  </div>
                  <p>
                    {source === "gamepad"
                      ? "LT 张开 · RT 闭合 · RB 精细速度"
                      : "F 张开 · R 闭合 · Shift 精细速度"}
                  </p>
                </div>
              </section>
              <section className="record-card card">
                <div className="card-header">
                  <span>
                    <Save size={17} />
                    Episode采集
                  </span>
                  <span
                    className={`record-dot ${live.recording?.active ? "recording" : ""}`}
                  />
                </div>
                <div className="record-body">
                  <label>任务描述</label>
                  <textarea
                    value={task}
                    disabled={live.recording?.active}
                    onChange={(e) => setTask(e.target.value)}
                    rows={2}
                  />
                  <div className="record-count">
                    <strong>{live.recording?.samples ?? 0}</strong>
                    <span>
                      FRAMES{" "}
                      <i>
                        {((live.recording?.samples ?? 0) / 20).toFixed(1)} s
                      </i>
                    </span>
                  </div>
                  {!live.recording?.active ? (
                    <button
                      className="primary wide"
                      disabled={!obs || !task.trim()}
                      onClick={() => send({ operation: "record/start", task })}
                    >
                      <CircleDot size={15} />
                      开始录制
                    </button>
                  ) : (
                    <div className="finish-buttons">
                      <button
                        onClick={() =>
                          send({
                            operation: "record/finish",
                            result: "success",
                          })
                        }
                      >
                        <Check size={15} />
                        成功
                      </button>
                      <button
                        onClick={() =>
                          send({
                            operation: "record/finish",
                            result: "failure",
                          })
                        }
                      >
                        <X size={15} />
                        失败
                      </button>
                      <button
                        onClick={() =>
                          send({
                            operation: "record/finish",
                            result: "aborted",
                          })
                        }
                      >
                        <Pause size={14} />
                        中止
                      </button>
                    </div>
                  )}
                  <p>原始输入、目标动作、模型状态与图像同步关联。</p>
                </div>
              </section>
            </div>
            <div className="shortcut-strip">
              <Keyboard size={15} />
              {source === "gamepad"
                ? "LB 允许动作 · RB 精细 · X 模式 · Y 坐标 · Start 录制 · A/B 结束"
                : "Space 允许动作 · A/D、S/W、Q/E 三轴 · Tab 模式 · C 坐标 · Enter 录制 · 1/2 结束"}
              <span>松开允许动作键即保持位置</span>
            </div>
          </>
        ) : tab === "datasets" ? (
          <div className="episode-layout">
            <section className="card episode-list">
              <div className="card-header">
                <span>
                  <Layers3 size={17} />
                  LeRobot Dataset v3
                </span>
              </div>
              {datasets.map((d) => (
                <button
                  key={d.id}
                  className={`episode-item ${dataset?.id === d.id ? "active" : ""}`}
                  onClick={() => {
                    setDataset(d);
                    setDatasetCursor(0);
                    setDatasetFrame(null);
                  }}
                >
                  <span className="outcome success">官方读取已验证</span>
                  <strong>{d.id}</strong>
                  <small>
                    {d.frames} 帧 · {d.episodes.length} 个episode
                  </small>
                </button>
              ))}
              {!datasets.length && (
                <div className="empty">完成导出后可在这里浏览。</div>
              )}
            </section>
            <section className="card episode-detail">
              <div className="card-header">
                <span>
                  <Video size={17} />
                  官方LeRobot / PyAV读回
                </span>
                <span>{loadingDataset ? "读取中…" : "本地模拟数据"}</span>
              </div>
              {dataset && datasetFrame ? (
                <div className="episode-content">
                  <img
                    src={`data:image/jpeg;base64,${datasetFrame.image_jpeg}`}
                    alt="LeRobot读回视频帧"
                  />
                  <input
                    aria-label="LeRobot帧"
                    type="range"
                    min="0"
                    max={dataset.frames - 1}
                    value={datasetCursor}
                    onChange={(e) => setDatasetCursor(Number(e.target.value))}
                  />
                  <div className="timeline-meta">
                    <span>
                      索引 {datasetFrame.index} / {dataset.frames - 1}
                    </span>
                    <span>
                      Episode {datasetFrame.episode_index} · 帧{" "}
                      {datasetFrame.frame_index}
                    </span>
                    <span>{datasetFrame.timestamp_s.toFixed(2)} s</span>
                  </div>
                  <p>{datasetFrame.task}</p>
                  <div className="section-label">关节观测与有效动作 / DEG</div>
                  <div className="review-joints">
                    {datasetFrame.state.slice(0, 6).map((v, i) => (
                      <div key={i}>
                        <label>J{i + 1}</label>
                        <strong>{((v * 180) / Math.PI).toFixed(2)}</strong>
                        <small>
                          目标{" "}
                          {((datasetFrame.action[i] * 180) / Math.PI).toFixed(
                            2,
                          )}
                        </small>
                      </div>
                    ))}
                  </div>
                  <p className="export-status">
                    夹爪 {(datasetFrame.state[6] * 100).toFixed(1)}% →{" "}
                    {(datasetFrame.action[6] * 100).toFixed(1)}% ·{" "}
                    {datasetFrame.reader}
                  </p>
                </div>
              ) : (
                <div className="empty">
                  <Video size={35} />
                  <p>
                    {loadingDataset
                      ? "正在加载官方数据读取器…"
                      : "选择一个已导出的数据集"}
                  </p>
                </div>
              )}
            </section>
          </div>
        ) : (
          <div className="episode-layout">
            <section className="card episode-list">
              <div className="card-header">
                <span>
                  <Video size={17} />
                  本地记录
                </span>
                <span>{episodes.length} 个</span>
              </div>
              {episodes.length ? (
                episodes.map((e) => (
                  <button
                    key={e.id}
                    className={`episode-item ${selected?.id === e.id ? "active" : ""}`}
                    onClick={() => selectEpisode(e)}
                  >
                    <span className={`outcome ${e.result}`}>
                      {resultText[e.result]}
                    </span>
                    <strong>{e.task}</strong>
                    <small>
                      {e.samples} 帧 · {(e.samples / 20).toFixed(1)} 秒
                    </small>
                  </button>
                ))
              ) : (
                <div className="empty">录制完成后，片段会出现在这里。</div>
              )}
            </section>
            <section className="card episode-detail">
              <div className="card-header">
                <span>
                  <Layers3 size={17} />
                  片段检查
                </span>
                {selected && (
                  <button
                    className="export-button"
                    disabled={
                      !selected.complete ||
                      selected.result === "aborted" ||
                      exportState === "running"
                    }
                    onClick={exportEpisode}
                  >
                    <Download size={15} />
                    {exportState === "running" ? "正在导出" : "导出LeRobot"}
                  </button>
                )}
              </div>
              {selected && row ? (
                <div className="episode-content">
                  <img
                    src={`/api/episodes/${selected.id}/${row.image}`}
                    alt="所选episode画面"
                  />
                  <input
                    aria-label="Episode帧"
                    type="range"
                    min="0"
                    max={rows.length - 1}
                    value={cursor}
                    onChange={(e) => setCursor(Number(e.target.value))}
                  />
                  <div className="timeline-meta">
                    <span>
                      帧 {cursor + 1} / {rows.length}
                    </span>
                    <span>{(cursor / 20).toFixed(2)} s</span>
                    <span>{resultText[selected.result]}</span>
                  </div>
                  <div className="section-label">关节观测与目标 / DEG</div>
                  <div className="review-joints">
                    {row.observation.joints_rad.map((v, i) => (
                      <div key={i}>
                        <label>J{i + 1}</label>
                        <strong>{((v * 180) / Math.PI).toFixed(2)}</strong>
                        <small>
                          目标{" "}
                          {(
                            (row.effective_action.joints_rad[i] * 180) /
                            Math.PI
                          ).toFixed(2)}
                        </small>
                      </div>
                    ))}
                  </div>
                  <p className="export-status">
                    {exportState === "done"
                      ? "✓ LeRobot导出与读取验证完成"
                      : exportState === "failed"
                        ? "导出失败，请查看详情"
                        : "模拟数据 · 观测先于有效动作"}
                  </p>
                  {exportLog && (
                    <details>
                      <summary>导出详情</summary>
                      <pre>{exportLog}</pre>
                    </details>
                  )}
                </div>
              ) : (
                <div className="empty">
                  <Video size={35} />
                  <p>选择一个片段查看图像和动作</p>
                </div>
              )}
            </section>
          </div>
        )}
        <footer>
          <span>
            <CircleDot size={12} />
            运动学模拟 · 不含接触抓取
          </span>
          <span>
            20 Hz · 本地数据 · 无自动上传 <ArrowUpRight size={12} />
          </span>
        </footer>
      </main>
    </div>
  );
}
