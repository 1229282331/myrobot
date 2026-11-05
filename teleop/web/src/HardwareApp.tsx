import { useEffect, useState } from "react";
import HardwareControlPanel from "./HardwareControlPanel";
type Source = {
  valid: boolean;
  available: boolean;
  reason?: string;
  age_ms: number | null;
  opening?: number;
  command_opening?: number;
  minimum_target_delta_counts?: number;
  pc_age_upper_ms?: number | null;
  device_frame_id?: number;
  metadata?: { uvc_pts_available?: boolean };
  valid_fraction?: number;
  decoded?: {
    robot_mode: number;
    enable_status: number;
    error_status: number;
    tool_index: number;
    user_index: number;
    controller_pose_raw_mm_deg: number[];
    payload_parameter_kg: number;
    tool_configuration_raw_mm_deg: number[];
  };
};
type Live = {
  control_stage?: string;
  capabilities?: {operator_control?: boolean};
  input?: unknown;
  candidate_action?: unknown;
  effective_action?: unknown;
  control?: {tick?: {requested_target?: unknown;last_accepted_action?: unknown;ik_result?: unknown;events?: unknown;pre_action_robot?: unknown}};
  observation?: {
    valid: boolean;
    joints_rad: number[] | null;
    tcp_position_m?: number[];
    tcp_quaternion_xyzw?: number[];
    model_link6_position_m?: number[];
    model_link6_quaternion_xyzw?: number[];
    gripper?: number | null;
  };
  sources?: Record<string, Source>;
  images?: { overview_jpeg?: string; wrist_jpeg?: string; depth_jpeg?: string };
  stats?: Record<string, unknown>;
  transport?: {
    round_trip_ms?: number;
    age_lower_ms?: number;
    age_upper_ms?: number;
    uncertainty_ms?: number;
  };
  preview?: Record<
    string,
    { device_frame_id?: number; age_upper_ms?: number; valid: boolean }
  >;
  recording?: {
    active: boolean;
    samples: number;
    pending?: boolean;
    phase?: string;
    expected_media?: number;
    verified_media?: number;
  };
  error?: string;
  pc_error?: string;
  associations?: Record<
    string,
    { valid: boolean; receive_delta_ms: number | null }
  >;
};
type Episode = {
  id: string;
  task: string;
  result: string;
  samples: number;
  complete: boolean;
};
type Row = {
  input?: unknown;
  candidate_action?: unknown;
  effective_action?: unknown;
  control?: Live['control'];
  image: string;
  elapsed_s: number;
  observation: { joints_rad: number[] };
  sources: Record<
    string,
    {
      media_pending?: boolean;
      file?: string;
      age_ms?: number;
      repeated?: boolean;
      valid_fraction?: number;
    }
  >;
  associations: Live["associations"];
};
export default function HardwareApp({
  controlBackend = false,
}: {
  controlBackend?: boolean;
}) {
  const [live, setLive] = useState<Live>({}),
    [task, setTask] = useState("E6与Gemini E真实观测链路检查"),
    [notice, setNotice] = useState(""),
    [tab, setTab] = useState("observe"),
    [episodes, setEpisodes] = useState<Episode[]>([]),
    [selected, setSelected] = useState<Episode | null>(null),
    [rows, setRows] = useState<Row[]>([]),
    [cursor, setCursor] = useState(0),
    [rowOffset, setRowOffset] = useState(0),
    [totalRows, setTotalRows] = useState(0);
  const refresh = () =>
    fetch("/api/episodes")
      .then((r) => r.json())
      .then(setEpisodes);
  useEffect(() => {
    let active = true,
      ws: WebSocket,
      retry: ReturnType<typeof setTimeout>;
    const connect = () => {
      if (!active) return;
      ws = new WebSocket(
        `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/state`,
      );
      ws.onmessage = (e) => {
        if (active) setLive(JSON.parse(e.data));
      };
      ws.onclose = () => {
        if (active) {
          setLive({ error: "观测连接断开：录制已中止，重连后需重新开始" });
          retry = setTimeout(connect, 1000);
        }
      };
    };
    connect();
    refresh();
    return () => {
      active = false;
      clearTimeout(retry);
      ws?.close();
    };
  }, []);
  const record = async (start: boolean) => {
    const r = await fetch(start ? "/api/record/start" : "/api/record/finish", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(
        start ? { task } : { result: controlBackend ? "success" : "observed" },
      ),
    });
    const d = await r.json();
    if (!r.ok) setNotice(d.detail);
    else {
      setLive((l) => ({ ...l, recording: d }));
      refresh();
    }
  };
  const robot = live.sources?.robot,
    decoded = robot?.decoded,
    row = rows[cursor - rowOffset];
  const moveCursor = async (value: number) => {
    setCursor(value);
    if (selected && (value < rowOffset || value >= rowOffset + rows.length)) {
      const offset = Math.floor(value / 100) * 100;
      const d = await fetch(
        `/api/episodes/${selected.id}?offset=${offset}&limit=100`,
      ).then((r) => r.json());
      setRows(d.rows);
      setRowOffset(offset);
    }
  };
  const sampling = live.stats?.sampling as
    | {
        rgb_selection_hz: number;
        depth_selection_hz: number;
        preview_hz?: number;
      }
    | undefined;
  const can = (
    live.stats?.can_interface as
      | {
          operstate: string;
          stats64?: { rx: { packets: number }; tx: { packets: number } };
        }[]
      | undefined
  )?.[0];
  const fps = (name: string) => {
    const s = live.stats?.[name] as { actual_fps?: number } | undefined;
    return s?.actual_fps?.toFixed(1) ?? "—";
  };
  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand">
          <div>
            <strong>E6 WORKSPACE</strong>
            <span>{controlBackend ? "实机遥操作验证" : "真实设备观测"}</span>
          </div>
        </div>
        <div className="section-label">工作空间</div>
        <button
          className={`nav-item ${tab === "observe" ? "selected" : ""}`}
          onClick={() => setTab("observe")}
        >
          设备观测
        </button>
        <button
          className={`nav-item ${tab === "episodes" ? "selected" : ""}`}
          onClick={() => {
            setTab("episodes");
            refresh();
          }}
        >
          观测记录
        </button>
        <div className="sidebar-bottom">
          <div className="sim-label">
            {controlBackend
              ? "HARDWARE · CONTROL VALIDATION"
              : "HARDWARE · OBSERVE ONLY"}
          </div>
          <p>
            E6 + Gemini E<br />
            {controlBackend
              ? (live.capabilities?.operator_control ? "手柄 / 键盘 · 受限实机控制" : "自动信号验证 · 操作员控制锁定")
              : "动作通道关闭 · 程序不发送CAN"}
          </p>
        </div>
      </aside>
      <main>
        <header>
          <span>工作空间 / 真实设备观测</span>
          <span>{live.observation?.valid ? "观测有效" : "等待有效新观测"}</span>
        </header>
        <div className="page-title">
          <div>
            <div className="eyebrow">OBSERVATION / E6</div>
            <h1>
              {tab === "observe"
                ? controlBackend
                  ? (live.capabilities?.operator_control ? "E6 实机遥操作" : "实机遥操作验证")
                  : "真实设备观测"
                : "原始片段检查"}
            </h1>
            <p>真实关节与RGB-D · 以X5接收时间关联 · 本地记录</p>
          </div>
          <div className="title-badge">{controlBackend?(live.control_stage==='shadow'?'影子模式 · 零动作':'受限实机 · 空载测试'):'只读模式'}</div>
        </div>
        {(notice || live.error || (live.pc_error && !live.observation?.valid)) && (
          <div className="notice">{notice || live.error || live.pc_error}</div>
        )}
        {!notice && !live.error && live.pc_error && live.observation?.valid && (
          <details className="recent-link-issue">
            <summary>当前观测已恢复 · 查看最近链路异常</summary>
            <p>{live.pc_error}</p>
          </details>
        )}
        {tab === "observe" ? (
          <>
            <div className="work-grid">
              <section className="card viewport">
                <div className="card-header">
                  机械臂模型示意<span>由实际关节驱动</span>
                </div>
                <div className="view-image">
                  {live.images?.overview_jpeg ? (
                    <img
                      src={`data:image/jpeg;base64,${live.images.overview_jpeg}`}
                      alt="实际关节驱动的模型示意"
                    />
                  ) : (
                    <div className="empty">等待实际关节</div>
                  )}
                </div>
                <div className="view-footer">
                  Link6原点 · 附件与夹爪开度未绘制
                </div>
              </section>
              <section className="card wrist">
                <div className="card-header">
                  Gemini E RGB<span>无线压缩预览 · 原图1280 × 720</span>
                </div>
                {live.images?.wrist_jpeg ? (
                  <img
                    className="wrist-image"
                    src={`data:image/jpeg;base64,${live.images.wrist_jpeg}`}
                    alt="真实RGB画面"
                  />
                ) : (
                  <div className="empty small">等待RGB</div>
                )}
                <div className="camera-meta">
                  源流 {fps("rgb")} fps · 选帧{" "}
                  {sampling?.rgb_selection_hz ?? "—"} Hz · 预览{" "}
                  {sampling?.preview_hz ?? "—"} Hz · 图帧{" "}
                  {live.preview?.rgb?.device_frame_id ??
                    live.sources?.rgb?.device_frame_id ??
                    "—"}{" "}
                  · PTS{" "}
                  {live.sources?.rgb?.metadata?.uvc_pts_available
                    ? "可用"
                    : "未知"}
                </div>
              </section>
            </div>
            <div className="bottom-grid">
              <section className="card">
                <div className="card-header">连接与源数据新鲜度</div>
                <div className="record-body">
                  {["robot", "rgb", "depth", "gripper"].map((n) => {
                    const s = live.sources?.[n];
                    return (
                      <p key={n}>
                        <strong>{n.toUpperCase()}</strong> ·{" "}
                        {s?.valid ? "有效" : (s?.reason ?? "等待")} · X5{" "}
                        {s?.age_ms?.toFixed(1) ?? "—"} ms · PC上界{" "}
                        {s?.pc_age_upper_ms?.toFixed(1) ?? "—"} ms
                      </p>
                    );
                  })}
                  <p>PC上界是原始源元数据的年龄，并非网络 ping。</p>
                  <p>
                    RGB预览年龄上界{" "}
                    {live.preview?.rgb?.age_upper_ms?.toFixed(1) ?? "—"} ms ·{" "}
                    {live.preview?.rgb?.valid ? "有效" : "尚未就绪或过期"}
                  </p>
                  <p>
                    传输年龄区间{" "}
                    {live.transport?.age_lower_ms?.toFixed(1) ?? "—"}–
                    {live.transport?.age_upper_ms?.toFixed(1) ?? "—"} ms ·
                    应用探测RTT{" "}
                    {live.transport?.round_trip_ms?.toFixed(1) ?? "—"} ms
                  </p>
                  <p>
                    CAN {can?.operstate ?? "未知"} · RX{" "}
                    {can?.stats64?.rx.packets ?? "—"} / TX{" "}
                    {can?.stats64?.tx.packets ?? "—"}
                  </p>
                  <p>
                    夹爪开度：
                    {live.observation?.gripper == null
                      ? "未知 · 当前遥测不可用"
                      : `${(100 * live.observation.gripper).toFixed(1)}%（实测）`}
                  </p>
                  {live.sources?.gripper?.valid && live.sources.gripper.command_opening != null && <p>
                    已发送开度目标 {(100 * live.sources.gripper.command_opening).toFixed(1)}%
                    {live.sources.gripper.minimum_target_delta_counts != null && <> · 更新粒度 {(100 * live.sources.gripper.minimum_target_delta_counts / 255).toFixed(2)} 个百分点，小输入先累计</>}
                  </p>}
                  {Object.entries(live.associations ?? {}).map(([n, a]) => (
                    <p key={n}>
                      {n} 接收差 {a.receive_delta_ms?.toFixed(1) ?? "—"} ms ·{" "}
                      {a.valid ? "关联有效" : "关联无效"}
                    </p>
                  ))}
                </div>
              </section>
              <section className="card">
                <div className="card-header">实际关节 / DEG</div>
                <div className="joint-grid">
                  {live.observation?.joints_rad?.map((q, i) => (
                    <div className="joint" key={i}>
                      <span>J{i + 1}</span>
                      <strong>{((q * 180) / Math.PI).toFixed(2)}°</strong>
                    </div>
                  ))}
                </div>
                <div className="record-body">
                  <p>
                    模式 {decoded?.robot_mode ?? "—"} · 使能{" "}
                    {decoded?.enable_status ?? "—"} · 错误{" "}
                    {decoded?.error_status ?? "—"}
                  </p>
                  <p>
                    User {decoded?.user_index ?? "—"} / Tool{" "}
                    {decoded?.tool_index ?? "—"}
                  </p>
                  <p>
                    控制器工具位姿 / mm, deg
                    <br />
                    {decoded?.controller_pose_raw_mm_deg
                      ?.map((v) => v.toFixed(2))
                      .join(" · ") ?? "—"}
                  </p>
                  <p>
                    URDF Link6 / mm（参考系未共同标定）
                    <br />
                    {(
                      live.observation?.model_link6_position_m ??
                      live.observation?.tcp_position_m
                    )
                      ?.map((v) => (v * 1000).toFixed(2))
                      .join(" · ") ?? "—"}
                  </p>
                  <p>
                    模型Link6 / xyzw
                    <br />
                    {(
                      live.observation?.model_link6_quaternion_xyzw ??
                      live.observation?.tcp_quaternion_xyzw
                    )
                      ?.map((v) => v.toFixed(4))
                      .join(" · ") ?? "—"}
                  </p>
                  <p>
                    Tool配置 / mm, deg
                    <br />
                    {decoded?.tool_configuration_raw_mm_deg
                      ?.map((v) => v.toFixed(2))
                      .join(" · ") ?? "—"}
                  </p>
                  <p>
                    负载参数 {decoded?.payload_parameter_kg ?? "—"} kg ·
                    非称重值
                  </p>
                </div>
              </section>
              <section className="card record-card">
                <div className="card-header">观测片段 · 20 Hz</div>
                <div className="record-body">
                  <label>检查描述</label>
                  <textarea
                    value={task}
                    onChange={(e) => setTask(e.target.value)}
                    disabled={live.recording?.active}
                  />
                  <div className="record-count">
                    <strong>{live.recording?.samples ?? 0}</strong>
                    <span>SAMPLES</span>
                  </div>
                  <button
                    className="primary wide"
                    disabled={
                      (controlBackend&&!live.capabilities?.operator_control)||
                      live.recording?.pending ||
                      (!live.recording?.active && !live.observation?.valid)
                    }
                    onClick={() => record(!live.recording?.active)}
                  >
                    {live.recording?.pending
                      ? "传输与校验中"
                      : live.recording?.active
                        ? "结束录制"
                        : "开始录制"}
                  </button>
                  <button
                    className="wide"
                    onClick={async () => {
                      const r = await fetch("/api/record/finish", {
                        method: "POST",
                        headers: { "Content-Type": "application/json" },
                        body: JSON.stringify({ result: "aborted" }),
                      });
                      if (r.ok) refresh();
                    }}
                    disabled={!live.recording?.active||(controlBackend&&!live.capabilities?.operator_control)}
                  >
                    停止采集本片段
                  </button>
                  {controlBackend&&!live.capabilities?.operator_control&&<p>自动验收会话中，录制由测试程序管理。</p>}
                  {live.recording?.pending && (
                    <p>
                      原始媒体 {live.recording.verified_media ?? 0} /{" "}
                      {live.recording.expected_media ?? 0}{" "}
                      已校验。请保持工作台运行；校验完成后片段才标为完整。
                    </p>
                  )}
                  <p>
                    原图与无损深度先在X5暂存，结束后回传。
                    <br />
                    {controlBackend
                      ? "保存输入、候选目标、确认目标及反馈"
                      : "input / effective_action = null"}
                    <br />
                    观测片段不导出动作训练数据。停止片段后实时观测继续。
                  </p>
                </div>
              </section>
            </div>
            <section className="card">
              <div className="card-header">
                深度预览320 × 240 · 原值640 × 480
                <span>
                  源流 {fps("depth")} fps · 选帧{" "}
                  {sampling?.depth_selection_hz ?? "—"} Hz · 有效{" "}
                  {(100 * (live.sources?.depth?.valid_fraction ?? 0)).toFixed(
                    1,
                  )}
                  %
                </span>
              </div>
              {live.images?.depth_jpeg ? (
                <img
                  style={{ maxWidth: 640, width: "100%" }}
                  src={`data:image/jpeg;base64,${live.images.depth_jpeg}`}
                  alt="原生深度预览"
                />
              ) : (
                <div className="empty">深度暂不可用，RGB与机械臂观测可继续</div>
              )}
              <p>
                预览为有损灰度图；原始记录保留uint16与零值 · 未对齐RGB · 不补洞
              </p>
            </section>
          </>
        ) : (
          <div className="episode-layout">
            <section className="card episode-list">
              {episodes.map((e) => (
                <button
                  className="episode-item"
                  key={e.id}
                  onClick={async () => {
                    setSelected(e);
                    setCursor(0);
                    setRowOffset(0);
                    setTotalRows(e.samples);
                    setRows(
                      (
                        await fetch(`/api/episodes/${e.id}?limit=100`).then(
                          (r) => r.json(),
                        )
                      ).rows,
                    );
                  }}
                >
                  <strong>{e.task}</strong>
                  <small>
                    {e.samples} 帧 · {e.result} ·{" "}
                    {e.complete ? "完整" : "不完整"}
                  </small>
                </button>
              ))}
            </section>
            <section className="card episode-detail">
              <div className="card-header">
                {controlBackend
                  ? "真实记录 · 输入与目标审计"
                  : "真实观测 · 无动作标签"}
              </div>
              {selected && row ? (
                <div className="episode-content">
                  <>
                    {row.sources.rgb.media_pending ? (
                      <div className="empty">此帧原图等待回传校验</div>
                    ) : (
                      <img
                        src={`/api/episodes/${selected.id}/${row.image}`}
                        alt="真实观测片段RGB"
                      />
                    )}
                  </>
                  <input
                    type="range"
                    aria-label="观测帧"
                    min={0}
                    max={Math.max(0, totalRows - 1)}
                    value={cursor}
                    onChange={(e) => moveCursor(Number(e.target.value))}
                  />
                  <p>
                    第 {cursor + 1} / {totalRows} 帧 ·{" "}
                    {row.elapsed_s.toFixed(3)} s
                  </p>
                  <div className="review-joints">
                    {row.observation.joints_rad.map((q, i) => (
                      <div key={i}>
                        <label>J{i + 1}</label>
                        <strong>{((q * 180) / Math.PI).toFixed(2)}°</strong>
                      </div>
                    ))}
                  </div>
                  <p>
                    RGB重复使用：{row.sources.rgb.repeated ? "是" : "否"} ·
                    深度有效率{" "}
                    {((row.sources.depth.valid_fraction ?? 0) * 100).toFixed(1)}
                    %
                  </p>
                  <pre>{JSON.stringify(row.associations, null, 2)}</pre>
                  {controlBackend&&<pre>{JSON.stringify({input:row.input,candidate:row.candidate_action,accepted:row.effective_action,ik:row.control?.tick?.ik_result,events:row.control?.tick?.events},null,2)}</pre>}
                </div>
              ) : (
                <div className="empty">选择一个观测片段</div>
              )}
            </section>
          </div>
        )}
        {controlBackend && <><HardwareControlPanel robotMode={decoded?.robot_mode} enabled={decoded?.enable_status} /><section className="card"><div className="card-header">请求 / 候选 / 已确认 / 实际</div><div className="record-body"><p>控制器确认表示接受目标；实际到位和停止分别由反馈核对。</p><details><summary>查看目标、IK与实际反馈明细</summary><pre>{JSON.stringify({input:live.input,requested:live.control?.tick?.requested_target,candidate:live.candidate_action,accepted_this_tick:live.effective_action,last_accepted:live.control?.tick?.last_accepted_action,actual_joints_rad:live.observation?.joints_rad,ik:live.control?.tick?.ik_result},null,2)}</pre></details></div></section></>}
        <footer>
          <span>{controlBackend?'Ctrl+C先停止控制，再释放本次资源；保持使能状态':'仅采集观测 · Ctrl+C释放本次资源'}</span>
          <span>{controlBackend?'法兰控制范围以验收档案为准 · 真实TCP另行验收':'真实TCP与连续运动另行验收'}</span>
        </footer>
      </main>
    </div>
  );
}
