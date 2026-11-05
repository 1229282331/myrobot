# v340 控制暂停时序与正式界面负载对照

2026-10-20。生产控制二进制、模型、所有有效期和几何阈值保持不变。上一轮 v339 已收齐失败证据且正常释放，属于实质进展。本轮继续定位，不声称交付。

## 四次暂停的不同来源

原数据是 `evidence/initial-query-v339/loaded`。`output/analyze_loaded_failures_340.py` 保存独立故障窗口到 `evidence/control-contention-v340/failure-windows.json`。窗口内的锁、方法、GC、线程时间互相重叠，不能相加或把墙钟减 CPU 全部算成调度延迟。

- Base 337：候选第一次被主机收到时剩余 8.753 ms，18.986 ms 后拒绝 `candidate_sample_deadline`。窗口内有一次 GC1 14.548 ms，但它发生于前一个周期，不能直接把全部 18.986 ms 归因于这次 GC。
- Base 348：主机收到时剩余 70.080 ms，74.717 ms 后拒绝 `original_candidate_deadline`。其中 UI 状态生成持锁 26.762 ms，后续 tick 等锁 11.380 ms、持锁 20.872 ms；状态线程实际 CPU 2.849 ms、16 次自愿切换，tick CPU 6.890 ms。当前记录未把 GIL、具体锁及操作系统调度完全分开。
- Base 664：回调 44.159 ms，夹爪检查阶段报告回调截止。
- Tool 577：回调 42.223 ms，同样报告回调截止。

候选到达过迟、到达后消费过迟、原生计算超时需要分别处理。

## 测试负载偏差

正式 `gateway.py` 的控制 WebSocket 在 `executor.state(True)` 后调用 `for_operator`，约 20 Hz；独立观察连接输出完整状态，约 5 Hz。v339 测试器 `ui_loop` 每次约 20 Hz 序列化完整状态，缺少对应的精简投影。原始双审计仍独立且完整。

本轮保留 `legacy` 原压力模式，并增加 `operator`：使用同一生产 `for_operator`、20 Hz 控制状态和独立 5 Hz 完整诊断状态。增加每路帧数和字节数，不缩短测试、不更改原始双审计、不改变候选或控制规则。

新对照基于同一 f88716de 候选与同一 Python 清单，Base / Tool 高速组合分别按 legacy/operator/operator/legacy 执行，各 36 秒、共八组。它只是负载归因实验；即使正式负载通过，也不能抹去旧 0.8x 实机失败或代替最终实机验收。

生成器 `output/prepare_operator_load_340.py`，可复现测试源在 `output/operator-load-v340`；启动、查询、取证与正常释放分别使用 `stage_operator_load_340.py`、`poll_operator_load_340.py`、`fetch_operator_load_340.py`、`finish_operator_shadow_340.py`。

## 图表口径修正

新增正式离线工具 `teleop-platform/scripts/plot_control_chain.py`。图中反馈年龄使用候选的 `pre_action_received_ns`，而不是计算开始时间；切换 epoch 时断开线段，显式标出全部方向拒绝，避免掩盖释放、重新操作和暂停。

旧图及统计原件保留；修正图在 `evidence/control-contention-v340/failed-chain.png`，相邻 JSON 保存指标。图表仅表示合成写入时序，不表示真实机械臂运动。

## 本轮运行记录

只读 shadow `20261020-141259-c92d0540` / exec24321；入口 `output/shadow_operator_load_340.py`，stopfile `output/shadow_operator_load_340.stop`。首个 45 秒就绪探针超时保留；同一进程随后正常就绪，没有重启。新读静止、未准备、Mode5 Enable1、负载及相机/模型健康；没有使能、清错、机械臂或夹爪位置命令。

隔离 job162508/start_ticks6041584/boot `de68973d-218f-46b0-84f5-ed09f8427efa`，目录 `/qualification/operator-load-v340`。最新状态必须通过该进程身份查询，不能据本报告假定仍运行。完成后取回原件、校验 SHA，并正常释放被动平台。

`for_operator` 和 IPC 心跳相关已有测试 10 项通过。它们验证投影及生命周期语义，不是负载性能证明。整个 Goal 仍 active，正式中高速、低速退离、Position assist 恢复、录制与最终发布等完整范围未被缩减。

## 最终结果：未通过

八组已完成，共 4435 次合成写入；七组没有暂停，`operator-tool-b` 的 538 次写入中出现两次暂停及求解屏障。全部八组无跳父候选、过期写入、跟随停止、写前拒绝或退离预算降速写入。隔离 job 正常结束但测试返回码为 1，`loaded/analysis.json` 的 `passed=false`。

- 候选 583：callback 45.149098 ms、compute 40.920608 ms、往返 47.972799 ms；夹爪阶段无剩余预算，耗时 2.739160 ms，回调超时拒绝。
- 候选 630：callback 42.043689 ms、compute 20.289825 ms、往返 45.105098 ms；夹爪检查在其预算内完成，但最终总回调仍超过 42 ms。计算范围之外的等待尚需分段核对，不能直接归因 DDS。

这表明旧故障不能仅归因测试器序列化负载过大。两组 legacy Tool 无暂停、一组 operator Tool 失败，也不足以得出简化输出改善或恶化时延的因果结论。原生适配器每请求发布关节状态并等待 CurrentStateMonitor，工具坐标转换仍依赖该状态；后续不能只删除等待而留下坐标不一致。

原包 276 文件、75,980,163 字节，远端/PC 压缩包及逐文件 SHA 已核对，保存于 `evidence/control-contention-v340/loaded`。`abba-curves.png` 已查看，保留输入释放及重启间隔并标出两次拒绝；不是实机运动曲线。

shadow 经 20 个新读未准备、静止、录制 idle 样本后正常退出 0，camera/gateway 均退出 0，CPU/Wi-Fi 恢复；PC app0/media1、cleanup 错误为空，原件保留于 `shadow-exit.json`。本轮作业全部结束，无控制 owner。生产原生版本仍为 `90fe4ad1…`，候选 `f88716de…` 未发布。

后续状态发布拆分改造见 `STATE_OBSERVATION_V341.md`；v340 使用的是拆分前 Python 源，不能将本轮结果当成 v341 的板端验证。
