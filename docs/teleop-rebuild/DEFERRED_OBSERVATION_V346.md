# v346 将录制展示计算移出控制线程

2026-10-20。**源码候选已完成板端带相机负载对照，但四组整体失败；未发布实机修复，不作为交付。**

v345 原候选 B 的 660 号结果已算出，却在控制线程同步生成录制状态期间错过 100 ms 有效期。v341 把计算移出锁，没有把它移出控制 tick，仍可能耽误可发送候选。详见 [v345 诊断](RETREAT_LOAD_V345.md)。

## 修改

- tick 在原采样时刻捕获 `StateObservation`，使用本地私有协议的字节快照冻结数据；后续实时状态或回执的原地修改不能改变旧记录。
- 现有独立 Audit serializer 进程解码快照，计算显示用 TCP/名义夹爪几何及说明，再写入原事件格式。原始反馈、动作回执、事件数量与顺序不变；不增加控制连接。
- Episodes 的结束判定与人类可读日志只读取同步捕获的生命周期字段，不等待几何渲染。`cycle.state` 和 `episode_finished.final_state` 在各自 Audit 进程中还原为完整状态。
- 字节快照仅来自当前进程，通过 Audit 自有管道传递，不能接收外部文件或网络提供的 pickle。控制、反馈新鲜度、停止、几何模型和截止判断均未变。

五文件原源、候选和可复现差异在 `output/deferred-observation-v346`。首轮深拷贝候选保留为 `attempt-deepcopy`，不覆盖其验证证据。两个 independent Audit 可以分别渲染同一快照；内容一致性须通过测试，不把重复计算当成无成本。

## 已验证与限制

首轮深拷贝候选完成 1,197 项后端测试、18 skip，但 PC 捕获中位 391 µs，明显高于旧同步状态约 85 µs，未采用。最终字节快照捕获 PC 中位约 90.6 µs，当前同步状态约 84.6 µs；它没有证明纯 CPU 加速，目标是避免调度等待与几何计算继续占住控制 tick。锁内捕获约 40 µs，冻结成本须在 RDK 再评估。

最终字节快照与原同步实现的 254 个保存状态逐字段全等，经实际 pickle roundtrip 后比较。原采样时钟、反馈接收时间、TCP、夹爪和能力说明均保留。文件 `evidence/deferred-observation-v346/byte-equivalence-wall.json`。

新增回归覆盖：捕获路径禁止调用 renderer；捕获后原地改变源反馈仍不改旧快照；双独立审计与原事件顺序一致；人类日志不触发几何渲染；正常录制结束和停止不确定时仍分别成功/aborted，并保留完整 final_state。最终字节版全后端验证入口 `output/run_backend_346b.py`，以 `tests-byte.json` 与 JUnit 为准。

已准备四组 inline/deferred/deferred/inline 的 Tool 完整链路对照，均固定同一 v344 诊断 native；因此隔离观察计算位置这一项变化，不混入线程优先级调整。入口 `stage_deferred_observation_346.py`、`poll_deferred_observation_346.py`、`fetch_deferred_observation_346.py`；新 shadow 使用独立 stopfile，不复用上一会话。

最终全后端 1,197 passed / 18 skipped，JUnit 共 1,215 项，66.765 秒。与五个最终源哈希绑定的结果在 `evidence/deferred-observation-v346/tests-byte.json`，无须对未改动源码重复跑通过项。

## RDK 相机负载结果：未通过

job185071 / start_ticks6941860 在 boot `de68973d…` 下完成，终止码 1，异常字段 null；四个 worker/native 均正常退出，日志没有丢弃或写入错误。本轮总 2,124 条合成写入，真实 Servo/IPC/NativeExecutor 配合 150 ms 合成反馈和 33 ms 回复，执行端网络隔离，没有物理动作。四组固定同一诊断 native `15173fcc…`，按 inline/deferred/deferred/inline 顺序，均 36 秒 Tool 位移与旋转组合。该诊断负载含额外观察和插桩，绝对时延不等同实机性能。

| 组别 | 合成写入 | 暂停 | callback 超时 | 其他 solver blocked | 写前过期 |
| --- | ---: | ---: | ---: | ---: | ---: |
| inline A | 528 | 2 | 2 | 0 | 0 |
| deferred A | 531 | 7 | 2 | 5 | 0 |
| deferred B | 552 | 1 | 1 | 0 | 0 |
| inline B | 513 | 7 | 1 | 5 | 1 |

不能把 17 次暂停全部算作独立几何超时：只有 6 个 `solver_callback_deadline`，另有 10 个较晚的 `official_solver_blocked`（其中含同一停点反复拒绝）和 1 个 `original_candidate_deadline`。后续 blocked 的起始因果仍需单独核对，不能仅按出现次序断言均由超时引起。所有组均没有过期发送、跳父链、跟随保护停机、预算降级后的实际合成写入；这不表示预算没有耗尽——6 个 callback 失败均发生过退离预算耗尽，最终被拒绝。

六个 callback 为 42.155–43.261 ms，条件数 20.095–20.656，低于奇异减速起点 30。各帧主线程 CPU 25.566–34.631 ms，墙钟比 CPU 多 7.703–17.544 ms，无缺页，存在非自愿切换；支持延迟包含调度等待，不能据此把全部差额归因单一来源。退离样本/初始化与之后夹爪完整检查共同占用原预算。单改录制展示没有消除这些失败。

inline B 的候选 565 已完成可行退离计算（callback 37.363 ms，倍率 1），worker 回复时反馈年龄 51.407 ms，最终写口在 100.794 ms 拒绝。窗口里同步 audit 状态只耗 2.382 ms，另外还有接收、观察/UI JSON 和 host_offer 等活动，无 GC；本次不能像 v345/660 一样归因于单个长 audit 调用。原 100 ms 截止未改变，也没有过期下发。

### 展示计算和记录完整性

计时器只保留 **至少 2 ms** 的作用域，以下不是全部 tick 的无偏分布，也不是生产运行成绩：

| 组别 | 保留样本数 | 中位 / P95 / 最大墙钟 ms |
| --- | ---: | --- |
| inline A 同步 audit | 553 | 3.348 / 8.647 / 62.207 |
| deferred A 捕获 | 522 | 3.525 / 8.727 / 17.728 |
| deferred B 捕获 | 510 | 3.519 / 8.766 / 19.110 |
| inline B 同步 audit | 526 | 3.305 / 10.581 / 37.913 |

deferred 两组没有写前过期，捕获最大值低于 inline，但中位未改善、仍有几何超时；路径随反馈/停止发生分歧，不能靠这四组宣布性能提升或根治。保留五文件候选，尚不发布实机修复；下一步优先处理共同的原生几何/调度长尾，并分离 host 接收与写入延迟。

四组连续与 episode 审计文件逐字节相同，各 594 / 604 / 611 / 589 个 cycle 全部还原完整 actual、gripper.geometry 与 capabilities.tcp。254 个状态的原采样语义全等验证及后端测试仍有效；它们证明记录语义，不证明控制实时性。

153 文件、38,143,225 字节原始压缩包以及每个成员 SHA 已验证，位置 `evidence/deferred-observation-v346/loaded`。分析入口 `output/analyze_deferred_observation_346.py` 保存全部失败窗口；`timing.png` 已生成并查看，保留暂停与重启间隙。原失败、初次就绪 503、深拷贝试验均保留。

## 正常释放及下一步

只读 shadow `20261020-164140-aac86601` 在新 20 个未准备、静止、录制 idle 状态后正常 stopfile 退出，exec43864 / fetch52746 均结束。末次反馈 Mode5 / Enable1 / Error0 / Collision0 / ID17517，负载匹配；只代表结束时状态，不沿用为下一次物理动作的当前证据。

camera/gateway 退出 0，CPU 恢复 schedutil、Wi-Fi 省电恢复 on，PC app0/media1、cleanup 错误空原样保留；本机 8765/8776/8777/8889/8766 无监听，当前没有平台控制 owner。生产 native 仍 `90fe4ad1…`，模型/配置/ACM/TCP/负载及原截止保持。未发送实机运动、Enable/Disable、清错、夹爪/参数写入或 Git 提交。

0.5× 局部笛卡尔往返实机曾通过，0.8× 曾因 44.089 ms callback 失败；J5 中高速、SDK 对照、实体手柄、录制与完整体验仍未验收。已如实向用户说明残留问题，Goal active、自主抓杯取消。下一步仍为诊断与修复，不标记阻塞或完成；下一物理动作须新反馈及画面。


## v348 后续定位

v346 中较晚的 official_solver_blocked 已由原始 FCL 证实为 Link3/base_link 自碰撞：七个此前放行的最终平滑目标漏查了整臂，反馈到位后才被当前状态监测拒绝。详见 FULL_ARM_SWEEP_V348.md。该发现不抹去原有六次 callback 超时与一次写前截止失败，也不将合成设备结果当作历史 J5 实机根因。
