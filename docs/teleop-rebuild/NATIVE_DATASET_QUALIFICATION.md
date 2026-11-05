# 原始动作记录与官方 LeRobot 接口：v97 离线资格验证

2026-10-08。基线 `bbe83fe`。本轮没有设备连接、机械臂位移或 CAN 发送。正常释放仍立即 Stop，稳定窗口允许 1 秒。**本报告只证明官方数据接口和原始事件解释，不代表完整真实示教、训练数据或平台交付。**

## 假设、改变与结果

假设：已有原始日志可以复用固定版 LeRobot 写入与读取，无需重新编写数据集格式；不完整动作标签必须显式拒绝。

- `native_dataset.py` 复用 `teleop/src/e6_teleop/exporter.py` 的官方 API 路线，调用固定 `8c920c4270460851cedd2737657584586d3dc66f` 的 `LeRobotDataset.create/add_frame/save_episode/finalize`。实际安装版本 0.6.2，Python 3.12，CPU libx264，PyAV 读取；无上游补丁。
- 新增只读 bundle 校验参数，缺失文件时拒绝，不回填或重新生成原记录。正常媒体回传默认行为保持。
- 七维动作取原始且确认的 ServoJ 字节目标（六关节转弧度），加发送时已经确认的 CAN 目标字节/255。观测取发送前官方 SDK 原包解析的实际关节及新鲜 CAN 回包。候选目标、下一帧状态均不能代替标签。
- 每行只代表一次已确认的机械臂发送事件。独立夹爪命令和暂停仍留在原始记录中；这里没有把它们虚构为机械臂命令。若臂发送时有更新的 CAN 请求尚未回包，则该行目标不确定，拒绝整个来源。
- RGB 使用原 `sdk_available_v2` 关联，并核对帧号、epoch、SDK 可用时刻、原包序号、SHA 与 100 ms 时效。缺失、未确认、未来观测或异常反馈均拒绝整个来源，不静默剔除失败行。

**时间语义：**官方 LeRobot 表中的 `timestamp=index/20` 只用于事件序号的浏览时间线，并非实际 20 Hz 采样。`source-events.jsonl` 保留发送、确认和关联观测的原 X5 时间，原始日志仍是事实来源；不跨主机相减，不宣称曝光同步。`qualification.json` 和验证报告均标记 `policy_ready=false`；不得直接当作控制策略训练或自动回放数据。

## 执行证据

证据目录：`evidence/native-dataset-v97`。

| 检查 | 结果 |
|---|---|
| 离线专项 | 58 项通过，覆盖有效标签、未确认 CAN、未来图、损坏原反馈、来源身份和只读校验 |
| 完整后端回归 | 360 项通过，4 项 POSIX 专用在 Windows 跳过 |
| 官方实际写入及 finalize | 两个明确 `qualification_fixture` 来源，分别 success 24 帧、failure 28 帧 |
| 官方标签读回 | 全部 52 个七维 state/action 和 success 标志一致；测试候选故意与发送字节不一致 |
| 视频 | 1280×720，52 帧全部 PyAV 解码，PTS 递增；逐帧关联原图，最大平均像素误差 2.263/255 |
| DataLoader | action/state `[8,7]`，RGB `[8,3,720,1280]` |
| 边界与来源 | 两个 episode 的帧序号、事件时间线、前后窗口 padding、52 行来源映射读回通过 |
| 原记录检查 | 20 段：13 个 shadow、7 个 hardware_demonstration；0 个具备本转换器所需的完整标签；原目录内容清单/大小/修改时间不变，bundle SHA 留存 |

原始失败保留：首个专项补充时测试文件括号写错，收集阶段失败（`targeted-reviewed.xml`）；修正后 58 项通过。官方 `official-run1` 已写出，但验证器把官方标量 bool 当作 `[N,1]`，读回失败；`lerobot.partial` 原样保留。改为官方实际 `[N]` 标量布局后，独立 `official-run2` 完整通过。未改变数据值、口径或原失败结果。

前端和设备控制实现本轮没有改动；31 项前端测试、正式构建及先前现场结果只按 `validation-reuse.json` 明确依赖复用，不称新实机验收。生产路径唯一改动是已有媒体校验器增加可选只读模式，已纳入专项和完整回归。

## 真实记录的缺口

`real-preflight.json` 保留所有来源和拒绝原因。原机械臂示教两段有 26/239 个发送事件，但没有 CAN 发送，且使用旧关联策略；独立夹爪片段有 CAN，没有 `motion_submitted`。中止片段仍保留且不导出。不升级历史策略，不把实际关节补为动作，也不跨片段拼接假定的夹爪目标。

正式页面仍仅提供原始片段浏览，`export_allowed=false`。新模块提供显式离线资格检查，**没有开放正式动作导出按钮**。后续需要在运动跟随问题解除后取得完整、同时可关联的真实机械臂/夹爪记录，再决定均匀时间数据的处理并验收。当前事件格式测试不满足该项完整交付。

## 复现

使用已固定的 `teleop/.venv`（依赖见 `teleop/pyproject.toml`、`teleop/uv.lock`），在仓库根目录：

```powershell
& teleop/.venv/Scripts/python.exe -B -X utf8 teleop-platform/scripts/qualify_native_dataset.py --output output/native-qualification-new

$env:PYTHONPATH = "$PWD/teleop-platform/src;$PWD/teleop-platform/upstream/dobot-sdk"
& teleop/.venv/Scripts/python.exe -B -X utf8 -m e6_phosphobot.native_dataset teleop-platform/runtime/episodes/<片段ID>
```

第一个命令只创建明确标为测试数据的来源和官方数据集；已有输出或 `.partial` 时拒绝覆盖。第二个命令默认只检查，完全离线。显式 `--output` 才尝试资格导出，任一来源不合格则整体拒绝，无上传和实机回放功能。

## 下一步与阻塞

普通 30 mm/s 的 J4 跟随偏差发生在首次 Stop 之前，最大 3.813°，不是微小收尾。此前提出的腕部背面支架/线缆接触事实，在最近至少三个连续 Goal 回合仍待答；无回复不能当作已排除。外围恢复、视频与数据接口工作已保存，本轮后按 Goal 机制报告 blocked，不继续重复外围测试。解除后从新状态做低幅单变量 ServoJ 时序对照，不能直接重跑失败的高速序列。主速度、首动时延、综合长时与实体手柄验收仍未完成。
