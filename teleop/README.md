# E6 遥操作、示教采集与真实设备观测

本地浏览器工作台，默认以末端六自由度增量控制驱动官方 E6 运动学模型，并录制、导出和浏览 LeRobot Dataset v3 数据。另提供 `hardware-observe` 后端，只读接入真实 E6 与 Gemini E；它没有动作通道。后端在启动时固定。

第三阶段增加 `hardware-control`，启动时固定 `shadow|live`，默认影子模式。正式入口为 `launch-control-live.cmd`：核对完整验收档案及源码、模型、CAN和参数SHA后才连接设备；缺少档案或源码变更时拒绝实体控制。当前验收结果见 `docs/实机遥操作生产集成与验收_2026-10-05.md`，不沿用旧版本的通过结果解锁新版。

启动固定夹爪配置 `--gripper-profile empty64-medium` 采用 Force64 / Vel25 / Acc25 / Dec25、目标速率10个百分点/秒；默认 `plan32` 保留原Force32 / Vel10、初测5个百分点/秒。力与速度是协议数值。shadow不建立CAN连接，live须点击独立初始化后才获得实际开度，不能用历史开度代替。

操作员界面支持PC端标准布局手柄与键盘、本地输入可视化、释放保持、停止发送/回包/静止三种证据、有限恢复以及示教记录。手柄用LB允许动作、RB精细、X切换平移/旋转、Y切换基座/工具、LT/RT连续开度，Start录制、A/B成功/失败、Back停止。键盘用Space允许动作、Shift精细、A/D、S/W、Q/E三轴，Tab/C切换、F/R夹爪，Enter录制、1/2成功/失败、Esc停止。模式和坐标切换须松开并回中，文本输入区域屏蔽控制。机械臂使能、夹爪初始化与准备控制分别执行，启动不会自动运动。

真实记录放在 `datasets/e6_teleop/hardware_demonstration/raw/`；原图和无损深度结束后回传，校验期间禁止控制。中止记录及首次失败快照保留，媒体未完成则不标完整；真实动作暂不导出LeRobot。`scripts/audit_hardware_episode.py EPISODE --output REPORT.json` 可离线核对原反馈、发送/确认目标、图像哈希、uint16深度与关联，无设备连接。

## 启动

已安装的本机可直接双击 `launch.cmd`，然后打开 **http://127.0.0.1:8765**。关闭终端或 Ctrl+C 退出；正常 Ctrl+C 同时结束模拟网关。两个端口被占用时拒绝启动，不接管其他进程。

新安装在根项目 PowerShell 执行：

```powershell
.\teleop\bootstrap.ps1
.\teleop\launch.cmd
```

## 真实观测启动

X5 网关独立部署，仅使用既有 Python 3.10 / Orbbec SDK 1.10.37 相机环境，加上固定版本 websockets 和 zstandard；X5 不安装 MuJoCo 或 LeRobot。部署程序不启动采图：

```powershell
.\teleop\.venv\Scripts\python.exe teleop/scripts/deploy_observer.py --host 192.168.1.13
.\teleop\.venv\Scripts\python.exe -m e6_teleop serve --backend hardware-observe --ssh-host 192.168.1.13 --remote-root /root/e6-observer-20261004 --media-hz 20
```

SSH 校验既有 known_hosts，使用既有密钥或本次进程的 `E6_X5_PASSWORD` 环境变量；不要把密码写进命令参数或提交。PC服务绑定127.0.0.1:8765；实时/原始回传隧道绑定127.0.0.1:8767/8769，均转发到X5本机8767。未指定 `--remote-root` 时仅转发已运行的网关，不接管或停止别人的进程；指定后通过 stdin EOF 结束自己启动的观测任务。

`--wifi-low-latency` 是可选对照参数，仅在本次 X5 观测会话中关闭 Wi-Fi 省电，正常退出、SIGTERM 或 SSH stdin EOF 时恢复原状态；不修改 NetworkManager 持久配置。关闭省电的 10 Hz 长测仍触发积压，因此不默认使用，也不把它称作根因修复。

Ctrl+C 结束 PC 和本次 X5 观测任务，释放相机与反馈连接。它不发送 E6 Stop、DisableRobot、CAN 指令或系统关机。页面“停止采集本片段”中止并保留当前记录，实时观测继续；关闭网页也不会触发实机下机。

模型仅由实际关节驱动，隐藏未经验证的附件和未知开度。URDF Link6 位姿与控制器当前 User/Tool 位姿分列展示，坐标约定和真实 TCP 尚未共同标定，不用于实机定位。夹爪始终显示“当前遥测不可用”；仅查询 can1 接口状态，不设置比特率、不启用接口、不发送任何帧。

## 观测协议与媒体

v2 声明 `backend=hardware-observe`、`capabilities.control=false`，`input/effective_action=null`。robot / rgb / depth / gripper 分别保存来源、序号、有效性、原因和 X5 接收单调时间；RGB 保留正确 UVC PTS 与 SDK 原字段。控制器反馈保留完整 1440 字节和固定 SDK 提交 `82bce656` 的解析结果。内部米、弧度、xyzw 约定保持一致。

E6 读取 30004；29999 白名单仅为 RobotMode / GetAngle / GetPose / GetErrorID。相机固定 SN AY4A1520111，RGB 1280×720、深度 640×480，均请求 30 fps，原生未对齐；页面报告实际源帧率。

观测源元数据与原始选帧按20 Hz发布；RGB与深度压缩预览通过独立WebSocket发布，默认20 Hz。RGB预览640×360、JPEG质量70，深度预览320×240灰度JPEG；每个预览保留自身帧ID与年龄，不代替原始源。`--media-hz 5|10|20`仅设置预览上限，RGB实际源流可能低于请求的30 fps。实时通道采用最新帧优先与有界队列。CLI默认模拟，不自动修改Wi-Fi省电。

录制时，X5异步暂存20 Hz实际选中的原JPEG与uint16深度，Zstandard无损保存；PC立即写原反馈、源元数据及帧引用。结束采集后通过独立SSH TCP连接限速回传（约1 MB/s），PC核对身份、长度、SHA256及尺寸，原JPEG不重新编码，深度转为无损PNG。原始媒体全部校验通过后才将片段标为完整；界面显示“传输与校验中”及进度，期间不能开始新片段。停止片段后实时观测继续。退出或回传失败保留不完整/中止记录，重连后不自动续录。

X5暂存位于网关工作目录`native_archives/SESSION/`，保留原始文件和清单供失败检查，不自动删除历史片段。单片段原始数据上限2 GiB，写盘队列上限128帧；预算、队列或磁盘异常都会中止。测试前确认X5可用空间；长片段和自动清理策略另行验收。PC最终数据仍位于本地独立目录，预览图不会混入原始图像。

PC 界面报告的是数据年龄上界，包含源采集、选帧、打包、传输与处理，不是 ping。SSH 连接关闭 TCP 小包合并；传输时钟用多次往返探测区间的交集给出上下界，保留 1000 ppm 相对漂移余量，30 秒后探测失效，不要求两台主机时钟同源。诊断程序 `scripts/diagnose_observation_latency.py` 绕过模型渲染与写盘，分别记录 ping、应用层 RTT、两路帧间隔、解包耗时与传输年龄。

关联仅相减同一 X5 时钟域的接收时间；超过 100 ms 标为无效。不同主机单调时钟不直接相减，传输新鲜度另用往返探测后的经过时间给出上界，包含往返不确定性和漂移余量。这不是曝光同步。超过 250 ms 的 E6 源反馈、500 ms 的 RGB 或传输不新鲜、断连、采集/处理队列溢出、写盘失败均中止录制；深度缺失/全零单独标记。重连不自动开始新片段。

真实记录位于 `datasets/e6_teleop/hardware_observation/raw/`，`dataset_kind=hardware_observation`。可在“观测记录”逐帧检查，禁止经 LeRobot 动作数据导出器导出；绝不使用关节状态补动作标签。源流缺号、20 Hz 抽样跳过、X5预览丢弃和浏览器预览丢弃分别统计。原模拟 v1 数据和导出器保持兼容。

离线逐帧核查原图、反馈、深度及关联：

```powershell
.\teleop\.venv\Scripts\python.exe teleop/scripts/verify_observation.py datasets/e6_teleop/hardware_observation/raw/EPISODE
```

最新无线验收见[无线低延迟观测优化](../docs/无线低延迟观测优化_2026-10-05.md)，此前实现与失败记录见[第二阶段真实设备观测验收](../docs/真实设备观测接入验收_2026-10-04.md)。实机连续控制、TCP、附件几何、抓放、训练和回放仍属后续阶段。

需要 uv、Node.js/npm 和 Windows x64。项目环境位于 `teleop/.venv/`，采用 uv 管理的 Python 3.12，避免本机 Conda 3.12.2 的 MuJoCo DLL 初始化问题。`uv.lock` 固定 Windows CPU 依赖；复现使用 `uv sync --frozen --extra dataset`，不要直接重新解析上游 CUDA 依赖。MuJoCo 3.14.0 使用本机 OpenGL 渲染；LeRobot 使用 torch 2.8.0+cpu 与 CPU H.264 编码。

开发前端：`cd teleop/web`，`npm ci`，`npm run dev`；另一个终端从根目录运行 `.\teleop\.venv\Scripts\python.exe -m e6_teleop serve`。Vite 代理 `/api` 与 `/ws` 到 PC 服务。发布界面使用 `npm run build`，由 PC 服务提供。

## 操作

先选择输入设备，摇杆回中、扳机松开，再点击“准备控制”。手柄须在浏览器支持的 standard 布局模式下；首次打开网页后可能需要按一下实体手柄使浏览器发现它。

| 操作 | 手柄 | 键盘 |
|---|---|---|
| 允许动作 | 按住 LB | 按住 Space |
| X/Rx、Y/Ry、Z/Rz | 左摇杆左右、左摇杆上下、右摇杆上下 | A/D、S/W、Q/E，后一个键为正方向 |
| 平移/旋转 | X | Tab |
| 基座/工具坐标 | Y | C |
| 25% 精细速度 | 按住 RB | 按住 Shift |
| 连续张开/闭合 | LT / RT | F / R |
| 软件停止 | Back | Esc |
| 开始录制 | Start | Enter |
| 成功/失败结束 | A / B | 1 / 2 |

模式和坐标切换需松开允许动作键并回中；输入设备改变需重新准备。夹爪变化同样需要允许动作，松开后保持目标开度。5% 径向死区连续重映射，平移最大 10 mm/s、角速度最大 5°/s、夹爪最大每秒 10% 开度，周期 20 Hz。文本编辑屏蔽控制快捷键并停止；失焦、隐藏、设备丢失、连接中断、租约过期或错误需重新准备。软件停止不能替代未来实机的硬件急停。

以上速度为模拟默认值。实机输入使用后端下发的能力配置，初测上限2 mm/s、1°/s、每秒5%开度；末端加速度2 mm/s²、1°/s²，关节2°/s与4°/s²。未通过实机验收前不开放5 mm/s、2°/s交付范围。

“示教数据”浏览原始 episode 与有效目标；中止和不完整记录保留但默认不能导出。“LeRobot 数据”通过固定版本官方读取器和 PyAV 读取实际导出视频，支持跨 episode 的索引浏览，不驱动模型或回放动作。

## 结构与协议

```text
web/                         React 19 + TypeScript + Vite，浏览器 Gamepad API / 键盘
src/e6_teleop/service.py      PC 服务（127.0.0.1:8765），输入桥接、录制与数据浏览
src/e6_teleop/gateway.py      独立模拟网关（127.0.0.1:8766），控制守护、IK 与渲染
src/e6_teleop/core.py         单控制者、租约、停止与增量积分
src/e6_teleop/model.py        固定官方模型；DLS IK，Link6 控制点
src/e6_teleop/exporter.py     官方 LeRobot writer → finalize → reader / DataLoader 校验
assets/e6/                   固定 URDF、网格、许可证与 SHA256
tests/                       离线状态机/运动学/记录与 WebSocket 测试
scripts/acceptance.py         双进程三段 30 秒模拟验收
```

浏览器与 PC、PC 与网关均分开控制和观测 WebSocket。网关只允许一个控制连接，握手产生 session。模拟v1每次input消费250 ms租约并返回下一租约；硬件v3使用同一服务端租约按递增序号续期，输入发送和确认接收独立运行，停止或故障撤销旧租约。PC不自动补发失败命令，浏览器重连清空待发操作，回中并显式准备后才能重新随动。

`InputCommand`：`operation=input`、`seq`、`lease`、`source`、`identity`、`twist=[vx,vy,vz,wx,wy,wz]`、`gripper_rate`、`frame=base/tool`、`mode=translation/rotation`、`deadman`、`fine`、`neutral`、`connected`、`focused`、`stop`、`raw`。PC 添加自身接收时间；连接层添加 session。内部单位米、弧度、归一化开度，外部四元数为 xyzw。

`Observation`：六轴 `joints_rad`、`tcp_position_m`、`tcp_quaternion_xyzw`、`gripper`、`frame_id`、`gateway_monotonic_ns`、`valid`、`is_simulated`。`EffectiveAction`：六关节和夹爪**实际采用的目标**、`requested_tcp`、`ik_residual`、`ik_result`、输入序号、应用时间与 active。IK 以当前关节为种子，残差上限 1 mm / 1°、关节跳变上限 0.2 rad；拒绝时保持当前目标并停止。

旋转用四元数左右组合，工具坐标的线速度也旋转到基座坐标，不直接累加欧拉角。模型以 URDF 原点、轴、限位和官方网格构建 MJCF；只更新运动学，不执行 MuJoCo 动力学步进。网格、代理夹爪和虚拟相机不构成接触或实机附件几何验收。

## 数据

默认目录为根项目 `datasets/e6_teleop/`（Git 忽略）。每段 raw 包含 `metadata.json`、逐帧 `samples.jsonl`、`images/` 和结束 `summary.json`，保留结果 success/failure/aborted、输入、末端请求、IK、有效动作、模拟状态及图像 SHA。中断退出产生无结束摘要的 raw，也可以浏览且禁止导出。

观测与图像在应用 `a_t` **之前**采集，动作标签来自同一行 `effective_action`，绝不用下一帧观测回填。分别保存浏览器单调毫秒、PC 和网关的 `time.perf_counter_ns`；这些是不同进程的时钟域，未来跨主机不得直接相减。

默认导出特征：

| 特征 | 形状及语义 |
|---|---|
| observation.state | float32[7]，6轴 rad＋夹爪归一化开度 |
| action | float32[7]，同一帧有效绝对关节目标＋夹爪目标 |
| observation.images.wrist | 360×640 RGB，CPU H.264 视频 |
| next.success | bool[1]，操作者指定该段结果，非抓取检测结果 |

LeRobot 固定 20 fps 时间轴使用 `index/20`，原始实际周期和时钟仍保留在 raw；这不是硬件同步。可用浏览器导出单段，或从根目录合并多段：

```powershell
.\teleop\.venv\Scripts\python.exe -m e6_teleop.exporter datasets/e6_teleop/raw/EPISODE_A datasets/e6_teleop/raw/EPISODE_B --output datasets/e6_teleop/lerobot/MY_DATASET
```

创建器使用官方 `create/add_frame/save_episode/finalize`；完成所有状态、动作、视频、边界、DataLoader 校验后，才把 `.partial` 目录转为完整结果。失败目录保留用于诊断，默认不显示为已验证数据集。任何数据都不会自动上传。

## 验证与边界

从根目录执行 `.\teleop\.venv\Scripts\python.exe -m pytest teleop/tests -q`；前端在 `teleop/web` 执行 `npm test`、`npm run build`。启动工作台后关闭持有控制权的浏览器页面，再运行 `scripts/acceptance.py` 和 `scripts/fault_acceptance.py`。验收脚本使用软件合成输入，不是轨迹自动回放。

验收记录见根项目 [docs/遥操作模拟闭环验收_2026-10-04.md](../docs/遥操作模拟闭环验收_2026-10-04.md) 和 `docs/evidence/teleop_simulation_20261004/`。实机 TCP、附件几何、碰撞、连续速度控制、CAN 与相机时钟适配另行验收。本阶段的关节状态、图像和结果均明确为模拟，不能用于证明真实抓放成功率。

来源与迁移边界见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 第三阶段影子验证与实机门槛

新网关源码仍由 `deploy_observer.py` 部署到独立目录。可用 `--skip-dependency-install --evidence PATH` 复用固定依赖并核查每份上传源码SHA256。双击 `launch-control-shadow.cmd` 启动影子展示，页面只查看真实反馈、模型与记录；自动验收会话禁止实体输入。

自动测试前关闭其他工作台。将随机生成的 `E6_TELEOP_HARNESS_TOKEN` 仅保存在本次环境中，再启动：

```powershell
$env:E6_TELEOP_HARNESS_TOKEN = & .\teleop\.venv\Scripts\python.exe -c "import secrets; print(secrets.token_urlsafe(32))"
.\teleop\.venv\Scripts\python.exe -m e6_teleop serve --backend hardware-control --control-stage shadow --controller-role harness --ssh-host 192.168.1.13 --remote-root /root/e6-control-20261005
```

另一个终端使用同一本次token运行测试。合成信号由生产TypeScript映射生成，经PC输入桥、服务端租约和X5守护进程；不直接发送关节目标：

```powershell
node teleop/web/scripts/synthetic-inputs.mjs output/control_mapping.json
.\teleop\.venv\Scripts\python.exe teleop/scripts/verify_synthetic_mapping.py output/control_mapping.json --output output/control_offline.json
.\teleop\.venv\Scripts\python.exe teleop/scripts/control_acceptance.py --stage shadow --suite matrix --seconds 600 --fixtures output/control_mapping.json --evidence output/control_shadow
```

`hardware-control` 使用v3，保留v1模拟与v2严格观测。X5独立守护进程独占29999及30004；控制SSH TCP与视频、原始媒体回传分离。控制点为User0、零偏移Tool0法兰；控制器InverseKin以最新实际关节为种子，PositiveKin核查1 mm/1°残差。URDF仅显示，不用常量补偿掩盖两套参考系差异。

控制后端默认预览10 Hz、RGB480×270、深度240×180、JPEG质量55；观测后端仍为20 Hz、640×360／320×240、质量70。原生采集和20 Hz选帧、记录不变，实际预览尺寸和质量由能力字段报告。后台解码、模型FK和验收证据写盘不占用输入事件循环。

X5相机的解码、缩放、深度预览、JPEG和哈希在两个有界工作线程中处理，仍按接收顺序发布；全部RGB帧均解码，原JPEG和uint16深度字节保持。退出时会排空守护进程结果管道，避免相机已释放但子进程仍等待队列写出的挂起。状态中的`first_fault`保留最初故障，后续旧租约拒绝不能覆盖它；只有显式重新准备成功才清除。

v3输入年龄与续期采用X5入口收到数据的单调时间，另保存取得控制锁后的处理时间；客户端提供的同名字段不能改变租约。正常松开允许动作键不再把进行中的只读IK/FK误判为异常，发送前检查释放状态；未确认Stop会立即撤销准备和旧租约。

本台AIC8800驱动的`iw power_save off`入口已由二进制确认只返回成功，因此`--wifi-low-latency`只表示cfg80211请求，不能认证固件已关闭省电。`diagnose_wifi_power_save.py`是需先释放设备会话的独立临时重载诊断，附5分钟自动回退，不是工作台启动动作。曾对照的关闭TX/RX聚合设置导致无线不可达，已自动恢复并移除该选项，不作为推荐配置。

`diagnose_native_control.py --tcp-rto-min-ms 20`提供独立只读诊断：仅临时添加E6的wlan0主机路由RTO下限，拒绝覆盖已有主机路由，退出恢复；它不修改应用命令重试或控制期限，也不是默认启动设置。`--query-tos 0`用于与原查询QoS标记对照。诊断采样的1秒查询期限只用于测量实际尾延迟，v55生产完整事务硬期限为75 ms。临时网络配置的影子测试必须加`control_acceptance.py --diagnostic-transport`，结果不能解锁live；旧无线E6诊断配置仍有超时，不建议直接用于实机运动。

机械臂断电期间，`wireless_control_fixture.py`可在X5运行写命令全部拒绝的替身控制器，复用实际守护、输入桥和SSH转发测试无线链路。其合成预览不来自相机，结果标记`test_fixture=true`、`eligible_for_hardware_acceptance=false`，不能解锁live，也不能替代真实IK/FK、ServoJ和原始示教验收。

替身默认仅监听X5本机8888／8889，启动token经标准输入传入，stdin EOF退出；PC用已有SSH配置转发到8878／8879。PC执行`wireless_control_fixture.py client --fixtures output/control_mapping.json --preview-url ws://127.0.0.1:8879 --evidence OUTPUT`进行10分钟链路测试；`wireless_control_faults.py --fixtures output/control_mapping.json --evidence OUTPUT`用于另一段新替身会话中的协议故障，不能指向实机live端点。源码和摘要保留在本轮证据中。

影子模式计算候选，不使能、不发送ServoJ/Stop或CAN。live只允许固定ServoJ参数 `t=0.05,aheadtime=50,gain=500`，一条未完成目标；超出末端50 mm、10°或各关节相对基准10°时拒绝。v55 保持50 ms控制周期，完整事务硬期限75 ms；已确认的50～75 ms短时尖峰记日志并容纳，跳过过时周期，不补发、不追赶。输入租约250 ms、反馈100 ms、RGB500 ms及Stop发送50 ms／确认1000 ms保持。旧记录没有新预算字段时仍按50 ms审计，不改判历史失败。使能与准备分离，使能后至少5秒稳定观察建立基准。初始化前夹爪开度未知；机器人零增量测试可以先于CAN初始化，任何未初始化开度输入被拒绝。

自动live脚本须提供 `--shadow-evidence`，验证600秒完整矩阵成功、覆盖一致、实机动作/CAN发送为零；失败证据不能通过这个门槛。实体操作员会话另外要求 `--acceptance-profile`，完整检查全部实机项目、源码与CAN驱动SHA及受测参数。用户授权的事务余量调整也必须完成新源码影子与实机复核；变更前的通过记录不能替代它。当前完成状态见生产集成报告和进度索引。

v3原始记录位于 `datasets/e6_teleop/hardware_demonstration/raw/`，包含原输入、请求/候选目标、IK结果、控制器接受事件、实际反馈和原图关联。接受目标与实际到位分别记录，失败后已接受但未完成的命令保留不确定性。夹爪观测也取自动作之前，停止未确认禁止重新准备；控制故障中止记录，回传期间暂停控制。shadow明确 `is_real_demonstration=false`，不能作为动作训练数据，所有真实记录均拒绝现有LeRobot导出器。

`verify_observation.py`同时支持v2与v3读回；`--allow-aborted`只核查保留的中止前缀，不把它改成完整记录。Ctrl+C先停止本次控制并释放相机、隧道与CAN资源，保持既有使能状态，不停放、不下使能、不关机；CAN停止通信不等于夹爪24V断电。

当前验收结论、阻塞及证据见 [第三阶段实机遥操作接入验收](../docs/实机遥操作接入与自动信号验收_2026-10-05.md)。

局部运动学只读实验位于`experiments/local_kinematics.py`，原实验及其禁止解锁标记保持。生产控制新增显式`--kinematics controller-local --kinematics-profile PATH`，默认仍为原生。两端从原始只读采样重新验证和拟合，准备时原生IK/FK复核，逐个实际反馈检查局部模型与控制器Tool0的1 mm/1°偏差；超出模型域或控制器重启即停止。不能用孤立系数或旧实验结果替代完整控制链路验收。

`--feedback-framing length-envelope`默认关闭，显式认可已观测到的精确8字节长度封套，并分别保留wire前缀和1440字节SDK载荷；坏包仍中止，hardware-observe仍严格只读和严格解析。算法队列暂停字节照实保留：Mode4/5空队列且目标实际一致可接受1；本项目已发Stop后，最多1000 ms内的Mode7空队列过渡还须精确匹配最后接受ID与目标，期间禁止新目标与重新准备。Mode10、错误/碰撞或暂停运行队列仍拒绝，不自动Continue。

采样与模型构建入口为`scripts/sample_controller_kinematics.py`和`build_kinematics_profile.py`；已使能只读采样显式指定`--expected-mode 5`，先检查5秒稳定，不自行使能。生产集成及当前实机验收进度见[实机遥操作生产集成与验收](../docs/实机遥操作生产集成与验收_2026-10-05.md)，原实验报告见[控制器局部运动学方案验证](../docs/控制器局部运动学方案验证_2026-10-05.md)。

自动测试启动档位为`--harness-limits initial|delivery`，分别请求2 mm/s、1°/s、5%/s和5 mm/s、2°/s、10%/s；只能用于自动harness，不能代替实体operator验收。`synthetic-inputs.mjs OUTPUT initial|delivery`生成生产映射信号；live要求影子中的模型SHA及全部限制与当前一致。手柄连续开度的实际受测速率以信号幅值为准，档位上限实现不等于已验证最大输入。

新的delivery信号使用满幅摇杆及扳机，initial仍使用30%手柄幅值；两档都直接运行生产映射。满幅160类信号已在MuJoCo读回验证，实机最大速度、开度与包络仍需对应的独立验收。

独立六轴验收可用`live_signal_acceptance.py --suite frames-fine --frame-filter base|tool --fine-filter normal|fine`分组。每组建立当前起点，成功返回该起点；故障结束该组，保留失败证据，新测试须先确认停止和新画面再显式准备。释放后的最多3秒等待只发送新回中输入，检查对应的新Stop和新鲜静止反馈；慢速精细位移最多等待30秒。上述是测试程序的等待期限，设备事务50ms、反馈100ms、租约250ms、RGB500ms及Stop确认1000ms保持原值。持续速度测试使用真实键盘映射请求满幅上限；30%摇杆结果不能作为最大速率证据。偶发尖峰允许表现为保护停止，待回中并显式准备后恢复。

控制和源状态分别使用独立SSH TCP连接，PC控制端口8770、状态端口8771；预览仍走8767，原始媒体回传走8769。Windows仅对控制和状态两个socket注册qWAVE `QOSTrafficTypeControl`非自适应流，关闭连接时释放。X5通过当前SSH会话的祖先sshd和双方地址端口精确匹配accepted socket，只给这两个连接设置DSCP56。视频、媒体、其他会话及全局sshd、网卡、注册表、路由和系统策略均不修改。API成功不证明线传包标记或AP优先级，仍按原保护期限停止，抓包结果单独记录。

相机先启动并取得新鲜RGB，再启动独立控制守护，不给启动期看门狗豁免。未准备且输入回中时，Dashboard连接空闲10秒后退役，随后2秒退让、1秒预热；不发设备指令，不在运动中换连接，失败事务不重发。

独立控制SSH协商流式压缩以减少重复状态的线传字节，不增加输入缓冲或动作重发。ServoJ端点完成须同时匹配准确ID、接受后50–150 ms内的新反馈、QTarget与实际目标，Mode7本身不是完成证据。Stop只发送一次、50 ms发送预算，在同一连接独立等待最多1000 ms完整回包；等待期间禁用查询与新目标，界面分开显示发送、确认与反馈静止。未确认Stop不自动重试，新的实机停止时间和位移尚待测量，不能宣称50 ms回包或实际停止已达标。

显式 `--ssh-rto-min-ms 20` 仅用于拥有X5网关的控制会话：临时新增到当前SSH PC peer的wlan0主机路由，设置TCP最小RTO；拒绝覆盖已有主机路由，退出核对本次路由未被外部改变再删除。作用范围是X5到该PC的所有TCP连接，不是全局sysctl，也不改变应用重发或保护时限。控制/状态SSH在网关就绪及路由生效之后新建；报告通过只读TCP_INFO保存实际socket RTO，它通常大于20 ms，不把路由下限称为实际重传时间。shadow/live/operator须匹配该参数。默认关闭，当前实机稳定性验收仍未完成。

显式 `--e6-rto-min-ms 20` 为同一控制会话新增到E6的独立主机路由，核查实际路由值并在退出时恢复；默认关闭，拒绝覆盖现有主机路由。Dashboard事件中的 `tcp_timing` 保存X5到E6连接的实际RTO；20 ms是路由下限，不能保证回包在20 ms内。shadow、live及operator验收配置必须一致，不增加应用重发，不改变50 ms运动事务期限。

已安装AIC驱动的cfg80211省电回调只返回成功，`iw`状态不等于固件省电状态。`stats.wifi_latency.driver_ps_on`单独报告加载模块的初始化参数，不宣称独立固件遥测；shadow/live必须一致，operator启动前还会核对验收档案中的 `wifi_driver_ps_on`。模块重载仅在无设备会话的独立诊断步骤进行，不由控制网关自动执行，当前实验没有修改持久配置。

已发送但未确认的ServoJ保留原Dashboard连接，仅供独立Stop使用。停止接收器按完整命令回显排空已知迟到运动回包；该动作继续标为不确定，不补写有效动作。错回包、重复回包或停止确认超时关闭连接，不能在控制器尚占用旧连接时另开连接发送Stop。

v23的ServoJ周期完成表示“达到跟踪门槛”，不表示控制器空闲或精确到位。须准确匹配执行ID、反馈在接受后50～150 ms且新鲜度≤100 ms，实际关节及QTarget均距接受目标≤0.05°，实际Tool0距请求≤1 mm/1°。先前0.0001°的QTarget精确回显条件在P/D轨迹过渡中误拒绝，原失败保留。最多一个未达到此门槛的目标；未通过仍在150 ms停止，不积压或补周期。释放后的Stop确认和Mode5静止另行判断，实体控制继续受完整验收门槛约束。

v24将回中、未准备状态下的TCP连接获取移到独立线程，最多一个获取任务；1秒资源连接期限不属于运动事务。获取期间不发任何设备指令，继续20Hz控制检查和新鲜回中输入，完成后仍须1秒预热及显式准备。获取失败在该会话中终止且不自动重试，退出等待获取线程结束并关闭其socket。运动/停止与50ms事务期限保持，准备不会等待一个尚未就绪的连接。

## PC无线、E6直连X5

本轮无线范围已明确为PC↔X5。推荐接线是PC通过现有Wi-Fi/SSH连接X5的`192.168.1.13`，E6网线直接连接X5的eth0；E6地址仍为`192.168.1.4`。X5的eth0使用`192.168.1.14/32`，只为E6增加`192.168.1.4/32`直连路由，不给eth0配置默认网关，不改变wlan0或PC到X5的连接。PC通过工作台访问E6，不需要转发整个局域网。

`scripts/configure_e6_direct_link.py`管理名称`e6-teleop-direct`且UUID明确归属的NetworkManager配置。SSH密码从已有`E6_X5_PASSWORD`环境变量或交互输入读取，SSH主机密钥仍严格核对。审计不改变网络；create/activate/persist/remove要求先释放本项目设备会话，拒绝覆盖已有eth0地址或被外部改变的配置。初始create使用`save no`，重启会丢失，不能称为已经永久配置。

```powershell
# 已创建配置的UUID见当次owner记录；证据文件名每次使用新名称。
.\teleop\.venv\Scripts\python.exe teleop/scripts/configure_e6_direct_link.py audit --uuid c9ba60b6-d0d3-4fc0-be96-0d7c9041816b --evidence output/control_direct_audit.json
.\teleop\.venv\Scripts\python.exe teleop/scripts/configure_e6_direct_link.py activate --uuid c9ba60b6-d0d3-4fc0-be96-0d7c9041816b --evidence output/control_direct_activate.json
```

persist必须提供已完成全部实机项目的`--acceptance-profile`，并核对当前直连路由与受测配置一致；随后核查配置文件位于`/etc/NetworkManager/system-connections/`并从该文件重新加载同一UUID。此步骤只证明落盘与加载，重启使用仍单独验收。remove仅删除指定且设置未变的本项目配置，不删除其他NetworkManager连接。

v32将实际`stats.e6_network`保存至影子/实机证据；operator配置必须绑定受测的eth0、源地址与无网关直连状态。检测到路线不同就拒绝启动，不自动转回无线继续运动。使用直连时不添加`--e6-rto-min-ms`，其旧wlan0诊断与现有E6主机路由不兼容。PC无线链路仍独立验收，直连本身不能替代600秒影子、停止、各轴、夹爪、故障及原始数据验收。
