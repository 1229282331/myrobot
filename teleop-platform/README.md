# E6 phosphobot 工作台

当前已发布实时 Workspace 3D 与模型工作空间，并通过正式网页局部平移／旋转回归；完整 Goal 仍暂停，尚不宣称满速、全空间或最终综合验收完成。左3D／右真实相机共享现有会话，扩大范围由MoveIt及装配模型判断。模型与实测边界、版本、使用和回滚见 `../docs/teleop-rebuild/WORKSPACE_MODEL_REPORT.md`，最新设备状态见 `../docs/teleop-rebuild/STATUS.md`。

本目录运行固定版本 phosphobot 完整应用、Dobot 官方 SDK 和 MoveIt Servo，通过适配层接入 E6。上游版本、实际调用路径和补丁见 `upstream.lock.json` 与 `../docs/teleop-rebuild/OPEN_SOURCE_REUSE.md`。

自动实机测试仍仅限已确认的固定局部包络，不能通过重新准备或重设基准扩大扫掠。日常平台使用模型工作空间；模型未包含桌面、周边物体和线缆。夹爪安装方向已统一为用户确认的B方向（绕安装轴−90°、板／相机不变），旧错误方向的间隙计算不代表实物间隙。当前MoveIt实机链路使用`ServoDuration 0.1`；启动默认shadow，选择档位本身不授予实机资格。

当前动态夹指碰撞盒随实际CAN开度变化，反馈过期回退名义全行程；独立指节仍是名义运动学。夹爪使用原生CAN速度与有界位置前置，普通／常用精细档已有空载回归，极低速仍受8bit协议分辨率限制。见[动态夹指](../docs/teleop-rebuild/DYNAMIC_GRIPPER_BOUNDS.md)、[平移接近减速修正](../docs/teleop-rebuild/ARM_PROXIMITY_FIX.md)及[连续开合](../docs/teleop-rebuild/GRIPPER_SMOOTH_CONTROL.md)。

## 抓握中心 TCP

默认控制点已改为两夹指工作区域内的固定名义抓握中心。平移移动此点；旋转保持此点目标 XYZ，由 MoveIt 补偿法兰位置。基座／工具改变输入轴方向；关节模式仍直接控制关节。

当前工具定义是相对法兰 `[0,0,151.4] mm`、轴与法兰平行，来自官方夹指 CAD 加已确认的 8 mm R4 板。它尚未经实物标定，也不随自适应夹指开度追踪实际接触中心，页面明确显示该状态。原始法兰反馈仍可在详情和记录中查看；控制器 Tool0 保持零偏移，无需写控制器 TCP 参数。

使用已验证的 C++ 求解入口：`Start-E6.ps1 -MotionProfile normal -WorkspaceMode model -MoveItApi cpp -TcpProfile omnipicker-r4 -PreviewCodec vp8`，默认 shadow；实机入口显式加 `-Stage live`，仍需手动准备。正常退出后可用 `-WorkspaceMode local -TcpProfile flange` 回到旧局部法兰语义；不能运行中切换。`native_servop` 对照入口须显式用 `-TcpProfile flange`，不会静默忽略非零工具偏移。新 TCP 已通过模型与正式网页影子检查，实物旋转精度仍待验证；本次没有扩大已验证工位空间。详 `../docs/teleop-rebuild/TCP_QUALIFICATION.md`。

## 找回退出前未回传的记录

先退出普通工作台，运行 `.\teleop-platform\Start-E6.ps1 -RecoverRecordings`。需要时加 `-Password`；只找回一段可加 `-Episode 片段编号`。此入口复用正式phosphobot片段浏览，只有文件回传，不连接E6／相机／CAN，也不启动控制。中止结果保留，校验完成后可看原图、输入、实际反馈和未确认动作。Ctrl+C结束；之后使用普通启动入口遥操作。未封存或缺少session终止记录的文件不会自动修补。详`../docs/teleop-rebuild/RECORDING_RECOVERY_QUALIFICATION.md`。

## 安装与启动（当前 Windows PC）

在仓库根目录执行一次：

```powershell
git submodule update --init
python teleop-platform/scripts/setup.py
python teleop-platform/scripts/setup_launcher.py
```

需要已安装 uv、Node/npm。前者构建原版应用＋受审补丁；后者建立独立 Python 3.12 启动环境，按带 SHA256 的依赖锁安装，核对 MediaMTX 制品。不会在 X5 宿主安装 ROS 或替换相机环境。

仅修改前端后，在设备会话退出后运行 `python teleop-platform/scripts/prepare_upstream.py --build-dashboard`。该入口构建并复制到上游实际服务的`resources/dist`，逐文件核验并写入`e6-build.json`；单独运行`npm run build`只更新开发构建目录，不能视为已经发布到工作台。

当前 X5 的独立 Jazzy 运行目录和相机路径在 `config/workstation.json`。运行时核对 Servo/KDL/平滑库、模型和配置的指定 SHA；新会话使用独立源码副本，不覆盖旧实验。全新 X5 的 ROS 安装见 `scripts/bootstrap_jazzy_root.py`、`install_jazzy_root.sh` 及资格报告，尚不宣称全新设备一键部署已验收。

SSH 复用本机 `~/.ssh/config`、已有密钥／agent、`known_hosts`；未知或变更的主机密钥拒绝连接。使用密码时只在终端隐藏输入一次，不写入配置、参数或日志。

```powershell
.\teleop-platform\Start-E6.ps1 -Password
```

默认 `shadow`，自动打开 `http://127.0.0.1:8765/control?tab=keyboard`。该模式读取真实状态和相机，计算候选，不使能、不发机械臂运动或 CAN。没有密码登录需求时省略 `-Password`。开发中要复用经进程身份检查的已有 PC 应用／MediaMTX，可加 `-ReuseLocal`；它不会抢占已有 X5 会话。

`-Stage live` 是显式实机开发入口，当前仍是候选配置，不能视为最终操作者发布。所有公开启动入口默认Servo单点时长`t=0.1s`，发送周期仍20Hz；旧`t=0.05s`仅允许shadow，实机入口在创建资源前拒绝。启动不会自动使能、初始化夹爪、清错或更改设备参数。实际运动测试必须从新实际姿态及当前现场事实开始，且仅限已经实际走过的局部路线，不能重设基准或累计漂移扩大线缆扫掠。

`-MoveItApi cpp`（Python为`--moveit-api cpp`）选择固定MoveIt 2.12.4官方C++接口，复用相同模型、平滑器、SDK、停止和记录链路。省略时，`model`工作空间默认C++，`local`默认ROS；推荐明确指定上述日常入口。参数必须在启动时选择，当前会话内不能切换；局部回归不代表全空间及最终综合验收。当前二进制及实测见`config/workstation.json`和`../docs/teleop-rebuild/ARM_PROXIMITY_FIX.md`。

`-MotionEngine native_servop` 可显式选择官方原生 ServoP 对照路线；默认仍为 `moveit_servo`。未平滑的原生路线只允许 `commissioning`。开发验收可加 `-NativeSmoothing ruckig` 使用固定 Ruckig 0.19.4 的活动输入加速度限制，并显式选择更高候选档位；选项可运行不代表该档已通过实机验收。路线与平滑模式在启动时固定并核对网关回读；原生路线不启动 ROS、不挂载其 proc，也不要求运动模型就绪。它与 MoveIt 共用正式网页、反馈线程、单控制者、停止和资源退出流程。原生笛卡尔记录的 `joints_rad` 为未知（null），实际关节另存 `observed_joints_rad`，不能拿它补成七维关节动作；官方命令与回包完整保留。具体资格见 `../docs/teleop-rebuild/NATIVE_ROUTE_REASSESSMENT.md` 和 `../docs/teleop-rebuild/NATIVE_INPUT_SMOOTHING.md`。

活动输入平滑与旧 `release_mode=ruckig` 释放实验互斥：松开允许动作、轴回中或故障仍立即请求 Stop，不发送剩余平滑轨迹。Ruckig 使用时间同步的二阶速度接口，零到满幅请求的斜坡为200ms，命令空间加速度为每通道档位速度的5倍；不宣称限制了控制器内部关节加速度或物理停止距离。反向期间保留完整候选来源、上一步速度、加速度及增量，便于逐步读回核验。

live 页面使用独立的 **Enable Robot** 按钮：选择输入并回中，设备未使能且新反馈、画面及空闲状态满足时可点击。发送一次官方SDK带参`EnableRobot(0.750,0,-14,46)`（质量kg、重心mm，沿用SDK默认isCheck=0），参数来自`config/workstation.json`。必须由实际Load／Center反馈确认一致，并完成新的至少5秒稳定观察，之后仍需单独点击 **Prepare Control**。已使能时不重复使能；重启后不假定重心参数仍保留。该路径已实机核验，详[负载启动](../docs/teleop-rebuild/PAYLOAD_STARTUP.md)。0.750kg是用户指定的固定估计，不是额定负载上限参数；重心仍为工程估算。

使能回包不确定时不自动重发，也不会自动清错或下使能。页面刷新、控制重连和软件退出不自动恢复运动；使能中退出会先收集已发送请求的回执，退出本身不撤销设备已经接收的使能请求。详见 `../docs/teleop-rebuild/ENABLE_QUALIFICATION.md`。

日常工作台采用固定 MoveIt Servo＋官方 ServoJ，保留平移、旋转和关节输入共用的临界阻尼柔和起步（tau=0.2s）、30°/s²规划关节加速度，以及立即释放停止。以下是当前明确参数的影子检查入口；早期ServoP对照试验见历史资格报告：

```powershell
.\teleop-platform\Start-E6.ps1 -Password -Stage shadow -MotionEngine moveit_servo -MoveItApi cpp -MotionProfile normal -WorkspaceMode model -TcpProfile omnipicker-r4 -PreviewCodec vp8
```

上例先启动不发送动作的shadow，便于核对输入、设备状态与相机；显式`-Stage live`才允许实机控制。准备前设置所需模式和倍率，按Shift／RB精细档；normal只是后端上限，不能把默认0.8x或最高1.0x视为已经完整验收。历史低速旋转0.25°/s实测见`../docs/teleop-rebuild/LOW_SPEED_ROTATION_DIAGNOSIS.md`，其中旧20°包络不再是本轮测试许可。省略MotionProfile仍为commissioning。默认不加诊断二维码，需要测时才添加 `-LatencyMarker`。当前服务运行时使用现有页面，先正常退出再启动，不运行第二套控制者。

速度档由同一个配置下发给网页、输入检查和 MoveIt。开发影子测试可运行
`Start-E6.ps1 -Password -Stage shadow -MotionProfile normal`；省略 `-MotionProfile` 仍为 `commissioning`。

| 档位 | 平移 | 旋转／关节输入 | 状态 |
|---|---|---|---|
| commissioning（默认） | 2 mm/s | 1°/s | 实机初测配置 |
| intermediate | 10 mm/s | 5°/s | 候选，待分级实机验收 |
| normal | 30 mm/s | 10°/s | 正式Y/Rz连续实机已有记录；完整速度和停止仍未通过 |

上游速度选择器是上述上限的倍率，精细键再乘25%；夹爪变化率独立，提高手臂档位不会改变夹爪力或速度。档位在启动时选择，启动器会核对X5实际报告。测试时`1.0x`才代表完整目标速度；本轮仍只使用已走过的路线，不能为达到速度指标扩大范围。详见`../docs/teleop-rebuild/SPEED_PROFILE_QUALIFICATION.md`。

`servo_duration_s`表示官方ServoJ/ServoP的单点运行时间`t`，与20Hz发送周期分别配置。默认及live固定使用0.1秒；shadow可显式保留0.05秒来复核历史。启动时核对X5回读，正常释放仍立即Stop。默认修正仅让公开入口与受测参数一致，不能当作首动响应、全速度或稳定性已经通过；历史对照见`../docs/teleop-rebuild/SERVO_TIMING_DIAGNOSIS.md`。

工作空间由`test_workspace.mode`选择，发布默认`model`：取消相对准备位置的平移球，保留模型关节两端各5°、奇异和装配碰撞检查。`local`（包括无mode的旧配置）保留固定中心50mm球；相对旋转／关节限制为null。启动参数`-WorkspaceMode local`可回滚，不能运动中切换。采样点只表示无模型碰撞的离散TCP见证，不是实时许可或全空间净空证明；桌面、物体和线缆未建模。

开发入口的“自动输入验收”可选“对称往返（首尾半周期）”：首尾各半时长、中间11段完整时长，仍为六轮、同样速度和总时长。该选项只改变合成输入波形，不更改控制器或平滑器，也不保证实际精确回到起点。原单侧往返与全部失败证据保留；始终以正式映射后的输入、实际反馈和停止记录判断结果。

## 退出、恢复与问题包

正式键盘／手柄页底部展开 **Logs**，使用其问题包导出工具。保存后点击下载；即使X5已断连，只要PC服务仍在也能导出。包内包含当前或最后接收状态、最近两次本地会话与片段的结构化记录、配置和版本SHA，过期状态、截取和遗漏会注明。每项最多256KiB、文本合计4MiB，不包含原图、密钥或终端/SSH日志；凭据字段已过滤，分享前可检查。导出只写本机`runtime/diagnostics`，不发送设备命令。磁盘失败后恢复空间并重试，不会覆盖上一份成功包。详 `../docs/teleop-rebuild/PROBLEM_REPORT_QUALIFICATION.md`。

在启动终端按 **Ctrl+C**。X5 先让现有网关走停止及反馈清理，再退出相机、ROS 子进程和本次挂载；PC 最后退出自有应用和视频服务。普通退出保持原有机械臂使能状态。复用的 PC 服务由其原始启动者管理，当前启动器不会结束它们。

输入连接恢复后回中并显式准备；服务崩溃后重新运行同一入口。它不重放旧输入、不自动恢复实机动作。遇到未知资源占用时会保留现场并拒绝接管，不使用全局进程名杀进程。进程退出失败或停止结果不确定不等于安全关机。

相机采集进程退出时，原始机械臂反馈及网页继续运行。释放输入，等待机械臂静止以及中止片段完成回传，再点击 **重新连接相机**。新画面确认后，回中并点击 **准备控制**；重连不会自动恢复输入或旧录制任务。重连失败时页面保留原因，需要处理原因后再次显式点击。无需为相机重连退出整套工作台，详见 `../docs/teleop-rebuild/CAMERA_RESTART_QUALIFICATION.md`。

每次启动的 PC 日志、源码 SHA、启动状态和退出结果在 `runtime/sessions/<会话ID>`。X5 的原始反馈、媒体及控制事件保存在 `config/workstation.json` 指定目录的 `sessions/<会话ID>`；媒体仍使用独立连接回传。导出最近一次本地诊断包：

```powershell
python teleop-platform/scripts/export_support.py --session latest
```

ZIP 留在本地，只包含白名单中的会话日志／配置／状态／SHA，不扫描 SSH 密钥、私有辅助脚本、环境变量或整个数据集。不要把影子候选标签当作实机有效动作。

视频诊断可在启动配置中设置 `latency_marker: true`（默认关闭）。正式画面会叠加逐帧二维码，并显示可下载的测量报告，原始RGB不受影响。报告注明内核或SDK接收起点、时钟误差和实际呈现帧率；它不代表曝光同步。当前静止影子实测约20fps、P95上界204ms，运动与录制的长时综合验证尚待完成，详见 `../docs/teleop-rebuild/VIDEO_TIMING_QUALIFICATION.md`。

录制回传失败时，在“浏览本地片段”查看原因并点击“重新校验回传”。若提示本地状态未落盘，先恢复本地磁盘写入能力；当前失败暂存于服务内存，重试成功后会连同原错误保存。重新校验使用原包校验值，不会自动准备控制或开始新录制。

采集区与片段详情会显示回传字节和进度。达到100%后仍需校验原图、动作关联与保存回执；显示“文件校验通过”才表示回传完成。该状态不改变片段原有成功／失败／中止结果。原图体积较大，默认1MiB/s无线媒体限速可能需要数分钟，期间控制保持暂停；完成后回中并重新准备。

“采集备注”可选填写场景／物料和操作者代号，随本次记录保存；录制、封存及回传期间不能修改。按钮和手柄Start使用同一份开始时快照。片段库支持搜索任务／备注／编号、筛选结果和来源；详情分开显示输入、候选、已接受目标、实际反馈、执行回执及图像关联。图片超过本周期100ms会单独提示，原图仍可检查；采图时新鲜度不能代替逐动作有效性。原始JSON在详情底部展开。浏览期间暂停控制，关闭后回中并重新准备；此处不回放动作。

原始片段的官方 LeRobot 离线资格检查见 `../docs/teleop-rebuild/NATIVE_DATASET_QUALIFICATION.md`。目前仅用明确测试来源完成官方写入、全帧读取和边界验证；真实七维示教仍缺少合格的组合记录。新事件时间线不是均匀采样训练数据，正式页面尚未开放动作导出。

## 回滚

先正常退出当前入口并检查该会话 `result.json` 及 X5 自有资源释放结果。保留工作区改动，在另一个 Git worktree 检出明确的历史提交；按该版本锁重新构建前端及启动环境。不要在有控制会话时切换当前目录或同时启动两套设备服务。

重建前基线 `efa3aa1` 和补丁在历史及 `docs/teleop-rebuild/evidence/baseline`；Jazzy 影子基线 `b4f0f3f` 可复现此前候选验证。历史版本同样有其未通过项，回滚不授予实机验收资格。X5 旧会话与源码副本会保留用于诊断，当前脚本不自动删除它们。

状态连接恢复：服务暂不可用时正式键盘/手柄页保留界面并每秒只读重试，可点击“立即重试连接”。服务回来不会自动准备或续动，回中后显式准备；详见docs/teleop-rebuild/STATUS_RECOVERY.md。
