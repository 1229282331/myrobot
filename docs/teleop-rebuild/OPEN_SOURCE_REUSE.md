# 开源复用与补丁登记

2026-10-17 v285：继续使用固定MoveIt Servo `1ade0e9dcf50dbbbc3a984b995c786cf12736235` 的IK、奇异处理、关节速度限制及AccelerationLimitedPlugin，仅增加可选同步碰撞倍率选择入口；默认与碰撞零倍率不被覆盖。`prepare_collision_retreat.py` 在已有 `prepare_arm_proximity.py` 结果上生成补丁和SHA，编译器通过 `E6_DIRECTIONAL_SERVO_SOURCE` 引用，原上游源码和BSD许可证保留。`collision_retreat.hpp` 是本项目有限候选方向策略，调用MoveIt/FCL实际碰撞和既有连续区间证明；盒面距离直接用官方geometric_shapes三角化以避免FCL mesh/box特征跳变，不手搓通用IK或碰撞引擎，不缩盒。动态夹爪矩阵更新使用MoveIt原读写锁，内容相同不重复写入。无前端库变化。精确调用路径、失败反例、312组方向对照、X5回归及局部实测见 [COLLISION_RETREAT_V285.md](COLLISION_RETREAT_V285.md)，可复现生成源和补丁在 `output/collision-retreat-v285/moveit_servo/`。

2026-10-16 v269：夹指改为八个官方CAD活动部件各自局部OBB，Trimesh生成局部盒体；复用既有官方夹爪四连杆闭合求解，生成256项名义空载姿态表。实际CAN开度选取共享表，MoveIt/FCL原生碰撞检查覆盖已发送、在途和下一待发送开度区间。完整URDFViewer仍负责加载、相机交互与渲染；正式适配仅绑定实际开度关节并展示同一碰撞几何。未知开度才使用由这些动态盒生成的全行程回退。具体来源、覆盖、版本、调用、失败证据和回滚见 [DYNAMIC_GRIPPER_BOUNDS.md](DYNAMIC_GRIPPER_BOUNDS.md)。以下早期版本的静态扫掠方案仅为历史。

2026-10-15 v263：附件改为 9 个简单盒体，实际调用 Trimesh 4.7.4 的 slice_faces_plane / bounds.oriented_bounds，SciPy 1.16.2 做坐标旋转；E6本体复用原 prepare_collision_meshes.py 的单连杆凸包。v262 CoACD仍归档，但当前生成路径不再调用。前端沿用完整URDFViewer0.13.1，使用Three0.186.1的EdgesGeometry为同一URDF盒体加边线、区分刚体/名义扫掠颜色。详 [COMPONENT_COLLISION_BOXES.md](COMPONENT_COLLISION_BOXES.md)。


2026-10-15 v262：碰撞几何实际复用 CoACD 1.0.14（MIT）做刚体分块，SciPy/Qhull 半空间交集生成保守外包，trimesh 读取与导出官方 CAD；未改上游算法。调用、固定依赖、发行物 SHA 与许可证见 `scripts/rigid_collision_geometry.py`、`scripts/gripper_collision_geometry.py`、`scripts/requirements-workspace-cad.txt`、`upstream.lock.json` 及 `patches/collision/`。本地适配重新覆盖每个原始三角面，独立检查实际导出的 STL；夹指仍明确为名义空载包络。详 [COLLISION_MODEL_REFINEMENT.md](COLLISION_MODEL_REFINEMENT.md)。


2026-10-15 v261：实际复用完整URDFViewer0.13.1（24feef2c267496d22d17bbbb5c1bf5b86dcf174f，Apache-2.0）和Three.js0.186.1（MIT）；官方viewer负责场景、URDF加载、渲染与OrbitControls，项目适配仅绑定实际反馈、TCP轴、采样点与错误/资源释放。固定源码、SHA、有限补丁及复现脚本在`teleop-platform/patches/viewer`和`prepare_workspace_viewer.py`；正式phosphobot第0010/0011补丁固定依赖并接入双视图。复用官方E6/OmniPicker网格、用户R4 STEP、成熟CAD/MoveIt FK/FCL生成共享装配及采样；夹爪包络名义性质明确标注。没有物理仿真器或自写IK。详[WORKSPACE_MODEL_REPORT.md](WORKSPACE_MODEL_REPORT.md)。

2026-10-12 v252：正式输入柔化使用既有SciPy1.11.4 `signal.cont2discrete(method="zoh")`实现标准两级一阶临界阻尼滤波，放在原CppCycle→官方MoveIt2.12.4 IK之前，原官方AccelerationLimitedPlugin及C++二进制不变，不另写运动学／通用轨迹优化器。`intent_ramp.py`为有限输入调理和状态记录，原网关负责停止。曾编译固定上游Ruckig插件速度接口适配，但离线旋转TCP漂移不合格，未在生产启用；实验源码和失败证据均归档，不列为正式依赖或实机通过。参数、公式、验证、回滚见J5_STARTUP_ACCELERATION.md v252节。

2026-10-11 v239：恢复固定phosphobot0.3.134/a5de051官方Layout、Sidebar/AppSidebar、TopBar、Footer及键盘／手柄页面布局，复用SpeedSelect、CameraStreamCard和原shadcn/Radix组件实现E6状态、紧凑录制栏和数据检查。移除自定义工作台样式覆盖，无新增UI库／主题。官方导航保持正常入口，进入未适配路由才提示。收敛0006、更新0008上下文，八补丁在固定干净源顺序应用后与工作副本20文件匹配；prepare脚本隔离Git index校验整链，避免覆盖本地修改。147前端测试及正式构建通过，无控制算法／后端协议改动。本轮复用路径、必要差异、截图、SHA与回滚见[UI_RESTORATION.md](UI_RESTORATION.md)。

2026-10-10 v236：对固定MoveIt Jazzy提交 `1ade0e9dcf50dbbbc3a984b995c786cf12736235` 的 `moveit_servo/src/utils/common.cpp` 维护有限方向退回补丁。上游虽计算leaving band，最终全方向仍在hard阈值停；补丁保留接近阈值、数值秩检查，仅允许改善条件数的低速退回，并在C++适配器复核官方平滑候选。`scripts/prepare_directional_servo.py`校验原SHA、生成独立副本／diff／来源；CMake实际编译原collision_monitor／servo／utils/common／utils/command，保留BSD许可，不重写IK／平滑器。不修改已安装ROS或子模块，旧二进制和清单保留。旧／新12方向、正常等价、反向阻止及正式网页实机小幅退回证据见DIRECTIONAL_SINGULARITY_RECOVERY.md；属于本地补丁，不称上游已修复。

2026-10-09 v230 TCP：继续使用固定MoveIt 2.12.4官方C++ Servo／KDL，未重写IK。在已核对E6模型的内存副本添加grasp_tcp fixed link并改变SRDF chain tip，C++仅将工具输入frame改为配置读取；原actual_fk／target_fk仍返回法兰供控制器独立核对。夹指几何复用OmniPicker官方xacro／STL，加已确认R2盘厚度；来源文件SHA、固定名义点的限制及构建源码／制品SHA见TCP_QUALIFICATION.md。正式phosphobot页面、mapper和录制继续使用原应用链路。

2026-10-09 v202：继续实际运行固定GamepadControlPage／SpeedSelect／GamepadUtils。0007为上游extractAnalogValues增加可选deadzone，保留默认15%；E6传0后采用既有5%径向映射，修复双重死区。0002补齐显式合成来源的上游准备入口；GamepadAcceptance仅在设备读取边界提供轴／按钮样本，调用同一正式hook与无线链路，不直接发目标。七补丁干净应用、112前端及正式构建通过；正式shadow36方向146候选来源/方向通过，4父节点未记录、运动StopCAN0。实际实体getGamepads路径没有被替换成全局shim，合成测试不冒称实体接收器验收。详GAMEPAD_SIGNAL_QUALIFICATION。

2026-10-09 v168工作台改版：仍实际运行固定phosphobot的KeyboardControlPage、GamepadControlPage、SpeedSelect、Recorder与CameraStreamCard；0002/0005更新布局及E6录制呈现，新增0006条件接入原Layout、TopBar、AppSidebar和ControlPage。`frontend/WorkbenchChrome.tsx`复用上游Sidebar/ThemeToggle/Button，`RecordingPanel.tsx`复用原Recorder处理函数、Input/Button/Dialog。没有另建应用或改变上游版本。六补丁干净应用／重复prepare、55前端、正式构建及正式影子录制通过，详UX_QUALIFICATION。非E6录制路径保留并有DOM测试。

2026-10-09 v163/v165：原图缓冲继续复用既有ArchiveStore、Python标准有界Queue及Zstandard，仅增加条目／在途载荷双预算；接收调度使用CPython官方`sys.setswitchinterval`，不是另造线程调度器。视频报告继续原MediaMTX／jsQR测量链，新增本机保存实际调用FastAPI、Starlette FileResponse及标准文件接口，正式phospho页面已下载并核对原字节SHA。源码分别为`media_archive.py`、`runtime_timing.py`、`video_reports.py`与`frontend/WebRTCPreview.tsx`。详细资格、失败和源码边界见同轮三份QUALIFICATION报告。

首轮UX调研见UX_RESEARCH.md及固定源码清单：DROID、Open-Teach只作源码／文档阅读，Foxglove、Rerun只作官方文档交互调研，均未被声称为运行依赖或本机实测。后续界面继续保留上游phosphobot页面、速度、录制及Radix/shadcn组件；研究借鉴与代码复用分别记录。

v150新增实际调用：`frontend/EpisodeBrowser.tsx`复用固定phosphobot的`components/ui/dialog.tsx`（Radix/shadcn），替换推走实时视频的内联展开列表。上游Dialog未修改。预览增加X5已安装FFmpeg/libvpx VP8的启动固定选项，继续使用原MediaMTX与官方reader，未改其信令。版本/命令/真实WebRTC codec回读和失败H.264对照见PREVIEW_DECODING_QUALIFICATION.md及video-diagnostics-v148至v150；默认仍H.264，VP8仅完成短时影子候选资格。

## 固定来源

| 模块 | 上游与固定提交 | 实际使用 | 本地差异 |
|---|---|---|---|
| 完整工作台 | [phosphobot](https://github.com/phospho-app/phosphobot/tree/a5de051197c879e3c685d1362f649ce54ee47a3c)，MIT，0.3.134 | `upstream/phosphobot/dashboard`完整构建；`phosphobot/app.py`完整API与静态页面运行 | 受审patch＋显式E6扩展目录；不以另写页面冒充原版 |
| 输入页面 | 同上 | `pages/control/KeyboardControlPage.tsx`、`GamepadControlPage.tsx`、`GamepadComponents.tsx`、`GamepadUtils.ts` | 补丁0002保留页面，增加E6允许动作/模式/坐标/最新意图；正式键盘页已驱动实机Z小幅往返 |
| 速度与相机布局 | 同上 | `components/common/speed-select.tsx`、`camera-stream-card.tsx` | 保留速度组件和相机卡片；增加实际E6状态及WebRTC渲染分支 |
| 视频传输与浏览器信令 | [MediaMTX v1.21.1](https://github.com/bluenviron/mediamtx/tree/048255986f7e04b859b4c4efe651448ec785ecd4)，MIT | 官方Windows程序、`internal/servers/webrtc/reader.js`；X5 FFmpeg libx264 | reader只把末行window赋值改为ESM导出；React包装帧率测量，未重写WebRTC。官方ZIP SHA见upstream.lock.json，解压EXE SHA见config/workstation.json |
| 仿真 | 同上 | PyBullet3.2.7、SO100Hardware，原页面方向卡片→`/move/relative`→模型→关节读回 | 上游原版资格测试，仅SO100示例，不称E6模型已验证 |
| E6命令与反馈 | [Dobot TCP-IP-Python-V4](https://github.com/Dobot-Arm/TCP-IP-Python-V4/tree/55ec1ec82201aaf2e6d54aa47b6272bbaa14c815)，MIT | `Motion.ServoP`、`RobotControl`；`MyType`与`FeedbackParser`，实机已调用 | 仅扩展TCP帧边界与反馈接收生命周期，不修改官方命令生成/解析字段，不使用其有缺陷的整机反馈循环 |
| Gemini E采集 | 已验证Orbbec SDK1.10.37及项目UVC PTS采集 | `camera_service.py`调用既有Camera/decode接口；固定SN AY4A1520111，MJPG1280×720、深度640×480 | 采集与编码独立；原SDK字段保留，预览附接收时间/帧号，不声称曝光同步 |

依赖：上游前端`package-lock.json`；后端本机解析结果保存在`teleop-platform/locks/phosphobot.uv.lock`，Python3.10.20。官方SDK上游未提交运行锁，X5读取使用numpy1.26.4及隔离目录requests2.32.5，后续网关将保存完整依赖清单。

## 已复现的差异

- 官方SDK `receive_text()`只读一次，返回TCP片段；`_feedback_loop()`丢弃粘包。`tests/test_sdk_upstream.py`保存复现。
- `dobot_transport.py`只处理流边界、sendall、命令回包关联和不确定回执，不生成运动学目标。超时命令不重发，迟到原回包不确认Stop。
- v12捕获80字节截短旧头加新头的异常原包，官方解析同样错位。适配器新增内嵌头拒绝和原字节证据，不修改SDK字段、不猜测重排；完整复现及只读部署见FEEDBACK_FRAMING_DIAGNOSIS.md。
- `feedback.py`不调用任何模型/视频/记录逻辑。订阅队列溢出只影响订阅者；原包保留，官方SDK解码在消费端进行。模型越界不能关闭接收。
- `patches/0001-local-simulation.patch`：给模拟机器人稳定设备名；网络/USB扫描只在连接设置弹窗打开时启动。原版可手动选择无名模拟臂，但不能自动选择。
- `patches/0002-e6-input-pages.patch`：保留上游键盘、手柄和相机组件，绕开E6不适用的自动回初始位/自动张爪路径。后端BaseRobot扩展只提供实际读数，写入统一走意图连接，旧写入/录制路径明确拒绝。
- `patches/0003-nonblocking-version-check.patch`：Windows/macOS上游异步版本接口同步requests阻塞事件循环；352.7ms最小复现和修正结果均归档，改为asyncio.to_thread。
- 网关固定依赖见`locks/gateway-requirements.txt`，X5独立target安装，没有安装phosphobot、MuJoCo或LeRobot。

## 尚未完成

官方LeRobot完整真实示教验收、最终速度/时延/稳定性验收。夹爪15–60%分级空载已有v74证据，不能代替最终综合运行。引用项目、原始审计日志和网页按钮开放均不等于完成这些能力。

v97新增[官方LeRobot](https://github.com/huggingface/lerobot/tree/8c920c4270460851cedd2737657584586d3dc66f)，Apache-2.0，固定提交`8c920c4270460851cedd2737657584586d3dc66f`/0.6.2。PC离线Python3.12沿用`teleop/uv.lock`，许可证`teleop-platform/licenses/LEROBOT-LICENSE`。`native_dataset.py`实际调用`lerobot.datasets.lerobot_dataset.LeRobotDataset.create/add_frame/save_episode/finalize`、官方读表/视频和PyTorch DataLoader，沿用原模拟exporter的官方API，不维护自写parquet/视频索引。未修改上游。两段明确测试来源共52帧全部标签、原分辨率视频及边界读回通过；20段本地真实/影子记录检查无合格完整七维来源，未开放正式导出。事件序号时间线不代表真实20Hz，详见NATIVE_DATASET_QUALIFICATION.md。

v35/v36新版适配实际使用2.12.4 `ServoNode`、官方KDLKinematicsPlugin、已审v34加速度插件和SciPy CubicHermiteSpline。新增补丁0003只追踪输入来源、复用已有回调锁及处理首状态等待的退出；宿主/隔离子进程不含自行求解IK的路径。完整正式桥接虚拟36方向、20Hz延迟速度及退出结果见JAZZY_QUALIFICATION.md，未等同新版实机通过。

## MoveIt备选路线（2026-10-07，方向已接入、持续速度未通过）

- [官方Dobot ROS2](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/0f21e9839888f188ac06b0a9fea622d13a29a74f)：固定提交`0f21e9839888f188ac06b0a9fea622d13a29a74f`，新增子模块。实际使用`dobot_rviz/urdf/me6_robot.urdf`、7个STL、`me6_moveit/config/me6_robot.srdf`。仓库根MIT，运动学插件源码另标BSD；没有运行它的设备bringup。KDL对照发现原几何/坐标不一致；现保留原件，使用明确几何总量与参考帧的等价模型，独立广域与雅可比验证见CONTROLLER_MODEL_ALIGNMENT.md。
- [MoveIt Servo 2.5.10](https://github.com/moveit/moveit2/tree/c283a36186a6f7a5985360e6674bf8fd0790e485/moveit_ros/moveit_servo)：固定提交`c283a36186a6f7a5985360e6674bf8fd0790e485`，BSD-3-Clause，新增子模块供审计。X5直接运行官方Debian `servo_node_main`，实际调用原Servo的Jacobian、奇异处理、Butterworth平滑、碰撞检查、JointJog/Twist和暂停/恢复服务。v27虚拟、v28网页影子及v29/v30网页ServoJ小往返已实际运行；持续速度不通过。仅元数据补丁patches/moveit/0001-input-provenance.patch，保存算法选中的输入类型/stamp，未改求解/平滑/保护。原构建失败、PC交叉编译记录和实际库maps保留。
- 原库输入配置基于固定版本`config/panda_simulated_config.yaml`，明确改E6组/帧/话题、SI速度单位、50ms周期及2°/s、4°/s²测试限值；未取消碰撞或奇异保护。原碰撞网格过密导致启动失败，采用[Trimesh4.8.3](https://github.com/mikedh/trimesh/tree/4.8.3)（MIT）/SciPy1.15.3 QHull生成保守碰撞凸包；保持原视觉STL。全部源顶点的包含关系、闭合性、制品SHA在`collision_hull_manifest_v19.json`。这不是校准后的真实/附件碰撞模型。
- KDL比较程序只调用系统`kdl_parser`、`ChainFkSolverPos_recursive`、`ChainJntToJacSolver`。没有加入自写通用运动学求解器。包清单、APT原计划及安装日志见`evidence/ros_setup_v17`；旧相机Python环境和既有网关依赖目录没有安装MoveIt的Python依赖。

诊断结果与限制见`MOVEIT_ROUTE_QUALIFICATION.md`。普通档30 mm/s、10°/s及完整正式网页MoveIt能力未验收；小幅实际往返不是完整交付。

- 新候选为[MoveIt Servo 2.12.4](https://github.com/moveit/moveit2/tree/1ade0e9dcf50dbbbc3a984b995c786cf12736235)，固定子模块`upstream/moveit2-jazzy`，BSD-3-Clause。在隔离Ubuntu24.04根目录安装官方ARM64包`2.12.4-1noble.20261006.043301`，先用原版`servo_node`、默认AccelerationLimitedPlugin、原JointTrajectory接口运行延迟plant对照。测试插值使用SciPy CubicHermiteSpline；不包含E6/CAN连接，不以虚拟结果替代正式网页验收。安装及运行脚本位于`teleop-platform/scripts/*jazzy*`。
- `patches/moveit/0002-acceleration-fine-input.patch`记录原加速度插件的非零微小增量、OSQP约束等价缩放与精度修正。`prepare_acceleration_patch.py`从固定源码按严格上下文生成平铺插件源码和SHA，命名空间/注册名独立，CMake将其安装到独立前缀；不是直接覆盖ROS文件。QP仍由官方OSQP求解，Jacobian/奇异/碰撞/限位仍由官方Servo执行。第一失败补丁及最终v34源码、构建和对照均保留，尚无实机资格。


- 视频测量新增 jsQR 1.4.0（https://github.com/cozmo/jsQR，Apache-2.0），通过上游dashboard package-lock及补丁0004固定；只用于诊断QR帧标识。编码使用现有OpenCV 4.11.0 QRCodeEncoder；媒体传输仍为FFmpeg/MediaMTX/WebRTC。
- OmniPicker复用本项目已验证的`omnipicker/tools/verify_can_control.py`，原样复制到`teleop-platform/src/e6_phosphobot/omnipicker_can.py`，SHA256 `44c99deed1a9159406147307619acfe0fd61f4f2a8602f0cb386c61b933dbc7b`。身份核查、SocketCAN设置、协议编码、真实反馈解析和回显排除均实际调用；`gripper.py`只适配独立工作线程及页面目标。

- 正常释放减速实验使用[Ruckig 0.19.4](https://github.com/pantor/ruckig/tree/a8db97a4e9c55e5160a3855f739fa3b270df8e4c)，MIT；源码提交、PyPI制品SHA记录于upstream.lock.json，许可证保存在licenses/RUCKIG-LICENSE。`trajectory.py`实际调用Velocity接口、update与pass_to_input，参考官方05速度与06停止示例；没有复制轨迹求解器或使用云端中间路点。适配层负责SI单位、基座/工具变换和四元数组合。
- `--release-mode ruckig`是显式实验配置，默认仍immediate。正常释放转零速度目标；故障直接Stop。分析独立报告减速首帧与Stop发送时间，不能把延后的Stop记为50ms内发送。新释放分析按true→false边沿关联，不用后来空闲心跳覆盖原释放时刻。

- 原始媒体复用第二阶段`teleop/device_gateway/e6_observer/archive.py`，基线SHA256 `da8f34257309631c90ef2a960d4a0f79774832bbc031076144021510771cee3e`，迁移到`media_archive.py`。保留原有Zstandard 0.25.0、有界队列、帧ID去重、SHA校验与会话所有权；仅扩展元数据入manifest及线程退出检查。`camera_archive.py`适配独立20Hz选帧和相机进程内的录制请求，未重新编写压缩器或替换预览协议。

- 正式原始示教沿用上游`components/common/recorder.tsx`，补丁0005修改E6分支的按钮语义与处理函数，保留原设置/相机组件；独立episode适配调用既有Audit/CameraArchive和Python标准ZIP/SHA、Zstandard及Pillow验证，不替代整个工作台。新增浏览在同一phosphobot控制页展开。LeRobot真实动作导出仍未验收。

- Jazzy来源补丁`0003-jazzy-trajectory-provenance.patch`已纳入固定清单：实际编译完整官方ServoNode，仅追加窗口来源和已有互斥锁的回调保护。v36隔离桥接及v39正式网页影子实际调用；2.5.10保留为失败历史，非当前主实现。
- 启动与SSH复用[Paramiko4.0.0](https://www.paramiko.org/)（LGPL-2.1）和[psutil7.1.3](https://github.com/giampaolo/psutil)（BSD-3-Clause）。实际调用SSHClient/RejectPolicy/open_channel/request_port_forward及本地监听进程查询；依赖版本与制品哈希在`locks/launcher.txt`。本地代码只管理已存在网关／相机／上游应用的资源顺序和日志，不新增通用通信协议或控制租约。

- v67/v68正式Jazzy→ServoJ端点适配改取上述固定官方`composeTrajectoryMessage`已发布的最后完整点；不使用其省略的未来点，不新增求解器。`TimedTrajectory.latest_complete`直接选取原数组并保留全窗口来源，原SciPy时间插值仅保留为工具。上游二进制与补丁未改；实际调用和实机局限见`ENDPOINT_QUALIFICATION.md`。

- v120诊断实际使用官方[py-spy 0.4.1](https://github.com/benfred/py-spy)二进制，MIT，PyPI wheel SHA和许可证在evidence/scheduling-v120；隔离会话目录运行、未修改系统Python。采样窗有调度过载，不能用于正常性能结论。
- v122/v123复用官方[rmw_fastrtps异步发布](https://github.com/ros2/rmw_fastrtps/blob/rolling/README.md)选项，不自行实现DDS队列。X5既有固定发行包Fast DDS2.14.7、rmw_fastrtps_cpp8.4.4、rclpy7.1.12版本读回在camera-cache-v121/dds-environment.txt；jazzy_host.py仅为会话子进程传入RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS，官方库/MoveIt二进制未修改。生产原始输入来源、KEEP_LAST(1)和有效期继续生效；返回不等于设备接收。虚拟两组144秒和正式影子六组分别保留结果，单次不复现不宣称根治。


### v125–v129 原生ServoP复评

仍使用固定Dobot SDK 55ec1ec82201aaf2e6d54aa47b6272bbaa14c815 的 `dobot_sdk/api/motion.py::Motion.ServoP`；统一启动器新增固定 `motion_engine` 选择，直接执行上游命令构建，无新本地IK/FK或模型拟合。网页沿用现有phosphobot完整应用及键盘正式链路。控制线程首动唤醒是本地设备适配调度补丁；输入不直接发送设备命令。新分析器用同一官方SDK重建并逐条核对发送文本，原生未知关节目标不以观测状态补填。上游版本、许可证和原页面补丁均未改变，实机低速证据见NATIVE_ROUTE_REASSESSMENT.md。

### v130 原生活动输入平滑

实际复用已固定 [Ruckig 0.19.4](https://github.com/pantor/ruckig) 的 `ControlInterface.Velocity`、`Synchronization.Time`、`Ruckig.update` 与二阶加速度限制；调用在 `trajectory.py`，没有本地通用轨迹求解器。上游库和MIT许可证不变。本地新增启动选项、档位加速度、完整步进来源；正常释放仍走既有即时 Stop。有限jerk的快速随机转向实验曾出现30mm/s请求下31.613mm/s速度范数，因此活动输入使用上游无限jerk（二阶）模式，不假称jerk有限。三档平移／旋转／关节随机转向在完整轨迹内采样核查，旧释放实验保留原jerk设置。候选审计直接重放固定Ruckig单步并核对原输入与位姿增量，反向残余全部计入。

### v141 页面状态恢复

保留固定phosphobot键盘/手柄页面与[SWR](https://github.com/vercel/swr)缓存及mutate接口。补丁0002只替换状态发现hook和错误提示，扩展文件useControlServerStatus.tsx设置GET超时/重试、缓存显示和手动重读；不替换输入或设备通信。SWR版本仍由既有上游及测试package-lock固定。原完整页面已在真实影子重启前后运行，源码与构建SHA见status-recovery-v141，未借此称完成新实机运动验收。

### v175 官方 C++ API 隔离候选

`scripts/servo_cpp_probe.cpp` 改编并保留固定MoveIt2.12.4提交 `1ade0e9dcf50dbbbc3a984b995c786cf12736235` 的BSD许可及 `demos/cpp_interface/demo_joint_jog.cpp` 初始化/调用方式，直接链接安装的官方 `moveit_servo_lib_cpp`，复用 `Servo::getNextJointState`、`setCommandType`、`resetSmoothing`。使用既有模型和已归档的加速度数值补丁，不新增IK或轨迹优化器。测试专用stdin/stdout替代官方demo的轨迹动作发布者；无设备SDK/CAN/动作接口。20Hz直接API步长与现生产100Hz ROS节点有明确区别，完整来源、参数和测量边界见CPP_API_QUALIFICATION。该探针目前仅做无设备虚拟验证，尚未替换正式桥接。

### v178 一键问题包

实际复用Python标准库`zipfile`/`hashlib`、现有会话与片段记录，以及固定phosphobot的`components/ui/button`。`problem_reports.py`只负责有限文件选择、脱敏、独立写入和下载，`ProblemReport.tsx`装入原ControlStatus；没有新增通用压缩库、上传服务或控制协议。完整应用原页面实际离线导出/下载已验证，源码与构建SHA见problem-report-v178；上游固定提交和许可证未变。

### v203 片段封存隔离

继续复用Python标准库`zipfile`/`hashlib`和`subprocess.Popen`，仅将既有ZIP/SHA工作放入不导入设备模块的独立解释器。原phosphobot Recorder/片段检查、Audit、相机暂存和原文件校验接口保持；上游提交、许可证、运动与媒体算法未改。完整调用与归属退出证据见BUNDLE_ISOLATION_QUALIFICATION。

### v206 采集阶段与准备按钮

沿用同一固定phosphobot的`KeyboardControlPage`／`GamepadControlPage`及原`Button`，补丁0002只增加基于既有录制阶段的禁用条件；E6 hook提供原因并避免在重连期间排队准备。没有替换上游输入、后台规则或增加租约。七补丁在固定原文件干净应用，前端源码／构建和PC-only正式页面证据见RECORDING_PREPARATION_QUALIFICATION。

### v176 正式C++桥接

`scripts/servo_cpp_adapter.cpp`继续复用同一固定官方库及demo许可，通过`MoveItBridge(api='cpp') → jazzy_cpp_worker → Servo::getNextJointState`接到原phosphobot输入和网关。没有本地IK或新轨迹优化器，目标FK由同一官方RobotState计算。新增部分仅为原状态时间等待、stdin/stdout适配、时效/epoch/种子来源及归属清理；正式页36方向影子已实际运行，默认仍ROS。官方库未修改，现有平滑插件原补丁/SHA不变；新adapter源码/二进制/执行配置在cpp-api-v176和live-jazzy-v176保存。实机发送仍由固定Dobot SDK和原Stop路径承担，本轮运动发送为零。

### v229 历史记录恢复

仍运行固定phosphobot完整应用；补丁0008只在其ControlPage内选择记录恢复页面，复用现有EpisodeBrowser／Inspector及上游Dialog、Button、Input、SWR，不新增自制通用控制界面。PC恢复入口复用已有Paramiko媒体隧道、TransferWorker；X5复用episode_http与原子元数据函数，新增部分仅为已结束session的有界目录发现和归属退出。没有设备驱动或运动学改动。正式HTTP重新取回v228中止片段、校验和浏览证据见RECORDING_RECOVERY_QUALIFICATION。


### v266 碰撞包络视觉复核

完整源 CAD 显示、按机械 link 的嵌套盒、精确并集显示和验证调用见 [COLLISION_VISUAL_REVIEW.md](COLLISION_VISUAL_REVIEW.md)。新增实际复用 CadQuery 2.6.1（Apache-2.0）的 OpenCascade Boolean union，Trimesh.contains 通过 Rtree 1.4.1（MIT）做独立内外判定；版本固定于 scripts/requirements-workspace-cad.txt。显示仍完整 URDFViewer 0.13.1 / Three.js 0.186.1，正式适配源及原 prepare_upstream 复制链不变。

### v271 整臂接近减速与夹爪扫掠

仍固定 MoveIt 2.12.4 / `1ade0e9`（BSD-3-Clause），FCL / Eigen 沿用原 Jazzy 环境。`prepare_arm_proximity.py` 在独立源码副本中复现有限 `CollisionMonitor` 补丁：原 ACM 做实际碰撞，临时距离矩阵排除整臂关节无法改变间距的部件对；KDL、Servo 平滑和既有方向奇异补丁保留。

`DynamicGripper → SweepDistanceCertificate` 实际调用 MoveIt `LinkModel` 的几何包围盒、`core::AABB::extendWithTransformedBox`、Eigen `exteriorDistance` 与 FCL `distanceSelf(SINGLE)`。新增适配仅为祖先链的三角不等式位移上界、已检查开度区间和结果完整性校验；不能证明时仍原 FCL 密集扫掠，未编写新的 IK、碰撞引擎或轨迹优化器。正式前端只在已有合成验收区增加两个有限时长组合输入，仍走 phosphobot 实际页面和原控制连接。源码、固定二进制及失败／通过证据见 [ARM_PROXIMITY_FIX.md](ARM_PROXIMITY_FIX.md)。

后续同轮修正复用 MoveIt RobotState 的关节轴、碰撞 AABB 和 FK，计算包含整个区间的材料点速度／加速度上界；采用 FCL 宽窄阶段分离，仅裁剪已经证明在整个区间分离的部件对。`useE6Control` 保持同一输入监听生命周期，能力刷新不重装 keyup 监听器。没有新增 UI 库、控制连接或替代求解器。

### v278 旋转几何计算优化

继续使用固定MoveIt 2.12.4 / `1ade0e9dcf50dbbbc3a984b995c786cf12736235`（BSD-3-Clause）及原Jazzy FCL/Eigen，无新增依赖。调用路径为`servo_cpp_adapter → DynamicGripper::check → SweepDistanceCertificate::{distances,preparePairs,checkPairs}`：局部坐标下复用`core::AABB::extendWithTransformedBox`；剩余部件对使用`collision_detection::createCollisionGeometry`、`CollisionData::enableGroup`、FCL `CollisionObjectd/getAABB().overlap`和MoveIt `collisionCallback`。

只缓存不可变形状/BVH，不缓存位姿、碰撞结论或许可。区间部件列表在原scene只读锁内使用；未知开度、附加物、非默认缩放/填充等仍走原整场景检测。没有自行实现新的碰撞引擎、IK或平滑器；原模型、碰撞矩阵及采样密度不变。原矩阵对照316,618次密集检查、X5原生重放和保留的失败证据详[GEOMETRY_BUDGET_V278.md](GEOMETRY_BUDGET_V278.md)。

### v286 持续退离与直接输入（验证中）

仍固定 MoveIt 2.12.4 / `1ade0e9`（BSD-3-Clause）、原 Jazzy FCL 0.7 / Eigen 和已核对的 AccelerationLimitedPlugin。`prepare_collision_retreat.py` 将退离选择放在官方速度/关节检查与最终平滑候选之后，拒绝时用该插件的原 `resetSmoothing` 恢复已提交状态，保留原接近减速路径。没有另写 IK 或平滑器。

`CollisionRetreatPolicy → SweepDistanceCertificate` 复用 `fcl::BVHModel<OBBRSSd>::getBV(0)`、原 RSS `distance(R,T,...)` 下界及 `fcl::distance` 完整精确查询。前一查询的三角面索引通过 FCL `distancePreprocessOrientedNode` 在当前位姿重算搜索上界；原生 `MeshDistanceTraversalNodeOBBRSS` 继续执行完整遍历与最近点后处理。固定 7 个 C++ 标准工作线程加调用线程只负责独立采样点查询，按原顺序验证完整结果，不新增运动队列或执行器。变换约定和 warm/cold 对照见 [RETREAT_CONTINUITY_V286.md](RETREAT_CONTINUITY_V286.md)，未更换模型或碰撞引擎。

前端沿用固定 phosphobot 0.3.134 / `a5de051` 的输入组件与 E6 hook，补丁 0012 仅更新 E6 键盘参考；未改变上游非 E6 默认操作。没有新增 UI 依赖或第二个控制连接。

- v286 的逐连杆区间裁剪复用现有 `SweepDistanceCertificate::velocityTravel` 材料点速度及二阶导上界，将同一上界保留为逐连杆值。仅固定夹爪开度的整臂退离使用；两个部件的运动上界之和加原 10 mm 阈值小于其初始距离时，才免除该部件对的重复精确距离查询。FCL 几何、真实碰撞 ACM、全臂 1 mm 采样数、最终 42 ms 截止均不变。实现位于 `scripts/sweep_distance.hpp`、`scripts/collision_retreat.hpp`；独立验证位于 `scripts/probe_collision_retreat.cpp`，许可证沿用 BSD-3-Clause。

- v286 原生对象复用：初始化时由原 Shape 的等值 clone 建立私有 FCL BVH；距离直接调用 FCL 原生 `distance(geometry, transform, ...)` 重载，碰撞查询复制已初始化的 `CollisionObjectd` 后仅更新 transform / world AABB，并继续用 MoveIt `collisionCallback`。避开对象构造时重复 `computeLocalAABB()` 及共享缓存写入，未重写距离或碰撞算法，原生精度、几何、ACM 和余量不变。验证包括原 2165 组 / 316618 次密集矩阵对照。
