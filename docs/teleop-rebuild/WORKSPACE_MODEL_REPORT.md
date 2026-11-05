# E6 实时 3D 与模型工作空间 · v261

> 2026-10-15 后续覆盖：v261 会话已 clean 退出，当前为 v262 `20261015-020622-223f5dc2`，模型 `a4e53045…346ba1`，入口 `output/collision_operator_262.py operator`，退出文件 `output/collision_operator_262_operator.stop`。v262 已替换原大盒，但用户要求从前端复核后确认仍有显著几何过包，贴合精度未合格；详 [COLLISION_MODEL_REFINEMENT.md](COLLISION_MODEL_REFINEMENT.md)。本报告下文的 v261 会话和几何数字保留为历史，不能当当前状态。v262 没有新增实机运动验收，完整 Goal 仍暂停。

2026-10-15。本阶段已发布实时 3D、装配碰撞模型和 `model` 工作空间，并完成局部实机回归。完整 Goal 仍暂停；物理仿真留在后续阶段。本文的模型检查与局部实测不能代替全空间实物验证。

## 使用与当前会话

打开 <http://127.0.0.1:8765/control?tab=keyboard>。左 Workspace、右 Gemini E，等宽等高；内容宽度不足 720 px 时上下排列。鼠标拖动旋转视角、滚轮缩放；按钮提供复位、展开、暂停 3D、采样点／碰撞几何开关。3D 不发送运动命令，不创建第二个控制连接。

正式 operator：`20261015-001856-20b439fa`，入口 `output/workspace_operator_261.py operator`，启动 exec 10586；正常退出文件 `output/workspace_operator_261_operator.stop`。发布回读为未准备、静止、Mode5/Enable1、无报警碰撞、录制 idle、相机有效、`synthetic_keys=false`。页面键盘／平移／基座／0.1x。未初始化夹爪，开度未知；没有重新使能、清错、负载写入或 CAN 初始化。

独立启动入口（先正常退出当前会话）：

```powershell
.\teleop-platform\Start-E6.ps1 -Stage live -MotionProfile normal -WorkspaceMode model -MoveItApi cpp -TcpProfile omnipicker-r4 -PreviewCodec vp8
```

需密码时加 `-Password`；不传 `-Stage live` 默认 shadow。启动不自动 Prepare。`model` 自动选择 C++ 求解入口，显式参数优先。原普通速度上限 30 mm/s、10°/s、精细 25%、tau=0.2 s 起步滤波、关节规划加速度 30°/s²、立即 Stop、ServoJ t=0.1 s/20 Hz、用户指定负载 0.750 kg/COM(0,−14,46) mm 与 TCP151.4 mm 均保持。

## 范围语义与统一装配

`model` 移除相对准备位置的 50 mm 球；实时许可仍由原 MoveIt Servo／KDL、官方关节范围每侧 5°余量、奇异处理和附件碰撞检查共同决定。平移保持目标姿态，旋转保持抓握 TCP 目标位置。450 mm 不作为新球形限制，采样点也不参与运动许可。`local` 保留原固定中心 50 mm 行为，旧配置无 mode 字段仍解释为 local。

模型 ID：`af50f9556077b0814732ed285964f8045e54c7126d2539c26e3204f1fce1f1b5`。控制、显示和采样引用同一清单及几何；Prepare 核对控制模型 ID，显示不一致时不绑定关节反馈。原始反馈、Stop、清理独立于显示。

装配采用已核对的 E6 串联模型，R4 STEP 米制转换和绕法兰 Z 轴 180°安装，夹爪 z=8 mm，相机中心 (0,−60,20.55) mm、外壳 89.82×25.1×25.1 mm，抓握点沿法兰 +Z 151.4 mm。凸包先对网格顶点按 10 µm 量化，再加 100 µm 数值余量，并核对原顶点包含性；显示网格先焊接重复顶点再减面，避免孔洞。仅新增 R4–Link6、R4–夹爪、R4–相机、夹爪–相机装配接触豁免；附件与肩肘前臂没有整组豁免。

夹爪采用 **70.5×155×174.408 mm 名义包络**：依据厂家端点外形、官方网格、链长高度界及额外余量；宽度比 125 mm 开口外形多 30 mm。厂家当前资料与用户 360 g 实物的机械一致性、被动指节全行程和受力自适应变形没有实物标定。用户已确认没有更多资料并接受按现有资料推进。它是工程近似，不能称为严格认证的全行程外包络。显示开度映射没有用于碰撞或伪造关节反馈。

**桌面、物体、人员和线缆未建模**。页面固定显示 `Environment not modeled` 和名义附件标记。反馈过期冻结，关节显示不裁剪成模型限位内的假值。

## 实际开源复用

| 来源 | 固定版本／许可 | 实际调用与本地差异 |
|---|---|---|
| [phosphobot](https://github.com/phospho-app/phosphobot/tree/a5de051197c879e3c685d1362f649ce54ee47a3c) | 0.3.134 / a5de051 / MIT | 完整官方页面、Card、Button、Badge、主题、输入与相机卡片；正式扩展在 `teleop-platform/frontend`，11 个补丁可复现 |
| [URDFViewer](https://github.com/gkjohnson/urdf-loaders/tree/24feef2c267496d22d17bbbb5c1bf5b86dcf174f) | urdf-loader 0.13.1 / 24feef2 / Apache-2.0 | 完整 `frontend/vendor/urdf-viewer-element.js` 负责场景、加载和 OrbitControls；`workspaceRenderer.ts` 仅绑定项目数据。有限补丁处理导入、加载失败、资源释放、像素比例和 Three 兼容 |
| [Three.js](https://github.com/mrdoob/three.js) | 0.186.1 / MIT | 原 viewer 使用 WebGL、相机、光照；项目增加 TCP／坐标轴及 Points。精确 npm lock 在补丁 0010 |
| MoveIt／KDL／FCL | MoveIt 2.12.4、KDL 1.5.1、FCL 0.7.0 | 保留现有正式 Servo 二进制及方向奇异退回补丁；PC WSL 独立使用 RobotState FK 与 FCL 碰撞。没有重写 IK 或通用轨迹规划 |
| 模型／几何 | 官方 E6 STL、OmniPicker xacro/STL、用户 R4 STEP、Gemini E 尺寸 | 原来源逐文件 SHA 在 `assets/workspace/manifest.json`；成熟 CAD／网格工具转换。未把名义装配称为实物扫描 |

版本与调用路径：`teleop-platform/upstream.lock.json`。Viewer 原源码 SHA 与有限补丁见 `patches/viewer/source.json`、`0001-lifecycle.patch`；`scripts/prepare_workspace_viewer.py` 复现。页面依赖和双视图修改为 `0010-workspace-renderer-dependencies.patch`、`0011-workspace-side-by-side.patch`。未新增 UI 主题、仿真器或设备控制接口。

## 验证结果

证据根目录为 `evidence/workspace-v261`，保留失败尝试，不覆盖旧证据。

| 检查 | 结果与范围 |
|---|---|
| 后端 | 正确固定环境全量 779 passed / 8 skipped；最终资产相关 35 项、启动器最终 29 项通过。错误 Python 环境的失败输出保留 |
| 前端 | 全量 159 项通过；反馈超时修改后的相关 98 项、暂停／恢复后的 Workspace 6 项通过；正式构建和全部 11 补丁复现通过 |
| 点集 | 固定 Sobol seed261，65,537 姿态，36,796 无模型碰撞姿态；10 mm 体素去重后 35,628 点，每点保存关节见证值。全量重新 FK/FCL，0 碰撞、FK 误差 0；缓存包含装配、TCP、关节范围余量和几何哈希 |
| 离线正式求解器 | 24 个基座／工具、正负三轴平移／旋转用例、释放重新输入、多轴；6 条较长平移中 3 条连续超过 65 mm。1,447 帧；当前模型独立检查实际／中点／候选共 4,341 次，0 模型碰撞，FK 最大差 3.53e−16 m |
| UI | 浅／暗、1280×720、1920×1080、窄屏堆叠、无横向溢出；真实／离线／过期／无相机／无手柄、加载失败、模型不符、图层、展开、卸载和输入隔离。原生 125% 快捷键未生效，仅检查等效 CSS 视口，不冒称原生缩放通过 |
| 性能 | 各 8 秒中立输入对照：3D 开／关转发 P95 0.174/0.251 ms，X5 回调 P95 0.851/0.941 ms，模型往返 P95 2.249/2.276 ms；相机有效、约20 fps。这是短时中立对照，不是主动运动延迟或长期负载认证 |

点集标为“模型可达范围采样”；未采到区域不能当作不可达，不填凸包。离线 Servo 首次写汇总遇到 numpy.bool_ 序列化错误，原始试验完整；修复后从完整原记录重算，未重跑实机。后续修补显示网格只改变视觉资产，`model-equivalence.json` 证明控制几何／运动学未变，并已用最终模型复核所有见证值和候选。

### 局部实机

正式网页→输入映射→PC无线→X5→原ServoJ链路。测试会话 `20261015-000023-d1c6492f`：键盘基座 Z±各0.5秒（请求3 mm/s），合成手柄基座 Rz±各0.5秒（请求1°/s）。不是实体手柄验收，也不是普通满速测试。

- 40 条 ServoJ／40 条确认，4 次 Stop 全确认；40 个候选来源和柔化状态核对无错误、无缺失父节点。
- 100,114 原SDK帧全部有效、无序号缺失、无报警碰撞，全部 Enable1。
- 全段 TCP 距原已确认中心最多45.134 mm；各关节峰峰最大0.240°。自动测试未扩大既有局部包络，没有通过重新 Prepare 重设原测试中心。
- 释放接收至 Stop 发出：1.098、1.024、0.992、1.559 ms。
- Stop 至250 ms严格静止窗口（各关节峰峰≤0.01°且空队列）：183.746、198.638、455.708、728.692 ms。Stop回包643–697 ms单独记录，不与发出或静止时刻混为一谈。
- 第一次尝试因相机滚出可见区域后浏览器暂停视频，`preview_stale` 阻止运动。保留该失败；没有放宽相机健康保护。随后让视频和测试控件同时可见完成四段。
- 测试后网页 Stop、恢复实体输入，未准备／静止／录制idle，再正常退出；所有自有控制子进程退出，CPU governor恢复schedutil。MediaMTX自有回收exit1按既有记录保留，整体clean=true。

原记录与重算：`live/raw-full.jsonl.gz`、`events-full.jsonl.gz`、`feedback.npz`、`analysis.json`、`files-sha256.json`；所有原始数据仍保留在该X5会话内。测试脚本从只读原记录生成报告，无补发运动。

## 限制与后续

扩大范围只做模型检查，未自动实走。全范围不同姿态的碰撞硬停与所有退回方向没有穷举认证；沿用正式求解器的可行性判定，不保证任意方向可离开任意边界。夹爪名义包络、TCP、相机安装仍非实物标定。历史 J5 跟随及失能位移问题没有因本次模型功能而被证明修复。

当前浏览器可能暂停完全滚出可见区域的视频，进而使现有预览健康检查暂挂控制；滚回新画面后需显式 Prepare。3D 增加的高度不能被误读为允许脱离真实视频继续盲控；这是已记录的使用限制。

后续 Simulation 独立接入，复用装配、坐标和上层接口，在执行适配处区分真机与模拟；本阶段未安装仿真器。完整平台耐久、满速、研究任务和全空间实物资格不在此报告中宣称通过。

## 回滚与复现

1. 松开输入、Stop，确认静止、录制结束；正常退出自有启动器，确认clean。不要强杀设备所有者。
2. 仅回到旧50 mm范围：下次启动加 `-WorkspaceMode local -MoveItApi cpp`，其它速度、TCP参数保持。
3. 连同旧UI／配置回滚：运行 `python teleop-platform/scripts/rollback_workspace_v261.py --check` 后运行同一命令去掉 `--check`。脚本仅使用已存SHA白名单，要求本地服务端口已释放，先另存当前构建／配置，再复制旧文件；不删除文件、不触碰设备。
4. 旧构建在 `output/workspace-v261/baseline`。重新发布新版UI用 `python teleop-platform/scripts/prepare_upstream.py --build-dashboard`；模型配置改回 `test_workspace.mode=model` 或启动显式 `-WorkspaceMode model`。
5. 模型生成 `scripts/build_workspace_assets.py` 使用 `.venv-cad`；PC WSL构建 `scripts/build_workspace_probe.sh`，采样 `sample_workspace.py --power 16`，全见证复核 `verify_workspace_map.py`。变更装配或几何后必须重新生成和核验，不能复用不同哈希的缓存。

所有修改在工作区和可复现补丁中；本轮未创建Git提交。
