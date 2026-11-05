# 夹指抓握中心 TCP：实现与验证

2026-10-09，v230。范围是本次用户要求的控制点修改，完整平台 Goal 仍暂停。本轮实现已经进入正式 phosphobot 页面、MoveIt、网关、启动器和原始记录；新的旋转轨迹尚未做实机精度验收。

## 控制定义与几何

默认 `omnipicker-r2`：相对控制器零偏移 Tool0 法兰平移 `[0,0,0.1504] m`，四元数 xyzw 为 `[0,0,0,1]`。平移改变此点位置，旋转保持此点目标 XYZ；基座／工具选项改变输入轴方向。关节模式仍直接控制所选关节，不保证 TCP 固定。

这个点是夹指工作区域内的**固定名义抓握参考点**。官方 OmniPicker 两侧远端内夹持面的面积加权中心高度分别约 144.113／142.596 mm；平均 143.355 mm，加已确认的 R2 盘 7 mm，取到 0.1 mm 为 150.4 mm。横向采用对称轴，工具轴与法兰平行。可复现脚本 `teleop-platform/scripts/derive_grasp_tcp.py`，源文件路径、SHA 与计算结果见 `evidence/tcp-v230/nominal-geometry.json`。

OmniPicker 为自适应连杆夹指，接触中心会随开度和物体变化。当前没有经过标定的开度到夹指角映射，因此不动态追踪接触中心，不声称这是实物 TCP 标定。界面、能力与数据均带 `nominal_cad_unmeasured`；工具定义 SHA256 为 `2c7ab2531f9d51c82989b84314f7fdb800b2c165f0c4f121dafcf025b0c28b79`。

## 实际实现

- 在核对后的固定官方 E6 模型内存副本上添加 `grasp_tcp` 固定 link，MoveIt 的 me6_group chain tip 改为此点。复用官方 MoveIt 2.12.4 C++ Servo／KDL，不新增 IK 或局部拟合。
- 工具输入 frame 与 chain tip 一致，明确启用 `apply_twist_commands_about_ee_frame`。C++ 返回的 `actual_fk/target_fk` 仍是法兰，继续独立对照控制器 Tool0。
- 网关以同一变换生成实际／候选 TCP；同时保留 `flange_pose`，核对模型与网关的工具定义一致。准备不重复施加偏移，不移动已建立的工作区原点，TCP 与法兰两套范围检查都保留。
- 控制器 Tool0 保持原零偏移，不写控制器 TCP 参数。非零 TCP 的旧原生 ServoP 路线明确拒绝，避免显示变了而执行点没变。
- 正式键盘、手柄共用该控制点。页面标明名义中心，详情保留原法兰；片段浏览区分新 `grasp_tcp` 记录和旧法兰记录，旧记录不回写。
- 启动选项 `-TcpProfile omnipicker-r2|flange`／`--tcp-profile` 在会话启动时固定。本机默认 `omnipicker-r2`；本次实测求解入口明确使用 `-MoveItApi cpp`。

## 已证明

| 检查 | 结果与边界 |
|---|---|
| 后端回归 | 完整第二轮 651 passed／8 skipped；第一轮 364 passed／3 failed／1 skipped 的 XML 保留，3 项是 AST fixture 未补 `profile.tcp`，修复测试夹具后复测。后来增加轻量 launcher 无数值依赖用例，相关 10 项通过。 |
| 前端 | 128 项通过，正式 `prepare_upstream.py --build-dashboard` 构建、复制并核验实际服务目录。 |
| 官方求解器无设备验证 | base/tool × 平移/旋转 × 三轴正负共 24 组，800 个逐帧记录；旋转 2°/s、1.5 s，平移 4 mm/s、1.5 s。旧法兰旋转对照使抓握点漂移约 5.050 mm；新 TCP 旋转数值漂移最大 8.47e-8 mm。该极小值只是模型数值结果，不是实机精度。 |
| 正式网页影子 | 键盘基座 Rx+、工具 Ry+、合成手柄基座 Rz+，各 4 个来源一致的候选；TCP 每组候选间保持，法兰相应补偿约 0.369／0.394／0.140 mm。使用实际正式 mapper、无线链路与 X5 求解器。 |
| 新数据读回 | `20261009T145020Z_1ca1dcd6` 为 `hardware_shadow`，499 周期、3141 原始反馈、1024 原图／1032 文件、0 个实机动作；全部回传校验，RGB／深度周期关联失效均 0。仍禁止当作真实示教或导出动作训练数据。 |
| 设备与资源 | 影子会话 `20261009-224800-a2ba051d` 保存 54220 原始反馈，无审计丢失、运动／Stop／CAN 事件。正常退出，设备 socket、相机占用、proc 挂载释放，CAN DOWN，RX/TX 保持 196756。 |

完整证据在 `evidence/tcp-v230/`。`shadow-analysis.json` 与 `shadow-audit-selected.json` 保留逐候选来源及法兰补偿；分析脚本为 `teleop-platform/scripts/analyze_grasp_tcp_shadow.py`。影子记录内实际关节峰峰最大约 0.007554°，仅为该段读回，不作运动精度认证。

## 制品及更正

新 C++ 制品 `qualification/cpp-api-v176/build-tcp230/build/servo_cpp_adapter`：SHA256 `ae43d156addbe00076aeb1d1ca4b62d2cffd4d0920cdb78814dd31c7cb98fef4`。其 LF 归一源码 SHA 为 `9d708f95ebbaf884e0c9dc5734c1950dca38123c3c25a732d84a883900751975`，固定 MoveIt 上游提交 `1ade0e9dcf50dbbbc3a984b995c786cf12736235`。

首份部署 manifest 的 source_sha256 误继承旧值；影子正常退出后已更正并读回核对，制品字节未变。原错误 manifest、原回滚 manifest 和配置分别保留，不能把旧字段当作本轮源码证据。构建实际快照在 `evidence/cpp-api-v176/build-tcp230/source.json`。

## 使用和未验证项

正式启动示例：`Start-E6.ps1 -Stage shadow -MoveItApi cpp -TcpProfile omnipicker-r2 -PreviewCodec vp8`；实机入口显式 `-Stage live`。启动不自动使能、准备或初始化夹爪。正常退出后以 `-TcpProfile flange` 可回到旧法兰语义；不要在运动中切换。回滚制品及配置先按 README 的会话退出流程执行，原文件在 `evidence/tcp-v230/previous-*`，禁止覆盖历史证据。

本轮未自动运动、未发 CAN；不把软件实现或影子通过当作实物标定／新轨迹实机验收。新版旋转会产生必要的法兰补偿，仍遵守既有工位和线缆范围，未扩大测试空间。上次操作会话中的夹爪 state2／fault0 是独立未解决结果，本次未重试、不因 TCP 改动宣布夹爪正常。完整速度、停止和长期综合目标继续按原 STATUS 保留。

交接时新版live会话`20261009-230018-60e13b49`已启动，页面显示E6 live／夹指抓握中心TCP，键盘旋转0.1x，保持未准备。新回读静止、使能且无报警，RGB约20fps，夹爪未初始化。证据为`live-handoff-state.json`及`workbench-live.jpg`；这证明部署与观察，不是实机新旋转验收。
