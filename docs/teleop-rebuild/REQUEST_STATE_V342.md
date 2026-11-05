# v342 请求状态统一与同步等待消除

2026-10-20。PC 语义对照、RDK 隔离构建及原故障 ABBA 回放通过；带相机负载的合并链路 v343 已结束，因一次 Tool 退离预算耗尽整体未通过。候选未发布到实机运动会话，不表示剩余卡顿已经修复。

## 改动及边界

适配器已经持有每次请求的实际关节状态，碰撞检查也使用该状态，但工具坐标变换原先仍从 CurrentStateMonitor 读取，因此每次必须先发布 JointState、再等待 ROS 回传。现在 REQUEST_STATE 模式把同一份实际状态显式传给 Twist/Pose 的机器人模型坐标变换；目标种子仍单独保留。后台模式继续读取 CurrentStateMonitor，外部坐标继续走原 TF 查询。

初始化仍等待首份完整状态，随后构造原 Servo/柔化器；仅初始化后的每请求 DDS 回传不再作为坐标数据来源。原始反馈/输入有效期、模型、ACM、奇异处理、37/42/50 ms 和 100/250 ms 限制均不变。不能把这个改动解释为接受旧反馈或取消碰撞检查。

模型指针不一致或非有限关节值，在使用请求进行坐标变换前拒绝。变换计算使用自有 RobotState 缓存并刷新 FK，避免修改调用者状态。

## 首轮失败与修正

首版把后台状态也用 const 指针读取；MoveIt 的 const 变换访问不会自动刷新脏缓存。PC 对照在后台模式的工具方向首先发现约 0.006847 rad 输出差异；未进入板端或实机。原失败在 `evidence/request-state-v342/c`、`d`，旧候选三文件补丁与清单在 `output/request-state-v342/rejected-a`。

修正为自有可更新的 FK 缓存后，增加“关节数值已改、缓存仍是旧姿态”的覆盖；没有降低误差门槛。另两个早期配置失败 `a`/`b` 是 PC 测试 CMake 包含未使用且未安装的 control_msgs/planning_interface 依赖；探针改为实际需要的依赖，未更改 RDK 生产构建依赖。

## PC 证据

最终 `f` 两个进程分别回报 `apply_twist_about_ee=true/false`，防止仅按命令行标签推断模式生效。每组：

- 55 个故障附近、其他姿态与开度样本。
- 13,640 次完整 Servo 输出/状态/碰撞结果比较，数值最大差为 0。
- 3,242 个非零运动输出；旧算法对滞后 monitor 敏感的 1,456 个输出，证明夹具能识别错误状态来源。
- 770 个外部 TF 命令，以及 Base、法兰、抓握 TCP 坐标中的正负六轴、组合、Pose 与 JointJog。
- 新模式缺少实际状态、旧模式误传实际状态、非有限数值与不同 RobotModel 四类拒绝。
- 后台模式在新鲜及滞后 monitor 下保留原行为；请求模式在后台滞后或请求 FK 缓存未刷新时仍与正确基线一致。

该探针为隔离坐标语义而关闭柔化；不能冒充完整适配器/生产柔化性能验证。`pc-qualified.json` 绑定全部候选源码 SHA，原 `e` 结果另存，未覆盖。PC 所有测试进程已终止。

## 正式保存与板端进度

可审查候选保存于 `teleop-platform/native/request-state-v342`，保留上游 BSD 许可证、62 文件完整源清单、父版本清单、三文件补丁和探针。它没有改写运行 pin。v341 的状态发布源与可复现补丁也已存入 `evidence/state-observation-v341`。

两个补丁均在全新的独立目录中通过 `git apply --check` 和实际应用；应用后 native 的 62 文件与状态发布的 4 文件 SHA 全部与候选一致，见 `evidence/request-state-v342/patch-proof.json`。

新只读主机检查：E6 `192.168.1.4` 经 eth0、源地址 `.14`；RDK boot `de68973d-218f-46b0-84f5-ed09f8427efa`，原控制锁可用、无控制进程。此检查没有打开设备控制或反馈连接，不是新姿态证明。

板端 job `167813`、start_ticks `6278225` 已退出 0，无异常；目录 `/qualification/request-state-v342`，持原独占锁、隔离网络/挂载、单路编译、1,900,000 KiB 虚拟内存上限。实际候选为 `37e8fc284b1b35b537b36bbd889be1b2363b8d8f826a7bb37daea86d9d0b00c8`，1,615,368 字节。原件已取回并验证 SHA，见 `evidence/request-state-v342/build`。

130 条保留的故障及 reset 请求完成 ABBA 共 520 条回放，保留生产柔化及全部预算。基线 f88716de 与候选各 242 条活动结果，均无硬拒绝、无退离或夹爪预算耗尽。387 次同种子比较目标关节最大差为 0；3 次首次非活动初始化因预算排除，未算入有效比较或性能通过。证据为 `replay-b/replay-b-summary.json` 与 `target-equivalence-detail.json`。

活动帧的状态等待中位数由 0.275708 ms 降至 0.000208 ms；总回调中位数 9.603685 → 9.420060 ms，P95 29.853199 → 29.131262 ms，最大值 30.698869 → 30.280951 ms。计算部分中位数约 6.114 → 6.161 ms，没有得到几何算法加速。这里只消除了一个同步等待，不能解释为整体卡顿已解决。`replay-b/curves.png` 已生成并查看，统计采用常规中位数与百分位数，原始数据不变。构建及回放均正常结束并恢复 CPU 配置；生产原生 `90fe4ad1…`、config `d44b3403…`、模型 `3b4003e4…` 不变。

## v343 合并链路结果

以下启动记录保留；最终十组 5,546 合成发送，0 暂停/过期发送/跳父候选/跟随停止，但高速 Tool 有一次退离预算耗尽，整体 `passed=false`。全部原件取回并验证哈希，只读 owner 已正常退出。详细结果、曲线与后续 v344 逐线程诊断见 [INTEGRATED_CHAIN_V343.md](INTEGRATED_CHAIN_V343.md)。

v341 锁外状态渲染与本轮 native 已合并到隔离测试。只读 shadow 会话 `20261020-151316-ecf4adb9`（PC exec `17476`）使用生产 native，承载真实反馈、相机及 3D；不接受运动输入。首个 45 秒就绪检查遇到 503 超时，失败原件保留；未重启，同一 owner 再读已确认未准备、静止、Mode5 / Enable1 / Error0 / Collision0、命令 ID17517、模型及负载匹配。刷新现有页面后核对 3D 实时和 RGB 20 fps，页面为 shadow，Prepare 禁用。夹爪未初始化是只读会话的预期状态。

隔离测试 job `171383`、start_ticks `6421659`、boot `de68973d-218f-46b0-84f5-ed09f8427efa`，目录 `/qualification/integrated-chain-v343`，使用候选 37e8fc28 与绑定 SHA 的 v341 Python。十组覆盖中/高速关节、中/高速交替、高速 Base/Tool 组合、Base/Tool 姿态辅助及两组重复。真实 Servo/IPC/NativeExecutor 配合合成 150 ms 设备响应，不连接设备执行口；状态投影 20 Hz、完整状态观察 5 Hz及双审计保留。测试未完成，不能计作通过。

查询 `output/poll_integrated_chain_343.py`，完成后 `fetch_integrated_chain_343.py`；取得全量结果和曲线后用 `finish_integrated_shadow_343.py` 正常释放，等待 exec `17476` 的退出结果。不得因检查超时重启 owner。证据在 `evidence/integrated-chain-v343`，本轮没有物理运动、使能/下使能、清错或夹爪位置写入。

v340 两次失败没有保留完整原生 state_wait 分段，故目前只是发现并消除一个可避免的同步依赖，不能断言两次超时都由 DDS 等待导致。仍需 v341 状态发布改造的 RDK 负载测试、相机负载完整链路及中高速实机联合动作。原低速退离、Position assist、录制与手柄验收范围继续保留，Goal active，自主抓杯取消。
