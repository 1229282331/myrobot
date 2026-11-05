# v332 日志写入与完整链路测试修正

2026-10-20，开发中，尚未交付。上一轮 v331 相机负载超时、日志线程退出不完整以及 high-tool 缺少汇总的失败全部保留。生产 native、模型、设备配置和控制阈值均未切换；本轮无物理运动、使能、清错、负载或夹爪位置指令。

## 修改

`operator_logs.py` 原先每条可见日志都重新打开、关闭文件。现在由原后台写线程持有追加流，复用 Python 标准库 `WatchedFileHandler` 的 inode 检查及轮转算法。每次最多合并处理 64 条已经排队的消息，随后刷新；不等待凑批，不设延迟计时器，空闲时的单条消息仍立即可见。队列容量、丢弃计数、异常记录与原独立结构审计保持。

追加流本身不是 `logging.Handler`，避免注册到解释器的 `logging.shutdown` 后，在主线程同步等待被磁盘阻塞的流。流关闭仍由写线程执行；主线程的关闭等待仍有界，若写线程未退出仍如实报告，不能把它伪装成成功。

`jazzy_host.solver_actual` 提取原生产代码已有的反馈投影，生产调用路径和测试器共用。源反馈时间、连接代、关节和工具反馈不改。旧测试器直接发送整个 compact SDK 状态，与生产发送字段不同；新测试器按正式投影发送。这是测试负载对齐，不能凭此解释掉历史实机故障。逐方法、锁、GC 和 JSON 诊断仍保留，测试器仍比生产多做记录。

新测试器在独立 `cleanup.json` 保存每项资源释放结果，某一清理异常后继续释放其他资源并保存原错误；保存数据前先写 `terminal.json`，汇总完成后再更新完成标记。SIGINT/TERM 记为中断，不当成功。六组分别采用 130 秒作业监护，避免之前整轮 420 秒监护截断最后一组结果。该时限仅约束隔离测试作业，**37/42/50 ms 及 100/250 ms 控制截止均未改**。

## 本机检查

- 最终生产候选源完整后端：1180 passed、12 skipped、14 个上游弃用警告，见 `evidence/chain-timing-v332/backend-tests-final.txt`。
- Linux 日志、管道和测试收尾专项：17 passed，见 `linux-tests-final.txt`。之后补充网络隔离检查与进程监护共 8 项通过，见 `linux-supervisor-final.txt`。
- 首次专项发现延迟打开流后没有更新 inode，导致多打开一次；Windows 和 Linux 同样失败。补齐标准库 `_statstream` 后通过。首个完整后端 1179 passed / 12 skipped 是中间候选，不冒充最终源验证。
- 保存的 high-alt 原生日志做本机 ABBA 回放，每轮 405 个块，原版文件打开 405 次、候选 1 次；四轮内容均逐字节一致、无丢弃、无写入错误且写线程退出。原版墙钟约 294–304 ms，候选约 33–80 ms，仅为 PC 有界回放，不能外推为板端实时改善或实机资格。原文和旧源保存在 `evidence/chain-timing-v332/log-replay`。

## RDK 隔离测试

固定使用候选 D `15596374412f22cb2577b10bbaa1148e0494d1286f80ad69ea45081e641c86d2`、原模型及原六组 36 秒输入：中/高速关节，中/高速位移旋转交替，高速同时位移旋转，高速 Tool 坐标。完整 Servo、IPC、SDK 解析和最终写口参与，但设备反馈延迟 150 ms、回包延迟 33 ms 均为合成，网络 namespace 无外部连接，不证明机器人动力学或实机完成。

第一轮在启动前失败：RDK 的新网络 namespace 自动带有 `tunl0`、`sit0`，二者均 DOWN、无地址、无路由。原检查要求接口名称只能有 `lo`，误拒绝默认隧道模板，0 次合成写入。19 文件、49,279 bytes 证据包已逐文件及双端 SHA 验证，保留 `evidence/chain-timing-v332/idle`，结果为 passed=false。

修正后的检查仅接受启用的 loopback 和上述两个无地址、未启用的默认模板，IPv4/IPv6 路由均必须属于 lo；Ethernet、启用隧道、有地址隧道、外部路由及未启动 lo 均有拒绝测试。新尝试另用 `chain-timing-v332-b`，不覆盖首次失败。

## 无相机负载的六组结果：通过

第二尝试 job138947 正常退出 0，CPU 恢复 schedutil。六组分别完成 590、595、592、594、592、593 次合成发送，共 3556 次；无暂停、跟随停止、退离 budget、跳父候选、过期实际写入或清理失败。每组均有完整 summary、terminal、监护回执；六个 native/worker 日志无丢弃/错误且写线程已退出，私有共享内存挂载均匹配。

213 文件、57,198,538 bytes 原始证据包已核对双端 SHA 及每个成员 SHA，`evidence/chain-timing-v332/idle-b/analysis.json` 为 passed=true。发送间隔各组中位 55.10–56.07 ms、P95 59.29–59.73 ms、最大 69.50 ms；发送时源反馈年龄最大 58.24 ms。仍不是严格 50 ms 节拍，也不能将合成反馈的曲线当成机械臂速度或动力学证据。

![六组软件时序](../../evidence/chain-timing-v332/idle-b/timing-curves.png)

曲线已逐图查看。原始 trace、回执和 source manifest 随证据保存。下一步相机负载仍是必要门槛，不能用本轮无相机结果替代。

## 相机负载复测：未通过（v333 收尾）

只读 shadow 会话 `20261020-114859-c6900366`、PC exec50623，入口 `output/shadow_chain_timing_332.py`，stopfile `output/shadow_chain_timing_332.stop`。新回读未准备静止、Mode5 Enable1、无报警碰撞、命令 ID17517，负载匹配、相机和模型有效；这是只读阶段，夹爪没有初始化。页面重载后 RGB 约20fps、3D实时，画面保存 `evidence/chain-timing-v332/shadow-page.png`。启动早期一次503保留为启动过程，未因此重复启动。

`stage_chain_timing_loaded_332.py` 校验完整 idle-b 通过与同 Python 哈希后运行相同六组；实际相机和只读网关是附加负载，合成写口仍在独立网络 namespace 内。此时同时运行被动真实网关和隔离合成链路，比单套生产系统负载更重；不能把它当成同负载的实机性能，但也不能因此解释掉先前实机失败。

隔离 job140822 已结束，exit=1。六组全部具有完整汇总、监护、worker/native 和日志退出记录，共3272次合成写入；212文件、54,499,513 bytes 原始证据包的双端 SHA 和每个成员 SHA 均已核对。`evidence/chain-timing-v332/loaded/analysis.json` 为 passed=false。

| 组 | 合成写入 | 暂停 | 结果 |
|---|---:|---:|---|
| 中速关节 | 544 | 1 | solver_deadline |
| 高速关节 | 548 | 0 | 本组通过 |
| 中速位移/旋转交替 | 548 | 0 | 本组通过 |
| 高速位移/旋转交替 | 552 | 0 | 本组通过 |
| 高速同时位移/旋转 | 549 | 0 | 本组通过 |
| 高速 Tool | 531 | 1 | solver_callback_deadline |

第一处失败为 candidate110/epoch1：往返280.847593ms，callback278.455264ms，getNextJointState计算区间274.714896ms。这个计算字段是墙钟时间，不能等同于CPU耗时，尚不能区分具体计算、缺页、锁或调度等待。

第二处失败为 candidate616/epoch9：往返45.163255ms，callback42.216176ms，计算区间37.382393ms，超出原42ms回调截止。它与第一处约281ms停顿不是同一种已证明原因。

全六组均无过期实际写入、跳父候选、过短发送间隔、跟随停止、退离budget写入、清理失败或中断。worker/native全正常退出，原生和人类可读日志无丢弃/写错，所有日志线程均退出。本次修复的日志收尾通过，不能据此将整个控制测试改成通过。各组同epoch发送间隔P95约67.47–72.58ms，最大99.390868ms；接受写入时源反馈最大年龄79.268195ms。

完整曲线保存于 `evidence/chain-timing-v332/loaded/timing-curves.png`，已查看。曲线保留了全部发送间隔（包括计划内中立与换向间隔），用独立标记显示两次被拒求解；没有删掉失败来绘制平滑曲线。这些仍是合成反馈，不是实机速度。

`inspect_native_memory_333.py` 在最后一组尚运行时进行了单次只读快照，保存 `evidence/chain-timing-v332/native-memory.json`。被动生产和隔离候选原生进程均 VmLck=0、VmSwap=0、RSS约84MB；宿主Python的memory-residency配置不能证明原生子进程也已锁定内存。此快照不构成缺页就是停顿根因的证明；本轮没有改内存或调度设置。

确认job结束后，`finish_timing_shadow_332.py` 保存20个新状态：全部未准备、静止、录制idle、Mode5 Enable1、无报警碰撞，命令ID17517不变。之后建立正常stopfile，exec50623实际退出0，shadow结果clean=true，camera/gateway均0，CPU schedutil与WiFi省电on恢复，PC app0/media1且cleanup错误空原样保留。目前无平台owner；没有新使能、下使能、清错或设备运动。最后姿态与画面不可当作下一次动作的新状态。

## 尚未证明

当前需要定位上述两种剩余超时，修正后重做相机负载，再完成正式网页中高速持续控制、工具坐标、位移旋转换向、J5 与官方 SDK 对照。下一诊断应保留失败原生完整reply并区分CPU与墙钟、缺页及等待；不要直接将所有超时归因于FCL，也不要把VmLck=0当成既定根因。生产native90fe4ad1、模型3b4003e4、配置d44b3403及原控制阈值未变；D15596374仍未live发布。原速度波纹、候选超时和实机验收缺项继续保留。Goal active，自主抓杯不恢复，未Git提交。
