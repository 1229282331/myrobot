# v334 原生求解停顿分段诊断

2026-10-20，开发中。上一轮 v333 已完成相机负载失败结果取证、曲线和正常释放，属于有证据进展；没有通过整体验收。本轮仍沿完整遥操作目标继续，不恢复自主抓杯。

## 尚待区分的问题

v332 相机负载六组3272次合成写入，出现两次暂停：

- 中速关节：native计算区间274.715ms、callback278.455ms、往返280.848ms。
- 高速工具坐标：native计算区间37.382ms、callback42.216ms、往返45.163ms。

旧 `compute_us` 是墙钟时间；不能直接称为CPU计算时间。失败回包中的一部分几何细节也未进入原 `CppCycle` 拒绝摘要。原生进程 `VmLck=0`、`VmSwap=0` 的快照说明宿主Python锁内存设置没有覆盖原生进程，但不是缺页根因证明。

## 本轮诊断改动

在 `output/native-timing-v334/source` 单独复制并逐文件核对 D15596374 的源树。新增原生19类阶段计时，记录：

- 单调墙钟和调用线程CPU时间；
- 轻/重缺页；
- 自愿/非自愿线程切换；
- 阶段起止时间及累计调用次数。

覆盖反馈发布、状态等待、Servo创建、求解器、参数更新、命令投影、IK、奇异处理、平滑、碰撞检查、场景读锁等待、世界/自身碰撞、距离、退离及其锁等待、夹爪几何检查。分段存在嵌套，不得把它们相加；退离内部并行线程CPU不计入调用线程CPU，墙钟与CPU的差值也不能直接等同于被抢占时间。

独立worker从当前正式 `jazzy_cpp_worker.py` 生成，仅在隔离副本中保存完整请求/回包，保留现有异步日志和有界资源释放；不拿旧同步日志worker替换新版本。记录保存在有上限的内存字符串中，退出后落盘。插桩有额外开销，结果用于定位，不能当作未插桩版本性能资格。

控制模型、ACM、负载/TCP、37/42/50ms及100/250ms截止不改。没有生产切换和实机运动。

## 检查与作业

WSL两编译单元验证通过：约25ms主动等待只消耗约0.016ms线程CPU、约15ms计算消耗约15ms线程CPU，8MiB触页记录2049次轻缺页，跨编译单元共享计数正确。原始输出和返回码保存在 `evidence/native-timing-v334/profile-check`。这只是诊断量具检查。

RDK在新目录 `/qualification/native-timing-v334` 单路构建，独占原会话锁，网络namespace无外部设备连接，私有tmpfs、1900000KiB编译虚拟内存上限。生产发布目录及pin不变。`output/native-timing-v334/launch.json` 为作业入口；须重新查 `/proc` 与terminal，不因轮询超时重启。

构建查询 `output/poll_native_timing_334.py`；成功后 `fetch_native_timing_build_334.py` 验证生产pin未变化并逐文件取回构建证据。后续已经准备两轮中速关节/高速工具坐标的相机负载诊断，共四组36秒，仍为独立合成写口。

构建已完成，job143345退出0，binary `fc76e68f4ae0124e71bf739bae2b3ad2e41197142b0c9902bfc9c470c554f92a`，1,608,512 bytes。构建证据逐文件双端SHA核对，生产pin和发布二进制无变化，CPU策略已恢复。

两处失败姿态的新旧ABBA共48请求（其中24活动）通过，目标最大差4.44e-16rad，所有活动status0；回调最大中速18.903ms、高速Tool30.119ms。没有复现旧281ms停顿，不称已修复。ARM同一量具验证也通过：25.078ms等待只用0.020msCPU，15.094ms计算用15.008msCPU，8MiB触页记录2049次轻缺页；证据 `evidence/native-timing-v334/smoke`。

新只读shadow `20261020-122244-3f851dfc`、exec46762，入口 `output/shadow_native_timing_334.py`，stopfile `output/shadow_native_timing_334.stop`，最长1200秒。启动期间两次503和旧页面离线画面保留为启动现象，未因此重启会话。新回读未准备、静止、Mode5 Enable1、无报警碰撞、ID17517，模型和相机有效。页面header恢复连接后主体仍保持旧离线状态，主动reload才恢复3D和20fpsRGB；此观察不能称自动恢复已通过，后续应区分只读重连与重新准备输入。截图 `shadow-page.png`。

四组36秒带相机诊断已完成，job146408退出1；不是通过。152文件、36,903,120字节证据包已双端及逐成员SHA核对，实际证据位于仓库根目录 `evidence/native-timing-v334/chain`。

| 组别 | 合成写入 | 非计划暂停 |
| --- | ---: | ---: |
| medium-joint-a | 551 | 0 |
| high-tool-a | 532 | 1 |
| medium-joint-b | 550 | 0 |
| high-tool-b | 548 | 0 |
| 合计 | 2181 | 1 |

高速Tool组candidate560/epoch9的callback为42.075ms、往返45.548ms。插桩callback墙钟42.014ms、调用线程CPU31.142ms，缺页计数均0、主动切换2次、非自愿切换12次。Servo墙钟35.836ms/调用线程CPU26.406ms；自身距离查询10.152/9.299ms；退离20.336/12.128ms；夹爪4.838/3.787ms。场景读锁和退离锁仅4.917及5.5微秒。本次不支持“场景锁竞争或缺页造成暂停”的解释；阶段嵌套，不能相加，辅助线程CPU未计入，不能把全部墙钟差归因调度。

这次退离本身已完整验证六个样本、从倍率0.003558提升到1，最小距离从1.837mm增加至4.488mm，行程5.568mm且退离预算未耗尽。后续夹爪检查遇到总回调截止后停止，说明正确退离仍可能因计算截止而卡顿。另一高速Tool组最大回调41.908ms，余量也不足。上轮约281ms长停顿未复现，原因仍未解决。插桩带来额外开销，本轮是定位证据，不等于未插桩生产版本的性能资格。

shadow20261020-122244-3f851dfc已取末20样本未准备、静止、录制idle、Mode5 Enable1无报警碰撞且ID17517不变，再经stopfile正常退出0。camera/gateway均退出0、CPU恢复schedutil/WiFi省电on，PC app0/media1、cleanup空原样保留。构建、短测、链路诊断及shadow均已结束，没有控制owner。未发本轮设备运动、Enable/Disable/ClearError或夹爪命令。

## 后续调度对照准备

2026-10-20新核对：官方MoveIt ServoNode调用`realtime_tools::configure_sched_fifo`，默认优先级40；当前直接使用Servo C++ API的适配器没有对应调用。该差异是待验证假设，不能直接认定为抖动根因。

独立短生命探针 `output/probe_native_scheduler_335.py` 已完成：RDK直连路由仍为192.168.1.4经eth0/src192.168.1.14，无机器人TCP连接、无gateway/camera/worker/native进程；探针只将自身从OTHER0切到FIFO40后恢复OTHER0并退出，父进程及全局配置不变，保留1秒CPU及50ms连续实时CPU资源上限。设备实时配额仍950000/1000000微秒。证据 `evidence/native-scheduler-v335/capability.json`。仅证明能力可用，尚未执行正常/FIFO负载ABBA对照，也未修改生产调度策略。

生产仍90fe4ad1，D15596374和诊断fc76e68f均未live发布。Goal active，完整中高速、正式网页及SDK对照验收仍待完成；不能称高可用系统已经交付。
