# v327 原生求解器日志阻塞修复（未发布）

## 已确定的阻塞来源

用户将 E6 重新直连 RDK 后，路由仍为 `192.168.1.4 dev eth0 src 192.168.1.14`。v326 静态反馈与相机检查正常；这不等于中高速运动通过。

在无物理设备 IO、无 ptrace 的完整 Servo/IPC/写口合成链路中，`chain-sched` 请求 1576 往返 73.642 ms、callback 70.679 ms。约 11 ms 间隔的五个连续内核采样都显示原生主线程处于 D 状态，停在 `do_get_write_access`；AArch64 syscall 64 为 `write`，参数为 fd 3、4096 字节。这段没有 CPU tick 或缺页增长。由此确定该次超时包含文件写入等待，不能解释为纯几何计算。

只关闭 ROS 外部文件日志的对照 `chain-logging` 仍失败：请求 1629 等待原生回复超时。连续三秒全部 260 次采样显示 `wait_transaction_locked`、D 状态，syscall 64 写 fd 2、112 字节。启动时 `/proc/PID/fd` 已将 fd 2 确认为该会话 `worker/servo.log`。这次没有调试器附加；不是网络或机械臂响应等待。该轮主测试汇总 `errors=[]` 并不能代表子进程成功，worker `exit.json` 明确记录 `Official solver response missing`；资格检查现在必须检查子进程退出详情。

曲线：`evidence/native-log-v327/native-file-io-stalls.png`，原件：`evidence/native-samples-v326/chain-sched`、`chain-logging`。均是合成设备反馈，不能当作实机曲线。更早 strace 记录的 247.849 ms 和 13.914 s 漏计时段还没有逐一归因，本轮发现不自动解释全部历史抖动。

## 修复

正式源文件 `operator_logs.py` 新增有界 `NativeLogDrain`，从原生 stderr 私有管道持续读最多 4096 字节，UTF-8 增量解码，不等待完整长行；交给既有 AsyncLog 后台落盘。`jazzy_cpp_worker.py` 改为 stderr 管道，同时通过 ROS 标准参数 `--disable-external-lib-logs` 关闭重复外部文件写入。console 原文、状态和错误继续写入原 `servo.log`，原逐周期结构审计、控制 IPC、所有运动截止与碰撞规则保持。

人类可读日志仍采用既有有界 best-effort 语义；队列满时显式统计丢弃块/字节，写入失败也计数，不能伪称日志完整。正式资格要求没有丢弃、写入错误或残留 drain/writer。新增磁盘阻塞测试证明 2 MiB stderr 输出不等待阻塞的文件写线程，同时确认有界队列和丢弃指标；UTF-8 碎片/无换行尾部保留、磁盘异常报告亦覆盖。专项 10 项通过。

本地改前源字节保存在 `output/native-log-v327/baseline` 及 SHA 清单。尚未发布 RDK 生产源；生产 native C `90fe4ad1…`、配置 `d44b3403…`、模型 `3b4003e4…` 不变。诊断使用 v326 原生候选 `830d7f3b…`，仅用于同二进制日志通道对照；该几何候选此前未通过，不因日志改动自动获得发布资格。

## 验证进度与范围

`chain-async-v326` 已完成同 90 秒中速交替测试：1477 写、0 流暂停/跟随停止/求解拒绝/跳父点/晚写/退离 budget；1755 请求及回复均对应，活跃时没有超过 50 ms 的往返。4 次 released_or_expired_input 保留。两个 audit 各 12797 原帧/4534 事件，计数一致。原生日志 120572 字节，EOF 正常、0 丢弃/写入错误/残留日志线程，原生退出 0；CPU 策略恢复。内核采样未再观察到原生主线程等待文件写入。这是首轮隔离通过，不能代替相机负载、实机和更长时间验证。

完整后端测试最初因选用环境缺依赖在收集阶段失败，原输出保留。最终使用项目正式 phosphobot `.venv`：1165 passed、8 skipped。专项 Linux 管道/阻塞文件验证亦已通过。未增加 RDK 生产依赖。四组中高速与 Base/Tool 组合链路改用 v325 `edd2fe16…` 原生候选和未插桩的正式 Python 修复，避免将 v326 已失败的几何调度推向实机；资格尚待完成。

未发本轮机械臂运动、Enable/Disable、清错、负载或夹爪位置命令。当前无平台控制 owner。未完成相机负载、完整中高速组合链路和正式实机复测；不称 J5 根因全部结案或系统交付。Goal active，自主抓杯取消，未 Git 提交。

## 四组最终结果：仍未通过整体资格

四组共 1973 次合成发送：中速交替 497、高速交替 498、高速 Base 同时位移旋转 497 均无暂停/拒绝/跳点/晚发；高速 Tool 481 写仍有 3 次 callback 拒绝，分别 42.254, 42.225, 42.439 ms。四个 worker 均正常退出、原生日志 0 丢弃/写错误/残留线程，两个 audit 计数分别与各组记录一致。日志隔离改善与几何候选仍失败必须区分，整体资格 false，未发布。原件 118 文件已 PC/RDK tar/SHA 双校验，归档仅移除 16 份重复 jsonl 明文。

10 个有 PC/RDK clean 退出证明的 2026-10-18 旧会话中，560 份重复大模型资产 1901542040 字节以 16 个内容哈希 gzip 双端保存并验证后转归档；代码及故障审计保留。恢复入口 `output/restore_retired_assets_327.py SESSION --check`，再去掉 --check 恢复；远端逐根目录 ARCHIVED-WORKSPACE-v327.json 有映射。完成后余量 2520014848 字节。隔离作业均结束、CPU 恢复，当前无平台 owner；下一步为剩余双段几何证明优化，不沿用旧姿态自动动作。
