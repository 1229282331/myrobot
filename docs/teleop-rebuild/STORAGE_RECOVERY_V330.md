# v330 存储压力处理与同代码负载复测

2026-10-20，开发中。当前候选 D 的相机负载软件链路仍未通过，不能据此称中高速实机控制已修好。

## 已确认的证据

根盘曾达到 99% 使用率、约 1.2 GB 可用，历史平台会话约 63 GB。内核采样实际观察到审计写入的 ext4 等待、jbd2/mmc 等待，以及 native 启动读取共享库时的等待。只能确认存在存储阻塞，尚不能将所有几何超时、调度延迟或 J5 现象归因于它。

对已退休会话的前 64 MiB 事件日志作有界抽样：22,041 条事件覆盖约 278.20 秒，事件文件约 241,264 bytes/s，其中 5,494 条 cycle 占约 62.92 MB。此数字不含原始 SDK 日志或视频，也不是所有运行阶段的平均值。证据为 `evidence/io-pressure-v330/journal-volume.json`。长期日志容量管理仍需纳入平台改进，单次归档不等于解决持续增长。

PC 独立实验将同一份 67,118,432 bytes 样本以标准 gzip level 1 保存为 8,814,712 bytes，约减少 86.9%，两轮逐字节解压哈希一致。两轮压缩 CPU 约 0.22–0.23 秒，仅为本机批处理结果，不能外推为 RDK 实时开销或上线资格。1 MiB 明文缓冲的批处理速度也更快，但可能延迟日志可见性；两种方法均未修改生产 Audit。证据为 `evidence/io-pressure-v330/compression-pc-experiment.json`。

## 无损归档

范围仅限两个已明确 clean 退出的会话：`20261018-144306-c02ecf61`、`20261017-191600-abb23d24` 的 `audit/raw.jsonl` 和 `audit/events.jsonl`。不处理 episodes，不更改历史失败结论。

每个文件先保留明文，RDK 压缩并校验解压 SHA256，PC 下载后验证压缩及解压 SHA256，再由 RDK 重验压缩件和原文。全部吻合后才移除重复明文；完整压缩件在 PC、RDK 各保留一份，原目录保留恢复索引。PC 证据目录为 `evidence/storage-archive-v330`，每文件 `.completion.json` 才是完成凭据。

四份均已完成，归档进程正常退出 0。原文总计 14,499,087,880 bytes，完整保存为 1,793,634,832 bytes 压缩件，减少重复磁盘占用 12,705,453,048 bytes；最后一次 RDK 可用空间为 13,826,404,352 bytes。第一会话原文 8,611,096,300 bytes 保留为 1,079,523,199 bytes 压缩件；第二会话原文 5,887,991,580 bytes 保留为 714,111,633 bytes 压缩件。两个恢复工具 `--check` 均返回 verified=2，未恢复明文占用空间；汇总见 `evidence/storage-archive-v330/summary.json`。

恢复入口（`--check` 只校验，不写回明文）：

```powershell
.\teleop-platform\.venv-launcher\Scripts\python.exe output/restore_journals_330.py 20261018-144306-c02ecf61 --check
.\teleop-platform\.venv-launcher\Scripts\python.exe output/restore_journals_330.py 20261017-191600-abb23d24 --check
```

省略 `--check` 时恢复原始文件字节及 mtime，需原会话不运行、取得独占锁且空间充足。归档过程中不启动控制 owner。

## 复测条件

`output/stage_storage_retest_330.py` 要求四个归档完成凭据，以及两组恢复校验各确认两个文件。158 个 Python 文件已与上一轮逐 SHA 比对一致。D native、模型、输入、37/42/50 ms 及 100/250 ms 截止不变。

同样运行 medium-joint、high-joint、medium-alt、high-alt、high-combined、high-tool 六组 36 秒输入，保留相机与只读 shadow 负载。真实 Servo/IPC/SDK 解析/最终写口参与，设备反馈及运动是合成的；该测试不连接物理运动写口，不证明机器人动力学。

完整判定同时覆盖暂停、solver barrier、退离 budget、跳父候选、过期写入、跟随停止、日志完整性和 native 退出。上一轮 medium-joint 清理超时缺 summary 仍记为未完成，其余五组一次候选截止暂停和一次退离 budget 仍保留为失败。不得用新结果覆盖旧证据。

被动 shadow `20261020-110525-0e406c77` 启动后，新状态未准备、静止、Mode5 Enable1、负载匹配，相机有效；已查看新 RGB 与 3D，视频约20fps。隔离 job 134528 在 `chain-loaded-after-storage-v330` 执行复测，launch 凭据在 `output/storage-retest-v330/chain-loaded/launch.json`。

## 复测结果：未通过

| 组 | 合成写入 | 暂停 | 备注 |
|---|---:|---:|---|
| medium-joint | 544 | 0 | 运动与退出通过 |
| high-joint | 537 | 1 | socket_candidate_deadline |
| medium-alt | 545 | 0 | 运动与退出通过 |
| high-alt | 541 | 0 | 退出回执中 human_logs.writer_alive=true，不算完整通过 |
| high-combined | 550 | 0 | 运动与退出通过 |
| high-tool | 未完整统计 | 未完整统计 | 420秒整轮任务总时限触发，缺少summary；worker自身exit0，不冒充整组通过 |

五份完整汇总共2,717次合成写入，无实际过期写入、跳父候选、跟随停止或退离budget。任务总时限是测试作业监护时间，与100ms反馈有效期、42ms控制回调截止不同。监护异常未产生exit.txt，保留原job.log和terminal.json的exit=null，不补写虚假的退出码。后续现场进程检查确认该隔离目录无残留进程。

168文件、50,507,342 bytes原始压缩证据包已核对双端SHA及每个成员SHA，目录`evidence/storage-retest-v330`。前后曲线为`storage-before-after.png`；图中缺失组、暂停、budget及日志线程退出问题均保留。完整结果`analysis.json`为passed=false，不能称清理空间已修复控制问题。期间一次新SSH握手8秒超时，随后同任务查询恢复；已有平台反馈、相机仍有效，未重启设备或重复启动测试。

high-joint的676号候选/epoch11：条件数25.877、碰撞倍率1、无退离预算耗尽；compute 4.195ms、callback23.574ms、worker roundtrip27.080ms。源反馈至worker接纳约33.862ms，计算完成至host收到约20.754ms，host收到至最终拒绝约23.927ms，累计约105.622ms，超过原100ms截止5.622ms。同窗口tick等锁35.351ms，tick自身wall32.691ms/CPU10.955ms，无GC事件。锁等待与计算存在重叠，不能把它们重复相加；也不能仅据此区分GIL、操作系统调度或测试记录器各自占比。原时序、候选与模型状态分别在`high-joint/failure-timing.json`、`rejected-candidate.json`、`rejected-model-state.json`。

测试器本身额外保留逐方法及JSON内存诊断，同时只读shadow与合成控制各有一个网关/模型进程。这是更重的压力组合，不能直接等同单正式平台的绝对时延；下一轮须核对测量开销，同时继续保留这轮失败。生产中的0.8x实机超时历史也仍然有效，不能用测试器开销解释掉全部实机问题。

结束前20次新回读均未准备、静止、Mode5 Enable1、无报警碰撞，命令ID17517未增，关节峰峰最大0.003434°，反馈年龄max8.227ms、相机有效。shadow经stopfile正常clean退出，camera/gateway均0、CPU schedutil/WiFi省电on恢复，PC app0/media1且cleanup错误空原样保留。当前无控制owner，未切生产native、未发实机运动、未使能/清错或改设备参数。下一物理动作仍须新SDK状态及新画面。候选尚未交付，Goal保持active。
