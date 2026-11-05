# v335 原生主线程调度对照

2026-10-20。目标是检验v334高速工具坐标暂停是否受线程调度影响，不预设其为唯一根因。生产90fe4ad1、模型3b4003e4、配置d44b3403和原截止均不变；没有设备运动。

## 对照范围

官方MoveIt ServoNode调用`realtime_tools::configure_sched_fifo`、默认优先级40；当前C++ API适配器没有该调用。RDK短命探针已证明自身可设置FIFO40并恢复普通策略，父进程和全局配置不变，证据在仓库根目录`evidence/native-scheduler-v335/capability.json`。

四组36秒全速工具坐标联合平移/旋转使用相同输入生成器、诊断binary fc76e68f、双审计、150ms合成反馈和33ms合成回包；顺序为OTHER、FIFO40、FIFO40、OTHER。保留相机和3D负载，独立网络namespace仅loopback，不连接真实机械臂执行口。

只在收到第一条非活动初始化回包后设置原生主线程，先核对已拥有子进程的PID、启动ticks和可执行路径。辅助计算、DDS、Python主机及相机线程不变，首尾记录所有原生线程策略。两变体均给测试原生进程设置200ms连续实时CPU上限，避免无界占用；没有改变设备控制截止。退出前验证策略并恢复OTHER0，然后走原清理流程。

精确包装类已用两个短命Linux子进程验证，每个含主线程及两辅助线程：normal和FIFO变体均通过，只改变预期主线程，恢复后三线程均OTHER0，子进程退出0；证据`wrapper-check.json`。这只是实验装置检查。

本地准备首轮遇Windows默认GBK解码错误，尚未启动实验；明确UTF-8后准备成功，保留工具错误记录。没有将失败算作测试通过。

## 运行入口与证据

当前只读shadow为`20261020-124151-dd27304a`，入口`output/shadow_native_scheduler_335.py`，exec92521，stopfile`output/shadow_native_scheduler_335.stop`。启动观察一次503，未重启；刷新后3D实时、RGB20fps，截图`evidence/native-scheduler-v335/shadow-page.png`。启动不沿用夹爪ready、不初始化、不Prepare。

独立job149248在`/root/e6-teleop-rebuild/jazzy-v32/rootfs/qualification/collision-snapshot-v321/native-scheduler-chain-v335`运行。以`output/native-scheduler-v335/chain/launch.json`和新`/proc`检查为准；工具超时不代表停止，不重复启动。

准备、包装类验证、启动、查询、收尾、取证与分析脚本分别为：

- `output/prepare_native_scheduler_335.py`
- `output/check_scheduler_worker_335.py`
- `output/stage_scheduler_chain_335.py`
- `output/poll_scheduler_chain_335.py`
- `output/finish_scheduler_shadow_335.py`
- `output/fetch_scheduler_chain_335.py`
- `output/summarize_native_scheduler_335.py`

## 完整结果

job149248已退出1。156文件、40,004,721字节证据包双端及逐成员SHA核对完成。2154次合成写入，整体未通过：

| 组别 | 写入 | 暂停 | 退离超预算后低倍率写入 | 回调中位/P95/最大 ms | 主线程非自愿切换 |
| --- | ---: | ---: | ---: | --- | ---: |
| normal-a | 542 | 0 | 0 | 10.216 / 23.902 / 40.884 | 620 |
| fifo-a | 539 | 0 | 0 | 9.776 / 29.009 / 39.049 | 49 |
| fifo-b | 544 | 0 | 1 | 9.724 / 12.746 / 41.960 | 35 |
| normal-b | 529 | 2 | 0 | 10.356 / 30.961 / 47.958 | 759 |

所有原生线程首尾策略验证通过，只有指定主线程在FIFO两组为优先级40，辅助线程保持OTHER；退出前全部恢复OTHER。四组worker/native及日志线程正常退出，无日志丢弃、写错、晚写、跳父点或跟随停止。

FIFO-b candidate567主线程CPU34.704ms、回调墙钟41.889ms（外层callback41.960ms），其中距离8.761ms、退离22.624ms、夹爪4.018ms。Servo与退离阶段主线程均无非自愿切换，仍在8个退离样本后耗尽原37ms预算，倍率留在0.004867。没有暂停不等于体验通过。退离含辅助线程工作，不能把墙钟与主线程CPU之差全算成抢占。

normal-b candidate575和628分别47.958ms及44.502ms回调后暂停。实时调度显著降低被抢占次数，但没有消除几何工作量导致的余量不足。本轮各组使用相同输入生成函数，而反馈和目标会随真实软件时序演变，姿态路径不完全相同；P95不能解释为严格同姿态算法加速。normal-a记录1次重缺页，但两次失败帧和FIFO超预算帧均无缺页，不能据此归因。

完整曲线`evidence/native-scheduler-v335/chain/scheduler-abba.png`已检查，三角标记退离预算事件、叉标记暂停，四图采用相同纵轴。完整原生请求/回包、策略证明及统计保存在同目录。插桩有开销，结果不能当作生产性能或实机动力学资格。

## 清理和下一步

shadow20261020-124151-dd27304a末20样本未准备、静止、录制idle后经stopfile正常退出0；camera/gateway退出0、CPU恢复schedutil/WiFi省电on，PC app0/media1、cleanup空原样保留。stage54360、shadow92521、取证74959和远端实验均已结束，没有RDK控制owner。没有设备运动、Enable/Disable/ClearError或夹爪写入，未切换生产调度。

继续检查能否将现有经过验证的距离查询加速复用于初始自距离阶段，以减少约9ms的距离计算。首先只在PC进行正式FCL与现有表面查询引擎的逐姿态数值/耗时对照；若转换几何表示造成差异，也必须保留并分析，不能直接替换原控制路径。入口`output/build_initial_distance_336.py`，证据`evidence/initial-distance-v336`，结果尚待。

旧281ms长停顿及完整中高速交付仍未结案，Goal active，自主抓杯取消。本轮调度设置不作为修复发布。
