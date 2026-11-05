# v347 明确标识几何辅助线程的调度实验准备

2026-10-20。**只用于隔离对照，未改生产调度，未发送设备动作。**

v345/v346 的失败帧同时存在主线程和几何样本等待。此前 v335 仅设置主线程 FIFO40，辅助线程仍为 OTHER，未解决所有预算耗尽。此次准备 normal、main、geometry 三种模式：最后一种只设置已确认的原生主线程及七个计算线程，不修改 DDS、Python、相机或其他线程。

在 v344 诊断源码中，只为 `RetreatDistanceWorkers` 的七个工作线程设置明确名称 `e6-distance-0` 至 `e6-distance-6`，在工作循环开始前执行，不改变几何内容或截止。全部 63 文件及单文件命名补丁保存在 `output/geometry-scheduler-v347`。

RDK 隔离构建 job187908 / start_ticks7056344 / bootde68973d 已退出 0，binary `0cb3d54091f91ac1f659fb273f2346615ae3c8bfeb7b6d0f914264dc21348b57`。证据已取回并逐文件校验，CPU 策略恢复；生产 pin 和 `90fe4ad1…` 二进制未变。

`output/geometry_scheduler_probe_347.py` 核对已拥有子进程的 PID、启动 ticks、可执行路径以及每个被选 TID 的启动 ticks 和名称。在初始化后、活动输入前设置策略；三组均配置子进程 200 ms 连续实时 CPU 上限，退出前逐线程恢复并保留收据。初始化中部分设置失败也会恢复已修改线程。

精确包装类在 RDK 三个独立短命子进程验证，每个含七个命名计算线程和一个无关线程。normal 选择主线程但不改调度；main 仅主线程 FIFO40；geometry 主线程加七个计算线程 FIFO40；无关线程始终 OTHER。三组结束后全部九线程恢复 OTHER0。结果 `evidence/geometry-scheduler-v347/wrapper-check.json`，不是实际控制性能资格。

准备期间进一步定位到最终目标漏检（详见 `FULL_ARM_SWEEP_V348.md`）。因此不将 v347 旧逻辑的耗时成绩当交付条件，后续应在补齐全臂路径检查后做调度与完整链路对照。v348 诊断构建继承本轮命名与计时；包装类尚未用于正式运动。
