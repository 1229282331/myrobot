# V357 方向受限时保留控制会话

2026-10-20。用户要求降低限制触发后的恢复负担。已部署四个 Python 文件；没有改变原生求解器、模型、几何余量或设备参数。

## 行为

| 情况 | 当前动作 | Prepare / 录制 | 恢复方式 |
| --- | --- | --- | --- |
| 求解器方向拒绝、几何拒绝屏障、零运动候选 | 丢弃候选，必要时 Stop 一次 | 保留 | 新鲜的反向、其他方向、回中或夹爪单独输入 |
| 计算/传递断档、候选过期、未提交父点、夹爪路径已变化 | 丢弃该代候选，必要时 Stop | 保留 | 同上，重新求解并验证 |
| 尚未发送的关节目标不连续 | 拒绝该点，不写设备 | 保留 | 同上 |
| 输入过期 / 控制策略切换 | 停止旧输入 | 原有行为保留 | 必须收到暂停后新鲜中立输入 |
| 真实跟随/停滞异常、设备报警、反馈失联、模型失败、Stop 失败或不确定 | 停止 | 保留原硬停处理 | 检查后明确恢复 |
| 显式 Stop、手柄断开、控制权丢失 | 停止 | 取消 Prepare | 明确恢复 |

持续按住同一受限方向或只改速度不能自动重试。改变方向不等于运动许可：仍需暂停后的服务器 echo、Stop 回执、健康静止反馈及起点重定、原候选时效/代号/父点、几何及跟随检查。旧目标清除后不会补发。夹爪单独输入不会附带机械臂保持命令。

已有普通 status 2/5/6 分支及 workspace 外向输入锁存保持，不新增对旧状态的反复 reset。新 `control_direction_limited` 事件每次暂停记录一次，持续按住不重复记录该事件；原始拒绝/设备回执证据仍保存。界面保留“可控制”，显示方向限制原因与恢复提示。

## 验证

- 完整后端 **1290 passed / 19 skipped**，原始输出 `evidence/control-recovery-v357/backend-tests-final.txt`。跳过项不是通过项。
- 新增覆盖：直接反向、其他平移/旋转/关节方向、参考系/辅助模式变化、回中、夹爪单独操作；旧 echo（即使 clock-bound 年龄有效）不得解锁；更严格的输入过期暂停不得降级。
- 验证同方向速度改变、一次 Stop/日志、录制不 abort；未收到 Stop 回执及未静止不得写新目标；回执和静止完成后仍须新的合格候选。
- 验证不连续候选零写入并保留 Prepare；已有真实跟随/跨 reset 大偏差、反馈/模型失效、停止失败、主动 Stop 与断连回归全部通过。
- 首轮 122 pass / 1 fail 是新夹爪测试替身缺少 `actual` 字段，原失败保存在 `focused-tests-a.txt`。补齐替身后完整回归通过，未因测试失败放宽生产检查。
- 本轮没有故意触发实机碰撞、停滞、超时或发送运动；因此不宣称真实故障恢复的运动验收。局部变更不代表整个遥操作长期/全范围、SDK 同轨迹、录制回放验收完成。

## 部署与当前状态

V356 owner `20261020-230927-34335769` 在20份新状态均未准备、静止、录制 idle 后，通过 `output/operator_alignment_356.stop` 正常退出。远端 camera/gateway 均0，CPU/WiFi恢复；PC app0/media1、cleanup空原样保留。

新唯一 owner **20261020-233413-2bb08a49**，执行会话 **66455**，入口 `output/operator_control_recovery_357.py`。退出入口 `output/operator_control_recovery_357.stop` 当前未创建；合成输入关闭。

20份新回读均未准备、静止、idle、Mode5 Enable1 Error0 Collision0；命令ID **27406** 未增加，负载匹配，夹爪自动只读连接 ready / sent_raw=null，没有位置初始化。Gamepad / Translation / Base / 0.8x 页面已刷新，3D实时、RGB20fps；实体手柄未识别，待用户唤醒。没有 Prepare、Enable、ClearError 或臂/夹爪运动。

新完整部署源清单与 release 相同，四文件远端 SHA 已独立核对；见 `deployed-verification.json`。生产原生 `d5551ee2…` / pin `682b721c…`、55/70/50/48ms计算预算、候选150ms/断档200ms、反馈100ms/输入250ms不改。V356停止对齐1°、首目标连续性及真实跟随保护不改。模型3b4003e4、TCP151.4、0.750kg COM(0,-14,46)、Force255/ACM不变。

## 可回滚副本

`output/control-recovery-v357/release` 保存实际V356四源、完整部署清单/配置、新四源及差异。

`output/rollback_control_recovery_357.py --check` 已通过。实际回滚必须先通过本轮 stopfile 正常释放并确认端口结束，再恢复四旧源、用正常 launcher 建立新会话部署；不在活动 owner 中热替换。

RDK 磁盘低于1GB，已仅对退出的V356 `audit/events.jsonl` 无损压缩。608282230字节明文，SHA `fbb31ff310c2600fa63a482e2d6621558ce8549ce897f57ca0f4d900014e7394`；79956577字节 gzip，SHA `1464bdb91cf8ca8d03dbdb04693b8218d71a5cf065639d13101f277d598c463f`。PC压缩件、解压后完整内容及远端副本全部验证后才回收重复明文；故障记录保留。远端路径变为同目录 `events.jsonl.v357.gz`，PC `evidence/control-recovery-v357/retired-v356-events.jsonl.gz`；原 raw 未改。归档后free1099206656字节，新日志仍会增长。

Goal仍 active，原自主抓杯取消，无Git提交。下一次实机动作必须新读状态与画面。
