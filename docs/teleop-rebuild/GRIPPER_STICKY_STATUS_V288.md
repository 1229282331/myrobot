# v288：夹爪 Blocked 后无法继续控制

2026-10-18。本轮修正 v287 的反向恢复误判。用户新确认夹爪空载、开合区域已清空；实测只动夹爪，没有发送机械臂运动。完整 Goal 继续暂停，未提交 Git。

## 原因

FW335 的 `fault=0/state=2` 会在实际已经反向移动、甚至到达目标后仍保留。v287 只等待状态码退出 2，且固定等待 500 ms；到期后把反向也记成受阻，形成几计数往返后再次锁住的行为。上一轮空载常规开合没有覆盖这段真实固件行为，不能作为受阻后恢复的实测证明。

原始事件固定前缀 198086687 字节，SHA256 `4b05d84f059c8672aa73e0820fdfb0ae4bd2c95ad39b6494400dd1ce2eda2f9f`，保存在仓库根目录 `evidence/gripper-blocked-v288/events-tail.jsonl.gz`。第一次反向命令为实际 raw208 → target212、速度1、力64、加减速25；软件在 509.523 ms 误判失败，609.390 ms 已实测张开两计数，1720.713 ms 达到212。三秒59个原包始终 fault0/state2。见 `diagnosis.json`、`incident-window.json`、`sticky-state-proof.png`。

## 修改

- 显式反向仍从实际开度提出最多四计数、速度字节1的初始目标，仍经过原 FCL 许可。
- 状态离开2，或实际沿请求方向前进至少两计数，才确认恢复。单计数抖动不构成恢复证据。日志保留原状态码及判断依据，不声称物体已释放或抓取成功。
- 无进展等待按原生速度估计：`max(0.5, 2 / (max(1, velocity) × 1.92) + 0.5)` 秒；速度1约1.542秒。1.92计数/秒/速度字节为已有空载记录的工程估计，不是厂商保证。此时间只用于判断停滞，不延长输入或几何许可有效期。
- 状态2保留期间持续检查实际方向进展；同向新目标不能重置无进展计时，提速也不能反向缩短已经处于慢速的起步窗口。若再次无进展，冻结目标并回到受阻观测，不自动反复试动。
- 到目标一计数内、速度0连续三次可以表示保持到位，仍不表示成功握持。非零fault、state3、错误帧和通信超时保持故障流程。
- force64、accel/decel25、raw12–255可用行程、原速度渐变、250ms输入过期、200ms反馈与FCL保护未变。前端、装配模型、求解器与臂控制参数未改。

源修改仅 `teleop-platform/src/e6_phosphobot/gripper.py`、`operator_logs.py`。未改变CAN协议，也未通过清错或重初始化消除这次固件残留状态。

## 验证

旧实现重放7失败/1通过，保留 `output/gripper-blocked-v288-before.log`。修改后相关后端200项通过；随后增加完整59包夹具，专项9项通过，其中8项与前者重叠，共201个不同用例。完整包重放使用生产解码器及原位置、速度、状态、fault字段。覆盖持续进展、真实再次停滞、单计数抖动、反向中停止、实际到位、真fault/state3。前端无修改，沿用v287构建与192项前端结果，不计作本轮新构建。

实测会话 `20261018-014028-0db0e64d`：正式网页键盘4段（首次恢复、闭合、释放后重开、再闭合）及合成手柄LT/RT各一段，0.2x，仅夹爪。合成输入经过正式映射与控制连接，不等同于实体无线接收器验收。

| 项目 | 结果 |
| --- | --- |
| 初始真实状态 | raw212，fault0/state2，Blocked方向未知，尚无初始化位置写入 |
| 首次反向 | target216、速度1、力64；602.608 ms 实际到214，仍state2，以 measured_reverse_progress 恢复 |
| 后续 | 超过初始四计数目标持续开合，最终首次段实际227；恢复后重新受阻0次 |
| 普通五段 | 内段零速回读0，实际约2.16–2.26可用行程百分点/秒 |
| 恢复首段 | 慢速试退及渐变期间保留4个内段零速度量化回读，不称整个恢复过程完全匀速 |
| 释放后 | 首个有效到位回报约356.9–486.0 ms，从网关收到中立输入计，不是端到端延迟 |
| 几何 | 4688结果，无拒绝/预算耗尽，最大5.511 ms，显示开度与几何映射错误0 |
| 许可等待 | 75次新目标等待许可/恢复事件保留，不能把事件名误读为75次实物碰撞 |
| 原始SDK | 30982帧，连续无缺号，ID22279始终不变，全部Mode5/Enable1/Error0/Collision0/空队列 |
| 臂动作 | 运动输入0、提交0；全会话关节峰峰最大0.013733°，不称严格零位移 |

本轮成功恢复的是实际残留受阻状态，夹爪为空载，没有进行新的物体抓取或被动指节接触形变标定。最初空载约40%停滞的机械原因仍未确定。

原始记录及图表位于 `evidence/gripper-blocked-v288/physical/`：`events.jsonl.gz`、`raw.jsonl.gz`、`analysis.json`、`sdk-analysis.json`、`recovery-analysis.json`、`recovery-detail.png`、`motion-detail.png`。首次恢复慢速段及所有原始量化值均保留，未平滑数据。

## 发布与回滚

旧日常v287和本次测试均正常clean释放，CPU/Wi-Fi策略恢复。两次本地media退出码1原样保留，无cleanup错误，不改写为媒体退出码0。

实际已部署的v287两个源文件保存于 `output/gripper-blocked-v288/baseline/`，本轮源文件在 `release/`，散列在 `manifest.json`。这份基线来自远端部署，不是较早的Git HEAD。

回滚先在网页Stop、静止、录制idle后离开Control，通过当前owner自己的stopfile正常退出；再执行 `teleop-platform/scripts/switch_gripper_sticky_v288.py --rollback --check` 和 `--rollback`，按原日常配置启动新的owner。脚本核验白名单与SHA、相关端口已释放且最新session结果clean。回滚会恢复已知500ms误判，仅用于诊断，不作为推荐日常版本。

当前日常会话及新鲜状态见文末发布记录。测试 stopfile `output/gripper_sticky_operator_288_test.stop` 已建立，不能直接复用该测试入口启动同名测试。

### 发布记录

日常owner `20261018-014709-9410913e` / exec40550，入口 `output/gripper_sticky_operator_288.py operator`，正常退出文件 `output/gripper_sticky_operator_288_operator.stop` 尚不存在。合成输入关闭；实际远端两源文件、部署清单、本地源与发布包SHA一致。

最终20新状态均未准备、静止、录制idle、Mode5/Enable1/Error0/Collision0/ID22279。夹爪自动观察接入raw209（可用开度约81.1%）、ready、contact=null、sent_raw=null，没有初始化位置写入；关节峰峰最大0.004807°。模型b4e1f8bc、v286求解器cb4055a9、安装B、TCP151.4mm、负载0.750kg/COM(0,-14,46)mm及原范围柔化保持。本轮未Enable、清错或改负载参数。

正式Gamepad页面已打开，0.2x、未准备、3D实时与RGB有效。页面当前显示没有检测到实体手柄，需唤醒/连接后才能Prepare；本轮不把合成手柄测试写成实体接收器已验证。证据 `final-release.json`、`sticky-operator-final-states.json` 及 `final-platform.png`。
