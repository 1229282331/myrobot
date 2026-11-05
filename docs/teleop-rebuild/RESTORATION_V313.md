# v313 PC / RDK 恢复交付

2026-10-19。PC 配置、正式前端和 RDK 运行版本已完成切换，当前使用 **v299 求解器 + v311 已修正的 Python 控制层**。这是经过验证的组合恢复，不是 v294 原样回退。现有记录及失败证据保留。

## 版本选择与保留项

- 首先测试了用户此前反馈顺畅的 v294。原适配层的持续输入重置缺陷已由协议夹具复现；该夹具不是完整求解器或实机回放。使用当前适配层进行板端隔离测试时，v294 求解器在当前姿态部分方向仍超时、没有候选，未发布该版本。
- 最终选择 J5 事件前的 v299 求解器，保留几何计算优化。当前模式为 Translation / Rotation / Joint；Position assist 能力关闭，正式源文件仅在能力可用时显示对应入口。
- Python 控制层与恢复前 v311 源码完全一致：保留同帧跟随误判修正、相邻实际写入指令连续性判断、真实停滞和突跳保护、拒绝锁存及写口屏障、录制回传恢复、夹爪快照时间修正和独立审计进程。
- 模型 `3b4003e4…`、安装 B、TCP 151.4 mm、负载 0.750 kg / COM (0,-14,46) mm、夹爪 Force255、原碰撞与关节范围保持。未改设备参数。

| 部件 | SHA-256 |
| --- | --- |
| RDK v299 native solver | `6314bdc0ff22586fc42c5be4e38b96a5f0e54380e9fd5e27e1a44a6785a94633` |
| 部署 pin | `acee116099971ab6ade97219418bef59610d8171be24b54cc904446ee232c69c` |
| PC workstation 配置 | `d401a6b9f2781f818cd93dcd1e32ea546c6ee4404bf3acb0f12a62864666c03e` |

精确源码、原始 v299 pin、修改后 pin、二进制、测试及文件哈希见 `output/restore-v313/release.json` 与 `release/`。RDK pin 为 `/root/e6-teleop-rebuild/jazzy-v32/rootfs/qualification/cpp-adapter.json`，二进制为其 qualification 下的 `geometry-recovery-v299/servo_cpp_adapter`。

## 验证范围

| 验证 | 结果与边界 |
| --- | --- |
| 后端回归 | 1114 通过，8 跳过 |
| 前端回归 / 正式构建 | 208 通过，正式构建成功 |
| RDK 网络隔离、无设备 IO 回放 | 5 个用例、1191 个接受候选；覆盖当前姿态基座/工具坐标平移与旋转及既有姿态；使用合成延迟反馈，不是实机验收 |
| 隔离运行异常 | 没有活动几何超时或意外拒绝；启动前 1 次 feedback_stale、正常释放 1 次拒绝保留 |
| 部署核对 | PC/RDK manifest 一致，gateway、cpp_cycle、joint_tracking 源码哈希与 v311 一致 |
| 正式页面 | Gamepad / Translation / Base / 0.2x，3D 实时、RGB 约 20 fps；浏览器尚未识别实体手柄 |
| 交接状态 | 新 20 样本全部未准备、静止、录制 idle；Mode5 / Enable1 / Error0 / Collision0；命令 ID5789 未变，负载匹配 |
| 实机运动 | 本轮未执行，没有重新 Enable/Disable、清错、机械臂或夹爪位置命令；不宣称运动体验已完成新一轮实测 |

夹爪以只读观察方式接入，实际回读约 29/255、sent_raw=null；这与界面校准后的百分比不是同一刻度。恢复没有初始化夹爪到新位置。新 owner 的早期审计快照 1011 周期无 Prepare / motion_submitted / 跟随停止或运动写入事件。不能由此推断此前用户会话没有运动。

J5 最初不跟随的设备/机构根因仍未确定；v307 实时退离候选仍未通过。本次恢复不代表全空间、全速度或实机回放验收通过。

## 当前平台与后续接管

- 唯一 owner：`20261019-224907-31ef4663`，统一执行会话 `67791`。
- 入口：`output/restore_operator_313.py operator`；stopfile `output/restore_operator_313_operator.stop` 尚未建立。
- 正式页面：<http://127.0.0.1:8765/control?tab=gamepad>。手柄需要在页面按实体按键唤醒，待出现名称、摇杆回中后由用户 Prepare。
- 旧 owner `20261019-215947-8b5036b3` 已通过 stopfile 正常退出。camera/gateway 退出0、CPU/Wi-Fi 恢复；本地 media 退出1、cleanup 错误列表为空，原结果保留。
- 完整 Goal 继续 paused，自主抓杯取消；本轮未提交 Git。

## 备份与撤销本次恢复

恢复前文件在 `output/restore-v313/baseline/`；更完整的项目文件归档在 `output/rollback-review-v312/`，这是项目文件备份，不是整机系统镜像。v294 失败尝试见 `output/restore-v313/rejected-v294/` 和 `evidence/restore-v313/isolated/`，v299 通过证据见 `evidence/restore-v313/attempt-b/`。

撤销入口是 `teleop-platform/scripts/switch_restore_v313.py`，恢复本次切换前的 L 求解器及原 UI / 配置，保留当前 Python 修正。必须先新读未准备、静止、录制结束，再通过当前 owner 的 stopfile 正常释放；不得在 owner 运行时调用，也不得新开第二个控制连接。脚本已包含端口、退出结果、文件哈希和 RDK 独占锁检查。

使用既有 launcher Python / 认证环境，先执行 `--check --rollback`，再执行 `--rollback`。切换后须新读状态并正常启动经核对的 owner；不能用旧姿态或旧夹爪 ready 作为下一次动作依据。

完整交接证据：`evidence/restore-v313/handoff-summary.json`、`handoff-states.json`、`deployed-verification.json`、`switch.json`、`previous-owner-exit.json`。页面截图：`evidence/restore-v313/handoff.png`。
