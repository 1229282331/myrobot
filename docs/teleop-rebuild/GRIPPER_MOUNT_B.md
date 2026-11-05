# 夹爪安装方向 B（2026-10-16）

用户在独立装配预览中确认方案 B。夹爪相对旧模型绕自身安装轴旋转 −90°；只改变 `controller_flange → omnipicker` 固定关节。R4 板、Gemini E、机械臂串联关节、碰撞形状及碰撞排除表保持原样。

正式模型：`67c7cc21dc632264267460f709e14fa34a500308467cc1dc4503b4b1f213a3d1`。统一配置在 `teleop-platform/config/gripper_mount.json`，完整 CAD 构建器与已有模型修订工具均读取它。控制和前端引用同一安装变换；这不是单独转动显示网格。

## 结果与验证

- 同一组本轮实际关节反馈，旧模型报告 `Link2/omnipicker` 碰撞，B 方向下不再碰撞；FK 计算的 TCP 位置完全一致。这支持安装方向错误造成了该姿态的误碰撞，不能据此断言全部姿态均无碰撞。
- 抓握 TCP 仍为轴上 151.4 mm，控制坐标轴定义不跟随模型修订改变。依原质量分布假设复算主件重心为 `(0.3731, −13.6655, 45.7149) mm`，整数毫米仍为 `(0, −14, 46)`。0.750 kg 固定负载及该重心配置不变，未写设备参数。
- 162 个相关测试通过，包括显示／控制装配一致、TCP、工作空间、夹爪、周期与启动回归。第一次检查脚本受 Windows 默认 GBK 读取影响，失败日志保留，明确 UTF-8 后通过。
- 使用与旧版相同的 65,537 个确定性关节样本，得到 37,682 个去重 TCP 点；逐一 FK/FCL 复查无碰撞，FK 误差为 0。另一次更密的采样及 137,304 个点的成功复查保存在 `output/gripper-mount-v268/power18`，不用于增加正式页面渲染负担。
- 原 v266b 会话正常 clean 退出、恢复 schedutil；随后启动 `20261016-202718-8c56d5ea`。原 v236 C++ 二进制保持不变，正式部署源码逐 SHA 核对。
- 正式页面展开、缩放及图层核对后，20 个新状态均未准备、静止、Mode5/Enable1/Error0/Collision0、命令 ID1、录制 idle；关节峰峰最大 0.004120°。相机有效。本轮没有 Prepare、运动、Enable、清错或参数写入。
- 会话切换释放了旧夹爪 worker；新会话未自动初始化夹爪，反馈标为 unknown/uninitialized，不能沿用旧会话的 35% 回读。

## 动态夹指仍未发布

本次线上使用原有全行程碰撞包络，只修正其安装方向。动态逐部件模型已同步为 B，暂存在 `output/dynamic-gripper-v267/mount-b-assets`，但 CAN 在途目标覆盖、FCL 扫掠与实时性等仍需完成验证。前端正式版本仍标注夹爪为名义闭合姿态，不把它称作真实开度。

用户确认的是安装方向，不是尺寸或被动指节的实物标定；桌面、环境、线缆仍未建模。此前错误安装方向下的间隙判断不得作为实物间隙依据。完整 Goal 保持暂停。

## 回滚与证据

回滚基线：`output/gripper-mount-v268/baseline`，保存原模型、构建、配置和切换前开发源码。旧会话部署记录和远端版本保留。

在未准备、静止、录制结束后正常退出自有会话，再运行：

```powershell
.venv-cad/Scripts/python.exe teleop-platform/scripts/switch_gripper_mount_v268.py --rollback --check
.venv-cad/Scripts/python.exe teleop-platform/scripts/switch_gripper_mount_v268.py --rollback
```

该脚本只切换模型，不发送设备命令、不终止进程、不删除历史版本。回滚会恢复已知错误的旧安装方向，仅用于诊断，不代表旧几何正确。

证据在 `evidence/gripper-mount-v268`：`mount-contract.json`、`current-pose-collision.json`、`witness-verification.json`、`tests.xml`、`post-release-report.json`、原状态与 `formal-mount-b.png`。
