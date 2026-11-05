# 项目工作约定

## 当前有效状态（V361，2026-10-23）

用户确认使用没问题后要求关闭，已经完成：唯一控制 owner 正常释放，camera/gateway 退出、CPU schedutil/WiFi on 恢复；独立持锁执行官方 Stop 和 DisableRobot 各一次，均返回 0；末帧 Mode4 / Enable0，无报警碰撞队列。CAN DOWN，RDK 接受 poweroff 后 SSH 三次超时，本机控制监听均关闭。当前没有控制 owner，物理断电由用户执行。

下使能期间关节有非零变化，尤其 J4 约 -8.82°、J5 约 -3.34°，随后稳定。不能称零位移或正常校准。J4 上电异响原因仍未查明，完整长期/全范围/SDK对照/录制回放验收未完成；自主抓杯保持取消。

下次必须用户要求继续，重新获取开机反馈、负载、姿态和现场新画面，不能沿用旧 ready。不得因代码维护或测试启动设备、使能、清错、改参数、停放补偿或发送机械臂/夹爪动作。实机动作严格按当次授权和新现场事实执行。

## 保留的生产约束

V358 Python 源来自本地旧提交 7efe9dcf，原生二进制 d5551ee2、pin 682b721c。关节运行偏差阈值 2° 持续至少 500ms；反馈100ms、输入250ms；callback/request55/70ms、名义积分/发送50ms、候选150ms/断档200ms。TCP151.4mm、质量0.750kg、COM(0,-14,46)mm及既有模型/ACM/Force保护保持。不能将历史局部通过当作全范围通过。

修改前阅读相关源码和平台文档，保存失败证据，不将失败改写为成功。不得删除或覆盖原始故障/录制归档；资源清理必须尊重单 owner 和会话身份，不全局杀进程。

## GitHub 维护（2026-11-05）

用户指定长期远端 `git@github.com:1229282331/myrobot.git`。当前目录继续用于开发，`main` 跟踪 `origin/main`；新功能默认使用 `codex/` 分支。初始远端提交只包含代码、当前必要资产、配置、文字文档和精确保留的离线样本。

原本地历史保留在 `codex/teleop-platform-rebuild` 等分支，切勿 `push --all`、`--mirror`，或将整段旧历史合并到新 main。原始文件继续留在本机忽略目录。迁移前 AGENTS/README/STATUS、Git索引和配置备份在 `output/github-publication-20261105/before/`；历史事实仍可从本地分支和归档查阅。

提交前审查 diff 并运行相关离线测试及 `python tools/check_repository.py`。不提交私钥、密码、token、环境、node_modules、录制和大日志。精确保留样本见 `tools/repository-policy.json`；不要放开整个证据目录。上游固定提交、补丁及模型字节校验必须可复现；完整流程见 `docs/GIT_WORKFLOW.md`。
