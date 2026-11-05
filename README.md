# MyRobot：E6 遥操作与数据采集

Magician E6、OmniPicker 夹爪和 Gemini E 相机的控制、建模与记录工具。PC 为 Windows，设备侧为 RDK X5。当前主要入口是 [E6 工作台](teleop-platform/README.md)。

本仓库从 2026-11-05 的本机源码和当前资产整理而来，保留配置、依赖锁、上游补丁、模型和必要离线测试样本。完整录制、实验日志、环境和历史归档仍留在原机器，未上传。文字报告中的部分历史证据链接因此只在原工作目录可用。

## 目录

| 路径 | 用途 |
|---|---|
| `teleop-platform/` | 当前 phosphobot 工作台、控制适配、MoveIt 原生源、网页、模型、测试 |
| `omnipicker/` | 独立夹爪上位机与官方三维资产 |
| `teleop/` | 早期模拟/观测工作台及 LeRobot 离线工具 |
| `dcw/` | 相机诊断、时间戳与采集工具；SDK 按其说明安装 |
| `tools/`、`tests/`、`config/` | 模型/CAD工具、历史离线控制测试、配置 |
| `docs/` | 使用、验收边界和 Git 维护说明 |

## 获取与安装

```powershell
git clone --recurse-submodules git@github.com:1229282331/myrobot.git
cd myrobot
git config core.hooksPath .githooks
```

当前平台需 Python、uv、Node/npm，随后按 [安装与启动](teleop-platform/README.md)执行：

```powershell
python teleop-platform/scripts/setup.py
python teleop-platform/scripts/setup_launcher.py
```

安装不包含全新 RDK 的完整环境配置。设备路径、二进制/模型校验值见 `teleop-platform/config/workstation.json`。当前原生发布文件保存在 [releases/v355](teleop-platform/releases/v355/README.md)，模型在 `src/e6_phosphobot/assets/workspace/`；保留其 SHA 校验，不重建后直接替换。

## 当前设备与验收边界

最后一次确认是 **2026-10-23 V361：机械臂已下使能，RDK 已关机，无控制 owner**。这是历史状态，不能据此判断现在已开机、使能或准备就绪。下一次操作须用户要求继续，并取得新反馈、负载/姿态及画面。

J4 上电异响原因、下使能后的非零姿态变化、完整长期/全范围/SDK录制回放验收仍未解决或未完成，自主抓杯保持取消。代码上传与离线测试不增加实机验收范围。详细约束见 [AGENTS.md](AGENTS.md) 和 [平台状态](docs/teleop-rebuild/STATUS.md)。

## 维护与检查

```powershell
git status
python tools/check_repository.py
```

检查器读取暂存区并核对文件范围、大小、常见凭据、模型和发布 SHA。后续使用 `origin` 和 `main` 维护，具体提交、分支、子模块及本地归档规则见 [Git 工作流](docs/GIT_WORKFLOW.md)。

第三方代码与模型保留各自的许可证、来源及固定版本，参见 `.gitmodules`、`teleop-platform/licenses/`、`upstream.lock.json` 和资产来源清单。
