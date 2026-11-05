# GitHub 仓库维护

远端：`git@github.com:1229282331/myrobot.git`，默认分支：`main`。

2026-11-05 从本机当前工作树建立新的源码初始提交。保留 E6 平台、OmniPicker、旧遥操作、相机工具、配置、依赖锁、模型、必要测试样本和文字报告。原本地分支及大体积实验历史不属于新的远端提交历史。

## 日常修改

```powershell
git status
git pull --ff-only
git switch -c codex/describe-change
# 修改、运行与本次变更相关的离线测试
git add <需要提交的文件>
python tools/check_repository.py
git commit -m "Describe the change"
git push -u origin HEAD
```

小改动可以直接在 `main` 提交后 `git push`；多人协作或较大变更使用分支和 PR。推送前读取差异，提交说明注明实际测试与未验收项。`git pull --ff-only` 遇到分叉会停止，此时检查远端改动再决定合并方式。

安装本地提交检查：

```powershell
git config core.hooksPath .githooks
```

检查器读取暂存区，限制单文件 50 MiB、整个源码树 300 MiB，拒绝普通日志目录、依赖环境、私钥文件和常见凭据格式，并核对当前模型及原生发布文件 SHA。推送钩子还会拒绝包含旧实验历史的分支。它不是完整的秘密检测器；仍需人工审查差异。GitHub Actions 对提交树执行同一检查。

## 只留在本机的内容

- `runtime/`、大部分 `evidence/`、`output/`、研究下载、录制与数据集。
- Python 环境、`node_modules`、构建输出、SDK 安装目录、私钥及本地认证配置。
- 迁移前的 Git 历史：`codex/teleop-platform-rebuild`、`codex/e6-teleop` 和 `master` 等本地分支。

不要使用 `git push --all`、`--mirror` 或把旧历史合并到新的 `main`。旧历史包含大量实验数据；需要某项修改时，审查后单独移植代码。迁移前的未提交文档、索引和配置备份在本机 `output/github-publication-20261105/before/`。

少量位于历史目录的测试/CAD输入通过 `tools/repository-policy.json` 精确保留。新增此类依赖时，只将实际需要的文件加入该清单，审查后用 `git add -f <文件>`；不要开放整个日志目录。

## 上游与复现

第三方项目保留为固定提交的 Git 子模块。phosphobot 工作树出现修改通常是已保存补丁和前端复制的结果，不要将构建产物作为新的上游源码提交。按 `teleop-platform/upstream.lock.json`、`patches/` 和 setup 脚本重建。

历史报告链接的完整原始记录保存在本机，GitHub 只包含精确保留的离线样本。历史回滚脚本需要相应本地归档，不能把这些脚本的存在当作全新克隆拥有完整旧运行时。当前 RDK 原生二进制和版本锁另存 `teleop-platform/releases/v355/`；不自动连接或部署设备。
