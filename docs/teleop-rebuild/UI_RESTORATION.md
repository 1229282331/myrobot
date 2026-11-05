# phosphobot 官方 UI 恢复（v239）

日期：2026-10-11。分支 `codex/teleop-platform-rebuild`；改动前主仓库提交 `4666f43b2f4607c3c4cab5f4f6705781987d8792`。本轮交付范围为 UI 和离线交互；完整平台 Goal 仍暂停。

## 结果

恢复 phosphobot 0.3.134 / `a5de051197c879e3c685d1362f649ce54ee47a3c` 的顶栏、侧栏、绿色标题、页脚、字号、间距和主题。控制页继续运行上游键盘／手柄页面，E6 状态、录制及数据检查使用同一套组件。

按用户最后修正，导航保持普通可点击链接，不显示 `Not adapted` 徽章、不置灰。Calibration、AI Training、AI Control、Admin Configuration、Network Management 进入后才显示 `Not adapted for E6`；直接访问也经过守卫，未适配的上游操作页面不挂载。账户与云端入口隐藏。

当前页面：http://127.0.0.1:8765/control?tab=keyboard 。页面保留未准备状态，供用户评价。

## 实际复用与必要差异

所有下列官方文件均来自上述固定提交，保留 MIT 许可。源目录为 `teleop-platform/upstream/phosphobot/dashboard/src`。

| 部分 | 实际复用 | E6 差异及原因 |
|---|---|---|
| 外壳 | Layout、Sidebar、AppSidebar、TopBar、Footer | 移除自定义研究工作台外壳和尺寸／配色覆盖；本地隐藏账户、推广、上游更新操作 |
| 控制输入 | KeyboardControlPage、GamepadControlPage、GamepadComponents、GamepadUtils、SpeedSelect | 保留现有 E6 hook 和处理函数；仅重排展示、按模式显示轴／关节选择、显示后端速度上限乘倍率 |
| 相机 | CameraStreamCard＋现有 WebRTCPreview | 空视频地址展示 `No camera stream`；总览复用同一组件，未增加相机启动流程 |
| E6 状态 | 官方 Card、Tabs、Select、Button、Badge | 连接取决于真实反馈新鲜度，HTTP 缓存快照也会过期；未知夹爪不显示历史开度 |
| 录制 | 官方 Recorder 工具栏视觉及 Button、Popover | 保留项目既有开始／成功／失败／中止／媒体回传逻辑；任务及备注放入设置弹层 |
| 数据 | 官方 Browse Datasets 页面分支、Table、Badge、Dialog、Card、Select、Slider | 数据来自 `/e6/episodes`；保留原图及逐采样检查，不启用上游删除失败记录、回放或真实动作导出 |

新增组件和展示适配位于 `teleop-platform/frontend`。`useE6Control.ts` 唯一修改是把已有 `speed` 暴露给展示层；本轮没有修改 E6 运动、停止、租约、按键映射、TCP、后端 API 或记录格式。

`0006-e6-workbench-layout.patch` 已收敛为本轮官方外壳／页面适配，`0008-recording-recovery.patch` 同步新上下文；原输入隔离、故障处理和原始手柄轴补丁保留。八个补丁在干净固定上游上可顺序应用，20 个受影响上游文件与工作副本匹配。`prepare_upstream.py` 通过隔离 Git index 比较整条补丁链，解决后续补丁改变早期上下文后重复 prepare 的误判；不重置工作副本。

## 验证与限制

- 147 项前端测试、8 个测试文件全部通过；正式上游生产构建和资源发布通过。
- 打包测试通过：叠加补丁可重复准备；拒绝不匹配的本地修改并保留原文件；真实 Git index 不变。
- 覆盖正常导航、未适配路由不挂载上游功能、未知／过期设备、无相机、模式展示、录制／回传失败、空记录、输入生命周期、页面浏览暂停与显式恢复。
- 正式浏览器核对导航、Dashboard、Camera Overview、键盘／手柄页及旧 `panel=episodes` 跳转。UI 改版没有新建第二条控制连接；只读状态组件使用共享 SWR 轮询。
- 只读查看成功 `20261009T104609Z_5b6538fc`、失败 `20261009T075029Z_3115cb6d`、中止 `20261009T120739Z_4654b673` 的真实记录。失败片段第一采样没有前置 RGB，界面如实显示；下一采样可见原图。364 个已有 episode JSON 文件 SHA 前后完全一致。本轮未重新逐一哈希所有图像文件。
- 检查当前窗口、1280×720、1920×1080和1536×864 CSS 视口，覆盖浅／暗主题，未发现页面／主内容横向溢出。短窗口使用页面滚动，详细项按需展开，没有另建狭窄控制滚动区。
- 浏览器缩放快捷键没有改变实际 DPR／视口，因此 **125% 浏览器缩放尚未实测**；1536×864 CSS 视口仅为1920×1080下125%的布局等效检查。没有把它写成原生缩放通过。

证据：[测试日志](evidence/ui-v239/frontend-tests.log)、[构建日志](evidence/ui-v239/build.log)、[打包测试](evidence/ui-v239/package-test.log)、[源文件 SHA](evidence/ui-v239/source-sha256.json)、[制品清单](evidence/ui-v239/build-manifest.json)、[记录完整性](evidence/ui-v239/record-integrity.json)。开发中失败日志保留于 `output/ui-v239-*`，未删除。

## 原版／改版对照

原版参考为此前在同一固定上游版本采集的历史原版截图，不冒称本轮重新运行截图。记录检查的三张截图采集于最后一次图像高度调整前；数据内容未变，最终原图展示采用 `object-contain` 和45vh高度上限。

| 参考 | 截图 |
|---|---|
| 固定版本历史原版手柄页 | [upstream_gamepad.png](evidence/upstream_gamepad.png) |
| 改版最终控制页 | [final.png](evidence/ui-v239/final.png) |
| 1280浅色键盘页 | [keyboard-1280-light.png](evidence/ui-v239/keyboard-1280-light.png) |
| 1280暗色手柄页 | [gamepad-1280-dark.png](evidence/ui-v239/gamepad-1280-dark.png) |
| 1920暗色 | [keyboard-1920-dark.png](evidence/ui-v239/keyboard-1920-dark.png) |
| 进入后提示 | [unavailable-route.png](evidence/ui-v239/unavailable-route.png) |
| 真实相机总览 | [camera-overview.png](evidence/ui-v239/camera-overview.png) |

## 本轮设备范围与资源归属

用户告知已开机后，使用 shadow 会话 `20261011-175544-2dd40258` 读取真实状态和显示相机。新反馈为 Mode4、Enable0、无报警／碰撞；CAN DOWN、RX0／TX0，夹爪遥测未知。视频浏览约16.6–20fps，仅为 UI 查看证据，不等同完整视频性能资格。

没有使能、运动、初始化 CAN 或实机输入验收。事件审计见 [session-event-counts.json](evidence/ui-v239/session-event-counts.json)，保存时只有反馈、周期、输入处理与时钟事件。历史实机资格不因 UI 改版而扩展。

现有 PC app PID26536被复用；shadow launcher exec47986的入口为 `output/ui_shadow_v239.py`。保留真实预览供用户评价。结束本轮预览时创建 `output/ui_review_v239_shadow.stop`，由 launcher 正常释放本次资源；复用的 PC app 不属于该 launcher。普通退出不下使能、不关机。PID与状态仅代表本轮保存时刻，下一操作须重新读取。

## 回滚

改动前375个文件及 SHA 清单保存在 `teleop-platform/runtime/ui-rollback-v239/`，包括 `served`、`frontend`、`patches` 和 `dashboard`。回滚前结束当前浏览器控制会话并正常停止本项目服务。

快速视觉回滚：将备份 `served` 内容覆盖到 `teleop-platform/upstream/phosphobot/phosphobot/resources/dist`，确认备份 `index.html` 及其引用资源 SHA 与清单相同，再重启 PC 服务。无需清空目录；未引用的新资源不执行。恢复后先做离线页面核对，不能据此自动使能设备。

源码回滚：保留本次修改后，恢复备份 `frontend` 和 `patches`，并基于固定上游干净副本重新应用旧补丁与构建。不要仅恢复生成目录后声称源码已回滚，也不要用整仓 reset/clean 删除已有证据或其他工作。
