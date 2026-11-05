# 工作台交互调研与设计取舍

2026-10-09。范围：两类研究示教系统、两类工程可视化工具，以及本项目实际运行的固定phosphobot。以下是设计依据，不是这些项目在E6上的运行资格。研究项目的源码只作阅读，未加入运行依赖；快照哈希见`evidence/ux-research/source-manifest.json`，下载副本留作本地研究材料。

## 研究与工程来源

| 来源与证据层级 | 核对结果 | 本项目采用／不采用 |
|---|---|---|
| [DROID采集流程](https://droid-dataset.github.io/droid/example-workflows/data-collection.html)，[源码33ae6a6](https://github.com/droid-dataset/droid/tree/33ae6a67274f36d2e29525b86f23a56616ef43a7)。阅读`droid/user_interface/gui.py`、`eval_gui.py`，没有运行其硬件软件 | 任务配置、相机页、练习与正式采集分开；成功／失败标签明确。源码有场景配置、控制器断连页和放大图像页，也含机器人复位及随机复位路径 | 借鉴任务上下文和结果标记，让图像与任务同时可见；不迁移Tk应用、强制姓名、6路相机假设及自动复位。GitHub API未识别该固定仓库许可证，不复制其实现进入本项目 |
| [Open-Teach](https://open-teach.github.io/)，[固定源码32a7d44](https://github.com/aadhithya14/Open-Teach/tree/32a7d44b33953066ff27312a7b2b4c294f4f52c5)，MIT。阅读README与`docs/teleop_data_collect.md`；未运行VR／机器人 | 文档区分遥操作与数据采集入口，采集机器人状态、相机视频、深度和时间信息；VR及多种机器人配置是其主要运行方式 | 借鉴控制与记录职责分开、每个传感源可检查的工作流。当前输入是PC键盘／手柄，不引入VR、Unity和另一套硬件控制链；现有原图、反馈及发送事件记录继续使用已验证适配 |
| [Foxglove布局](https://docs.foxglove.dev/docs/visualization/layouts)、[回看](https://docs.foxglove.dev/docs/visualization/playback)、[曲线](https://docs.foxglove.dev/docs/visualization/panels/plot)。2026-10-09官方在线文档阅读，非本机亲测；在线文档未提供固定源码提交 | 任务布局、标签面板、统一时间定位和选中对象详情服务调试。回看可以保留某话题最近值，布局保存／共享涉及其产品配置与服务 | 借鉴“布局服务任务”、图像与曲线联动、详情按需显示。E6数据要同时显示源帧时间与年龄，不能把回看保持的旧值画成当前有效。暂不嵌入需额外服务／席位的完整产品，也不声称复用其前端代码 |
| [Rerun Blueprint](https://docs.rerun.io/dev/concepts/visualization/blueprints/)、[时间轴](https://docs.rerun.io/dev/reference/viewer/timeline/)。2026-10-09官方dev文档阅读，非本机亲测；这是文档版本，不冒称固定SDK资格 | 布局与记录分开；视图、选择详情、时间面板各有职责，能按时间与实体检查多模态数据 | 借鉴独立视图状态、统一样本游标和按需详情。本轮继续phosphobot页面与现有记录格式；尚无Rerun SDK导入或替代记录器，不把设计借鉴算代码复用 |

## 实际复用的应用底座

[phosphobot a5de051](https://github.com/phospho-app/phosphobot/tree/a5de051197c879e3c685d1362f649ce54ee47a3c)，MIT，0.3.134，已经在本机完整构建和运行；详细调用及补丁见OPEN_SOURCE_REUSE.md。

- 保留`dashboard/src/pages/control/KeyboardControlPage.tsx`、`GamepadControlPage.tsx`，通过已有E6分支提供输入，不建另一个平行控制页。
- 继续使用上游`speed-select.tsx`、`recorder.tsx`及相机卡片；视觉改进复用现有Button、Card、Tabs、Tooltip、Dialog和Tailwind设计变量。
- `frontend/EpisodeBrowser.tsx`已经实际使用上游Radix/shadcn Dialog，修正历史片段列表挤走视频的问题。这个改进有既有v150正式页证据；不能据此宣布新整页设计通过。
- 已有WebRTC官方reader及原始记录链路保持。报告保存经过Blob链接失败对照，改为由本机FastAPI保存、回执后提供Starlette FileResponse链接；正式浏览器下载与本机原文件SHA一致。48项前端测试和构建通过，细节及同源检查的证据边界见VIDEO_REPORT_SAVE_QUALIFICATION.md。

## 设计决策

1. **一个主要操作视图。** RGB占主要区域；输入、模式、坐标、速度和停止入口在旁侧固定位置。主视图始终标明影子／实机，显示线缆限制内的受测范围。验收脚本与大段JSON收进诊断区域，不能挡住视频。
2. **记录任务清楚、手续简短。** 任务描述与场景备注可复用上一段；不强制实名。开始、成功结束、失败结束、中止保留沿用当前真实语义；“封存”“回传”“校验完成”分别显示，不能点击结束即称保存成功。
3. **数据检查使用独立模式。** 片段列表按结果、日期与任务筛选，选定样本统一驱动图像、输入、发送／确认、实际反馈详情。无效关联与未知值保留。回看图像明确标为历史，退出浏览不会续动；不增加机器人自动回放。
4. **恢复以影响范围呈现。** 网络、机械臂、夹爪、相机、存储各自显示健康状态；恢复入口说明将做什么。模型或录制错误不终止反馈读取。普通释放与方向边界属于正常状态，不显示满屏红色故障。
5. **可见功能必须可用。** E6配置下将未适配的训练、自动回放、VR、校准写入入口移出主要导航，并说明能力范围；保留上游模拟路径。不能让可点击菜单暗示已完成实机能力。
6. **主界面不承担完整诊断渲染。** 统计按需展示、记录详细数据供问题包下载；避免每帧序列化长历史、每轮重绘大曲线。视觉变更只复测受影响项；正式输入生命周期变化必须补真实链路测试。

## 实施与验收状态

调研与上述取舍已形成；正式工作台整体改版尚未开始。下一步在静止／影子会话使用真实状态与片段，落实布局、任务元数据和导航，再作既定分辨率／缩放／焦点走查。实机只使用用户新限定的已验证局部空间，不为界面演示扩大动作。最终仍以WORKBENCH_UX_PLAN.md的工作流和性能要求验收。
