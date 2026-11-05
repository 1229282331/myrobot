# 控制面板与原生日志台（v243，2026-10-11）

本轮只交付界面与日志功能，完整 Goal 保持暂停。未准备控制、未使能、未初始化夹爪、未主动发送运动。

## 改动

- 主面板保留操作、固定两行状态区、实际 TCP、夹爪实际开度，以及关节模式下的实际角度。删除候选/接受目标、停止回执、帧号和设备诊断详情。详细映射进入 Controls reference；官方可交互输入按键仍保留。
- Camera recovery 移到已有 CameraStreamCard 旁；问题包导出移到日志工具栏的 Popover。
- 实际扩展 phosphobot 0.3.134 / `a5de051197c879e3c685d1362f649ce54ee47a3c` 的 `dashboard/src/components/custom/LogsStream.tsx`，保留原训练日志调用兼容性。页面底部只挂载一个日志台，默认收起，展开后输出区固定 256 px。支持 Control / PC / Camera / Servo、暂停跟随、复制、清空显示、下载。无新 UI 库或控制连接。
- 前端按原文显示文本，UTF-8 流式解码，最多 5,000 行且 2 MiB；没有把 JSON 审计事件转换成网页卡片。收起会取消请求，展开/切换来源读取该来源最新文本。
- 程序使用标准 Python logging 和容量 1,024 的非阻塞队列。格式化、状态变化去重、堆栈处理及写盘在日志线程进行。队列丢弃和写入失败不进入控制故障路径。高频输入、运动目标、反馈时序为 DEBUG；默认 INFO 输出连接、准备、状态变化、停止、边界、夹爪和记录结果。启动输出当前 TCP、速度和范围。
- Servo 的 JSON stdout IPC 与结构化审计保持不变；Servo 日志包含原生 ROS/MoveIt stderr 和独立的人读日志。

## 通道与文件

网页 GET `/e6/logs` 查询来源；GET `/e6/logs/{source}/stream?cursor=…` 获取最多 64 KiB 原始文本及下一游标；空闲轮询间隔 500 ms；GET `/e6/logs/{source}/download` 下载打开文件时的长度快照。开始浏览只载入最近 64 KiB，继续追加直到显示缓存上限。游标包含会话、文件身份、偏移及尾部锚点，检测换文件、截断和重新增长。

来源只接受四个固定 ID，浏览器不能指定路径。PC 通过既有 8777 媒体 SSH 转发读取 X5 的日志 HTTP 服务，未复用控制消息连接。文件读取与下载不获取运动/模型锁、不读取设备。

| 来源 | 会话登记文件 |
|---|---|
| Control | X5 当前会话 `control.log` |
| PC | 当前 PC 应用的 `runtime/logs/pc-{session}.log` |
| Camera | X5 当前会话 `camera/human.log` |
| Servo | 当前模型子进程目录 `servo.log`，原生 ROS 输出保持原文 |

日志读取不删除或修改文件。原始 `.jsonl` 审计继续独立保存；问题包导出仍按原有结构化字段脱敏策略执行，单独的日志下载为本地原文。

## 验证

- 83 项相关前端检查通过：49 输入生命周期、19 页面、9 问题包、6 日志流。覆盖分块中文、多行堆栈、断线续读、轮转重置、暂停滚动仍接收、清空不回放、来源切换和卸载取消、字节/行双上限、训练接口兼容、日志焦点不发允许动作、不创建第二控制连接。
- 96 项相关后端检查通过；PC 日志线程改为应用生命周期内启动后，13 项相关检查再次通过。覆盖有界队列、阻塞写入/写盘失败隔离、路径白名单、文件轮转/copytruncate、原文下载及媒体/PC 转发。
- 正式 TypeScript/Vite 构建通过；九个上游补丁在独立 Git index 中干净应用，再运行准备脚本也通过。改动保存在 canonical frontend 与 `0009-native-log-console.patch`，未只修改生成目录。
- 真正页面检查四路文件可读、复制与原文一致、清空不再显示历史文本。浅/暗主题及 1280×720、1920×1080、原窗口均无横向溢出；恢复原深色主题与窗口尺寸。日志开合前后操作卡片文档位置均 401.333 px、高度均 489.333 px；日志输出高 256 px。
- 最终只读回读：未准备、静止、录制 idle、模型 ready、四路来源 available。最终会话检查到 1,609 周期、817 中立输入，允许动作计数 0，运动/Enable/CAN/录制事件 0。此为本轮截面检查，不能替代运动验收。

首次测试中的两个 Windows 换行夹具断言、一个旧面板文本精确查询断言已修正。最初使用不完整测试环境导致 scipy/phosphobot 缺失，改用项目已有完整上游环境后通过。没有改运动判定来绕过测试。

## 当前会话与回滚

v242 会话正常 clean 退出，旧独立 PC 进程通过其专用控制台 Ctrl-C 退出；第一版 v243 会话也正常 clean 退出，PC app exit 0。最终运行 `20261011-220215-9b859051`，launcher exec `2963`，入口 `output/native_console_v243b.py`，正常退出文件 `output/native_console_v243b_live.stop`。

沿用 workstation.json：平移 50 mm、相对旋转/关节上限 null、官方关节绝对范围两端各内缩 5°；普通档 30 mm/s、10°/s，原 v236 二进制 SHA 不变，合成输入关闭。夹爪重建会话后未知，等待用户独立初始化。本轮没有改范围、控制协议、CAN 或 TCP。

原构建保存在 `output/native-console-v243/dist-before`；可在未准备、静止且录制结束后用其恢复服务的静态资源目录，再刷新页面。新的只读日志后端可保留。其余旧变更、历史记录和失败证据未清理。

证据目录：[evidence/native-console-v243](evidence/native-console-v243)。包含构建/测试输出、源码 SHA、原始日志片段、会话切换状态、动作计数及截图；完整现场副本和回滚构建留在 `output/native-console-v243`。

![控制区](evidence/native-console-v243/final-controls.png)

![日志台](evidence/native-console-v243/final-logs.png)
