# 起动回调唤醒验证

2026-10-09，v166／v167。已缩短部分软件等待，尚未证明完整平台达到浏览器输入至实机首动P95≤200ms。

## 假设与修改

v160四组正式输入中，X5收到输入至首发ServoJ为57.7–114.3ms，首发至实际任一关节变化超过0.01°约115.7–138.2ms。浏览器至实机首动P95区间258.85–278.26ms，原失败及阈值保持。

隔离适配器已有首候选唤醒网关，不重复添加该机制。本次发现ROS服务完成、首条有效轨迹到达后仍可能等待下一次10ms定时pump。复用rclpy GuardCondition，在这两个事件上唤醒原pump；由同一单线程executor串行处理，并保留原新鲜度、epoch、来源、暂停状态和模型检查。后续轨迹不触发额外唤醒；20Hz发布／设备发送间隔、官方MoveIt算法、ServoJ(t=.1, aheadtime50, gain500)、停止逻辑均不变。

不缩短原暂停转换间隔、不外推目标、不修改统计阈值。源码修改位于`jazzy_worker.py`，没有修改官方Servo二进制。

## 同机虚拟对照

`qualify_jazzy_startup.py`运行生产MoveItBridge与固定Jazzy节点，使用既有150ms一阶虚拟plant；没有导入设备SDK或连接E6／CAN。各32次450ms输入，覆盖J6、基座旋转、工具旋转及反向。每次释放后检查旧候选不可取，后续目标保持20Hz。完整输入／候选／运行时诊断、执行源码与SHA均归档。

| 指标 | baseline | callback-wake |
|---|---:|---:|
| 首候选生成P50／P95 | 52.88／62.99ms | 48.58／56.77ms |
| 首候选到主进程P50／P95 | 56.14／68.67ms | 52.48／60.70ms |
| 到主进程最大值 | 151.68ms | 88.46ms |
| 产生有效候选／释放清空 | 32／32 | 32／32 |

两个顺序实验仅支持这组有限对照，不能保证所有无线／负载条件都有相同收益。冷启动样本保留在统计中；没有删除最大值。虚拟plant不是E6辨识模型，结果不计入实机主动时间、速度或停止资格。

## 正式页面影子复核

v167会话`20261009-014156-b29d5cf4`，固定phosphobot正式键盘页、无线PC↔X5、真实相机与原始E6反馈同时运行。六次0.5秒输入为基座Rx±、工具Rx±、J6精细±。55个候选及完整窗口来源均匹配正式输入，来源错误0，方向余弦中位数最小0.9999786。六组首候选生成P95=56.74ms，到主进程P95=65.73ms；控制周期首次呈现候选P95=83.57ms只是采样上界。没有设备接受／实机运动结果，不替代实机复核。

24266条官方SDK原始反馈全部有效、缺号0，全部Mode5／Enable1／Error0／Collision0。运动、Stop、CAN发送均0。正常退出exit0，PC／X5自有连接、相机、ROS和proc挂载均释放，CAN DOWN、RX/TX196367未增加。保持原使能，不停放或下机。

同轮补验PC视频报告的最终同源保护：正式页面保存成功，原报告695个样本，P95区间178.54–215.31ms；短时观测不代替最终长时视频资格，解码丢帧4原样保留。原字节与页面截图见v167目录。

## 回归与后续

49项起动／桥接专项通过；完整后端460通过、5平台条件跳过。前端代码未改，沿用前轮48项与正式构建，不虚称重新测试。

后续实机仅在用户已走过并验证的局部路线内检查新组合，不能借prepare重设基准累计扩大扫掠。大范围仅影子／模拟。仍需实机起动与停止、组合记录、最终稳定性及产品化工作台验收；本报告不解锁更大运动空间。

证据：`evidence/startup-v166/{baseline,callback-wake}`、`evidence/live-jazzy-v167`。分析命令：

```powershell
python teleop-platform/scripts/analyze_page_motion.py docs/teleop-rebuild/evidence/live-jazzy-v167
python teleop-platform/scripts/analyze_shadow_startup.py docs/teleop-rebuild/evidence/live-jazzy-v167
python teleop-platform/scripts/audit_moveit_page.py docs/teleop-rebuild/evidence/live-jazzy-v167/session/x5/audit --provenance-profile jazzy-timed-window --output docs/teleop-rebuild/evidence/live-jazzy-v167/candidate-audit.json
```
