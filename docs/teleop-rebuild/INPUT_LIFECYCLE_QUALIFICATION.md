# 输入连接交接修正

2026-10-07，v43。此轮只验证正式React输入组件的行为，没有连接实机或声称通过真实运动验收。

## 复现、改变、结果

假设：输入源切换关闭旧WebSocket时，旧连接的迟到回调仍可能作用于新连接。结果：在`d6576fd`的正式`useE6Control`中复现。新连接已收到可控制状态，旧连接的关闭事件会将页面状态改为未连接，同时清空键盘输入；旧message也没有连接身份检查。这能造成短暂错误状态和按键丢失，不将其认定为此前所有实机不动问题的唯一根因。

修正只在正式hook的message/close回调中核对当前WebSocket身份；message还要求连接处于OPEN。后端控制、停止、速度、租约和设备参数均未改变。

采用固定Vitest3.2.4、React Testing Library16.3.0、jsdom26.1.0运行实际hook和上游GamepadUtils，React19.1.0、SWR2.3.4与当前工作台一致。只替代网络WebSocket和手柄硬件，不复制输入映射。依赖与完整传递依赖固定于 `teleop-platform/frontend-tests/package-lock.json`，测试没有设备或CAN网络入口。

7组测试通过，覆盖：

- 100次按下/释放：发送正确活动与中立意图，不额外发送会话停止，不重建连接。
- 20次回中后的模式/坐标切换；按住允许动作时拒绝切换。
- 20次键盘/手柄输入源交接：旧连接关闭及消息不能覆盖新状态或清掉新按键。
- 10次断连/显式重连：断连后不创建新连接；点击准备后等待新状态，先发中立意图，再发准备，不携带旧轴输入。
- 输入框、文本区、下拉框获得焦点时释放键盘输入，编辑快捷键被屏蔽；离开后可再次输入。
- 手柄移除发送停止，重新出现后不自动准备。
- 页面失焦停止，并清除键盘按住状态。

这些次数是DOM测试中的事件次数，不是无线、真实浏览器或实机启停次数。控制器实际动作、物理停止、真实手柄、第二页面争抢和最终30分钟标准仍需正式整机验收。

## 证据及复现方式

`evidence/input-lifecycle-v43/initial-tests.xml`保留首次测试环境缺少getGamepads的7项设置失败，未计为产品故障。补齐jsdom缺失的手柄API后，`reproduction-tests.xml`为6通过、1个真实旧连接回调复现失败。修正后`final-tests.xml`为7通过。没有删除或改判失败。

旧hook取自提交d6576fd并保存为before-useE6Control.ts；最终源码、依赖锁和制品SHA均归档。`prepare_upstream.py`已将正式扩展复制到固定phosphobot，`tsc -b && vite build`通过，输出index-FGIsqH6I.js。本轮不为前端生命周期改动重复长时实机测试。

```powershell
cd teleop-platform/frontend-tests
npm ci --ignore-scripts
npm test
```

下一步仍是当前boot的独立使能与新Jazzy正式页面实机对照，等待已提出的现场事实答复。本轮软件结果不放开尚未验收的实机能力。
