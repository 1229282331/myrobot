# 内嵌反馈头：复现、修复和边界

2026-10-06。v12只读session的85627个原包中，序号18061含两个有效长度/魔数头：偏移0时间戳1791299307804，偏移80时间戳1791299307953。后半数据因此整体错位80字节。官方MyType/FeedbackParser也把电流读作QActual；数值有限、旧头魔数正确，原校验漏检。

这不是已证明的101°实机跳动或碰撞。该帧的RunQueuedCmd=32、CollisionState=32同样来自错位；前后正常帧维持静止。原坏帧、前后四帧和旧collector巨大峰峰值保留，不能把旧派生统计写成真实运动。异常位于录制结束之后；6821帧的episode原数据没有该帧。没有对应PCAP，尚不能断言截短源头在控制器、驱动或应用。

`FeedbackStream`现在检测包内另一个长度/魔数头，拒绝发布，停止使用当前连接并重连；不删除80字节后猜测修复。错误保存拒绝缓冲区前65536字节、总长度和完整SHA（超长明确标注截断）。`FeedbackReader`独立保持生命周期，网关把连接/拒绝事件送入审计和活动episode。会话不能因重连自动续动，既有代际检查与显式准备保持。

原坏帧五种拆包位置、普通payload中单独1440值、拒绝证据及新连接恢复测试通过。47项相关和113项完整后端通过。影子部署v13framing约151秒，18458原包、无运动/Stop/CAN，关节峰峰最大0.009613°，没有再次发生该包型；短期未复现不等于设备源缺陷已修复。新相机正常关闭，旧tmpfs健康发布退出补丁也在本轮部署。

证据：`evidence/v12_feedback_outlier_neighbours.json`、`feedback_nested_header_tests.xml`、`rebuild_feedback_framing_full.xml`、`executed_shadow_v13framing/sha256.json`、`shadow_v13framing_input_timing.json`。完整原包仍在runtime/shadow-v12paced及shadow-v13framing。修复只涉及TCP结构与诊断，没有运动学近似或关节异常阈值过滤。
