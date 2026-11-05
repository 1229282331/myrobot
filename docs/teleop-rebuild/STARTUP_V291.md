# v291 重启后使能与平台接通

v291 重启后使能与平台接通（2026-10-18）覆盖当前：用户要求使能并打开平台。新boot cfda066e-836d-4c3d-9faa-74493a10e4fd，约39GiB可用；726只读SDK帧全Mode4 Enable0无报警碰撞/队列，RGB/深度新采集成功并已查看。开机COM回零。独立官方SDK按现有Payload策略仅发送一次EnableRobot(0.750000,0,-14,46,0)，0回包并核验实际负载/COM和新5秒稳定窗口，1681原SDK无缺号报警碰撞；使能J4+0.856934/J5−0.855560/J6−0.690765°等变化保留，不称零位移。随后启动唯一日常operator20261018-123541-e9850927/exec32399，入口output/startup_operator_291.py operator，stopfile output/startup_operator_291_operator.stop尚未建立；自有PC服务由正常owner管理，不复用旧共享前端。20新状态未准备静止idle/Mode5 Enable1 Error0 Collision0/ID1，峰峰max0.002747°；夹爪自动反馈连接ready/raw0/sent_raw=null，无位置初始化；视频及模型反馈有效、合成输入关闭。原模型b4e1f8bc/v286 cb4055a9/B安装/TCP151.4/负载0.750与COM(0,-14,46)/v289 Force255及速度柔化范围保持。未Prepare/主动臂运动/开合/清错；本轮仅使能及在线静止，不是新运动验收。浏览器URL安全策略阻止自动访问页面，未绕过，给用户正式链接/刷新入口，不称完成页面视觉验收。完整Goal仍paused。证据evidence/startup-v291、报告STARTUP_V291.md。下方关机和旧会话均历史，下一动作新读。

平台入口：http://127.0.0.1:8765/control?tab=gamepad

退出沿用当前owner的stopfile，先新读未准备、静止及录制idle；不复用旧stopfile。新启动和使能原始回读、来源散列、模型/配置及相机图像保留在证据目录。启动初期状态接口503随后随gateway就绪恢复，未为此重复使能。
