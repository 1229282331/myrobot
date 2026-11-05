# 当前 RDK C++ 运行时备份

从已验证的本地 `output/final-delivery-v355/release/` 逐字节复制 `servo_cpp_adapter` 和 `cpp-adapter.json`。V356–V358 的后续修改位于正式 Python 源码中；本目录不复制过时的 Python 或重复的模型文件。

- 二进制：ARM64 / RDK X5 Jazzy 环境，SHA256 `d5551ee2cc564dd5d1fbb07141b65a9ce768e432af502e870790c01b6cb1cdc1`。
- pin：SHA256 `682b721c972a4c0b021f88d94727b6b4509800615ef1b3bf169c9b21e57c61a6`。
- 构建源：`../../native/solver-timing-v352/source/`，其中保留上游来源与补丁链。
- 正式配置：`../../config/workstation.json`。
- 本地原始验收报告：`../../../docs/teleop-rebuild/FINAL_DELIVERY_V355.md`。

`manifest.json` 校验本目录的两个发布文件。它们保留现有版本，不能证明新设备已经安装所需 ROS、共享库和相机环境。发布到 GitHub 不触发设备连接、使能、运动或部署。
