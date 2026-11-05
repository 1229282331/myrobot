# 第三方来源

本项目局部采用 phosphobot 的 standard Gamepad API 提取方式，重新实现 E6 末端控制、允许动作、租约和采集语义，不维护 phosphobot fork。

- phospho-app/phosphobot，固定提交 `a5de051197c879e3c685d1362f649ce54ee47a3c`，参考 `dashboard/src/pages/control/GamepadUtils.ts`。Copyright (c) 2025 phospho，MIT；完整许可证保存在 [assets/phosphobot_LICENSE](assets/phosphobot_LICENSE)，源快照在根项目 `research/teleop_survey_20261004/sources/phospho-app__phosphobot/`。
- Dobot-Arm/DOBOT_6Axis_ROS2_V4，固定提交 `ec201c40d5db63185a3593ec9c39f80fe25aa527`。原始 E6 URDF 和七个 STL 保存在 `assets/e6/`，上游 MIT 许可证及 SHA256 清单随资产保存。没有将模型转换结果称为实机 TCP 或绝对精度验收。
- Hugging Face LeRobot，固定提交 `8c920c4270460851cedd2737657584586d3dc66f`，0.6.2，Apache-2.0。通过官方 Dataset v3 写入、finalize 和 PyAV 读取接口使用，不复制 writer。
- Diffusion Policy，参考平移/旋转模式和动作记录设计，没有复制项目代码。
- MuJoCo 3.14.0、React 19、Vite、TypeScript、Lucide、PyAV 和其他依赖保留各自许可证，版本与下载哈希见 `uv.lock` 和 `web/package-lock.json`。LeRobot CPU H.264 编码通过 PyAV 的 libx264 完成；FFmpeg/libx264 的许可适用于其二进制，未将依赖二进制提交到本仓库。
