# 本机 RGB-D 相机测试

用户购入称为 DaBai DCW 的相机。2026-09-27 实物由官方 SDK 识别为 **Gemini E**，SN `AY4A1520111`、固件 `RD3481`、深度 PID `065C`、彩色 PID `055C`。目录按用户要求保留 `dcw/`。官方 USB 表分别列出 DCW 与 Gemini E，尚未确认两者的商业别名/地区版本关系；采集始终展示设备真实回读名称。

## 使用

- **双击 `launch.cmd`**：同时预览 RGB 和深度。按 **Q / Esc** 或关闭预览窗口停止并释放相机。
- **双击 `test_camera.cmd`**：运行 60 秒双流测试，保存帧率、逐帧元数据和一组图像快照。
- **双击 `viewer.cmd`**：打开配套官方 OrbbecViewer v1.10.37。使用前先关闭其他采集程序，避免占用相机。

预览左侧 RGB 为 1280×720 MJPEG @30，右侧深度为 640×480 Y12 @30（SDK 解包为 uint16）。实际帧率显示在窗口内。图像保持相机原始方向；当前未旋转、未做 RGB/深度像素对齐、未做硬件同步。深度伪彩固定显示 0.2–2.5 m，黑色表示无效值；色带范围不是本机实测量程或精度保证。

第一次插线后若只有 RGB 而没有 `ORBBEC Depth Sensor`，安装官方驱动并重新插拔 USB。应由主机直连接口供电；不用 E6 或夹爪供电。

2026-09-27 曾出现移动相机后 USB 设备消失、随后采集超时；用户更换数据线后反馈恢复。详见 [移动断连与换线复核](docs/移动断连与换线复核_2026-09-27.md)。安装时固定线缆并留活动余量，避免插口承受拉力。

## 文件位置

| 路径 | 内容 |
|---|---|
| `.runtime/` | 本目录独立 Python 3.11 运行时和包环境，NumPy 2.2.6、OpenCV 4.12.0.88 |
| `vendor/` | 官方 SDK、Viewer、驱动包 |
| `downloads/` | 可重新下载的 SDK 安装压缩包 |
| `tools/obsdk.py` | 按 v1.10.37 C 头文件编写的最小 ctypes 绑定及设备探测 |
| `tools/followup_profiles.py` | 有界IR分辨率/RGB640×360对照，详见RGB帧率与IR亮点复核记录 |
| `tools/camera.py` | 双流采集、预览、统计、超时停止与资源释放 |
| `tools/rgb_probe.py` | 明确指定 DirectShow 索引的 RGB 排障工具，不代表深度可用 |
| `captures/<run_id>/` | 每次采集的报告、CSV，启用快照时还有图像及原始深度 |
| `logs/` | SDK 与启动/安装日志 |
| `docs/evidence/` | 已归档的设备、驱动、验收证据 |
| `tests/` | 不连接相机的解码、深度单位与帧号统计测试 |

项目程序、依赖包、下载文件和采集结果都在本目录。**Windows 驱动必须注册到系统 DriverStore**；官方安装器还创建了 `C:\Program Files\Orbbec\ASTRA`，这是系统驱动安装的部分，不是可完全便携的 Python 依赖。

最终运行时是本目录 `.runtime/` 下的独立 Conda 前缀环境，不依赖 OmniPicker 环境；未往 OmniPicker 环境安装/修改包。

## 命令行

在 `dcw/` 中运行：

```powershell
.\.runtime\python.exe tools\obsdk.py --output logs\device.json
.\.runtime\python.exe tools\camera.py --mode color --seconds 10
.\.runtime\python.exe tools\camera.py --mode depth --seconds 10
.\.runtime\python.exe tools\camera.py --mode both --seconds 60 --snapshots
.\.runtime\python.exe tools\camera.py --mode both --preview --seconds 0
.\.runtime\python.exe -m unittest discover -s tests -v
```

多台设备时必须用 `--serial` 选择目标；单台自动使用其回读身份。模式不支持时直接报错，不静默替换分辨率/帧率。

## 数据语义与边界

- `frames.csv` 每行对应一帧，保留流名、帧号、SDK 设备时间戳、SDK 系统时间戳、主机单调接收时间、分辨率、格式、字节数和深度尺度。接收时刻不等于曝光时刻，两路没有已验证硬件同步。
- 2026-09-27 验收发现：本机 SDK v1.10.37 的 RGB `device_timestamp_us` 增长速率约为主机时间的 15003 倍，不能用来同步；深度该字段与 SDK 系统接收时间一致，也不能当作独立曝光时钟。2026-10-02 已在 Linux V4L2 读到 USB 描述符的 15 MHz 时钟及原始 UVC PTS，验证了单位换算问题，新增可选正确换算字段，详见下文。原 SDK 字段继续保留。`host_perf_counter_ns` 供高精度接收间隔统计，不能替代曝光同步。
- `depth_raw.npy` 为无损 uint16 原始深度。毫米值 = 原值 × 对应 `depth_snapshot.json` 的 `depth_scale_mm`。零值是无效深度，不填补。
- `color.png`、`depth.png` 是各流首次满足快照条件时保存的图像，并非已同步/对齐的一对训练样本；后者只是伪彩。不要拿两图同一像素直接做 RGB-D 对应。
- `report.json` 中 `status=completed` 表示采集正常结束；还应检查 `data_validation`、有效深度帧数、帧率和断流。全零深度不是测距通过。
- 应用层任一路超过 3 秒无新帧即停止，预览超过 0.5 秒变暗并标注 STALE；不自动重新连接续采。SDK 内部可能有短暂重试，日志保留。
- 不升级固件、不清除标定、不写 Flash、不关闭 LDP，不连接或操作机械臂/夹爪。流启动/停止是相机正常采集操作。
- Windows 与 X5 的双流连通和有界采集已分别验证；测距精度、RGB 近焦、30 分钟稳定性、标定和机械臂时序同步仍未完成验收。

## 重建环境

需要 Windows x64 与已安装的 Conda。关闭所有本目录程序后执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Conda 'C:\ProgramData\miniconda3\condabin\conda.bat'
```

setup 从官方 v1.10.37 发行页下载 SDK 和 Viewer，使用本轮本地记录的 SHA-256 固定版本，创建本目录独立 `.runtime` 运行时并安装锁定依赖；不会自动改固件。驱动在 `vendor/orbbec-v1.10.37/viewer/driver/SensorDriver_V4.3.0.22.exe`，只需安装一次。本机已通过官方 INF / `pnputil` 安装为 `oem22.inf`（4.3.0.22）。

来源：[官方 v1.10.37](https://github.com/orbbec/OrbbecSDK/releases/tag/v1.10.37)、[官方 Windows 驱动说明](https://github.com/orbbec/OrbbecSDK/blob/v1.10.37/doc/tutorial/English/Environment_Configuration.md)、[官方 SDK 支持列表](https://www.orbbec.com/developers/orbbec-sdk/)。

## 二手验收工具

`tools/healthcheck.py --inspect` 只读枚举传感器、模式、属性权限和出厂参数。`--preset both/color/1080p/depth/ir/d2c --seconds N` 显式启动对应流，记录逐帧元数据、逐秒质量统计、开始/结束状态及快照。`d2c` 使用原生 640×360 硬件对齐模式；正式预览仍不对齐。`--poll-status` 仅用于状态关联排障，控制查询会影响接收间隔，不用于帧率验收。

计时前有明确记录的 1 秒接收预热；PNG 压缩与写盘在独立线程，缓冲有界。`tools/analyze_health.py captures/<run>` 离线检查帧间隔和 SDK 时间戳速率。内参查询需在实际双流启动后使用；未启动/仅 IR 时的管线参数不能作为标定丢失证据。深度时序波动只有在相机与目标均静止时才能解释为重复性。

完整项目与待完成实物验收见 [二手相机验收](docs/二手相机验收_2026-09-27.md)。

## RDK X5 采集

`tools/obsdk.py` 现在支持 Windows DLL 与 Linux ARM64 共享库。Linux 使用官方 **v1.10.37 ARM64** 包，不能换成不支持本机 Gemini E 的 v2 包。将 SDK 解压到独立目录，保留 ZIP 内的符号链接；`python -m zipfile -e` 会将这些链接解成短文本文件，导致 `file too short`。

2026-10-02 实机目录为 `/root/installed-audit-20261002_2142/`，SDK 在 `vendor/OrbbecSDK_v1.10.37/SDK/`。使用 X5 已有 Python 3.10.12、NumPy 1.26.4、OpenCV 4.11.0，未修改全局 Python 包或安装系统驱动。`tools/rdk_healthcheck.py` 默认只打印离线预览；显式 `--execute` 后才开流：

```bash
export DCW_SDK_ROOT=/root/installed-audit-20261002_2142/vendor/OrbbecSDK_v1.10.37/SDK
python3 /root/installed-audit-20261002_2142/tools/rdk_healthcheck.py --execute --preset both --seconds 60 --output /root/installed-audit-20261002_2142/new_capture
```

输出目录必须不存在。工具使用独立 SDK 接收线程、128 帧有界处理队列，OpenCV 内部固定单线程；`--decode-workers 1..4` 可选择并行解码数，默认1。最多有 `2×workers` 个待完成解码任务，按接收顺序输出并保持图像与元数据配对；快照在独立单线程保存，最多一个未完成快照。队列满、解码失败或任一路超过 3 秒无帧即中止，不补帧、不自动重连。`processing.csv` 逐帧保存解码耗时与接收到完成的时间。可选 `--stop-file /absolute/path`，文件出现后取消本次采集并释放设备。支持 `both`、`1080p`、`d2c`、`ir-native`（512×384/30）、`ir-high`（1024×768/10）等配置，均明确选择已枚举模式。

默认对每张 RGB 帧解码。X5 上旧单解码线程的1080p双流约9秒触发处理队列满；2026-10-02 新增2个解码worker后，V4L2与原LibUVC各60秒逐帧处理完成，RGB1473/1482张全部解码，队列峰9/8，无帧号缺口。两次实收仅24.55/24.68fps，不能称1080p30fps验收通过。`--color-decode-stride 3` 用于抽样诊断，仍记录每张接收帧元数据，报告区分 `processed_frames` 和 `color_decoded_frames`；未解码图像的内容没有被验证。长期采集优先沿用已有10分钟记录的720p配置，但本轮720p也出现约24.45fps，帧率需按每次实测检查。

接收线程与处理线程使用同一 X5 单调时钟，但这不构成曝光同步。原RGB设备时间戳仍有约15000倍单位问题，不能直接用于同步；SDK系统时间与主机接收时间也不能替代已标定的相机和机器人同步。停止阶段的SDK警告另行保留，不能用 `status=completed` 单独宣称全部正常。

### 可选UVC PTS换算

仅对本台Linux Gemini E的V4L2后端验证。`DCW_SDK_CONFIG` 指定独立SDK XML，默认仍使用SDK原配置。复制原XML，仅将全局 `<LinuxUVCBackend>Auto</LinuxUVCBackend>` 改为 `<LinuxUVCBackend>V4L2</LinuxUVCBackend>`；不修改原SDK文件或相机属性。本轮两份配置与源码位于 `/root/remaining-issues-20261002_2226/`，可直接复现：

```bash
export DCW_SDK_ROOT=/root/installed-audit-20261002_2142/vendor/OrbbecSDK_v1.10.37/SDK
export DCW_SDK_CONFIG=/root/remaining-issues-20261002_2226/config_v4l2.xml
python3 /root/remaining-issues-20261002_2226/tools/rdk_healthcheck.py --execute --preset both --seconds 320 --decode-workers 2 --uvc-pts --output /root/remaining-issues-20261002_2226/new_pts_capture
```

程序按SN匹配RGB USB设备，读实机UVC描述符时钟，解析SDK提供的Linux UVCH元数据。新增 `uvc_pts_32`（原始32位计数）、`uvc_pts_unwrapped`（展开回绕后的计数）、`uvc_pts_clock_hz`、`uvc_pts_ns`（计数×1e9/实读时钟）、`uvc_pts_wrap_count`、`uvc_kernel_receive_ns`（内核接收时刻）、`uvc_pts_available`。原SDK字段原样保留。无metadata时图像仍保留、PTS为空；损坏/不一致metadata、逆跳/重置或超过3秒跳变中止采集，不制造时间。只支持连续采集内的回绕展开，重连后重新建立时钟。

320秒实测7825张RGB全部取得PTS，跨一次约286.331秒的32位回绕后严格递增，9607张深度均有效、两路无帧号缺口。稳态计时速率比约1.000107，仍有设备时钟漂移及接收延迟；`uvc_pts_ns` 不是UTC/主机时刻，也未验证为曝光时刻、RGB-D同步或机器人同步。Windows和LibUVC没有启用这条修复路径。

根因、代码、原始证据与其他待解决事项见[剩余问题公开资料与修复验证](../docs/剩余问题公开资料与修复验证_2026-10-02.md)。

本轮反复使用SDK的LibUVC后，曾出现控制接口部分绑定、流接口未绑定及视频节点消失。已确认无SDK进程/USB句柄后，仅重绑定已识别的RGB控制接口，恢复uvcvideo和视频节点并通过V4L2采图。当前SDK工具并不自动执行这项内核绑定修复，后续切换V4L2前应检查节点与占用。过程及完整结果见[安装后联合验收](../docs/整机安装后夹爪与相机联合验收_2026-10-02.md)。

`tools/geometry_check.py captures/<run> --second 30` 可从健康检查保存的深度快照和同模式内参导出米制PLY点云；零值排除，不补洞。仅适用于本机当前未镜像、深度畸变项为零的回读参数。可用 `--plane-roi X Y W H` 对人工确认的平面区域做连续性诊断；平面残差不等于绝对测距误差。
