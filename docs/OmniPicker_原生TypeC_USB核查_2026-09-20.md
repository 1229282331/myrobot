# OmniPicker 本体 Type-C：官方 USB 实现核查

日期：2026-09-20。适用对象为当前官方手册对应的 OmniPicker/REF-CLI，用户照片 SN 指向 20 PCBA。未取得实物 USB 描述符、设备导出的接口定义或运行数据。本轮未运行厂商 EXE、未导入其夹爪通信模块、未连接任何硬件。

## 结论与证据级别

| 问题 | 本轮结论 | 证据范围 |
|---|---|---|
| 本体 Type-C 能否直接控制夹爪 | 能。官方 REF-CLI 支持 `ref0.motor.ctrl.set_pos(0.5)` 等中间开度命令 | 官方手册第 6.2.3 节 |
| 官方上位机是否通过 USB-CAN 桥接 | 已查明该调用路径为 PyUSB → USB Bulk → Fibre 远程对象访问，不是六芯口 CAN 帧封装 | 官方 EXE 的静态字节码分析 |
| 本体 Type-C 是否还有未公开 CAN 桥接功能 | 未找到公开依据。没有全量分析夹爪固件，不能将未发现上升为所有硬件/固件绝对不可能 | 公开资料与本轮检查的边界 |
| RDK/Linux 是否具有实现路径 | 有。USB 传输层包含非 Windows 分支、内核驱动分离及 udev 权限错误处理，Python/libusb 可作为移植方向 | 软件结构证据，不是 ARM64 实测 |
| 是否有现成的原生 USB Linux/Python 即用 SDK | 本轮未核实到。官方入口是 Windows EXE，不能在 RDK 上直接运行；也不能保证 `pip install fibre` 就是匹配的实现 | 官方入口与公开仓库排查 |
| 是否能读取属性/反馈 | 软件具有远程属性读取机制，设备上线时通过协议端点 0 读取 JSON 接口定义，动态创建对象/属性/方法 | 具体实物的实际开度、速度、扭矩字段/单位/采样率仍需设备接口定义验证 |
| 是否能反复更新开度 | 官方已支持 USB 位置命令，可作为逐周期控制的开发入口 | 未实测重复下发频率、延迟、丢包、在途目标更新及长期稳定性 |

因此不必为“通过本体 Type-C 控制开度”再购买 USB-CAN。可以把原生 USB 作为到货后优先验收的路径，同时保留已有 RDK 板载 CAN 作为有明确帧协议的替代方案。

## 官方软件静态证据

下载来源：[REF-CLI v1.0.3.exe](https://www.zhiyuan-robot.com/file/ueditor/php/upload/file/20250114/x1/REF-CLI%20v1.0.3.exe)，链接来自 [OmniPicker 官方手册](https://www.agibot.com/filepage/265.html)第 6.2.1 节。

- 文件大小：37,964,212 字节。
- SHA-256：`f90f4a90218ea63983fc1395362604236d159e2c3d49683e033a3bd272ad9b18`。
- PyInstaller 归档标明 Python 3.10；提取归档和 PYZ 索引、解压选定模块，再用 xdis 6.3.0 跨版本反汇编。
- 只把字节码作为数据分析，没有执行厂商程序或其模块。

关键模块与可复查文件保存在 `research/grippers/omnipicker_usb_2026-09-20/`：

| 模块/文件 | 静态核查结果 |
|---|---|
| `main.data.dis.txt` | 官方入口导入 `fibre`，调用 `launch_shell`，交互对象品牌名为 `ref`/`REF` |
| `fibre.usb_bulk.marshal.dis.txt` | `USBBulkTransport` 导入 `usb.core`/`usb.util`，发现 USB 接口并选取 IN/OUT 端点，调用端点 read/write |
| `fibre.discovery.marshal.dis.txt` | `search_devices` 调用 USB 发现；协议端点 0 返回 JSON 接口描述，据此构建 `RemoteObject` |
| `fibre.protocol.marshal.dis.txt` | 远程端点操作带序号、ACK、接口 CRC 和读写数据，属于 Fibre 对象访问协议 |
| `fibre.remote_object.marshal.dis.txt` | `RemoteProperty.get_value/set_value` 和 `RemoteFunction.__call__` 将属性读写及方法调用映射为远程端点操作 |
| `fibre.shell.marshal.dis.txt` | 创建交互 Python/IPython 环境，暴露远程设备对象，并提供配置、升级等命令 |

USB 层的有效 VID 列表是 `[0x1001, 0x0D28]`，忽略 PID `0xDF11`。这是软件扫描规则，不是用户这台夹爪的实测 VID/PID；后续要从实物描述符确认，不能盲写 udev 规则或假定一个固定 `/dev/ttyUSB0`。

**连接副作用：**`USBBulkTransport.init()` 在非 Windows 系统分支调用 `dev.reset()`，并可能分离已绑定的内核驱动。因此直接运行原始发现/连接函数不是严格无副作用的“只读检查”。到货后应先只读取系统枚举信息，再检查并适配连接逻辑；不能启动任意现成例程扫描全部 USB 设备。

## 公开项目排查：不要混淆 USB-CAN 与原生 USB

- [zyf617/OmniPicker_usb2can](https://github.com/zyf617/OmniPicker_usb2can/tree/c5a0a824c08a08ab331a561926edca885155e59b)：README 明确使用外置 USB-CAN；源码打开 `/dev/ttyUSB0` 并发送 `AT+ET` 给透传适配器。不是插本体 Type-C 的例程。该构造函数也会发送配置命令，不适合作为只读发现工具。
- [motor-and-motion-control-lab/OmniPicker_SDK](https://github.com/motor-and-motion-control-lab/OmniPicker_SDK)：基于 Linux SocketCAN 的 C++ SDK。
- [HysteriaC/omnipicker_ros2](https://github.com/HysteriaC/omnipicker_ros2)：README 标明测试硬件为外置 ZQWL-UCANFD-100K。
- [ruiofang/OmniPicker](https://github.com/ruiofang/OmniPicker)：Python/PyQt GUI，后端为 SocketCAN 或 RS485。
- [KLMmotion 示例集合](https://github.com/KLMmotion/KernalMind-OmniGripper-Control-Examples/blob/main/README_CN.md)：USB Python/C++ 示例对应另一个 OmniGripper；智元 OmniPicker 章节是 CAN ROS2。不能混用两者。

## 对本项目的实际接线与下一步

```text
RDK X5 的 USB Host ── USB 数据线 ── OmniPicker 本体 Type-C
独立 24V 电源 ── 原六芯线的 VIN/GND ── OmniPicker
原六芯线 CAN_H/CAN_L 暂不连接，散尾绝缘
```

本体 Type-C 通信路径不需要 CAN 接口切换、不要求为了使用 USB 把 20 板改焊为 RS485。六芯线的 CAN 选择与 USB 数据路径应分开理解。

这仍然是两路物理连接：USB 通信线与六芯供电线；本轮没有增加 Type-C 单线电机供电证据。沿机械臂可以将两路整理为同一束，保留各自接头和电气隔离，不等于一根普通 USB 线承担 24V 电机供电。

到货后的合理顺序：

1. 固定夹爪并预留闭合空间，按独立台架条件接电；官方默认上电自动闭合，不能当纯静态过程。
2. 用官方 Windows REF-CLI 确认原生 USB、固件信息及中间开度行为。
3. 在 RDK 上先读取 USB 枚举/描述符，明确 VID/PID、配置、接口、端点和绑定驱动。
4. 提取实际设备导出的 JSON 接口定义，核对真实状态属性及其单位；没有反馈字段时不伪造反馈。
5. 用匹配的 Fibre/PyUSB 实现编写最小硬件适配，去除自动复位/自动运动等不合需要的默认行为，再测周期下发、时间戳、超时与反馈。

本轮交付的是可追溯的协议核查，不是已经能在 RDK 运行的驱动或已验收的控制频率。
