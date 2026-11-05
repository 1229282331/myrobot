# OmniPicker 接线与通信：官方资料与实操案例复核

核查日期：2026-09-21。用户要求重新检索网上接线/通信实例。本轮重新读取官方在线手册、知乎实操全文及关键图，取得四个社区项目的 README，没有运行设备程序、连接夹爪或操作 E6。

## 本轮新增结论与纠正

1. 查到“只接一对正负极”的具体实操记述，不能继续把“双 VIN / 双 GND 都接”写成已经证实的硬性必要条件。保留双线是当前稳妥设计建议，官方手册只定义了针脚，并未在所查章节明确要求双针必须同时连接。
2. 本体 USB-C 可直接控制夹爪已得到官方手册与社区截图的交叉支持，不是只能配置、必须再走 CAN。没有新增 USB 单线电机供电或 PD 证据。
3. 官方电压仍为 24V DC；个人案例的 12V 用法不替代产品额定电压，不证明全部版本的力、速度与峰值工况。
4. E6 腕部供电能力、配对插头料号仍未确认。第三方在独立电源上使用成功不构成 E6 取电已验证。

## 实操来源：TonyTnT《智元 OmniPicker 调试》

- 页面：[知乎原文](https://zhuanlan.zhihu.com/p/1929191831956100863)，页面标记编辑于 2025-07-18。
- 硬件自述：NVIDIA AGX Orin + OmniPicker。
- “夹爪信息查看/修改”段落原文：

> 使用线缆从夹爪的typec连接到windows机器，参照官方文档 6.2 节，然后给夹爪上电（通过上面1号和6号接入12v电源）……

- 之后使用官方上位机显示 `ref0`，修改配置，再以 `/dev/ttyTHS0`、115200 串口从 Orin 控制夹爪。
- [接头照片](https://pica.zhimg.com/v2-5c2927cf94268e080d17ae803d03ca4e_1440w.jpg)标注该观察面下 1→6；六根线均为黑色，说明不能把官方示意图的线色视作所有实物的保证。
- [控制板照片](https://pic3.zhimg.com/v2-25e4236c75363f6a30d189f54eb6a54a_1440w.jpg)可见六位拨码开关，按官方规则属于 30 PCBA；照片板上 `Ver1.2` 与 20/30 PCBA 分类是不同字段。
- [USB 上位机截图](https://pica.zhimg.com/v2-c5ff6a2c2f9b99f5a0606704c33786e2_1440w.jpg)显示 REF-CLI Ver1.0.3、已连接 `ref0`、`fw_version=333`、`can_node_id=1`、`motor` 子对象及状态请求/配置方法。

证据边界：作者明确记述用 1/6 单对取电，但图片未显示电源侧完整线束接法、实际电流、负载、温升或长期循环记录。本项目用户照片指向 20 PCBA，不把 30 板个人案例提升为用户实物额定工况验证。

## 官方六针定义

来源：[官方手册 §5.1](https://www.agibot.com/filepage/265.html)。本轮重新下载快照。接口文字为 `6PIN socket with lock at a distance of 1.25mm`，没有列出连接器厂牌/精确料号。

| 针号 | 20 PCBA | 30 PCBA |
|---|---|---|
| 1 | VIN | VIN |
| 2 | VIN | VIN |
| 3 | CAN_H（其他模式 UART TX / RS485 B） | CAN_L（其他模式 UART RX / RS485 B） |
| 4 | CAN_L（其他模式 UART RX / RS485 A） | CAN_H（其他模式 UART TX / RS485 A） |
| 5 | GND | GND |
| 6 | GND | GND |

上表按本轮手册逐项记录；串口引脚随硬件模式/板型使用，不能按 CAN H/L 推导 RS485 A/B。针号按官方与实物核对，不根据照片左右或线色猜测。

主供电规格 §2.4 为 24V DC，§5.1 输入定义写明不超过 24V DC。板上 `30V 5A max` 丝印不能替代产品输入额定值和峰值电流规格。双 VIN/GND 通常用于同网供电，但用户实物内部连通和单针额定电流尚未实测。

## 三条主要接法，以及 UART 案例

### 本体 USB-C

```text
24V DC 电源 ── 六芯口 VIN / GND ── OmniPicker
Windows USB ── USB 数据线 ── OmniPicker 本体 Type-C
六芯口的 3/4 通信线不用，分别绝缘
```

官方 §6.2 明确先供电、再 USB-C 接电脑，REF-CLI 可运行 `ref0.motor.ctrl.set_pos(0.5)` 设置 50% 开度。这个示例是运动命令，本轮未执行。该路径无需外置 USB-CAN。

前轮官方程序静态分析已确认 PyUSB/USB Bulk/Fibre 通信；原生 USB 可以作为 RDK/Linux 移植路径，但目前没有经过本项目实物验收的 ARM64 驱动。不能将“Windows 官方工具可用”写成“直接在 RDK 运行同一个 EXE”。详见 `OmniPicker_原生TypeC_USB核查_2026-09-20.md`。

### CAN / CAN-FD

```text
24V 电源 ── VIN / GND ──┐
                        ├── 原六芯复合线 ── OmniPicker
本地 CAN 控制器 ── H/L ─┘
```

CAN 地参考按接口说明连接，电机回流走电源负极；使用 RDK X5 板载 CAN 可不买 USB-CAN。20 板默认焊接为 CAN，30 板默认拨码 3/4 ON。经典 CAN 为 1 Mbps、80% 采样点；CAN-FD 为仲裁 1M、数据 5M，不能只凭支持 CAN-FD 就认定接口支持 5M。

官方 CAN 协议要求固件不低于 3.3.0。可下发位置、速度、力矩、加减速，收到控制帧后返回状态；不能假定无命令时自动持续广播。CAN 总线两端的终端电阻按拓扑配置；夹爪默认板载 120Ω，两节点情况下主机端也需适当终端。

### RS485

六芯口仍提供供电，3/4 切为 RS485 B/A；主机需物理 RS485 收发接口，例如 USB-RS485。20 板切换涉及焊接跳线，30 板使用拨码；还需匹配固件里的模式、波特率和地址。自定义串口协议与 Modbus RTU 不相同，Modbus RTU 需固件 ≥3.3.5。

### UART / TTL

知乎案例使用该路径：先通过原生 USB 配置，再由 Orin UART 控制。它是另一种物理通信模式，不等于 USB-C 被配置成普通串口。使用开发板 UART 还须核对电平、收发交叉和地参考；不将未核实的电平直接接线。用户照片指向 20 板，没必要为了照搬此案例而焊改已有 CAN 默认配置。

## 社区项目：实际用的是什么接口

本轮重新获取 README，保存在 `research/grippers/omnipicker_wiring_2026-09-21/`。以下为社区项目自述，未编译/运行或做硬件验收。

| 项目 | 实际路径 | 对本项目的用途与边界 |
|---|---|---|
| [zyf617/OmniPicker_usb2can](https://github.com/zyf617/OmniPicker_usb2can) | Python pyserial → 支持透传的外置 USB-CAN | 不是本体原生 USB 示例；前轮源码会初始化发送 `AT+ET`。原源码 CAN 数据的速度/力矩顺序还需按当前官方帧表复核，不可直接当验证完毕驱动 |
| [motor-and-motion-control-lab/OmniPicker_SDK](https://github.com/motor-and-motion-control-lab/OmniPicker_SDK) | C++ Linux SocketCAN | 适合研究 RDK 板载 CAN 适配。README 的 500Hz 循环只是示例口径，未验证用户实物性能；`query_states` 会重发控制帧，不是无动作的纯读 |
| [HysteriaC/omnipicker_ros2](https://github.com/HysteriaC/omnipicker_ros2) | ROS2 → ZQWL-UCANFD-100K 外置转换器 | 自述测试 Ubuntu 24.04/Jazzy；提供目标开度/力矩等服务和状态话题，不是本体 Type-C 驱动 |
| [ruiofang/OmniPicker](https://github.com/ruiofang/OmniPicker) | Python/PyQt5 → SocketCAN 或 RS485 | 有经典 CAN、CAN-FD、自定义串口和 Modbus RTU 路径。Modbus 模式的硬件和软件前提不能省略；不要求用户为 GUI 默认模式去改硬件 |

## 对用户当前方案的判断

- “六芯口只接电源，本体 Type-C 通信”的架构明确成立，不需要把 CAN 数据送入 E6 的 PNP 口。
- 单对供电有个人实操记述，双对均接应表述为保留原电源路径的建议，不是缺一对就一定不工作的结论。
- 使用现有六芯线把双 VIN / 双 GND 分别合接仍简单，不增加电源数量。若用户选择只接 1/6，一对触点/线材承担全部电流是否足够仍需规格或实物受控验收。
- E6 作为电源来源的能力核查与 OmniPicker 的接线定义是两个独立问题；本轮没有解决 E6 腕部连续/峰值电流和精确配对接头问题。
- 官方默认上电会自动使能并闭合归零，首次接电不是静态只读检测。
