# RDK X5 / Orange Pi 5 Pro 作为 OmniPicker 与 D435i 边缘控制器

核查日期：2026-09-19。本文只讨论接口和架构，不执行 E6、夹爪或相机动作。

2026-09-20 更新：用户已购 OmniPicker、随货仅六芯线；照片 SN 前缀指向 20 PCBA。最新法兰、六芯逐针映射、RDK SH1.0 三芯接口与首次上电步骤见 [到货接入方案](OmniPicker_E6_到货接入方案_2026-09-20.md)。默认上电会自动闭合归零；CAN 反馈按命令响应，不能把等待数据等同于已有周期广播。

## 结论

2026-09-20 补充：原生 Type-C 也有直接开度控制路径。已静态确认官方 REF-CLI 使用 PyUSB/USB Bulk/Fibre，并存在非 Windows 代码；RDK 移植与实际反馈字段尚待实物验证。不要再把 USB 只描述为配置工具或认为必须外置 USB-CAN，详见 [原生 USB 核查](OmniPicker_原生TypeC_USB核查_2026-09-20.md)。本页 CAN 仍是有公开帧定义的推荐备选。

优先使用 RDK X5：它的官方硬件资料明确列出 CAN FD 接口，板上集成 TI TCAN4550，支持经典 CAN 1 Mbps；同时有 4 个 USB 3.0 Host 口，适合 D435i。Orange Pi 5 Pro 可用，但 40Pin 上的 CAN1_RX/CAN1_TX 是控制器逻辑信号，仍需外置 3.3V CAN 收发器或 USB-CAN。

OmniPicker 运行时采用 CAN，主供电按独立 24V DC 设计。Type-C 可能为低功耗接口电路提供 VBUS，但官方手册没有给出 Type-C 的供电电压/电流或 USB-PD 规格，因此不能把它当作电机运行电源；两者可以共用一根物理复合线，但必须是不同的芯线，不能把 24V 接到开发板 GPIO 或 CAN 引脚。

推荐的复合线保留 OmniPicker 6Pin 的全部定义：

- 1、2：VIN（24V，两个针脚并用）
- 3、4：CAN 信号（具体 H/L 方向按硬件版本）
- 5、6：GND（两个针脚并用）

版本 20：3=CAN_H，4=CAN_L；版本 30：3=L，4=H。采购后必须按 SN/板型和原装线确认，不能只按插头外观接线。

## 推荐拓扑

```text
主电脑 --Wi-Fi--> RDK X5 --SocketCAN--> CAN_H/CAN_L
       \--Wi-Fi--> E6（仍由原来的网络接口控制）
                         |
                    本地分线盒
                 /              \\
       独立 24V 电源             复合线（沿机械臂）
                              OmniPicker 6Pin

RDK X5 USB 3.0 Host ---------------- Intel D435i
```

复合线在开发板/机械臂底座附近分开：两根供电芯线接带保险丝的 24V 电源，两根 CAN 芯线接 RDK X5 的 CAN 接口，另外两根保留为并联的电源回路。这样沿 E6 运动部分只有一条线束。

## RDK X5

官方硬件页列出 CAN FD 接口；CAN 使用页说明板上集成 TCAN4550，支持经典 CAN 和 CAN FD，经典 CAN 速率最高 1 Mbps，并提供 CAN_H、CAN_L、GND 三线接口。OmniPicker 官方协议兼容经典 CAN 1 Mbps，因此首轮用经典 CAN，不需要 CAN-FD 数据段。

RDK X5 的 CAN 接口不是 24V 电源输出；OmniPicker 的电机电源必须来自独立 24V 电源。RDK X5 的 5V 供电只给开发板和 USB 外设使用。

## Orange Pi 5 Pro

Orange Pi 官方参数页写明 40Pin 可配置 UART、PWM、I2C、SPI、CAN、GPIO；官方 V1.1 引脚图把 CAN1_RX_M1/CAN1_TX_M1 引到 40Pin（按标准 40Pin 编号为物理 9/11）。这仍是 SoC 的 3.3V 控制器信号，不是 CAN_H/CAN_L，不能直接连接 OmniPicker。需要 3.3V 逻辑兼容的 CAN 收发器/HAT，或改用 USB-CAN。

如果用 Orange Pi，D435i 占用唯一的 USB 3.1 Host 口；USB-CAN 可以插 USB 2.0，但要注意驱动和供电。综合线缆与软件调试，RDK X5 更适合作为第一块边缘控制板。

## OmniPicker 运行控制

官方手册给出 CAN 下行帧：位置、速度、力矩、加速度、减速度均为 0–255，可连续改变；上行帧返回故障码、状态、当前位置、速度和力矩。CAN 固件需不低于 3.3.0。手册要求先给夹爪上电，再用 USB-C 连接 Host；USB-C 主要用于 Host 软件配置、升级和调试。即使某台实物能通过 VBUS 点亮控制电路，也不能据此推断电机能由 USB-C 运行，额定运行电源仍按 24V 6 针输入准备。

## 线缆与终端

CAN_H/CAN_L 用一对双绞线，整条线采用柔性拖链线或柔性屏蔽线；VIN/GND 使用足够截面积的并联芯线。RDK X5 有 120Ω 终端电阻开关，OmniPicker CAN 板默认带 120Ω 终端；两节点、较高波特率时按官方说明在总线两端保留终端，禁止在线缆中间重复并联终端。所有插拔必须断电，OmniPicker 手册明确禁止带电热插拔。

24V 电源的额定/峰值电流仍需按实物版本或卖家规格确认；在确认前不要让 E6 末端 24V 给夹爪供电，也不要让开发板 USB/GPIO 给夹爪供电。

## 官方资料

- [D-Robotics RDK X5 硬件说明](https://d-robotics.github.io/rdk_doc/Quick_start/hardware_introduction/rdk_x5/)
- [D-Robotics RDK X5 CAN 使用](https://d-robotics.github.io/rdk_doc/Advanced_development/hardware_development/rdk_x5/can)
- [Orange Pi 5 Pro 官方参数](http://www.orangepi.cn/html/hardWare/computerAndMicrocontrollers/parameter/Orange-Pi-5-Pro.html)
- [Orange Pi 5 Pro 官方引脚/硬件图](http://www.orangepi.cn/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5-Pro.html)
- [AGIBOT OmniPicker 产品手册](https://www.agibot.com/filepage/265.html)
