# USB-CAN（PCAN黑壳）接入RDK X5前模式核查

核查日期：2026-10-01（Asia/Shanghai）。本轮仅阅读厂商网页、PDF和Linux驱动资料，未连接RDK或转接器、未发CAN帧、未改夹爪配置或刷写固件。

## 当前现场状态

用户确认已购买USB-CAN转接器，OmniPicker已接通24V，六针线已接到转接器；尚未把转接器插入RDK X5。接线完成属于用户反馈，本轮未独立核对线序、电压、接地或终端电阻。转接器资料由用户指定芯路城资源98、107。

历史实测本台OmniPicker固件335、CAN节点ID=1；此前铭牌指向20 PCBA，六针3=CANH、4=CANL。当前重新上电后的使能状态没有读回，不能沿用历史下使能结果。X5是否已经解决9月29日启动问题，本轮也没有新增验证。

## 开关结论

厂商《PCAN固件使用说明书》第5页§1.2照片为USB-TO-CAN、MODEL:UTC-T01，顶部三档标记为120R、0R、BOOT。网页标题为PCAN黑色外壳，用户实物具体铭牌和当前开关位置尚未确认。

| 档位 | 厂商定义 | 本次使用 |
|---|---|---|
| 120R | 正常通信，接入120Ω终端电阻 | 只有转接器和夹爪两个节点、转接器端未另接终端电阻时，选这一档 |
| 0R | 正常通信，断开120Ω终端电阻 | 转接器端已有外接120Ω电阻，或转接器位于多节点总线中间时才选；不表示H/L短接 |
| BOOT | 刷写固件 | 正常连接不用这一档 |

因此，若现在已在120R，无需切换；若在0R或BOOT，本次两节点直连应在USB未供电时改到120R，再插入X5。以实物文字标记定位，不按照片左右位置盲拨。这里改变的是转接器开关，不是夹爪内部CAN/RS485选择，也不切换PCAN/candleLight固件。

两节点总线应在两个物理末端各有一个约120Ω电阻。OmniPicker官方§5.2说明R47出厂焊接120Ω；用户这台二手夹爪是否仍保留，尚未测量。厂商转接器手册括号中的“两个设备至少一个电阻”不能替代两端终端的工程配置。夹爪端保留R47，转接器端选120R，不再额外并联第三只电阻。

如需电阻核验，先关闭24V、拔掉全部USB供电，再量H/L；约60Ω仅是两个120Ω并联的理想参考。本模块参考原理图还有串联保护/开关电路，实物读数可能偏离，不把严格60Ω作为单独合格标准。禁止在带电总线上测电阻。

## 通信参数和接线

- 采用经典CAN，1,000,000 bit/s。资源107中的500k为教程示例，不是OmniPicker工作速率。
- 本台历史CAN ID为1，控制和反馈均DLC=8。使用标准数据帧，不能按通用教程发送浮点数或改成远程查询帧。
- 官方OmniPicker资料标注80%采样点；实际转接器能设置哪些位时序由其固件和Linux驱动决定，初始化后检查实际值，不把教程的控制器时钟数字当作所有节点必须相同的参数。
- 对已有20板线序，3接转接器CANH、4接CANL，夹爪电源GND节点接转接器GND。电机回流直接回24V电源负极，24V正极不能接转接器CANH/CANL/GND。
- 当前使用USB-CAN，不经过X5板载J18，故X5的J19终端跳帽不参与这条总线。

这些是接入建议，本轮没有确认实物线序或进行通信验收。详细夹爪协议及20/30板差异见[既有CAN接线说明](RDK_X5_CAN接线与VLA接入_2026-09-27.md)。

## Linux接入：先枚举，再配置

保留现有PCAN固件，先检查USB识别。厂商网页明确PCAN模式没有COM口；不能套用串口转CAN的slcan流程。

PEAK官方Linux页面说明PCAN-USB从Linux主线3.4起有SocketCAN驱动，模块名peak_usb。核对Linux源码，其经典PCAN-USB匹配VID:PID=0c72:000c。该事实证明存在Linux驱动路径，不证明芯路城APM32兼容固件或用户X5镜像已验证成功。

厂商附带烧录PDF是STM32/candleLight/PCAN参考教程；资源98正文明确当前主控为APM32、BOOTLoader不同。没有证据要求为了Linux预先刷candleLight或更新BOOTLoader。BOOT只在明确刷写任务时使用。

插入X5后，先在X5终端运行以下只读命令：

```bash
uname -r
lsusb
ip -brief link
sudo dmesg | tail -n 60
```

X5自身板载CAN可能已经占用can0，USB转接器可能得到can1；接口名不能预先固定。后续用`ethtool -i canX`及`readlink -f /sys/class/net/canX/device`核对是否是USB/peak_usb，再初始化对应接口。

经典PCAN-USB驱动向Linux报告8MHz位时序时钟。在1Mbps下有8个时间量子，80%不能精确表示；Linux可能选到75%。这是驱动位时序的离散限制，与夹爪端80%不必逐值相等，仍应通过总线错误计数和实物收发验收判断兼容性。本轮未进行初始化或收发，不能提前声称可用。

初次枚举无需发送夹爪控制帧。OmniPicker公开CAN协议没有独立只读查询，通常收到控制帧才返回反馈；只监听无数据不能据此判断线。不要把全零8字节或固定目标帧当成只读探测。

## 来源与证据

1. [芯路城PCAN黑壳客户资料（用户提供）](https://www.xinlucity.com/?s=resourcedetail/index/id/98.html)：使用说明、参考原理图、刷写资料、APM32提示。
2. [PCAN使用说明PDF](https://testxinlu.oss-cn-beijing.aliyuncs.com/static/upload/file/warehouse/2026/05/07/1778141194624953.pdf)：第4页正常连接，第5页§1.2三档定义与照片。
3. [芯路城C006-PCAN-View教程（用户提供）](https://www.xinlucity.com/?s=resourcedetail/index/id/107.html)：Windows软件流程，500k示例。
4. [PEAK官方Linux驱动说明](https://www.peak-system.com/fileadmin/media/linux/index.php)：SocketCAN、内核版本、peak_usb。
5. [Linux PCAN USB核心头文件](https://raw.githubusercontent.com/torvalds/linux/master/drivers/net/can/usb/peak_usb/pcan_usb_core.h)、[设备匹配代码](https://raw.githubusercontent.com/torvalds/linux/master/drivers/net/can/usb/peak_usb/pcan_usb_core.c)、[经典PCAN-USB位时序](https://raw.githubusercontent.com/torvalds/linux/master/drivers/net/can/usb/peak_usb/pcan_usb.c)：本轮下载的是master快照，后续网页可能变化，以归档和SHA-256清单为本轮证据。
6. [OmniPicker官方手册](https://www.agibot.com.cn/DOCS/PM/X1)§5.2、§6.3：R47、CAN参数；引用项目既有官方快照，不是本轮新增实测。

网页、PDF、文本抽取和关键页渲染在`../research/usb_can_pcan_2026-10-01/`；来源及SHA-256在该目录`source_manifest.json`。

## 同日后续：X5已通过SSH只读预检

用户随后提供本台X5远程连接信息，并明确授权保存到项目记忆、尝试登录。实际从本机WLAN 192.168.1.7连接192.168.1.13:22成功，以root完成一组只读命令，随后SSH退出。凭据按用户要求存于根目录AGENTS.md，不在本节重复。

- 实测系统Ubuntu 22.04.5 LTS、aarch64、内核6.1.83；根分区117G、剩余104G。
- `modinfo peak_usb`找到`/lib/modules/6.1.83/kernel/drivers/net/can/usb/peak_usb/peak_usb.ko`，含经典PCAN-USB的0c72:000c匹配别名。只确认驱动文件存在，没有手动加载模块或确认它已绑定转接器。
- `/usr/bin/candump`可执行。未安装或升级依赖。
- `ip -brief link`已有can0，状态DOWN；未配置该接口，也未单独追溯来源。USB转接器新增接口仍需插入后确认，不能直接选can0。
- `lsusb`仅列USB2/USB3根Hub及Genesys Hub，尚未出现转接器。没有CAN实物收发结果。

现在可以让用户确认转接器开关为120R，再将USB端插入X5。下一步仅核对枚举、实际VID/PID、驱动绑定和新增CAN接口；夹爪动作验证仍为后续步骤。此次成功登录确认X5当前已启动，不证明9月29日灯光现象的原因已查明。

结构化只读预检证据：`evidence/rdk_x5_ssh_20261001/pre_usb_can_check.json`。本轮未改远端系统配置、启停CAN、发帧、刷写或操作夹爪/E6。

## 同日后续：插入后枚举与纯监听初始化通过

用户明确反馈已将转接器插入X5。实际通过SSH取得：

| 项目 | 实测结果 |
|---|---|
| USB设备 | Bus001 Device003，VID:PID=0c72:000c；lsusb按ID显示PEAK System PCAN-USB |
| USB产品字符串 | XCAN-USB；不是实测为PEAK原厂硬件 |
| 驱动 | peak_usb已自动加载；sysfs驱动链接=/sys/bus/usb/drivers/peak_usb |
| 新接口 | can1，parentbus=usb、parentdev=1-1.2:1.0 |
| 既有接口 | can0，m_can、parentbus=spi、parentdev=spi5.0；本轮未操作 |
| 运行时配置 | 经典CAN、bitrate1000000、请求sample-point0.8、listen-only on；初始化成功 |
| 实际位时序 | 采样点0.750、clock8000000、tq125ns、prop-seg2、phase-seg1=3、phase-seg2=2、sjw1 |
| 有界监听 | timeout10s candump，0条报文，退出124为预期到时退出 |
| 发送 | 未运行cansend/cangen/canplayer；TX报文0 |
| 计数 | RX报文0、RX/TX errors0、bus-errors0、bus-off0；RX dropped1在监听前已出现，10秒后仍1，原因未定位 |
| 最终状态 | can1 DOWN/STOPPED；运行时仍保留1Mbps、75%、listen-only on |

执行的CAN接口命令如下，已完成清理，没有建立开机自动配置：

```bash
ip link set can1 type can bitrate 1000000 sample-point 0.8 listen-only on
ip link set can1 up
timeout 10s candump -tz can1
ip -details -statistics link show can1
ip link set can1 down
```

初始探测中的shell循环因本机PowerShell转义错误，未取得设备路径；随后用明确的`/sys/class/net/can1/device`及`/sys/class/net/can1/device/driver`重新独立核对成功。监听模式不参与ACK，0报文时错误计数0只能描述本阶段接口状态，不能作为电气通信验收。

结果确认本台转接器能被X5枚举、由peak_usb绑定并初始化SocketCAN。仍未验证夹爪端线序、实物终端电阻、开关位置、夹爪接收控制和返回反馈。公开OmniPicker反馈由控制帧触发，只有监听时静默是可能情况，不能据此判定故障或正常收发。

后续明确进行CAN动作验证时，应先在DOWN状态切换`listen-only off`并核对正常模式，再按当时现场条件执行有界低速动作。不可直接沿用当前listen-only参数尝试发送，也不能将全零帧伪装成只读查询。

原始SSH工具输出及结构化监听摘要已归档至`evidence/rdk_x5_ssh_20261001/usb_can_detection_raw.json`和`usb_can_listen_raw.json`，保留终端ANSI转义及原始换行。本轮除can1临时接口配置外，无依赖安装、固件/夹爪参数写入或夹爪/E6动作。
