# RDK X5 + PCAN USB-CAN：OmniPicker低速控制与反馈验收

日期：2026-10-01，Asia/Shanghai。实机窗口18:24:38–18:24:56，由设备记录UTC转换。承接[转接器模式与接入核查](USB_CAN_PCAN接入前模式核查_2026-10-01.md)。

## 结论

**主电脑SSH → X5本地SocketCAN → USB-CAN → OmniPicker动作 → CAN状态反馈 → 主电脑证据归档，整条低速控制与反馈链路已实测通过。**

用户明确授权继续验证整套CAN控制，并确认当前夹爪固定稳妥、行程清空、人在现场且可以切断24V。没有操作E6。

约18秒内按20Hz发送360帧，收到360条有效设备反馈，全部故障码0；设备报告运动状态72次、到位状态288次，无堵转或物体掉落状态。位置有真实的逐步变化，与目标分别记录，没有用发送回显当反馈。

本次仅为空载15%–35%区间的一组低速验收，没有验证全行程、带载、力标定、长时间采集或断网急停。现场平稳性和声音已向用户询问，暂待回复，不套用历史USB测试的观察结果。

## 实测配置与结果

| 项目 | 结果 |
|---|---|
| X5 | 192.168.1.13，Ubuntu22.04.5、aarch64、Python3.10.12、内核6.1.83 |
| 转接器 | VID:PID=0c72:000c，产品字符串XCAN-USB，peak_usb驱动；不是据此认证PEAK原厂硬件 |
| CAN接口 | can1；can0为X5板载SPI/m_can，本轮未操作 |
| 协议 | 经典CAN，标准数据帧，ID=1，DLC=8 |
| 位时序 | 1Mbps，实际采样点75%；官方手册夹爪端标80%，本轮正常收发验证了该组合的兼容性 |
| 控制限幅字节 | Vel25、Force64、Acc25、Dec25，约为协议最大字节的9.8%/25.1%/9.8%/9.8%；不是实测mm/s、A、N或N·m |
| 周期 | 每50ms发一次当前目标以触发设备反馈，每段6秒 |
| 主机反馈率 | 19.997Hz；由主机接收间隔计算 |
| CAN应用往返 | 中位0.778ms、P95 0.827ms、P99 0.848ms、最大0.862ms；不含主电脑SSH往返，不是设备采样时刻或机械到位时间 |
| CAN错误 | RX/TX errors、bus-error、error-warning、error-passive、bus-off均0 |
| 丢弃计数 | RX dropped从测试前1到结束1，没有新增；旧计数原因未定位 |
| 最后位置 | 设备回报38/255=14.90196%，最后状态0（到位） |

| 阶段 | 目标 | 编码目标 | 设备位置变化 | 连续三帧到位确认 | 反馈数 |
|---|---:|---:|---|---:|---:|
| 1 | 15% | 38 | 0→38 | 1.201s | 120 |
| 2 | 35% | 89 | 38→89 | 1.501s | 120 |
| 3 | 15% | 38 | 89→38 | 1.501s | 120 |

8位归一化量化使15%/35%对应14.90%/34.90%；这不是毫米定位误差或绝对精度测量。少数保持阶段位置在邻近一个字节间波动，图中保留原值。

![目标与真实反馈及主机CAN往返](evidence/can_control_20261001/can-control.png)

## 协议核对与回显排除

本轮联网读取[智元官方手册](https://www.agibot.com.cn/DOCS/PM/X1)§6.3，并读取社区SocketCAN实现作交叉核对；按官方定义实现，没有运行社区示例。该社区SDK的connect()会发全零帧作为探测，实际含闭合目标，故本工具没有使用该初始化路径。

下行顺序：Reserved0、Pos、Vel、Force、Acc、Dec、Reserved0、Reserved0；本次15%帧为`00 26 19 40 19 19 00 00`，35%帧为`00 59 19 40 19 19 00 00`。

上行顺序：Fault、State、Pos、Vel、Force、D5–D7回显。0=到位、1=运动、2=堵转、3=物体掉落。故障非0、未知状态、异常帧、回显不符或反馈超时300ms时取消剩余目标，不自动重试或重连续跑。

发送socket关闭内核loopback与RECV_OWN_MSGS，使用recvmsg标志排除本机来源，拒绝与下行数据完全相同的潜在适配器回显。360条匹配反馈的接收flags均0；D5–D7与当次命令相符，位置从0到38、再89、再38，速度/力矩字段保留原始无符号字节。没有擅自推断速度或力矩的符号编码和物理单位。

## 最后状态与停止含义

程序结束时已关闭socket并将can1设为DOWN；另一次独立SSH读回确认DOWN/STOPPED、RX/TX各360包、错误计数0、原始证据和程序SHA-256与本机一致。运行时保留1Mbps/75%，listen-only已经off；没有创建开机自动配置或启动后台控制进程。

**夹爪仍通电，最后为约15%目标保持；本次没有下使能。** 公开8字节CAN协议没有独立下使能命令，关闭接口/停发不等于电机下电，也未验证通信看门狗。停止文件只能取消后续发送；危险动作仍须切断24V，或后续接入已验证的本体USB下使能路径。

CAN每帧携带速度/力矩/加减速限幅。本轮没有本体USB前后属性快照，因此不声称六项RAM参数保持不变或恢复。没有Flash保存、固件升级、校准、清错或更改节点ID。

## 可复用程序

本机工具：`omnipicker/tools/verify_can_control.py`，仅标准库，默认离线预览；只有`--execute`访问硬件。预检必须匹配peak_usb与0c72:000c，并且接口处于DOWN；使用文件锁避免同一工具多实例。动作只允许15%–35%、最多4段/30秒，输出文件禁止覆盖。现场仍应排除其他CAN发送者。

X5上已部署相同版本：

```text
/root/omnipicker-can/verify_can_control_0ec93b527cae.py
```

以下是离线预览，不启动CAN：

```bash
python3 /root/omnipicker-can/verify_can_control_0ec93b527cae.py
```

以下命令**会驱动夹爪**，应在现场条件及动作范围获得授权后使用，输出文件名需每次更换：

```bash
python3 /root/omnipicker-can/verify_can_control_0ec93b527cae.py \
  --execute --interface can1 --can-id 1 --targets 15,35,15 \
  --dwell 6 --hz 20 --timeout 0.3 \
  --output /root/omnipicker-can/runs/NEW_RUN.json
```

不要把该有限时长验收脚本当成已经完成VLA长时驱动、Studio CAN后端或具备断网下使能的网关。后续可以沿用已验证帧编码和真实状态解析开展对应集成。

9项新增离线测试覆盖目标/限幅边界、本机与适配器回显、非法帧/ID、故障与堵转、回显尾部、到位必须匹配位置、反馈超时不重试和停止取消。实机测试只执行本次一组，不为测试而追加运动。

## 证据

- `evidence/can_control_20261001/can_trial01_20261001.json`：从X5复制的原始完整报告；X5原文件SHA-256=f581b022aa3cc03f21ae54b55defab517a4fd0ac2ce6b6fe8dc815dadec68593。
- 同目录`analysis.json`、`samples.csv`、`can-control.png`：离线派生数据和图，由`omnipicker/research/analyze_can_control.py`用既有omnipicker环境生成；没有新增依赖。
- 同目录`verification_metadata.json`：用户授权/现场条件、部署和独立SSH复核工具输出、9项离线测试摘要。
- 联网官方/社区资料及SHA-256：`omnipicker/research/can_control_2026-10-01/source_manifest.json`；社区固定提交819a15c6…，只作为交叉核对。

固件335、内部SN3954464D和节点ID1是此前本台USB实测身份信息；本轮CAN不能直接读取固件或SN，没有重新取得这些属性。
