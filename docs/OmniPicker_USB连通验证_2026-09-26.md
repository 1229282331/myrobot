# OmniPicker 原生 USB 连通验证

时间：2026-09-26，约 21:00（Asia/Shanghai）。用户确认夹爪已通过 USB 连接本机，并接好 24V 供电。本次使用官方 REF-CLI v1.0.3 实际连接成功，未执行电机运动或修改设备参数；供电来源、接线载流能力和机械安装状态未独立检查。

后续更新：同日 21:09 在用户授权并确认现场条件后完成 [基础动作验收](OmniPicker_基础动作验收_2026-09-26.md)。下文保留首次只读连接阶段的证据与边界；最新动作结果见该记录。

## 实测结果

| 项目 | 实测值 |
|---|---|
| USB VID / PID | `1001 / 03EF` |
| Windows USB 序列号 | `20773954464D` |
| USB 串口 | `COM29`，接口 `MI_00`，驱动 `usbser` |
| REF 接口 | `MI_02`，名称 `REF Interface`，驱动 `WINUSB` |
| Windows 设备状态 | 三个相关设备项均为 `OK`，错误码 `0` |
| 上位机版本 | REF-CLI `1.0.3` |
| 连接提示 | `已连接 REF 00003954464D 设备作为 ref0` |
| `device_type` | `1` |
| `serial_number` | `0x3954464D`，上位机显示补零为 `00003954464D` |
| `fw_version` | `330`，按官方 `OmniPicker-fw-ota-v330.bin` 对应版本 `3.3.0` |
| `can_node_id` | `1`，是本台读回值，不是手册示例的 `8` |
| `motor.error_code` | `0`，本次读取时无故障报告 |
| `encoder_type` / `debug_flag` | `3` / `56`，仅记录原值 |

USB 的 WinUSB/Fibre 通道已验证可发现设备、读取属性和导出接口定义。`COM29` 只是同时枚举出的串口，本次没有打开串口或验证串口协议；CAN 通信也未测试。

## 软件与检查过程

官方程序 SHA-256：`f90f4a90218ea63983fc1395362604236d159e2c3d49683e033a3bd272ad9b18`，与此前静态核查一致。下载来源及协议分析见 [原生 Type-C USB 核查](OmniPicker_原生TypeC_USB核查_2026-09-20.md)。

1. 只读检查 Windows USB 枚举与驱动状态，确认仅有一台匹配设备。
2. 核对官方程序启动、发现回调和 Windows USB 初始化分支，没有启动时的电机动作调用；非 Windows 分支的 USB reset 不能照搬为本机行为。
3. 最初将 Windows USB 序列号传入 `--serial-number`，软件显示 `Ignoring device with serial number 3954464D`。USB 描述符序列号和固件导出的 32 位 `serial_number` 不同，不能混用。
4. 改为 `--serial-number 3954464D shell` 后，官方软件明确报告已连接 `ref0`。在命令行管道模式下直接使用 `ref0` 曾出现 `NameError`；检查官方发现回调后，从其现有 `interactive_devices` 字典绑定本地变量，再成功读取设备。
5. 读取根对象属性，并保存设备已导出的 `_json_data` 接口定义。正常退出最终检查会话，当前没有留下持续占用 USB 的 REF-CLI 进程。

本次成功读取使用的上位机交互语句如下（变量赋值仅在电脑端绑定对象）：

```python
import fibre.shell as _ref_shell
ref0 = _ref_shell.interactive_devices["ref0"]
print(ref0)
```

没有调用 `request_state`、`set_pos`、`clear_error`、`save_config`、`reboot`、校准或固件升级，也未连接 E6 控制接口。用户接电前后的自动归零动作未由本次观察验证。

## 实物接口定义与边界

- 已取得这台设备的完整 JSON 接口定义，包含 `motor.ctrl.set_pos`、`set_vel`、`set_cur`、`set_tor` 和 `set_mit`，本轮仅检查函数定义，没有调用。
- 本次定义中未见实际位置、速度、电流等实时反馈属性。不能仅凭设置函数存在就认定已经获得闭环观测；后续仍需核对其他遥测途径。
- `motor.config.enable_on_boot`、限流、速度等配置属性存在，但本轮未进一步读取其值，也未改变使能状态。`error_code=0` 不能代替当前使能状态或动作验收。
- 本轮证明原生 USB 通信可用，没有验证夹爪开合、位置语义、频率、断线行为或负载能力。照片推断的 20 PCBA 仍不能由 USB 接口枚举单独证实。

## 原始证据

- [Windows USB 枚举及程序校验](../local-network/omnipicker-usb-enumeration-2026-09-26.json)
- [官方 REF-CLI 成功读取日志](../local-network/omnipicker-ref-cli-2026-09-26-readonly.log)
- [实物 USB JSON 接口定义](../local-network/omnipicker-usb-schema-2026-09-26.json)

这三份文件已加入 Git 忽略规则的精确白名单，以供后续复核；初次试连接的临时日志仍按默认规则留在本机。
