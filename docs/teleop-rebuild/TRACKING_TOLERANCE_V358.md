# V358 关节偏差容差：2° / 500 ms

2026-10-21。按用户要求，仅修改 `joint_tracking.py` 的偏差判定与诊断字段；原生控制器、模型、负载及速度配置保持。

## 最终语义

- 同帧控制器目标与实际角度：任一关节绝对偏差 **>2°**，连续 **≥500 ms** 才触发 `joint_tracking_sustained`。恰好2°允许，回到范围内即清零该关节计时。
- 执行轨迹偏离：偏离最近250 ms实际发送轨迹范围 **>2°**，连续 **≥500 ms** 触发 `joint_execution_lag`。250 ms是原轨迹历史范围，500 ms是本轮新的超限持续时间；二者与反馈新鲜度不同。
- 疑似停滞：完整500 ms窗口内，指令或控制器目标偏差持续 **>2°** 且实际关节变化峰峰 **<0.02°** 才触发。原0.35°/250 ms判定已替换。
- 原先“单帧超过2°立即停止”已合入上述持续判定，避免其提前绕过500 ms容忍时间。
- 停止后重新对齐 / Prepare 的容差改为 **≤2°**，以当前实际姿态建立首目标起点。停止回执、静止确认和Prepare前原5秒观察流程保留；500 ms是运行中偏差持续时间，并非把所有状态等待缩短为500 ms。
- 首条0.095°及后续按真实写间隔计算的指令连续性、100 ms反馈/250 ms输入新鲜度、几何检查与设备报警均保持。没有关闭故障检测或替代设备碰撞设置。
- V357方向拒绝保留Prepare/录制的行为保持；持续超过本轮容差仍是跟随故障停止。

## 验证与范围

完整后端 **1307 passed / 19 skipped**，见 `evidence/following-threshold-v358/backend-tests-final.txt`。覆盖正负2°边界、499/500 ms、回到范围内重置、不同关节计时独立、停止后±2°Prepare及夹爪释放、超2°拒绝、真实停滞、历史突跳、反馈失联和Stop确认。首轮两项测试断言放错位置导致2 fail / 123 pass，修正测试位置后完整通过，原失败输出保留；未为通过测试另行放宽生产参数。

刚才的报警：V357命令ID28522，J4 `joint_tracking_sustained`，同帧误差1.472881°，设备Error0/Collision0。独立抓取报警前约2秒的242帧原SDK（序号连续）与17条真实运动写入，在PC离线逐帧重放：

- 原规则4帧拒绝，首次J4持续偏差1.429857°；
- 新规则0拒绝；本段最大J4偏差1.472881°；
- 原始捕获325568字节，SHA `5e06203256cf83c166f48c7a2165b646cccca5df63dbda5663176c134c17590e`；
- 结果 `incident-replay.json`，曲线 `incident-replay.png` 已查看。

逐帧离线采样频率与真实网关轮询不同，不把重放首次触发时刻冒充实机时刻。记录在原Stop时结束，不能推断继续运动后的偏差；本轮不制造接触/停滞实机故障，不称全范围实机验收完成。

## 备份与部署

`output/tracking-tolerance-v358/release` 保存实际V357 guard、完整部署清单/配置、新源及差异；新guard SHA `7643555fdbb9051a3e11d8eb06f38780b665cb68fbde3a4a4426e279866de065`。

回滚入口 `output/rollback_tracking_tolerance_358.py --check` 已通过。实际回滚须先正常释放本轮owner、确认端口结束，再恢复旧guard并用正常launcher建立新会话；不得热替换活动进程。

V357 owner `20261020-233413-2bb08a49` 经新20状态未准备、静止、录制idle后，stopfile正常clean退出；camera/gateway0、CPU/WiFi恢复，PC app0/media1、cleanup空原样保留。新阈值部署状态追加于下方，下一动作须新读。

## 已接入正式平台

新唯一owner **20261021-000700-e34a1e5f** / exec **73847**，入口 `output/operator_tracking_tolerance_358.py`，退出文件 `output/operator_tracking_tolerance_358.stop` 未创建。新20状态未准备、静止、idle、Mode5 Enable1 Error0 Collision0，命令ID28522未增加，负载/模型/相机匹配，夹爪只读ready、sent_raw=null。远端源码SHA与候选相同，独立读取类参数证实2° / 500000000ns；启动审计0 motion_submitted、0 enable_requested、0 gripper_initialize。

页面重载后3D与RGB约20fps恢复，保留Gamepad / Translation / Base，并恢复用户切换前的0.6x。实体手柄需唤醒；没有Prepare或本轮助手运动，夹爪当前接触/Blocked反馈原样保留，没有释放夹持物。截图 `restored-page.png`。本轮无Git提交，Goal保持active，自主抓杯取消；下一实机动作须新读状态和画面。

## 磁盘归档

切换前RDK free仅7933952字节。对已经clean退出的V357会话，远端低优先级只读压缩流传至PC；压缩件与完整解压内容SHA验证，再重核远端原明文的长度、mtime及SHA，回收该重复明文，上传压缩件并核对双端SHA。原故障/运动记录保留，没有触碰活动owner日志。

- events：712438043字节 → 96011304字节，原SHA `40fd0a174d611f8c0a95c7661fceda6459b7132dc09ea329dbdf3fd60c66e228`，gzip SHA `aa30fc967e8e4713b55297ed1280bdf136e5c3689f661de40942a3d010c42dba`。
- raw SDK：402878464字节 → 44394268字节，原SHA `158e4fdf4c0003587e452c4cbf5f76a6178ba86f51e1ca7a950a07eb5e6ebd74`，gzip SHA `87d239cabab959e62b402ce3751d14b9708f1a66b5f0d7a364474b12bc86f162`。

PC副本与索引在 `evidence/following-threshold-v358/retired-v357-{events,raw}*`。远端原会话audit路径现在分别是 `events.jsonl.v358.gz` / `raw.jsonl.v358.gz`；读取历史脚本应使用归档，不再假定明文路径存在。新owner启动及核对后free739594240字节，新日志仍会增长。
