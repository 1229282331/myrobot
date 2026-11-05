# V359 下使能与关机

v359 用户要求下使能及RDK关机完成（2026-10-21，覆盖当前）：代码此前已提交7efe9dcf，未推送。新20状态确认未准备静止录制idle、新RGB已看后，通过stopfile释放唯一owner20261021-000700-e34a1e5f。gateway0；camera未在15秒退出导致原result clean=false、CPU/WiFi暂保留，原失败不改。随后独立持锁确认全部子进程及后代已自行退出、无发信号，恢复CPU schedutil/WiFi on及卸载本会话proc，resource-recovery.json保留。PC app0/media1、cleanup空。独立锁内新静止SDK反馈后官方Stop和DisableRobot各一次0回包，988原帧连续无报警碰撞队列，末98帧Mode4 Enable0、峰峰max0.006180°。下使能姿态变化约[-0.010452,-0.200493,+0.023376,-7.059402,+0.059052,+1.311493]°后稳定，无重使能/补偿/停放/清错/改参/夹爪位置命令。CAN DOWN RX/TX338295，无E6连接/相机占用；sync后systemd接受5秒poweroff，随后WLAN源192.168.1.7三次TCP22超时，PC8765/8776/8777/8889/8766无监听。当前无控制owner，设备物理断电由用户执行；下次须用户要求继续后重新读取开机/反馈/负载/姿态和新画面，不沿用旧ready。本轮仅关机，未新增运动验收或Git提交；完整长期验收仍未完，自主抓杯保持取消。证据docs/teleop-rebuild/evidence/shutdown-v359。下方均历史。

原会话退出超时与随后资源恢复分别保留，不把恢复收据改写为原会话 clean。设备下使能后新反馈确认稳定，再请求 RDK 关机；没有自动回待机位。
