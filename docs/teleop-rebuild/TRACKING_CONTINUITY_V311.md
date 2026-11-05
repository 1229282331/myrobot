# v311 指令连续性误停修正

v310 只修了持续跟随误差，把控制器执行中的目标误差与最新未执行目标的领先量分开。本次剩余问题在 candidate 连续性检查：用相邻写口的约28–37ms允许步长，同时比较了候选到滞后QTarget的累计距离，造成误停。

本次改为连续发送时仅比较相邻两次实际下发目标；首条指令仍锚定已稳定的控制器目标。候选对实际角度的2°硬界、同帧持续跟随检查、停滞、跨重置指令连续性及设备报警处理保持。

最初五次事件中相邻指令最大步长约0.324–0.496°，小于对应0.866–1.132°界；原错误比较却为0.884–1.274°。退役后完整会话共有8次，均由同一条件触发，修正后八次全部重放通过。证据含被拒候选、原指令和原SDK帧，retired-capture.json保留2321发送与2321确认事件；不能把这些旧版用户操作当成新版运动验收。

验证：旧版新增5例全部失败，修正后完整1114通过、8skip。另有两方向真实突跳例及原跨reset7.86°跳变例仍拒绝；两份历史v304实际不跟随记录仍在原时刻/原因拦截。八次完整退役事件独立重放通过。组合核验脚本先在科学Python完成重放后因缺paramiko退出，随后使用原launcher环境完成只读SSH核验；记录保留，没有因此改生产依赖。

上线：只有joint_tracking.py变化，源SHA在RDK独立核对。旧owner正常释放后启动新owner；中途工具中断没有终止新owner，后续通过进程、生命周期、实时状态确认。没有重复启动、自动Prepare或新设备命令。新会话只验证静止与网页连接，未完成新版本实机运动验收。页面恢复为用户切换前的Gamepad/Rotation/Base0.5x，RGB约20fps，3D实时。真实停滞与突跳保护未关闭。

回滚入口：用launcher Python运行 teleop-platform/scripts/switch_tracking_v311.py --check --rollback，正常释放owner后执行 --rollback；脚本核对源码、配置、日志证明、端口和远端独占锁。随后应使用回滚版本匹配的日常启动入口及新stopfile名，不能重放已经存在stopfile的入口。

v311 连续性误停修正已上线并完成交接（2026-10-19，覆盖当前）：用户多次要求撤全部跟随保护，已明确未执行全关闭；用户随后同意继续修正。v310旧owner最终8次joint_command_discontinuity均混用了滞后QTarget与两次写口时间差；仅改joint_tracking.py为相邻实际下发目标差，保留首次起步锚点、2°硬界、同帧0.75°/250ms、停滞、100ms新鲜度及设备报警检查。旧5夹具全失败，新22专项包含在完整1114通过/8skip；退役会话8原事件重放全通过，旧v304两真实异常仍同刻同因拦截。源SHA9bcc8b0e已独立在RDK核对；L fdf3f684/model3b4003e4/TCP151.4/负载0.750与COM(0,-14,46)/Force255不变。旧owner20261019-214938-1cc1d956经新20静止未准备idle后stopfile正常clean退出，camera/gateway0、CPU/WiFi恢复，media1/cleanup空保留。切换工具被用户中断但进程已继续完成，已新查确认无需重复启动。现唯一日常owner20261019-215947-8b5036b3，入口output/tracking_operator_311.py operator，stopfile output/tracking_operator_311_operator.stop未建；PC执行PID9208/父14536仅线索、统一execID未返回不可猜。新20状态未准备静止idle/Mode5 Enable1 Error0 Collision0/ID2958不增，负载匹配、夹爪只读ready actual12/255 sentnull。新owner审计快照6791周期无Prepare/动作/跟随停止；正式Gamepad/Rotation/Base0.5x已恢复，RGB约20fps与3D实时，手柄需按键唤醒。无本轮助手运动/新Enable/清错/改参/夹爪位置指令，未进行新版本实机运动验收；原J5根因与v307实时退离未解决，不称全部遥操作问题根治。Goal paused、自主抓杯取消、未Git提交。证据evidence/tracking-v311、报告TRACKING_CONTINUITY_V311.md，回滚switch_tracking_v311.py（正常释放后先--check --rollback）。下方会话均历史，下一动作新读。
