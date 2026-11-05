# v229 历史中止片段恢复

2026-10-09。解决v228正常退出时片段已在X5封存、PC尚未回传，新会话无法发现旧session文件的问题。没有改运动算法或发设备动作。

## 实现与使用

在项目根目录运行：

```powershell
.\teleop-platform\Start-E6.ps1 -RecoverRecordings
# 只恢复指定片段：
.\teleop-platform\Start-E6.ps1 -RecoverRecordings -Episode 20261009T120739Z_4654b673
```

复用已有SSH设置、phosphobot完整应用、episode_http、TransferWorker及文件校验器。启动一条媒体SSH连接，X5仅运行已结束session目录的媒体服务；PC打开专门的恢复页面。不启动gateway、MoveIt、相机、MediaMTX或CAN，不建立E6观察／控制连接。控制WebSocket在接受前拒绝，上游设备写接口保持拒绝。

结束用Ctrl+C，退出自有PC和媒体资源，机械臂已有使能状态保持。此模式不能与live、验收代输入或复用现有服务组合。普通启动不自动扫描所有历史目录，避免在控制期间追加历史批量传输。

目录仅识别项目session命名及匹配的owner/result终止记录，文件必须有封存SHA；路径逃逸、符号链接、重复episode编号和损坏元数据拒绝并报告。缺少终止记录或未封存的目录保留、不自动修补。校验回执只改transfer.json，不改原始结果、数据或bundle。失败显式重试沿用原TransferWorker，原失败部分下载保留。

正式页面复用原EpisodeBrowser／Inspector；回传阶段显示进度，文件验证后自动载入采样，避免把尚未下载的数据展示为“没有周期”。恢复页面不挂载键盘或手柄控制页。

## 测试与真实恢复

- 初次专项50通过、1个Windows符号链接权限跳过。完整后端644通过／8跳过（7既有平台项＋符号链接），后续环境修复专项12通过／1跳过；未改运动模块，不重跑实机长测。
- 前端既有片段／录制16项及正式build通过。新增已选中片段从transferring→verified后自动取周期用例，片段专项12通过；两次测试夹具失败分别为Dialog隐藏测试按钮、夹具关闭SWR首次取数，原输出保留，修正夹具后通过，生产逻辑未为迁就测试改变。
- 首次X5恢复启动因system Python未加载已安装zstandard而失败；日志与退出结果保留。修复为复用session_runner同一`gateway_dependencies`导入路径，不安装新包。
- 新恢复会话 `20261009-203015-deeb3666`，完整下载v228包155559882字节，SHA与先前SFTP备份一致；1196原图／1204文件、3980原SDK、154动作关联全部校验。正式远端verified回执已取得，原标签仍aborted/complete=false。
- 原SFTP恢复目录整体移至PC `runtime/recovery-preserved/`，未删除；正式恢复在原episode目录从空目录重新HTTP下载，不以已有文件跳过传输冒称完整回传。
- 正式phosphobot页面实看该中止条目、629周期、第一帧原图与有效关联；恢复期间只读资源审计无E6连接、无相机占用、无ROS proc挂载，CAN计数保持不变。
- 正常退出PC和X5媒体服务均exit0；PC8765/8776/8777/8889/8554监听空，退出后的新只读资源审计仍无设备／相机连接。原metadata、summary、bundle、raw/events、associations六项SHA与SFTP备份全部一致。末周期按原记录显示在途命令未确认。

证据：`evidence/recording-recovery-v229/`；运行部署及源码清单在对应`runtime/recoveries/`归档。此功能解决已封存历史片段的找回，不宣称掉电前未封存记录已可自动重建，也不改变v228的运动与停止未达标结论。
