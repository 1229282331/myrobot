# v266 碰撞包络与显示复核

2026-10-16。按用户要求结合正式渲染器反复核对，而非只检查覆盖是否通过。完整 Goal 保持暂停；本轮没有 Prepare、实机运动、夹爪初始化、清错或负载写入。

## 发现与修正

1. 旧显示网格经过过度 QEM 简化，夹爪颈部和连杆局部形状失真。恢复官方源 CAD 的完整显示网格；碰撞模型继续独立使用简单几何，未把高面数显示网格送入碰撞计算。
2. 夹爪固定本体原来两个盒，将细颈和宽外壳合并。现在按 CAD 高度 28.5、56.5 mm 分为三个盒，颈部包络约 59.16×59.31 mm（含每面 1 mm 余量），去掉肩部两侧多占的约 11–12 mm 空区。
3. 夹指原来四个机械分组盒，现在八个官方 CAD link 各一盒。各子盒沿原父盒的坐标轴拟合，所有角点严格位于原父盒内，避免重新旋转拟合产生新的外扩。固定名义驱动范围 −50°..+5°、每面 5 mm 余量和被动指节零姿态保持。联合体积由 Monte Carlo 估计约 1055 cm³ 减至约 982 cm³，约减少 7%；不是全面贴合或全行程标定。
4. 用 CadQuery/OpenCascade 对同一批八盒做精确 Boolean union，仅去掉显示中的重复内部面和交叉边线。控制仍读八个盒，显示占用体积相同；并集加载失败时显示原盒体。黄色明确标为 `Full-stroke fingers`，灰色夹爪是名义闭合姿态。

最终模型：`87fa1c31cfa04e67513f00ea0ce7a0fead19c04c733e1a3f3018f225bc04abeb`。
附件共 14 盒：R4 两盒、相机一盒、固定本体三盒、活动夹指八盒。E6 本体仍为七个原 CAD 凸包。TCP 151.4 mm、安装方向、碰撞排除表、速度、柔化、停止、负载和关节两端 5° 不变。

## 视觉和几何证据

离线预览使用正式 `workspaceRenderer.ts` 与完整 URDFViewer，分别检查闭合、名义全开、前、侧、顶、透视和整臂装配；不是另绘示意图。证据在 `evidence/collision-review-v266/final-*.png`，正式页面截图另存。

- 原 CAD 覆盖：七个本体凸包、R4/固定本体切分面连续，八指节各 122 个独立角度（含端点）通过。
- 显示并集闭合，体积 982307.854 mm³；10,000 个确定性随机点与控制八盒联合体占用判定零差异。
- WSL 固定 MoveIt/FCL：65,537 姿态采样，39,144 无碰撞，10 mm 体素去重为 37,825 个见证；所有见证 FK/FCL 独立重算通过。
- 正式 X5 C++ 求解器离线 1,397 帧，通过基座/工具、多轴、固定 TCP 旋转、释放重启及超过旧 50 mm 球的连续轨迹检查。独立复核实际/中点/候选共 4,191 次碰撞检查，无碰撞。
- 最终版只恢复显示网格；与完成求解器回归的候选 `281782c6…` 的 URDF、SRDF、附件和全部碰撞几何逐字节相同，最终版仍重新复核全部见证与轨迹。详 `cad-display-control-equivalence.json`。
- 后端相关 64 项、前端工作空间/输入/手柄 102 项通过，含并集失败回退、图层开关和卸载后资源释放。固定 phosphobot 正式构建通过。
- 首次正式页面发现旧采样点集被浏览器缓存，触发 `Reachability model mismatch`，截图保留。模型元数据与点集读取改用 `cache: no-store`；9 项受影响检查与正式构建再次通过，随后正常退出首个 v266 会话再发布，未热替换运行资源。

## 性能与未证明项

相同 1,224 姿态的 X5 FCL 对照中，碰撞姿态数 407→404。首轮新模型 P50/P95 为 11.599/34.466 ms，旧模型 10.477/18.443 ms。一次 ABBA 定向复核确认这是实际代价：旧 P95 18.421–18.465 ms，新 34.402–34.444 ms；新最大约 59.5 ms。没有降低碰撞检测频率、删除检查对或放宽阈值来掩盖。

碰撞检查沿用独立 10 Hz 线程。上述正式离线轨迹的控制回调 P95 2.044 ms、最大 20.041 ms，1,397 帧无回调达到 50 ms。此证据支持本次有限模型更新，但不等于全工况实时性能或新增实机运动验收。

黄色表示整个名义开闭行程，因此在闭合状态下仍明显宽于灰色夹指。完整被动指节受力形变、线缆和环境未标定/未建模，不能因图形看起来空就再压缩。该限制不改写为已解决。后续若需要随实际开度收缩碰撞体，需可靠关节/开度映射及形变范围，不用当前显示映射替代。

角度分段、全局高度/横向切片、各连杆独立旋转拟合等未采用方案和失败日志均保留在 `output/collision-review-v266/`。测试首次错误为选错不含 SciPy 的解释器；改用已有 CAD 环境依赖后通过，未改测试要求。

## 发布与回滚

发布包 `output/collision-review-v266/publish` 只包含 manifest 白名单资产与采样文件，未混入离线预览辅助文件。最终前端为 `dashboard-cachefix`，原版 dashboard 和配置基线均保存在同目录 `baseline`。

`teleop-platform/scripts/switch_collision_review_v266.py --check` 校验成套模型和构建；`--rollback --check` 校验旧版。只有当前会话正常退出且端口关闭后才能执行切换/回滚；原目录通过重命名保留，无破坏性清理。

发布前 20 个新状态确认未准备、静止、录制 idle、空队列且无报警，v265 会话正常 clean 退出，CPU governor 恢复。新会话保持既有使能与参数，不重新 Enable；运行会话见 STATUS 顶部。此次只读视觉和模型更新，不称扩大范围已实机验收。

## 开源复用

- 完整 URDFViewer 0.13.1 / Three.js 0.186.1：版本及许可证沿用既有清单；调用 `STLLoader`、`EdgesGeometry`，正式适配位于 `frontend/workspaceRenderer.ts`。
- [CadQuery 2.6.1](https://github.com/CadQuery/cadquery)（Apache-2.0）：`build_workspace_assets.py` 使用 OpenCascade 的盒体、刚体变换、Boolean union 和 tessellation。未自行实现实体布尔运算。
- [Trimesh 4.7.4](https://github.com/mikedh/trimesh)（MIT）：沿用切面和定向盒拟合；独立验证使用 `contains`，依赖固定 [Rtree 1.4.1](https://github.com/Toblerity/rtree)（MIT）空间索引。版本见 `scripts/requirements-workspace-cad.txt`。
- 正式前端源通过原 `prepare_upstream.py` 与固定补丁链进入 phosphobot；本次未另建 UI 主题或控制链路。

最终正式页：模型/点集/实时关节一致，RGB约20 fps；包络开关、展开和鼠标旋转后20个新状态均未准备、静止、无报警、空队列，命令ID仍为1，关节峰峰最大0.004120°。最终会话 `20261016-193321-915b7117`，exec `67321`，入口 `output/collision_review_operator_266b.py operator`，正常退出文件 `output/collision_review_operator_266b_operator.stop`。
