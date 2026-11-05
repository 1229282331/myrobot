# v329 Jazzy 共享内存生命周期修复（候选）

2026-10-20 后续状态：四份旧审计日志的无损归档及恢复检查已完成，减少重复占用约12.7GB。但同D候选、同Python源的相机负载复测仍未通过：高速关节一次发送前截止拒绝、高速交替退出日志线程未结束、最后Tool组缺少总时限后的汇总。平台已正常释放，当前无owner，候选仍未live发布。完整新证据和时序见 `STORAGE_RECOVERY_V330.md` 与 `evidence/storage-retest-v330`；下方旧“准备归档”等描述为当时过程记录。

当前未发布。E6 直连已恢复，但软件隔离测试继续出现几何时限失败和进程清理延迟，不能将全部问题归因于路由器。

## 原始证据和判断边界

v328 C 六组完整链路的 medium-alt 运动段没有暂停，结束后 native 进程在 unlinkat/do_get_write_access 中出现91个D态样本，约5.57秒，随后被测试器结束，缺少worker退出凭据。原失败保留，不能算完整通过。

根文件系统检查发现 `/root/e6-teleop-rebuild/jazzy-v32/rootfs/dev/shm` 是 eMMC ext4 上的普通目录，不是 tmpfs。仅设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 并没有消除 FastDDS 对共享内存文件的操作。

v329 在独占锁、独立网络及私有挂载 namespace 内做 disk-a / ram-a / ram-b / disk-b 对照。每组1秒合成输入，只跟踪原生子进程 unlink/unlinkat，不连接机器人。原目录文件保留；RAM组仅在子namespace覆盖128MiB tmpfs。160文件/2,375,971字节证据包逐成员及双端SHA校验。

strace确认原生进程删除 `/dev/shm/fastrtps_*`、`/dev/shm/sem.fastrtps_*`。四组worker均干净退出，没有复现旧多秒停顿；unlink最大耗时依次0.521/0.305/0.295/0.551ms。由于使用ptrace且输入仅1秒，这不是性能资格，也不能用这轮证明旧437.985ms首帧等待的根因。证据：`evidence/runtime-shm-v329/analysis.json`。

## 源码修复

`jazzy_host.isolated_runtime_argv` 在正式worker原有独占进程组内增加私有mount namespace，将该rootfs的dev/shm挂为128MiB tmpfs，权限1777并设置nosuid/nodev/noexec。启动失败即拒绝进入worker，不回退到磁盘目录。参数通过argv传递；根目录和逃逸符号链接拒绝。

仍由原有SIGINT/TERM/KILL和进程组清理管理全部子进程。挂载不传播至父会话，最后一个子进程退出后由内核释放，原ext4文件不删除。每个worker会话保存`shm-mount.json`作为实际挂载证据。模型、ACM、TCP、负载及37/42/50ms、100/250ms全部保持。

基线源通过重建匹配v322实际release清单哈希，保存在`output/runtime-shm-v329/baseline/jazzy_host.py`；补丁`private-shm.patch`。尚未对生产进行切换。

## 验证范围

- 正式Windows后端：1,167通过、9跳过；Linux针对测试3通过。两个独立小venv缺pytest，Linux完整套件缺ruckig/phosphobot的初次收集失败保留；最终完整测试使用项目既有正式环境。
- WSL生命周期实测因sudo需要密码未运行，不计通过；随后RDK正常退出、非零退出和SIGTERM三类生命周期均通过，父挂载和原文件保持。
- D空闲完整软件链路六组共3,542次合成写入，无暂停、拒绝或退离预算耗尽，六worker退出0。此项不测量实机动力学。
- 带相机的首轮负载测试未通过：medium-joint退出等待超时，缺summary；另外五组共2,720次合成写，high-alt一次original_candidate_deadline暂停，high-tool一次退离预算耗尽。167文件、51,574,605字节原始压缩包经双端及逐成员SHA核对；失败未覆盖为成功。
- high-alt候选523到达主机时，其源反馈已经约68 ms；随后控制锁等待约21 ms、tick约37 ms，发送前超过原截止约10 ms。相关窗口无GC事件，但不能据此认定全部长尾来源已明。时序见`evidence/runtime-shm-v329/loaded-failure-timing.json`。
- 次轮只读shadow `20261020-095215-ce246272`发生`audit_raw_overflow`，不算健康平台负载资格。新内核只读诊断直接观察到audit子进程`do_get_write_access`与jbd2/mmc写等待，根盘99%、约1.2 GB空闲。两项证据相关，但尚不证明所有超时均由存储容量造成。
- 三组加内核观察的诊断全部清理完成，medium-joint 536写、high-alt 502写/4暂停、high-tool 516写/2暂停及1退离budget；诊断负载及同时磁盘盘点的影响保留，不作为性能资格。两只读shadow均经stopfile正常退出，原始反馈命令ID17517不增；当前无控制owner。未切生产native、未本轮实机运动或设备参数写入。

## v330 存储阻塞后续处理

只读容量盘点发现历史平台会话约63 GB。准备对明确已clean退休的`20261018-144306-c02ecf61`、`20261017-191600-abb23d24`原始审计文件做无损归档；不改写记录内容，不处理episodes。每文件先保留原件，压缩并核对PC/RDK压缩与解压SHA；全部核对后才移除重复明文。归档工具`output/archive_journals_330.py`，恢复工具`output/restore_journals_330.py SESSION --check`。完成凭据保存在`evidence/storage-archive-v330`；在出现completion凭据前不能称已归档。

自主抓杯继续取消，Goal保持active。下一次物理动作前新读SDK反馈与画面。
