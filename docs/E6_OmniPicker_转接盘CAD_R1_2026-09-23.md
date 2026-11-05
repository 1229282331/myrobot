# 转接盘 CAD R1 交付记录

后续版本：用户要求增加E6外圈8个M3备用位，已交付 [R2](E6_OmniPicker_转接盘CAD_R2_2026-09-23.md)，本次加工以R2为准。R1保留作为历史记录。

2026-09-23，用户确认此前保留原后盖的单片铝盘方案，要求 CAD 和螺钉采购单。已离线完成，并非实物试装结果。

## 文件

- 加工单件 STEP：`output/cad/E6_OmniPicker_adapter_R1/E6_OMNI_ADAPTER_R1.step`
- 二维 DXF：同目录 `E6_OMNI_ADAPTER_R1.dxf`
- 加工图与采购单：`output/pdf/E6_OmniPicker_转接盘加工图与采购单_R1.pdf`（2页）
- 同目录 README、CSV采购单、JSON设计参数/检查记录、3张实际BREP渲染、参考装配STEP。
- 打包文件：`output/cad/E6_OmniPicker_adapter_R1.zip`
- 离线生成：`tools/build_e6_omnipicker_adapter.py`（CadQuery 2.6.1、ezdxf 1.4.3，项目独立 `.venv-cad`）；PDF生成：`tools/build_e6_omnipicker_adapter_pdf.py`（bundled Python/ReportLab）。没有执行任何机器人、夹爪或网络控制。

## 相比初设的定稿变化

1. 保持6061-T6、Φ65×7、PCD50四个Φ6.6 M6过孔、PCD25四个M3。
2. M6固定DIN7991头部，B面沉头Φ12.4×90°；选M6×12，沉入后实际穿出并非严格5mm，验收4.8-5.5、距实物孔底至少0.5。
3. M3由初选6长改8长；指定NBK **SSHS-M3-8-SD**，头径5、高1.3、A2-50、1.5内六角，厂家中国尺寸表已核实。原后盖沉孔Φ6×1.5。避免普通SSHS头径6贴边及普通圆柱头过高。
4. 机械臂面A增加Φ34×深1mm浅槽作为避让设计量；不作定位止口，不装定位销。M3攻通至浅槽，入口Φ3.4×90°，材料厚6mm；STEP用底孔简化，PDF明确攻牙。理论体积20766.43mm³，盘重56.07g，未含螺钉。

## 验证及未验证项

单件STEP导出再导入为1个有效实体；外形与8个孔中心反提取通过。DXF审计通过。官方后盖真实STEP归一化到B面后体积交集0，四孔同轴；简化螺钉头/杆包络与后盖无交集，M6与盘也无交集。M3螺纹配合未做真实牙型建模。

发现二维图夹持厚度2.5与当前官方STEP的2.8966存在差异；这两种口径下M3×8名义伸入5.1-5.5，距A槽底0.5-0.9，均已列出；**未证明用户20板实物就是其中一款后盖**。需实测后盖孔距、沉孔、夹持厚度、金色螺母/隔柱及贴合面。

E6中央结构只有官方平面图，无完整实测三维包络；浅槽仅覆盖Φ34内且突出≤0.8的条件，需在下料前核对。未建模PCB、完整夹爪、实际插头/线缆扫掠；未做FEA/承载验证。整机J4/J5保持问题仍未解决。

## 螺钉来源

- [NBK官方小头超短头尺寸表](https://www.nbk1560-chn.com.cn/products/specialscrew/nedzicom/lowsmallheadscrew/SSHS-SD/)：通过浏览器可见表核实 M3-8-SD 的L8/D1=5/L1=1.3/B1.5、0.5g、A2-50；PDF直链403，没有绕过。
- [Brighton-Best DIN7991规格](https://www.brightonbest.com/download/pds/PDS_Metric_Hexagon_Socket_Countersunk_Head_Cap_Screws.pdf)：M6头径11.57-12、高3.3、4mm内六角；DIN/ISO不可随意互换。原PDF与提取文本在 `research/grippers/omnipicker_mount_2026-09-23/fasteners/`。
- 后盖、M2.5×8及E6接口来源沿用机械转接方案中的官方资料。M2.5侧螺钉仅按官方示例备购，头型须和实物原件比对。

首件图已渲染视觉检查。未采购、联系商家、发送资料或安装。
