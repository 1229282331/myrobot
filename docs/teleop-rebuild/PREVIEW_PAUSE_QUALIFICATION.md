# 预览暂停复核（v211）

2026-10-09。生产前端、视频编码和500ms新帧判定均未修改。

## 假设与证据

v210暂停附近曾截屏，时间邻近不能证明截屏导致冻结。保留页面后由v211重连保存视频测量原始报告；connection1为v210的3065样本，connection2为新会话，不能合并冒充同一次测量。

旧故障前接收/解码从约20fps降至10–11fps，随后约42fps追赶；jitter约10–15ms升至70–91ms、freezeCount1→7。解码约0.5–1ms、QR约2–4ms且全报告最大6.6ms，getStats仍约每秒回调。未发现浏览器主线程冻结证据。packetsLost=0仅覆盖PC MediaMTX至本机浏览器，不能排除X5至PC无线/SSH TCP抖动。QR源时间给出的延迟上界曾约1.4秒且时钟误差区间增宽。

stop-input.json记录preview_stale。该时刻X5相机valid、年龄87.970ms，E6年龄8.765ms，问题发生在采集之后到浏览器接收/呈现之间，编码、SSH、无线的具体占比仍未确定。暂停及显式重准备成功，无需重启；不能为消除该正常保护而放宽500ms。

## 临时省电对照

X5 wlan0原power_save=on，aicwf_sdio驱动。只对独立20Hz、500字节、TCP_NODELAY=1的SSH echo做on/off/off/on，同一连接每组600包全部返回。P95分别20.9585/21.109/21.0094/20.9824ms，max78.8535/29.1769/29.6544/30.3143ms。未显示稳定P95改善，不采用生产变更；finally恢复on并回读。未改PC无线设置。

## 结果与下一步

正式v211 shadow：71807官方SDK帧有效无缺号，全Mode5/Enable1/Error0/Collision0，候选、运动、Stop、CAN均0；无新episode。正常退出并核对设备/C++/相机/proc资源空。继续原Z路线的正式实机随动验证，保留本次失败与恢复事实，不无限重跑静止诊断。

证据：evidence/preview-pause-v211中的原视频报告、stop-input、network-readonly、power-save-echo-results及探针；evidence/live-jazzy-v211中的原包、events、motion、timing-and-model与资源核查。分析首次误用不含SciPy的3.12，随后使用既有3.10环境完成；设备测试未重跑。
