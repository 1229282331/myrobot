"""Single-owner control engine; UI submits intents through a bounded mailbox."""
from __future__ import annotations

from collections import deque
import copy
import math
import threading
import time
import uuid

from .device import LIMIT_NAMES, MockDevice, NativeDevice, opening_fraction
from .feedback import CdcFeedback, SimulatedFeedback, STALE_SECONDS


class Mailbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.commands = deque()
        self.latest = None
        self.stop_event = threading.Event()
        self.urgent = deque()

    def put(self, kind, value=None):
        with self.lock:
            if kind in ("stop", "disconnect", "close"):
                self.commands.clear()
                self.latest = None
                self.stop_event.set()
                if kind == "stop":
                    if not any(x[0] == "stop" for x in self.urgent):
                        self.urgent.appendleft((kind, value))
                elif not any(x[0] in ("close", "disconnect") for x in self.urgent):
                    self.urgent.append((kind, value))
            elif kind == "target":
                if not self.stop_event.is_set(): self.latest = value
            elif len(self.commands) < 8:
                self.commands.append((kind, value))

    def take_command(self):
        with self.lock:
            if self.urgent:
                return self.urgent.popleft()
            return self.commands.popleft() if self.commands else None

    def take_target(self):
        with self.lock:
            item, self.latest = self.latest, None
            return item

    def cancel(self):
        with self.lock:
            self.latest = None
            self.commands.clear()
            self.stop_event.set()


class ControlEngine:
    def __init__(self, emit=lambda *_: None, clock=time.perf_counter, factory=None, feedback_factory=None):
        self.emit = emit
        self.clock = clock
        self.factory = factory
        self.feedback_factory = feedback_factory
        self.feedback = None
        self.feedback_latest = None
        self.feedback_failed = False
        self.feedback_started = 0.0
        self.feedback_times = deque(maxlen=101)
        self.feedback_trail = deque(maxlen=201)
        self.last_feedback_publish = -math.inf
        self.last_debug_check = -math.inf
        self.box = Mailbox()
        self.device = None
        self.baseline = {}
        self.current = {}
        self.modified = False
        self.recovery = {}
        self.last_send = -math.inf
        self.last_poll = -math.inf
        self.sequence = None
        self.quit_requested = False
        self.state = dict(connected=False, busy=False, simulated=True, compatible=False,
                          enableState="unknown", locked=False, faultCode=-1, rtt=0.0,
                          firmware="—", serial="—", canId="—", ackTarget=-1.0,
                          testActive=False, testProgress="未运行", status="未连接", limits={}, baseline={})
        self.state.update(mayBeEnabled=False,modified=False)
        self.state.update(connectionId="", feedbackRequired=False, feedbackValid=False,
                          feedbackStatus="disconnected", feedbackMessage="连接后开始接收",
                          feedbackPosition=None, feedbackVelocity=None, feedbackColumn1=None,
                          feedbackAgeMs=-1.0, feedbackHz=0.0, feedbackPort="—", feedbackTrail=[])

    def publish(self):
        self.state["limits"] = dict(self.current)
        self.state["baseline"] = dict(self.baseline)
        self.state["modified"] = self.modified
        self.emit("state", dict(self.state))

    def log(self, message, level="info", **fields):
        self.emit("log", copy.deepcopy({"message": message, "level": level, "monotonic": self.clock(), **fields}))

    def submit(self, kind, value=None):
        if kind == "target" and (self.state["testActive"] or not self.state["connected"]):
            return
        self.box.put(kind, value)

    def _require(self, enabled=False, motion=False):
        if not self.device or not self.state["connected"]:
            raise ValueError("请先连接设备")
        if not self.device.compatible:
            raise ValueError("未知固件或接口：仅允许读取")
        if self.state["locked"]:
            raise ValueError("控制已锁定，请处理故障或重新连接")
        if enabled and self.state["enableState"] != "enabled":
            raise ValueError("请先在本程序中使能")
        if (enabled or motion) and self.state["feedbackRequired"]:
            if not self.state["feedbackValid"] or self._feedback_stale():
                raise ValueError("设备反馈不可用，核对 CDC 后重新连接")

    def _feedback_stale(self):
        return (self.feedback_latest is None or self.feedback_failed or
                self.clock() - self.feedback_latest["monotonic"] > STALE_SECONDS)

    def _target_cancelled(self):
        return self.box.stop_event.is_set() or (self.state["feedbackRequired"] and
                (self._feedback_stale() or bool(getattr(self.feedback, "error", None))))

    def _record_feedback(self, rows, stream_valid=True):
        if rows:
            self.emit("observation", [dict(row, source="simulation" if self.state["simulated"] else "usb_cdc",
                debug_flag=None if self.state["simulated"] else 56,
                clock="host_perf_counter_receive", position_resolution_fraction=.01,
                stream_accepted=bool(stream_valid and not self.feedback_failed)) for row in rows])

    def _close_feedback(self):
        if self.feedback:
            self.feedback.close()
            rows, error = self.feedback.poll()
            self._record_feedback(rows, stream_valid=not error)
        self.feedback = None
        self.feedback_latest = None
        self.feedback_times.clear()
        self.feedback_trail.clear()
        self.state.update(feedbackValid=False, feedbackStatus="disconnected", feedbackMessage="未连接",
                          feedbackPosition=None, feedbackVelocity=None, feedbackColumn1=None,
                          feedbackAgeMs=-1.0, feedbackHz=0.0, feedbackTrail=[], feedbackPort="—")

    def _start_feedback(self, device):
        self.feedback_latest = None
        self.feedback_failed = False
        self.feedback_started = self.clock()
        self.last_debug_check = self.clock()
        self.last_feedback_publish = -math.inf
        self.feedback_times.clear()
        self.feedback_trail.clear()
        self.state.update(feedbackRequired=bool(self.state["simulated"] or device.firmware == 335),
                          feedbackValid=False, feedbackStatus="waiting", feedbackMessage="等待 CDC 数据…",
                          feedbackPosition=None, feedbackVelocity=None, feedbackColumn1=None,
                          feedbackAgeMs=-1., feedbackHz=0., feedbackTrail=[], feedbackPort="—")
        if not device.compatible or not self.state["feedbackRequired"]:
            self.state.update(feedbackStatus="unsupported", feedbackMessage="当前固件的 CDC 格式尚未验证")
            return
        try:
            if not self.state["simulated"] and device.read("debug_flag") != 56:
                raise ValueError("debug_flag 不是已验证的 56；未修改设备配置")
            self.feedback = (self.feedback_factory(device) if self.feedback_factory else
                             SimulatedFeedback(device, self.clock) if self.state["simulated"] else
                             CdcFeedback(device.serial, self.clock))
            self.feedback.start()
            self.state["feedbackPort"] = self.feedback.port_name
            self.log("模拟反馈已启动" if self.state["simulated"] else "CDC 只读接收已启动", action="feedback_start",
                     port=self.feedback.port_name, debug_flag=None if self.state["simulated"] else 56,
                     clock="host_perf_counter_receive", position_resolution_fraction=.01)
            self._read_feedback()
        except Exception as exc:
            if self.feedback:
                self.feedback.close()
                self.feedback = None
            self._feedback_problem(str(exc))

    def _feedback_problem(self, message):
        if self.feedback_failed: return
        self.feedback_failed = True
        self.state.update(feedbackValid=False, feedbackStatus="unavailable", feedbackMessage=message,
                          feedbackPosition=None, feedbackVelocity=None, feedbackColumn1=None)
        self.box.take_target()
        self.log("反馈不可用：" + message, "error", action="feedback_lost")
        # A read-only connection must never change the existing motor state.
        # Stop only motion this program has enabled (or an active sequence).
        if self.state["enableState"] == "enabled" or self.state["testActive"]:
            self._trip("反馈中断，动作已取消：" + message)
        else:
            self.publish()

    def _read_feedback(self):
        if not self.feedback: return
        now = self.clock()
        rows, error = self.feedback.poll()
        previous = self.feedback_latest["monotonic"] if self.feedback_latest else None
        for row in rows:
            if previous is not None and row["monotonic"] - previous > STALE_SECONDS:
                error = error or "接收时间出现超过 300 ms 的间断；需重新连接"
            previous = row["monotonic"]
        self._record_feedback(rows, stream_valid=not error)
        if rows:
            self.feedback_latest = rows[-1]
            self.feedback_times.extend(row["monotonic"] for row in rows)
        age = now - self.feedback_latest["monotonic"] if self.feedback_latest else None
        self.state["feedbackAgeMs"] = max(0., age * 1000) if age is not None else -1.
        if error:
            self._feedback_problem(error)
        elif age is not None and age > STALE_SECONDS:
            self._feedback_problem("超过 300 ms 未收到新数据；需重新连接")
        elif age is None and now - self.feedback_started > 2:
            self._feedback_problem("2 秒内未收到有效数据；需重新连接")
        if self.feedback_failed: return
        if self.feedback_latest:
            r = self.feedback_latest
            span = self.feedback_times[-1] - self.feedback_times[0]
            hz = (len(self.feedback_times)-1)/span if span > 0 else 0.
            self.state.update(feedbackValid=True, feedbackStatus="simulated" if self.state["simulated"] else "live",
                              feedbackMessage="模拟生成 · 非实机观测" if self.state["simulated"] else "CDC 正在接收",
                              feedbackPosition=r["position_fraction"] * 100,
                              feedbackVelocity=r["velocity_fraction_s"] * 100,
                              feedbackColumn1=r["column1_unverified"], feedbackHz=hz)
            if now - self.last_feedback_publish >= .05:
                self.last_feedback_publish = now
                self.feedback_trail.append(dict(t=r["monotonic"], position=r["position_fraction"]*100,
                                                target=self.state["ackTarget"]))
                self.state["feedbackTrail"] = list(self.feedback_trail)
                self.publish()

    def _cancel_sequence(self):
        self.sequence = None
        self.state["testActive"] = False

    def _disable(self):
        if not self.device or not self.device.compatible:
            raise ValueError("没有可下使能的兼容设备")
        if self.state["enableState"] == "disabled" and not self.state["mayBeEnabled"]:
            return  # FW330 may reject redundant transitions; retain our confirmed result.
        self.device.request_state(False)
        self.state["enableState"] = "disabled"
        self.state["mayBeEnabled"] = False
        self.log("下使能请求已确认", action="disable")

    def _restore(self):
        if self.modified:
            if self.state["enableState"] != "disabled":
                raise ValueError("先下使能，再恢复参数")
            for name, value in self.baseline.items():
                self.current[name] = self.device.set_limit(name, value)
            self.modified = False
            self.recovery.pop(self._identity(),None)
            self.log("连接时的六项原参数已恢复并读回", action="restore", values=self.current)

    def _trip(self, error, transport=False, attempt_stop=True):
        self.box.cancel()
        self._cancel_sequence()
        self.state["locked"] = True
        self.state["status"] = str(error)
        self.log(str(error), "error")
        try:
            if not attempt_stop or not self.device: raise ValueError("通信失败，未重试写操作")
            self._disable()
        except Exception as exc:
            self.state["enableState"] = "unknown"
            self.state["status"] += " · 停止未确认"
            self.log("停止未确认：" + str(exc), "error")
        if transport and self.device:
            if self.modified: self.recovery[self._identity()] = dict(self.baseline)
            try:
                self._close_feedback()
                self.device.close()
            except Exception: pass
            self.device = None
            self.state["connected"] = False
        self.publish()

    def _identity(self):
        return (self.state["simulated"], self.state["serial"])

    def _connect(self, selection):
        if self.device:
            raise ValueError("请先断开当前设备")
        self.box.cancel()
        self._cancel_sequence()
        simulated, serial = selection
        self.state.update(busy=True, status="正在连接…")
        self.publish()
        device = (self.factory() if self.factory else (MockDevice(serial) if simulated else NativeDevice(serial)))
        try:
            device.open()
            fault = device.fault()
            limits = device.limits() if device.compatible else {}
        except Exception:
            device.close()
            raise
        self.device = device
        device.cancel_motion = self._target_cancelled
        identity = (simulated,f"{device.internal_serial:012X}")
        self.baseline = dict(self.recovery.get(identity,limits))
        self.current = dict(limits)
        self.modified = identity in self.recovery
        self.last_send = -math.inf
        self.last_poll = self.clock()
        self.box.stop_event.clear()
        self.state.update(connected=True, busy=False, simulated=simulated, compatible=device.compatible,
                          enableState="unknown", mayBeEnabled=True, locked=bool(fault), faultCode=fault,
                          firmware=f"{device.firmware // 100}.{device.firmware // 10 % 10}.{device.firmware % 10}",
                          serial=f"{device.internal_serial:012X}", canId=str(device.can_id),
                          ackTarget=-1.0, testActive=False, testProgress="未运行",
                          status="已连接 · 状态未知" if device.compatible else "已连接 · 只读模式")
        self.state["connectionId"] = uuid.uuid4().hex
        self.log("模拟设备已连接" if simulated else "原生 USB 已连接（只读初始化）", action="connect",
                 serial=self.state["serial"], firmware=device.firmware, limits=limits, simulated=simulated)
        self._start_feedback(device)

    def _command(self, kind, value):
        if kind == "connect":
            self._connect(value)
        elif kind in ("stop", "disconnect", "close"):
            self._cancel_sequence()
            self.state["testProgress"] = "已停止"
            if kind == "stop":
                self._disable()
                self.state["status"] = "已下使能"
            else:
                if not self.device and value == "disable" and (self.state["mayBeEnabled"] or self.modified):
                    raise ValueError("设备已断连，须重新连接才能确认下使能并恢复参数")
                if self.device and value == "disable":
                    self._disable()
                    self._restore()
                elif self.device and value != "keep":
                    raise ValueError("退出方式无效")
                self._close_feedback()
                if self.device: self.device.close()
                if value == "keep":
                    self.recovery.pop(self._identity(),None)
                    self.modified = False
                    self.log("按选择关闭连接；未下使能，RAM 参数保持",action="leave_keep")
                self.device = None
                self.state.update(connected=False, enableState="unknown", ackTarget=-1.0, status="未连接")
                if kind == "close": self.quit_requested = True
            self.box.stop_event.clear()
        elif kind == "enable":
            self._require(motion=True)
            if self.state["testActive"]: raise ValueError("请先停止测试")
            if self.state["enableState"] == "enabled": return
            if self.device.fault() or not self.device.read("motor.config.calib_valid"):
                raise ValueError("存在故障或校准无效")
            if self.device.read("motor.config.ctrl_mode") != 5:
                raise ValueError("设备不是已验证的位置斜坡模式")
            self.state["mayBeEnabled"] = True
            self.state["enableState"] = "unknown"  # A lost ACK may still have energized the motor.
            self.device.request_state(True)
            self.box.stop_event.clear()
            self.state.update(enableState="enabled", status="使能请求已确认")
            self.log("使能请求已确认；未发送开度", action="enable")
        elif kind == "apply":
            self._require()
            if self.state["enableState"] != "disabled": raise ValueError("请先下使能，再应用参数")
            if set(value) != set(LIMIT_NAMES): raise ValueError("参数集合不完整")
            for name, number in value.items():
                if not math.isfinite(number) or not 0 < number <= self.baseline[name] + 1e-7:
                    raise ValueError("参数必须大于0且不超过连接时原值：" + name)
            self.modified = True  # Even a partial write must be recoverable.
            for name, number in value.items():
                if self.box.stop_event.is_set(): raise ValueError("参数应用被停止操作取消")
                self.current[name] = self.device.set_limit(name, number)
            self.state["status"] = "临时参数已应用并读回"
            self.log(self.state["status"], action="apply", values=self.current)
        elif kind == "restore":
            self._require()
            self._restore()
            self.state["status"] = "原参数已恢复"
        elif kind == "clear":
            if not self.device or not self.device.compatible: raise ValueError("设备不可清错")
            if self.state["enableState"] != "disabled": raise ValueError("请先确认下使能")
            self.device.clear_fault()
            fault = self.device.fault()
            self.state.update(faultCode=fault, locked=bool(fault), status="清错后故障码：" + str(fault))
            if not fault: self.box.stop_event.clear()
            self.log(self.state["status"], action="clear_fault")
        elif kind == "test":
            self._require(enabled=True)
            if self.state["testActive"]: raise ValueError("测试已在运行")
            a, b, dwell, cycles = value
            opening_fraction(a); opening_fraction(b)
            if not math.isfinite(dwell) or not 0.5 <= dwell <= 60 or not 1 <= cycles <= 100 or cycles != int(cycles):
                raise ValueError("停留0.5–60秒，往返次数1–100")
            self.box.take_target()
            self.sequence = {"targets": [a] + [x for _ in range(int(cycles)) for x in (b, a)], "index": 0, "deadline": 0., "dwell": dwell}
            self.state.update(testActive=True, testProgress="准备运行", status="往返测试中")
            self.log("开始有限次往返测试", action="test", a=a, b=b, dwell=dwell, cycles=cycles)
        elif kind == "mock_fault":
            if isinstance(self.device, MockDevice): self.device.error = 7
        elif kind == "mock_timeout":
            if isinstance(self.device, MockDevice): self.device.fail_next = True
        elif kind == "mock_feedback_pause":
            if isinstance(self.feedback, SimulatedFeedback): self.feedback.paused = True

    def tick(self):
        command = self.box.take_command()
        if command:
            try:
                if command[0] not in ("connect", "stop", "disconnect", "close"):
                    self._read_feedback()
                self._command(*command)
            except ValueError as exc:
                self.state.update(busy=False, status=str(exc))
                self.log(str(exc), "warning")
                # Failed closing must never close the window or pretend success.
                if command[0] in ("stop", "disconnect", "close"):
                    self.state["status"] = "停止未确认 / 退出未完成：" + str(exc)
            except Exception as exc:
                self.state["busy"] = False
                self._trip(exc, transport=True, attempt_stop=command[0] not in ("stop", "disconnect", "close"))
            self.publish()
            return
        if not self.device or not self.state["connected"]: return
        try:
            now = self.clock()
            self._read_feedback()
            if now - self.last_poll >= 0.2:
                fault = self.device.fault()
                self.last_poll = self.clock()
                self.state.update(faultCode=fault, rtt=round(self.device.rtt_ms, 2))
                self.emit("telemetry",dict(monotonic=self.clock(),fault_code=fault,rtt_ms=self.device.rtt_ms,
                    enable_request=self.state["enableState"],ack_target=self.state["ackTarget"]))
                if fault and not self.state["locked"]:
                    self._trip("设备故障码：" + str(fault))
                    return
                self.publish()
            if self.feedback and not self.state["simulated"] and not self.feedback_failed and now - self.last_debug_check >= 1:
                self.last_debug_check = now
                if self.device.read("debug_flag") != 56:
                    self._feedback_problem("debug_flag 已改变，数据列含义失效；需重新连接")
                    self.publish()
            if self.state["locked"] or self.state["enableState"] != "enabled":
                self.box.take_target()
                return
            target = None
            if self.sequence:
                seq = self.sequence
                if now >= seq["deadline"]:
                    if seq["index"] == len(seq["targets"]):
                        self.box.take_target()
                        self._cancel_sequence()
                        self.state.update(testProgress="已完成 · 保持末次目标", status="往返指令已完成")
                        self.log("往返指令完成，未用位置反馈判定到位", action="test_completed")
                        self.publish()
                    else:
                        target = seq["targets"][seq["index"]]
            elif now - self.last_send >= 0.05:
                target = self.box.take_target()
            if target is not None and not self.box.stop_event.is_set():
                self._require(enabled=True)
                call_started = self.clock()
                if self.device.set_opening(target):
                    self.last_send = self.clock()
                    self.state["ackTarget"] = round(target, 1)
                    self.state["status"] = f"目标 {target:.1f}% · 指令已确认"
                    self.log(self.state["status"], action="set_opening", target=target,
                             call_started_monotonic=call_started, target_unit="percent")
                    if self.sequence:
                        self.sequence["index"] += 1
                        self.sequence["deadline"] = self.clock() + self.sequence["dwell"]
                        self.state["testProgress"] = f'{self.sequence["index"]} / {len(self.sequence["targets"])} 段'
                    self.publish()
        except Exception as exc:
            self._trip(exc, transport=True)

    def release_resources(self):
        """Process shutdown only: normal UI exits already made the user's choice."""
        try:
            self._close_feedback()
        finally:
            if self.device:
                self.device.close()
                self.device = None
