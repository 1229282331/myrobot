"""Simulation gateway state machine; leases are evaluated only on the gateway clock."""
from dataclasses import dataclass
import math
import secrets
import time
import numpy as np
from .math3d import integrate


@dataclass(frozen=True)
class Settings:
    period_s: float = .05
    lease_s: float = .25
    translation_m_s: float = .01
    rotation_rad_s: float = math.radians(5)
    gripper_s: float = .1


class InputError(ValueError):
    pass


def vector(value, length):
    if not isinstance(value, list) or len(value) != length:
        raise InputError(f"Expected {length} numeric values")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise InputError("Input must contain finite numbers")
    return np.asarray(value, dtype=float)


class GatewayCore:
    def __init__(self, model, clock=time.perf_counter_ns, settings=None):
        self.model, self.clock = model, clock
        self.settings = settings or Settings()
        self.owner = None
        self.lease = None
        self.lease_ns = 0
        self.input_ns = 0
        self.input = None
        self.seq = -1
        self.armed = False
        self.reason = "请连接输入设备并准备控制"
        self.last_tick = clock()
        self.tick_id = 0
        self.target_position, self.target_quaternion = model.pose()

    def stop(self, reason):
        self.armed = False
        self.reason = reason
        self.target_position, self.target_quaternion = self.model.pose()

    def new_lease(self):
        self.lease = secrets.token_urlsafe(24)
        self.lease_ns = self.clock()

    def connect(self):
        if self.owner is not None:
            raise InputError("已有页面持有控制权")
        self.owner = secrets.token_urlsafe(24)
        self.input = None
        self.seq = -1
        self.stop("已连接：回中并松开允许动作键，然后准备控制")
        self.new_lease()
        return self.status()

    def disconnect(self, owner):
        if self.owner == owner:
            self.stop("连接已断开，需要重新准备")
            self.owner = None
            self.input = None

    def validate_owner(self, owner):
        if owner != self.owner or self.owner is None:
            raise InputError("没有控制权")

    def command(self, data):
        self.validate_owner(data.get("session"))
        operation = data.get("operation")
        if operation == "stop":
            self.stop("操作员停止，需要重新准备")
        elif operation == "arm":
            f = self.input
            if not f or (self.clock()-self.input_ns)/1e9 > self.settings.lease_s:
                raise InputError("需要新鲜的回中输入")
            if not f["connected"] or not f["focused"] or not f["neutral"] or f["deadman"] or f["stop"] or any(f["twist"]) or f["gripper_rate"]:
                raise InputError("请回中、松开扳机和允许动作键")
            self.armed = True
            self.reason = "已准备：按住允许动作键随动"
            self.target_position, self.target_quaternion = self.model.pose()
        elif operation == "input":
            try:
                if data.get("lease") != self.lease or (self.clock()-self.lease_ns)/1e9 > self.settings.lease_s:
                    raise InputError("输入租约过期或已被消费")
                if type(data.get("seq")) is not int or data["seq"] <= self.seq:
                    raise InputError("输入序号必须递增")
                twist = vector(data.get("twist"), 6)
                grip = data.get("gripper_rate")
                if type(grip) not in (int, float) or not math.isfinite(grip) or abs(grip) > self.settings.gripper_s + 1e-9:
                    raise InputError("夹爪变化率越界")
                if np.linalg.norm(twist[:3]) > self.settings.translation_m_s + 1e-9 or np.linalg.norm(twist[3:]) > self.settings.rotation_rad_s + 1e-9:
                    raise InputError("末端速度越界")
                for key in ("connected", "focused", "deadman", "fine", "stop", "neutral"):
                    if type(data.get(key)) is not bool:
                        raise InputError("输入状态必须为布尔值")
                if data.get("frame") not in ("base", "tool") or data.get("mode") not in ("translation", "rotation"):
                    raise InputError("未知坐标系或控制模式")
                if data["mode"] == "translation" and any(twist[3:]) or data["mode"] == "rotation" and any(twist[:3]):
                    raise InputError("输入与控制模式不符")
                if data.get("source") not in ("gamepad", "keyboard") or not isinstance(data.get("identity"), str) or not 0 < len(data["identity"]) <= 256:
                    raise InputError("未知输入设备")
                raw = data.get("raw", {})
                if not isinstance(raw, dict):
                    raise InputError("原始输入必须为对象")
                # Source/frame changes cannot redirect a held stick during active control.
                if self.input:
                    keys = ("source", "identity", "frame", "mode")
                    changed = any(data[k] != self.input[k] for k in keys)
                    if changed and (data["deadman"] or not data["neutral"] or any(twist) or grip):
                        raise InputError("切换设备、模式或坐标系前必须松开并回中")
                    if changed and any(data[k] != self.input[k] for k in ("source", "identity")):
                        self.stop("输入设备已切换，需要重新准备")
                self.seq = data["seq"]
                self.input_ns = self.clock()
                self.input = {k: data[k] for k in ("source", "identity", "frame", "mode", "connected", "focused", "deadman", "fine", "stop", "neutral", "raw") if k in data}
                self.input.update(twist=twist.tolist(), gripper_rate=float(grip), seq=self.seq,
                                  pc_input_monotonic_ns=data.get("pc_input_monotonic_ns"), gateway_received_ns=self.input_ns)
                if data["stop"] or not data["connected"] or not data["focused"]:
                    self.stop("停止／输入断开／页面失焦，需要重新准备")
                elif not data["deadman"]:
                    self.target_position, self.target_quaternion = self.model.pose()
            except (ValueError, KeyError, TypeError):
                self.stop("输入被拒绝，需要重新准备")
                raise
            finally:
                self.new_lease()
        else:
            self.stop("未知命令，需要重新准备")
            raise InputError("未知命令")
        return self.status()

    def status(self):
        return dict(session=self.owner, lease=self.lease, armed=self.armed, reason=self.reason,
                    backend="simulation", hardware_connected=False, seq=self.seq)

    def tick(self):
        now = self.clock()
        dt = (now-self.last_tick)/1e9
        self.last_tick = now
        self.tick_id += 1
        if dt <= 0 or dt > .15:
            self.stop("控制循环延迟，需要重新准备")
            dt = 0
        if self.input and (now-self.input_ns)/1e9 > self.settings.lease_s:
            self.stop("输入看门狗超时，需要重新准备")
        before_position, before_quaternion = self.model.pose()
        observation = dict(joints_rad=self.model.q.tolist(), tcp_position_m=before_position.tolist(),
                           tcp_quaternion_xyzw=before_quaternion.tolist(), gripper=self.model.gripper,
                           gateway_monotonic_ns=now, frame_id=self.tick_id, valid=True, is_simulated=True)
        f = self.input
        active = bool(self.armed and f and f["deadman"] and f["focused"] and f["connected"] and not f["stop"])
        requested = None
        errors = None
        ik_result = dict(attempted=False, accepted=False, reason=None)
        q_target, g_target = self.model.q.copy(), self.model.gripper
        if active:
            scale = .25 if f["fine"] else 1.
            twist = np.asarray(f["twist"]) * scale
            position, quaternion = integrate(self.target_position, self.target_quaternion, twist[:3], twist[3:], dt, f["frame"])
            requested = dict(position_m=position.tolist(), quaternion_xyzw=quaternion.tolist())
            ik_result["attempted"] = True
            try:
                q_target, errors = self.model.solve(position, quaternion)
                ik_result["accepted"] = True
                g_target = float(np.clip(self.model.gripper + f["gripper_rate"]*dt*scale, 0, 1))
                self.target_position, self.target_quaternion = position, quaternion
            except ValueError as exc:
                ik_result["reason"] = str(exc)
                self.stop(str(exc))
                active = False
        # o_t is captured BEFORE applying a_t, including the rendered image.
        images = {name: self.model.render(name) for name in ("overview", "wrist")}
        action = dict(joints_rad=q_target.tolist(), gripper=g_target, active=active, input_seq=self.seq,
                      gateway_applied_ns=self.clock(), requested_tcp=requested, ik_residual=errors, ik_result=ik_result)
        self.model.set_state(q_target, g_target)
        return dict(observation=observation, effective_action=action, input=f, status=self.status(), dt_s=dt), images
