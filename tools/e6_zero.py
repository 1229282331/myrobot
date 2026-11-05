"""Preview or execute one slow E6 joint move to [0, 0, 0, 0, 0, 0].

Python standard library only. Default: read-only preview.
Protocol reference: Dobot TCP-IP-Python-V4, commit
82bce656c6e1a1c71599901baa93ed45bb50963d, MovJ / EnableRobot / Stop.
"""

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import logging
import math
from pathlib import Path
import re
import socket
import sys
import time

try:  # Supports both `python tools/e6_zero.py` and module imports in tests.
    from .probe_e6_readonly import read_reply
except ImportError:
    from probe_e6_readonly import read_reply


LOG = logging.getLogger("e6_zero")
TARGET_DEG = (0.0,) * 6
TOLERANCE_DEG = 0.1
MAX_START_CHANGE_DEG = 1.0
MAX_TRAVEL_DEG = 180.0
MAX_SAMPLE_SECONDS = 1.0
POLL_SECONDS = 0.1
STABLE_SAMPLES = 3
# Conservative sanity bounds from the official me6_robot.urdf, commit
# ec201c40d5db63185a3593ec9c39f80fe25aa527. Not collision or path validation.
JOINT_BOUNDS_DEG = tuple(math.degrees(v) for v in (6.27, 2.356, 2.6878, 2.7925, 3.0194, 6.27))


class E6Error(RuntimeError):
    """Protocol, state, or execution failure; never automatically recovered."""


def parse_reply(raw, command):
    if raw == "Control Mode Is Not Tcp":
        raise E6Error("机器人不在 TCP/IP 模式，请在 DobotStudio Pro 中检查设备模式。")
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*\{(.*?)\}\s*,\s*(.*?)\s*;", raw, re.DOTALL)
    if not match:
        raise E6Error(f"无法识别回复：{raw!r}")
    error_id, payload, echo = match.groups()
    if re.sub(r"\s+", "", echo) != re.sub(r"\s+", "", command):
        raise E6Error(f"回复与请求不匹配：请求 {command!r}，回复 {raw!r}")
    if int(error_id) != 0:
        raise E6Error(f"控制器拒绝 {command}，ErrorID={error_id}，原始回复：{raw}")
    return payload.strip()


class Dashboard:
    """One connection, one outstanding command, no retries or auto-reconnect."""

    def __init__(self, ip, timeout=1.0):
        self.ip = ip
        self.timeout = timeout
        self.connection = None

    def __enter__(self):
        self.connection = socket.create_connection((self.ip, 29999), timeout=self.timeout)
        LOG.info("连接 %s:29999，本机地址 %s", self.ip, self.connection.getsockname()[0])
        return self

    def __exit__(self, *exception):
        if self.connection is not None:
            self.connection.close()

    def command(self, command, *, timeout=None):
        LOG.debug("TX %s", command)
        command_timeout = self.timeout if timeout is None else timeout
        self.connection.settimeout(command_timeout)
        self.connection.sendall(command.encode("ascii"))
        raw = read_reply(self.connection, command_timeout)
        LOG.debug("RX %s", raw)
        return parse_reply(raw, command)


@dataclass(frozen=True)
class State:
    mode: int
    joints: tuple
    received_at: float


def alarm_codes(value):
    """Support [] and nested per-controller/per-servo arrays; fail closed."""
    if not isinstance(value, list):
        raise E6Error("GetErrorID() 未返回报警数组。")
    result = []
    for item in value:
        if isinstance(item, list):
            result.extend(alarm_codes(item))
        elif type(item) is int:
            if item != 0:
                result.append(item)
        else:
            raise E6Error(f"无法识别报警数据：{item!r}")
    return result


def validate_joints(joints):
    if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
        raise E6Error("关节反馈必须是六个有限角度值。")
    for index, (value, bound) in enumerate(zip(joints, JOINT_BOUNDS_DEG), 1):
        if abs(value) > bound:
            raise E6Error(f"J{index}={value:.4f}° 超出本脚本的模型检查范围 ±{bound:.4f}°。")


def read_state(robot, clock=time.monotonic):
    started = clock()
    try:
        mode = int(robot.command("RobotMode()"))
        joints = tuple(float(v.strip()) for v in robot.command("GetAngle()").split(","))
        errors = alarm_codes(json.loads(robot.command("GetErrorID()")))
    except (ValueError, TypeError) as error:
        raise E6Error(f"状态解析失败：{error}") from error
    received_at = clock()
    if received_at - started > MAX_SAMPLE_SECONDS:
        raise E6Error("本轮状态查询耗时超过 1 秒，拒绝使用过期状态。")
    validate_joints(joints)
    if errors:
        raise E6Error(f"存在报警 {errors}；脚本不会自动清错。")
    return State(mode, joints, received_at)


def require_idle(state):
    if state.mode not in (4, 5):
        raise E6Error(f"仅允许未使能(4)或已使能空闲(5)状态开始，当前 RobotMode={state.mode}。")


def max_error(joints):
    # Absolute joint coordinates: never wrap 360 degrees to zero.
    return max(abs(a - b) for a, b in zip(joints, TARGET_DEG))


def check_start_unchanged(before, after):
    change = max(abs(a - b) for a, b in zip(before.joints, after.joints))
    if change > MAX_START_CHANGE_DEG:
        raise E6Error(f"确认后起始关节角变化了 {change:.3f}°，请重新预览路径。")


def check_motion_envelope(start, current):
    for index, (origin, actual) in enumerate(zip(start, current), 1):
        if not min(origin, 0.0) - 2.0 <= actual <= max(origin, 0.0) + 2.0:
            raise E6Error(f"J{index} 偏离起始角到零位之间的预期范围，停止回零。")


def move_command(speed_percent, accel_percent):
    for name, value in (("速度", speed_percent), ("加速度", accel_percent)):
        if type(value) is not int or not 1 <= value <= 5:
            raise E6Error(f"本脚本的{name}比例只允许 1～5%。")
    # Explicit joint= is essential: pose={0,...,0} is a different target.
    return f"MovJ(joint={{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000}},a={accel_percent},v={speed_percent},cp=0)"


def stop_best_effort(robot):
    try:
        robot.command("Stop()")
        LOG.error("控制器已接受 Stop()；请在现场确认机械臂停止。")
    except (Exception, KeyboardInterrupt) as error:
        LOG.error("无法确认 Stop() 被接收：%s。运动可能仍在执行，必要时使用硬件急停。", error)


def run_to_zero(robot, *, execute=False, speed_percent=5, accel_percent=5,
                motion_timeout=180.0, confirm=input, clock=time.monotonic, sleep=time.sleep):
    if not math.isfinite(motion_timeout) or not 5 <= motion_timeout <= 600:
        raise E6Error("运动超时必须在 5～600 秒之间。")
    command = move_command(speed_percent, accel_percent)
    initial = read_state(robot, clock)
    require_idle(initial)
    LOG.info("当前关节角(度)：%s", [round(v, 4) for v in initial.joints])
    LOG.info("目标关节角(度)：%s", list(TARGET_DEG))
    LOG.info("各关节转角(度)：%s", [round(-v, 4) for v in initial.joints])
    LOG.info("命令速度/加速度比例：%s%% / %s%%；到位容差：%.1f°", speed_percent, accel_percent, TOLERANCE_DEG)
    if max_error(initial.joints) > MAX_TRAVEL_DEG:
        raise E6Error("单个关节需要转动超过 180°，本脚本拒绝直接回零；请先规划中间姿态。")
    if max_error(initial.joints) <= TOLERANCE_DEG:
        LOG.info("本次采样已在全零容差内，不发送使能或运动命令。")
        return "already_at_zero"
    if not execute:
        LOG.info("只读预览结束。将使用的运动指令：%s", command)
        LOG.info("执行时使用 --execute；届时还会要求现场确认。")
        return "preview"

    LOG.warning("这是直接关节运动，不自动避障。低速不会减少关节转动幅度。")
    answer = confirm("确认底座已固定、整条运动路径已清空、可操作急停，并允许必要时使能及回零。\n输入 GO ZERO 执行，其他输入取消：")
    if answer.strip() != "GO ZERO":
        LOG.info("已取消，未使能、未下发运动。")
        return "cancelled"

    # Confirmation can take arbitrarily long: always obtain a new state afterwards.
    current = read_state(robot, clock)
    require_idle(current)
    check_start_unchanged(initial, current)
    if current.mode != initial.mode:
        raise E6Error("确认期间机器人模式发生变化，请重新运行并检查控制来源。")

    action_attempted = False
    try:
        if current.mode == 4:
            action_attempted = True  # An acknowledgement timeout does not prove rejection.
            # Enable may take several seconds while the controller changes state;
            # this timeout is separate from the 1 s freshness budget for reads.
            robot.command("EnableRobot()", timeout=15.0)  # No payload or centre-of-mass changes.
            deadline = clock() + 10.0
            while True:
                current = read_state(robot, clock)
                require_idle(current)
                check_start_unchanged(initial, current)
                if current.mode == 5:
                    break
                if clock() >= deadline:
                    raise E6Error("使能后 10 秒内未进入空闲状态。")
                sleep(POLL_SECONDS)

        # Includes the case that enabling took several seconds.
        current = read_state(robot, clock)
        if current.mode != 5:
            raise E6Error(f"下发运动前未处于使能空闲状态：{current.mode}")
        check_start_unchanged(initial, current)
        if clock() - current.received_at > MAX_SAMPLE_SECONDS:
            raise E6Error("下发前状态已过期。")

        action_attempted = True
        queue_id = robot.command(command)
        if not re.fullmatch(r"\d+", queue_id):
            raise E6Error(f"运动回复中的队列 ID 无效：{queue_id!r}")
        LOG.info("控制器已接受运动，队列 ID=%s；开始通过实测角度确认到位。", queue_id)
        deadline = clock() + motion_timeout
        stable = 0
        next_progress = clock()
        while True:
            if clock() >= deadline:
                raise E6Error("等待回零超时。")
            current = read_state(robot, clock)
            if clock() >= deadline:
                raise E6Error("等待回零超时。")
            if current.mode not in (5, 7):
                raise E6Error(f"运动期间进入非预期状态 RobotMode={current.mode}。")
            check_motion_envelope(initial.joints, current.joints)
            error_deg = max_error(current.joints)
            stable = stable + 1 if current.mode == 5 and error_deg <= TOLERANCE_DEG else 0
            if stable >= STABLE_SAMPLES:
                LOG.info("回零完成：%s；保持使能空闲，不自动下使能。", [round(v, 4) for v in current.joints])
                return "reached_zero"
            if clock() >= next_progress:
                LOG.info("RobotMode=%s，最大关节误差 %.3f°", current.mode, error_deg)
                next_progress = clock() + 1.0
            sleep(POLL_SECONDS)
    except (Exception, KeyboardInterrupt):
        if action_attempted:
            stop_best_effort(robot)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description="E6 全零关节姿态：默认只读预览，--execute 才执行。")
    parser.add_argument("--ip", default="192.168.1.4", help="机器人 IP")
    parser.add_argument("--execute", action="store_true", help="现场确认后使能(若需要)并执行回零")
    parser.add_argument("--speed-percent", type=int, choices=range(1, 6), default=5)
    parser.add_argument("--accel-percent", type=int, choices=range(1, 6), default=5)
    parser.add_argument("--timeout", type=float, default=180.0, help="运动到位超时，5～600 秒")
    parser.add_argument("--log", type=Path, help="日志路径，默认在项目 local-network 目录")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 5 <= args.timeout <= 600:
        parser.error("--timeout 必须为 5～600 秒的有限数值")
    if args.execute and not sys.stdin.isatty():
        parser.error("--execute 必须在交互终端运行，以便现场确认；不接受管道输入。")

    log_path = args.log or (Path(__file__).resolve().parents[1] / "local-network" /
                            f"e6-zero-{datetime.now():%Y%m%d-%H%M%S-%f}.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    LOG.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    for handler in (file_handler, console_handler):
        handler.setFormatter(formatter)
        LOG.addHandler(handler)
    try:
        LOG.info("日志：%s", log_path)
        with Dashboard(args.ip, timeout=1.0) as robot:
            run_to_zero(robot, execute=args.execute, speed_percent=args.speed_percent,
                        accel_percent=args.accel_percent, motion_timeout=args.timeout)
        return 0
    except KeyboardInterrupt:
        LOG.error("用户中断；没有自动重连或重发运动。")
        return 130
    except (E6Error, OSError, ValueError, EOFError) as error:
        LOG.error("未完成：%s", error)
        return 1
    finally:
        for handler in (file_handler, console_handler):
            LOG.removeHandler(handler)
            handler.close()


if __name__ == "__main__":
    raise SystemExit(main())
