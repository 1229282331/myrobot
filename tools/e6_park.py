"""Preview or execute one slow E6 move to a compact parking pose.

The parking pose is a conservative starting candidate for an E6 placed near a
table edge.  It is not a collision guarantee: the operator must still watch
the physical arm and have the emergency stop available.

Python standard library only.  Default: read-only preview.  ``--execute``
performs one explicitly confirmed joint move and keeps the robot enabled and
idle after reaching the target.
"""

import argparse
from datetime import datetime
import logging
import math
from pathlib import Path
import re
import sys
import time

try:  # Supports ``python tools/e6_park.py`` and package imports in tests.
    from .e6_zero import (
        Dashboard,
        E6Error,
        JOINT_BOUNDS_DEG,
        MAX_SAMPLE_SECONDS,
        MAX_START_CHANGE_DEG,
        POLL_SECONDS,
        STABLE_SAMPLES,
        TOLERANCE_DEG,
        alarm_codes,
        check_start_unchanged,
        read_state,
        require_idle,
        stop_best_effort,
    )
except ImportError:
    from e6_zero import (
        Dashboard,
        E6Error,
        JOINT_BOUNDS_DEG,
        MAX_SAMPLE_SECONDS,
        MAX_START_CHANGE_DEG,
        POLL_SECONDS,
        STABLE_SAMPLES,
        TOLERANCE_DEG,
        alarm_codes,
        check_start_unchanged,
        read_state,
        require_idle,
        stop_best_effort,
    )


LOG = logging.getLogger("e6_park")
TARGET_DEG = (0.0, -60.0, 120.0, 0.0, 45.0, 0.0)
MAX_TRAVEL_DEG = 180.0


def max_error(joints):
    return max(abs(actual - target) for actual, target in zip(joints, TARGET_DEG))


def validate_target():
    for index, (value, bound) in enumerate(zip(TARGET_DEG, JOINT_BOUNDS_DEG), 1):
        if not math.isfinite(value) or abs(value) > bound:
            raise E6Error(f"目标 J{index}={value}° 超出模型检查范围 ±{bound:.4f}°。")


def move_command(speed_percent, accel_percent):
    for name, value in (("速度", speed_percent), ("加速度", accel_percent)):
        if type(value) is not int or not 1 <= value <= 5:
            raise E6Error(f"本脚本的{name}比例只允许 1～5%。")
    values = ",".join(f"{value:.6f}" for value in TARGET_DEG)
    return f"MovJ(joint={{{values}}},a={accel_percent},v={speed_percent},cp=0)"


def check_motion_envelope(start, current):
    """Reject a feedback sample outside the direct start-to-target box."""
    for index, (origin, target, actual) in enumerate(
        zip(start, TARGET_DEG, current), 1
    ):
        low = min(origin, target) - 2.0
        high = max(origin, target) + 2.0
        if not low <= actual <= high:
            raise E6Error(
                f"J{index}={actual:.3f}° 偏离起点到停放目标的预期范围，停止运动。"
            )


def run_to_park(
    robot,
    *,
    execute=False,
    speed_percent=3,
    accel_percent=3,
    motion_timeout=180.0,
    confirm=input,
    clock=time.monotonic,
    sleep=time.sleep,
):
    """Preview or execute the one-way move; never retries a motion command."""
    validate_target()
    if not math.isfinite(motion_timeout) or not 5 <= motion_timeout <= 600:
        raise E6Error("运动超时必须在 5～600 秒之间。")
    command = move_command(speed_percent, accel_percent)
    initial = read_state(robot, clock)
    require_idle(initial)
    LOG.info("当前关节角(度)：%s", [round(v, 4) for v in initial.joints])
    LOG.info("候选 park_pose(度)：%s", list(TARGET_DEG))
    LOG.info(
        "各关节预计转角(度)：%s",
        [round(target - actual, 4) for actual, target in zip(initial.joints, TARGET_DEG)],
    )
    LOG.info(
        "命令速度/加速度比例：%s%% / %s%%；到位容差：%.1f°",
        speed_percent,
        accel_percent,
        TOLERANCE_DEG,
    )
    if max_error(initial.joints) > MAX_TRAVEL_DEG:
        raise E6Error("单个关节需要转动超过 180°，脚本拒绝直接移动；请先规划中间姿态。")
    if max_error(initial.joints) <= TOLERANCE_DEG:
        LOG.info("本次采样已在 park_pose 容差内，不发送使能或运动命令。")
        return "already_at_park"
    if not execute:
        LOG.info("只读预览结束。将使用的运动指令：%s", command)
        LOG.info("执行时使用 --execute；届时还会要求现场确认。")
        return "preview"

    LOG.warning("这是直接关节运动，不自动避障；请全程观察并准备急停。")
    answer = confirm(
        "确认底座已固定、桌边和运动路径已清空、可操作急停，并允许低速移动。\n"
        "输入 GO PARK 执行，其他输入取消："
    )
    if answer.strip() != "GO PARK":
        LOG.info("已取消，未使能、未下发运动。")
        return "cancelled"

    current = read_state(robot, clock)
    require_idle(current)
    check_start_unchanged(initial, current)
    if current.mode != initial.mode:
        raise E6Error("确认期间机器人模式发生变化，请重新检查控制来源。")

    action_attempted = False
    try:
        if current.mode == 4:
            action_attempted = True
            robot.command("EnableRobot()", timeout=15.0)
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
                raise E6Error("等待 park_pose 到位超时。")
            current = read_state(robot, clock)
            if clock() >= deadline:
                raise E6Error("等待 park_pose 到位超时。")
            if current.mode not in (5, 7):
                raise E6Error(f"运动期间进入非预期状态 RobotMode={current.mode}。")
            check_motion_envelope(initial.joints, current.joints)
            error_deg = max_error(current.joints)
            stable = stable + 1 if current.mode == 5 and error_deg <= TOLERANCE_DEG else 0
            if stable >= STABLE_SAMPLES:
                LOG.info(
                    "park_pose 到位：%s；保持使能空闲，不自动下使能。",
                    [round(v, 4) for v in current.joints],
                )
                return "reached_park"
            if clock() >= next_progress:
                LOG.info("RobotMode=%s，最大 park_pose 关节误差 %.3f°", current.mode, error_deg)
                next_progress = clock() + 1.0
            sleep(POLL_SECONDS)
    except (Exception, KeyboardInterrupt):
        if action_attempted:
            stop_best_effort(robot)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description="E6 候选 park_pose：默认只读预览，--execute 才执行。")
    parser.add_argument("--ip", default="192.168.1.4", help="机器人 IP")
    parser.add_argument("--execute", action="store_true", help="现场确认后执行低速 park_pose")
    parser.add_argument("--speed-percent", type=int, choices=range(1, 6), default=3)
    parser.add_argument("--accel-percent", type=int, choices=range(1, 6), default=3)
    parser.add_argument("--timeout", type=float, default=180.0, help="运动到位超时，5～600 秒")
    parser.add_argument("--log", type=Path, help="日志路径，默认在项目 local-network 目录")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 5 <= args.timeout <= 600:
        parser.error("--timeout 必须为 5～600 秒的有限数值")
    if args.execute and not sys.stdin.isatty():
        parser.error("--execute 必须在交互终端运行，以便现场确认；不接受管道输入。")

    log_path = args.log or (
        Path(__file__).resolve().parents[1]
        / "local-network"
        / f"e6-park-{datetime.now():%Y%m%d-%H%M%S-%f}.log"
    )
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
            run_to_park(
                robot,
                execute=args.execute,
                speed_percent=args.speed_percent,
                accel_percent=args.accel_percent,
                motion_timeout=args.timeout,
            )
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
