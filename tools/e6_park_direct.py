"""One-command entry point for the validated E6 parking pose.

Run from the project root:

    python tools/e6_park_direct.py

This intentionally executes the move without an interactive confirmation,
because it is a user-requested convenience entry point.  The underlying
preflight, feedback checks, motion envelope, timeout and Stop() handling from
``e6_park.run_to_park`` remain enabled.  It never retries a motion command and
does not disable the robot after arrival.
"""

from datetime import datetime
import logging
from pathlib import Path
import sys

try:
    from .e6_park import Dashboard, E6Error, LOG, run_to_park
except ImportError:
    from e6_park import Dashboard, E6Error, LOG, run_to_park


IP = "192.168.1.4"
SPEED_PERCENT = 3
ACCEL_PERCENT = 3
MOTION_TIMEOUT = 180.0


def main():
    log_path = (
        Path(__file__).resolve().parents[1]
        / "local-network"
        / f"e6-park-direct-{datetime.now():%Y%m%d-%H%M%S-%f}.log"
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
        LOG.warning("一键 park 入口：将直接执行已验证的 [0,-60,120,0,45,0]° 姿态。")
        LOG.info("日志：%s", log_path)
        with Dashboard(IP, timeout=1.0) as robot:
            run_to_park(
                robot,
                execute=True,
                speed_percent=SPEED_PERCENT,
                accel_percent=ACCEL_PERCENT,
                motion_timeout=MOTION_TIMEOUT,
                confirm=lambda _prompt: "GO PARK",
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
