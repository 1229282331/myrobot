"""Bounded real SocketCAN receive-timeout check at the already held 25% target.

The device remains connected. A temporary socket filter hides its response
from this process, checking that missing feedback blocks the next target.
Default is offline preview. Interface DOWN does not disable the motor.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

import verify_can_control as base
import verify_can_stability as stress


class FeedbackLossAcceptance(stress.Acceptance):
    def profile(self):
        self.report["harness_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.open()
        self.step(25, label="feedback_loss_prepare25")
        marker = len(self.report["events"])
        before = base.ip_snapshot(self.args.interface)
        self.channel.sock.setsockopt(base.SOL_CAN_RAW, 1,
                                     struct.pack("=II", self.args.can_id ^ 0x400,
                                                 base.SFF_MASK | base.CAN_EFF | base.CAN_RTR))
        try:
            self.step(25, label="injected_receive_timeout_at25")
        except base.CheckFailed as exc:
            if not str(exc).startswith("No valid feedback before deadline"):
                raise
            self.report["expected_feedback_timeout"] = repr(exc)
        else:
            raise base.CheckFailed("Hidden feedback did not stop the control path")
        stage = self.report["stages"][-1]
        stage["expected_feedback_failure"] = True
        attempts = self.report["events"][marker:]
        tx = [e for e in attempts if e["kind"] == "tx"]
        if len(tx) != 1 or not tx[0]["send_confirmed"] or stage["feedback"]:
            raise base.CheckFailed("Timeout caused a retry or accepted hidden feedback")
        self.report["timeout_kernel_before"] = before
        self.report["timeout_kernel_after"] = base.ip_snapshot(self.args.interface)
        marker = len(self.report["events"])
        self.cancelled = True
        try:
            self.step(35, label="must_not_send35_after_timeout")
        except base.CheckFailed:
            pass
        else:
            raise base.CheckFailed("Next target was allowed after missing feedback")
        if any(e["kind"] == "tx" for e in self.report["events"][marker:]):
            raise base.CheckFailed("Next target escaped after missing feedback")
        self.report["feedback_loss_gate_passed"] = True
        self.close()
        self.cancelled = False
        self.sleep(.5)
        self.open()
        self.step(25, label="feedback_loss_explicit_resume25")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({"mode": "offline_preview", "target_percent": 25,
                          "injection": "Socket receive filter hides one response; 300ms timeout; no retry",
                          "motor_disable_available": False}, indent=2))
        return 0
    if not args.output:
        parser.error("Execution needs a new --output path")
    args.profile = "feedback-loss"
    args.full_stroke = False
    args.interface = "can1"
    args.can_id = 1
    args.stop_file = "/root/omnipicker-can/STOP"
    stress.Acceptance = FeedbackLossAcceptance
    return stress.execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
