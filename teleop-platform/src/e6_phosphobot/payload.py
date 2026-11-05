"""Configured total load estimate, applied by the official EnableRobot command.

The mass is the user's fixed 0.750 kg estimate; the center retains the R4
assembly engineering estimate. Neither is a new physical measurement or a
qualification for arbitrary objects. Controller reboot persistence is not
assumed. Verification uses the SDK's actual Load/Center fields, never a command
acknowledgement alone.
"""
from dataclasses import dataclass
import math

FEEDBACK_FIELDS = ('load', 'load_center_x', 'load_center_y', 'load_center_z')


@dataclass(frozen=True)
class Payload:
    mass_kg: float = .750
    center_mm: tuple = (0., -14., 46.)

    @classmethod
    def from_config(cls, value=None):
        if value is None:
            return cls()
        if not isinstance(value, dict) or set(value) != {'mass_kg', 'center_mm'}:
            raise ValueError('Payload requires mass_kg and center_mm')
        mass, center = value['mass_kg'], value['center_mm']
        if type(mass) not in (int, float) or not math.isfinite(mass) or not 0 < mass <= .75:
            raise ValueError('E6 payload estimate must be finite, >0 and <=0.75 kg')
        if (not isinstance(center, (list, tuple)) or len(center) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in center)):
            raise ValueError('Payload center requires three finite coordinates in mm')
        return cls(float(mass), tuple(float(v) for v in center))

    def description(self):
        return dict(mass_kg=self.mass_kg, center_mm=list(self.center_mm))

    def enable(self, robot):
        x, y, z = self.center_mm
        # Keep the SDK/controller default isCheck=0. That optional physical
        # check is not the feedback comparison performed by this application.
        return robot.EnableRobot(self.mass_kg, centerX=x, centerY=y, centerZ=z)

    def mismatch(self, status):
        values = [status.get(k) for k in FEEDBACK_FIELDS]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            return '负载参数反馈缺失或无效'
        expected = (self.mass_kg, *self.center_mm)
        # Numerical serialization tolerances only, not physical calibration
        # tolerances: 1 micro-kg and 0.001 mm.
        if any(abs(a-b) > tol for a,b,tol in zip(values, expected, (1e-6, .001, .001, .001))):
            return (f'负载参数不一致：实际 {values[0]:g} kg / {values[1:]} mm；'
                    f'要求 {self.mass_kg:g} kg / {list(self.center_mm)} mm')
        return None
