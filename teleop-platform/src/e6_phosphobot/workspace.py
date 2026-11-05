"""Session-fixed site test range; independent of robot joint/collision limits."""
from dataclasses import asdict, dataclass, fields
import math


@dataclass(frozen=True)
class TestWorkspace:
    __test__ = False
    position_m: float = .05
    rotation_deg: float | None = 10.
    joint_deg: float | None = 10.
    mode: str = 'local'

    def __post_init__(self):
        if self.mode not in ('local', 'model'):
            raise ValueError('Unknown workspace mode')
        for field in fields(self):
            if field.name == 'mode':
                continue
            value = getattr(self, field.name)
            if value is None and field.name in ('rotation_deg', 'joint_deg'):
                continue  # Explicitly use model joint limits instead of a relative cap.
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('Test workspace requires finite positive numbers: '+field.name)

    @property
    def uses_model_range(self):
        return self.mode == 'model' or self.rotation_deg is None or self.joint_deg is None

    def velocity_boundary(self, origin):
        return None if self.mode == 'model' else dict(origin_m=list(origin), radius_m=self.position_m)

    @classmethod
    def from_config(cls, value=None):
        if value is None:
            return cls()
        if not isinstance(value, dict) or set(value)-{f.name for f in fields(cls)}:
            raise ValueError('Unknown test workspace fields or format')
        return cls(**value)

    def description(self):
        value = asdict(self)
        # Preserve the legacy wire shape and stored recording metadata.
        if self.mode == 'local':
            value.pop('mode')
        return value
