"""Session-owned Linux CPU governor, without changing frequency/thermal limits."""
from pathlib import Path


def configured_governor(config):
    value = config.get('cpu_governor', 'unchanged')
    if value not in ('unchanged', 'performance'):
        raise ValueError('cpu_governor must be unchanged or performance')
    return value


class SessionCpuPolicy:
    def __init__(self, mode, *, root='/sys/devices/system/cpu/cpufreq', event=None):
        self.mode = configured_governor({'cpu_governor': mode})
        self.root = Path(root)
        self.event = event or (lambda *args, **kwargs: None)
        self.policies = []

    def apply(self):
        if self.mode == 'unchanged':
            return self.snapshot()
        paths = sorted(self.root.glob('policy[0-9]*'))
        if not paths:
            raise RuntimeError('CPU frequency policies unavailable')
        # Preflight all policies before changing any. Receipts survive failures
        # so the supervisor can report/restore exactly what it owns.
        for path in paths:
            available = (path/'scaling_available_governors').read_text().split()
            if self.mode not in available:
                raise RuntimeError(f'{path.name}: governor {self.mode} unavailable')
            original = (path/'scaling_governor').read_text().strip()
            if original not in available:
                raise RuntimeError(f'{path.name}: unknown original governor')
            self.policies.append(dict(policy=path.name, original=original,
                                      applied=self.mode, changed=False, restored=False))
        self.event('cpu_policy_original', **self.snapshot())
        for entry in self.policies:
            if entry['original'] == self.mode:
                continue
            entry['changed'] = True  # A failed write may still have taken effect.
            path = self.root/entry['policy']/'scaling_governor'
            path.write_text(self.mode+'\n')
            if path.read_text().strip() != self.mode:
                raise RuntimeError(f"{entry['policy']}: CPU governor readback differs")
        self.event('cpu_policy_applied', **self.snapshot())
        return self.snapshot()

    def restore(self):
        errors = []
        for entry in reversed(self.policies):
            if not entry['changed'] or entry['restored']:
                continue
            try:
                path = self.root/entry['policy']/'scaling_governor'
                current = path.read_text().strip()
                if current == entry['original']:
                    entry['restored'] = True
                    continue
                if current != entry['applied']:
                    raise RuntimeError('externally changed governor retained: '+current)
                path.write_text(entry['original']+'\n')
                if path.read_text().strip() != entry['original']:
                    raise RuntimeError('restore readback differs')
                entry['restored'] = True
            except Exception as exc:
                errors.append(f"{entry['policy']}: {exc}")
        self.event('cpu_policy_restored', **self.snapshot(), errors=errors)
        if errors:
            raise RuntimeError('; '.join(errors))
        return self.snapshot()

    def snapshot(self):
        return dict(mode=self.mode, policies=[dict(entry) for entry in self.policies])
