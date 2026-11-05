"""Session-owned Wi-Fi power saving policy; never reconnects the interface."""
import re
import subprocess


class SessionWifiPolicy:
    def __init__(self, config, *, event=None, run=subprocess.run):
        self.mode = config.get('wifi_power_save', 'unchanged')
        self.interface = config.get('wifi_interface', 'wlan0')
        if self.mode not in ('unchanged', 'off'):
            raise ValueError('wifi_power_save must be unchanged or off')
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', self.interface):
            raise ValueError('Invalid Wi-Fi interface')
        self.event = event or (lambda *args, **kwargs: None)
        self.run = run
        self.original = None
        self.changed = False
        self.restored = False

    def command(self, *args):
        result = self.run(['iw', 'dev', self.interface, *args],
                          capture_output=True, text=True, timeout=3, check=True)
        return result.stdout.strip()

    def read(self):
        match = re.fullmatch(r'Power save:\s*(on|off)', self.command('get', 'power_save'))
        if not match:
            raise RuntimeError('Unrecognized Wi-Fi power saving readback')
        return match.group(1)

    def apply(self):
        if self.mode == 'unchanged':
            return self.snapshot()
        self.original = self.read()
        self.event('wifi_policy_original', **self.snapshot())
        if self.original != self.mode:
            self.changed = True  # Retain ownership even after a partial failure.
            self.command('set', 'power_save', self.mode)
            if self.read() != self.mode:
                raise RuntimeError('Wi-Fi power saving readback differs')
        self.event('wifi_policy_applied', **self.snapshot())
        return self.snapshot()

    def restore(self):
        if self.changed and not self.restored:
            current = self.read()
            if current != self.original:
                if current != self.mode:
                    raise RuntimeError('Externally changed Wi-Fi policy retained')
                self.command('set', 'power_save', self.original)
                if self.read() != self.original:
                    raise RuntimeError('Wi-Fi power saving restore differs')
            self.restored = True
        self.event('wifi_policy_restored', **self.snapshot())
        return self.snapshot()

    def snapshot(self):
        return dict(mode=self.mode, interface=self.interface, original=self.original,
                    changed=self.changed, restored=self.restored)
