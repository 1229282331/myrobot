from types import SimpleNamespace
import subprocess
import pytest
from e6_phosphobot.wifi_policy import SessionWifiPolicy


class Radio:
    def __init__(self, state='on'):
        self.state = state
        self.calls = []
        self.fail_write = False

    def run(self, args, **kwargs):
        self.calls.append(args)
        assert kwargs['timeout'] == 3 and kwargs['check']
        assert args[:3] == ['iw', 'dev', 'wlan0']
        if args[3] == 'set':
            self.state = args[-1]
            if self.fail_write:
                self.fail_write = False
                raise subprocess.TimeoutExpired(args, 3)
        return SimpleNamespace(stdout='Power save: '+self.state)


def test_default_no_radio_io():
    radio = Radio()
    policy = SessionWifiPolicy({}, run=radio.run)
    policy.apply(); policy.restore()
    assert not radio.calls


@pytest.mark.parametrize('original', ['on', 'off'])
def test_apply_restore_exact_original(original):
    radio = Radio(original)
    policy = SessionWifiPolicy({'wifi_power_save': 'off'}, run=radio.run)
    policy.apply()
    assert radio.state == 'off'
    policy.restore(); policy.restore()
    assert radio.state == original
    writes = [a for a in radio.calls if a[3] == 'set']
    assert len(writes) == (2 if original == 'on' else 0)


def test_partial_failure_restores():
    radio = Radio(); radio.fail_write = True
    policy = SessionWifiPolicy({'wifi_power_save': 'off'}, run=radio.run)
    with pytest.raises(subprocess.TimeoutExpired): policy.apply()
    assert radio.state == 'off'
    policy.restore()
    assert radio.state == 'on'


def test_malformed_readback_rejected():
    radio = Radio('unknown')
    policy = SessionWifiPolicy({'wifi_power_save': 'off'}, run=radio.run)
    with pytest.raises(RuntimeError): policy.apply()
    assert not policy.changed


@pytest.mark.parametrize('config', [{'wifi_power_save':'on'}, {'wifi_interface':'wlan0; reboot'}, {'wifi_interface':''}])
def test_invalid_config(config):
    with pytest.raises(ValueError): SessionWifiPolicy(config)
