"""Temporary network parameter ownership and evidence binding, no IO."""
import pytest
from e6_observer.wifi_latency import WifiLatency
from e6_teleop.tunnel import Tunnel


def test_temporary_reorder_readback_restores_exact_original(tmp_path):
    parameter=tmp_path/'reorder';parameter.write_text('50\n')
    setting=WifiLatency(False,10);setting.reorder_path=parameter
    with setting:
        assert parameter.read_text()=='10\n'
        assert setting.report['reorder_original_ms']==50
        assert setting.report['reorder_effective_ms']==10
        assert not setting.report['firmware_power_save_verified']
    assert parameter.read_text()=='50\n'


def test_reorder_failure_and_external_change_never_overwrite_other_owner(tmp_path):
    parameter=tmp_path/'reorder';parameter.write_text('unknown\n')
    setting=WifiLatency(False,10);setting.reorder_path=parameter
    with pytest.raises(ValueError,match='Unrecognized'):setting.__enter__()
    assert parameter.read_text()=='unknown\n'
    parameter.write_text('50\n');setting.__enter__();parameter.write_text('20\n')
    with pytest.raises(ValueError,match='outside this session'):setting.__exit__()
    assert parameter.read_text()=='20\n'


def test_reorder_only_with_owned_gateway_and_operator_accepted_value():
    with pytest.raises(ValueError,match='owned'):Tunnel('127.0.0.1',wifi_reorder_ms=10)
    with pytest.raises(ValueError,match='10 ms'):Tunnel('127.0.0.1',remote_root='/owned',wifi_reorder_ms=5)
    with pytest.raises(ValueError,match='acceptance'):
        Tunnel('127.0.0.1',remote_root='/owned',control_stage='live',control_role='operator',
               acceptance_profile={'wifi_reorder_ms':None},wifi_reorder_ms=10)
    accepted=Tunnel('127.0.0.1',remote_root='/owned',control_stage='live',control_role='operator',
                    acceptance_profile={'wifi_reorder_ms':10,'media_hz':10},wifi_reorder_ms=10)
    assert accepted.wifi_reorder_ms==10


@pytest.mark.parametrize('value',['Y','N'])
def test_driver_ps_readback_is_distinct_from_cfg80211_and_never_written(tmp_path,value):
    parameter=tmp_path/'ps_on';parameter.write_text(value+'\n')
    setting=WifiLatency(False);setting.driver_ps_path=parameter
    with setting:
        assert setting.report['driver_ps_on']==value
        assert not setting.report['firmware_power_save_verified']
    assert parameter.read_text()==value+'\n'


def test_unknown_driver_ps_parameter_fails_before_network_changes(tmp_path):
    parameter=tmp_path/'ps_on';parameter.write_text('unknown\n')
    setting=WifiLatency(True);setting.driver_ps_path=parameter
    with pytest.raises(ValueError,match='driver PS'):setting.__enter__()
    assert not setting.changed
