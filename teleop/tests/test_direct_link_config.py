"""Network ownership and acceptance guards, using no remote device."""
import importlib.util
import json
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('direct_link', Path(__file__).resolve().parents[1]/'scripts/configure_e6_direct_link.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
UUID = 'c9ba60b6-d0d3-4fc0-be96-0d7c9041816b'


class Network:
    def __init__(self):
        self.calls = []
        self.filename = '/run/NetworkManager/system-connections/e6-teleop-direct.nmconnection'
        self.name = module.NAME
        self.settings = dict(module.FIELDS, **{'ipv4.routes': '192.168.1.4/32'})
        self.carrier = '1'
        self.connections = ''
        self.deleted = False
        self.persist_works = True

    def __call__(self, *args):
        self.calls.append(args)
        if args[:2] == ('ss', '-Htnp'):
            return self.connections
        if args[:3] == ('nmcli', '-t', '--escape'):
            return '' if self.deleted else ':'.join((UUID, self.name, self.filename)) + '\n'
        if args[:2] == ('nmcli', '-g'):
            return self.settings[args[2]] + '\n'
        if args[:2] == ('cat', '/sys/class/net/eth0/carrier'):
            return self.carrier + '\n'
        if args[:4] == ('ip', '-j', 'route', 'get'):
            direct = args[4] == '192.168.1.4' and not self.deleted
            return json.dumps([dict(dev='eth0' if direct else 'wlan0', prefsrc='192.168.1.14' if direct else '192.168.1.13')])
        if args[:4] == ('ip', '-j', 'route', 'show'):
            return json.dumps([dict(dev='wlan0', gateway='192.168.1.1')])
        if args[:4] == ('ip', '-j', 'address', 'show'):
            return json.dumps([dict(ifname='eth0', addr_info=[])])
        if args[:3] == ('nmcli', 'con', 'modify'):
            if self.persist_works:
                self.filename = self.filename.replace('/run/', '/etc/')
            return ''
        if args[:3] == ('nmcli', 'con', 'delete'):
            self.deleted = True
            return ''
        if args[:3] == ('nmcli', 'con', 'load') or args[:4] == ('nmcli', '--wait', '10', 'con'):
            return ''
        raise AssertionError(args)

    def mutations(self):
        return [call for call in self.calls if any(v in call for v in ('modify', 'delete', 'load', 'up', 'add'))]


@pytest.mark.parametrize('operation', ['activate', 'persist', 'remove'])
def test_device_session_blocks_every_network_mutation(operation):
    network = Network()
    network.connections = 'ESTAB 0 0 192.168.1.14:50000 192.168.1.4:29999 users:(("python3",pid=1,fd=2))'
    link = module.DirectLink(network, UUID)
    with pytest.raises(ValueError, match='Release'):
        getattr(link, operation)(None) if operation == 'persist' else getattr(link, operation)()
    assert not network.mutations()


@pytest.mark.parametrize('change', ['name', 'settings', 'routes'])
def test_external_profile_changes_are_not_overwritten(change):
    network = Network()
    if change == 'name':
        network.name = 'unrelated-project'
    elif change == 'settings':
        network.settings['ipv4.addresses'] = '192.168.1.20/24'
    else:
        network.settings['ipv4.routes'] = '192.168.1.4/32 192.168.1.1'
    with pytest.raises(ValueError, match='mismatch|externally'):
        module.DirectLink(network, UUID).remove()
    assert not network.mutations()


def test_no_carrier_or_wrong_accepted_network_never_mutates():
    network = Network()
    network.carrier = '0'
    link = module.DirectLink(network, UUID)
    with pytest.raises(ValueError, match='carrier'):
        link.activate()
    network.carrier = '1'
    with pytest.raises(ValueError, match='acceptance'):
        link.persist(dict(interface='wlan0'))
    assert not network.mutations()


def test_persistence_verified_by_disk_filename_and_owned_reload():
    network = Network()
    expected = dict(robot_ip='192.168.1.4', interface='eth0', source='192.168.1.14', gateway=None, direct_ethernet=True)
    result = module.DirectLink(network, UUID).persist(expected)
    assert result['disk_reload_verified'] and not result['temporary'] and not result['reboot_verified']
    assert result['before']['pc_route'] == result['after']['pc_route']
    assert result['before']['default_routes'] == result['after']['default_routes']
    assert result['robot_commands_sent'] == result['can_frames_sent'] == 0
    assert all('wlan0' not in call for call in network.mutations())


def test_successful_nmcli_reply_does_not_prove_persistent_storage():
    network = Network()
    network.persist_works = False
    expected = dict(robot_ip='192.168.1.4', interface='eth0', source='192.168.1.14', gateway=None, direct_ethernet=True)
    with pytest.raises(RuntimeError, match='filename'):
        module.DirectLink(network, UUID).persist(expected)
    assert not any('load' in call for call in network.calls)


def test_removal_targets_only_recorded_uuid():
    network = Network()
    result = module.DirectLink(network, UUID).remove()
    assert result['removed']
    assert network.mutations() == [('nmcli', 'con', 'delete', 'uuid', UUID)]


def test_creation_uses_exclusive_temporary_keyfile_with_recorded_identity():
    network = Network()
    network.deleted = True
    def runner(*args):
        if args[0] == 'python3':
            network.calls.append(args)
            assert 'os.O_EXCL' in args[2]
            assert args[3].startswith('/run/NetworkManager/system-connections/')
            assert 'uuid=' + UUID + '\n' in args[4]
            assert 'never-default=true' in args[4]
            network.filename = args[3]
            return ''
        if args[:3] == ('nmcli', 'con', 'load'):
            network.deleted = False
        return network(*args)
    result = module.DirectLink(runner, UUID).create()
    assert result['temporary'] and result['uuid'] == UUID
    assert not any('add' in call or 'modify' in call for call in network.calls)


def test_creation_refuses_existing_identity_before_keyfile_write():
    network = Network()
    with pytest.raises(ValueError, match='already exists'):
        module.DirectLink(network, UUID).create()
    assert not any(call[0] == 'python3' for call in network.calls)
