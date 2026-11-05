"""Configure only the owned E6 Ethernet host route; never touch robot commands.

SSH stays on WLAN. New configurations are temporary until a complete hardware
acceptance profile is supplied to persist. Passwords come from the environment
or an interactive prompt and are never written into reports.
"""
import argparse
import getpass
import json
import os
from pathlib import Path
import re
import shlex
import uuid

import paramiko

NAME = 'e6-teleop-direct'
FIELDS = {
    'connection.id': NAME,
    'connection.interface-name': 'eth0',
    'connection.autoconnect': 'yes',
    'connection.autoconnect-priority': '999',
    'ipv4.method': 'manual',
    'ipv4.addresses': '192.168.1.14/32',
    'ipv4.never-default': 'yes',
    'ipv6.method': 'disabled',
}


class DirectLink:
    def __init__(self, run, owner_uuid, pc_ip='192.168.1.7'):
        self.run = run
        self.uuid = str(uuid.UUID(owner_uuid))
        self.pc_ip = pc_ip

    def route(self, address):
        rows = json.loads(self.run('ip', '-j', 'route', 'get', address))
        if len(rows) != 1:
            raise ValueError('Ambiguous route')
        return rows[0]

    def profiles(self):
        output = self.run('nmcli', '-t', '--escape', 'no', '-f',
                          'UUID,NAME,FILENAME', 'connection', 'show')
        return [dict(zip(('uuid', 'name', 'filename'), line.split(':', 2)))
                for line in output.splitlines() if line]

    def idle(self):
        connections = self.run('ss', '-Htnp')
        if any(re.search(r'192\.168\.1\.4:(29999|30004)\b', line)
               and not line.startswith('TIME-WAIT') for line in connections.splitlines()):
            raise ValueError('Release this project device session before network changes')

    def owned(self):
        matches = [row for row in self.profiles() if row['uuid'] == self.uuid]
        if len(matches) != 1 or matches[0]['name'] != NAME:
            raise ValueError('Owned UUID/name mismatch; no configuration modified')
        for field, expected in FIELDS.items():
            actual = self.run('nmcli', '-g', field, 'con', 'show', 'uuid', self.uuid).strip()
            if actual != expected:
                raise ValueError('Owned profile was changed externally: ' + field)
        routes = self.run('nmcli', '-g', 'ipv4.routes', 'con', 'show', 'uuid', self.uuid).strip()
        if routes != '192.168.1.4/32':
            raise ValueError('Owned E6 host route was changed externally')
        return matches[0]

    def snapshot(self):
        return dict(carrier=self.run('cat', '/sys/class/net/eth0/carrier').strip(),
                    eth0=json.loads(self.run('ip', '-j', 'address', 'show', 'eth0')),
                    e6_route=self.route('192.168.1.4'), pc_route=self.route(self.pc_ip),
                    default_routes=json.loads(self.run('ip', '-j', 'route', 'show', 'default')),
                    profiles=self.profiles())

    def create(self):
        self.idle()
        before = self.snapshot()
        if any(row.get('addr_info') for row in before['eth0']):
            raise ValueError('eth0 already has an address; refuse replacement')
        if any(row['uuid'] == self.uuid or row['name'] == NAME for row in before['profiles']):
            raise ValueError('Profile already exists; use its recorded UUID to audit')
        # NM 1.36 treats UUID as immutable even in `nmcli con add`. Load a
        # new /run keyfile with the recorded identity instead of trying to
        # modify an automatically generated UUID. /run is lost on reboot.
        filename = '/run/NetworkManager/system-connections/e6-teleop-direct-' + self.uuid + '.nmconnection'
        keyfile = ('[connection]\nid=' + NAME + '\nuuid=' + self.uuid
                   + '\ntype=ethernet\ninterface-name=eth0\nautoconnect=true\nautoconnect-priority=999\n'
                   '\n[ethernet]\n\n[ipv4]\nmethod=manual\naddress1=192.168.1.14/32\n'
                   'route1=192.168.1.4/32\nnever-default=true\n\n[ipv6]\nmethod=disabled\n')
        writer = ('import os,sys; from pathlib import Path; p=Path(sys.argv[1]); '
                  'assert p.parent==Path("/run/NetworkManager/system-connections"); '
                  'fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600); '
                  'f=os.fdopen(fd,"w"); f.write(sys.argv[2]); f.close()')
        self.run('python3', '-c', writer, filename, keyfile)
        self.run('nmcli', 'con', 'load', filename)
        return self.finish(before)

    def finish(self, before):
        profile = self.owned()
        after = self.snapshot()
        if after['pc_route'] != before['pc_route'] or after['default_routes'] != before['default_routes']:
            raise RuntimeError('PC/default route changed; retain evidence and stop')
        return dict(uuid=self.uuid, name=NAME, before=before, after=after,
                    profile=profile, temporary=profile['filename'].startswith('/run/'),
                    scope='owned eth0 E6 /32 only; PC/default remain on WLAN',
                    robot_commands_sent=0, can_frames_sent=0)

    def activate(self):
        self.idle()
        self.owned()
        before = self.snapshot()
        if before['carrier'] != '1':
            raise ValueError('Ethernet cable has no carrier')
        self.run('nmcli', '--wait', '10', 'con', 'up', 'uuid', self.uuid, 'ifname', 'eth0')
        result = self.finish(before)
        route = result['after']['e6_route']
        if route.get('dev') != 'eth0' or route.get('prefsrc') != '192.168.1.14' or route.get('gateway'):
            raise RuntimeError('Direct E6 route did not become active')
        return result

    def persist(self, accepted_network):
        self.idle()
        self.owned()
        before = self.snapshot()
        route = before['e6_route']
        actual = dict(robot_ip='192.168.1.4', interface=route.get('dev'), source=route.get('prefsrc'),
                      gateway=route.get('gateway'), direct_ethernet=route.get('dev') == 'eth0' and not route.get('gateway'))
        if before['carrier'] != '1' or actual != accepted_network or not actual['direct_ethernet']:
            raise ValueError('Current link differs from completed hardware acceptance')
        # Default nmcli modify saves the profile, unlike con add save no.
        self.run('nmcli', 'con', 'modify', 'uuid', self.uuid, 'connection.autoconnect', 'yes')
        result = self.finish(before)
        filename = result['profile']['filename']
        if not filename.startswith('/etc/NetworkManager/system-connections/'):
            raise RuntimeError('Persistent profile filename not verified')
        # Reload from disk and verify the same UUID/settings again. This does
        # not reconnect the device or reload unrelated NetworkManager profiles.
        self.run('nmcli', 'con', 'load', filename)
        result['disk_reload_verified'] = self.owned()['filename'] == filename
        if not result['disk_reload_verified']:
            raise RuntimeError('Persisted profile identity changed on disk reload')
        result['reboot_verified'] = False
        return result

    def remove(self):
        self.idle()
        self.owned()
        before = self.snapshot()
        self.run('nmcli', 'con', 'delete', 'uuid', self.uuid)
        after = self.snapshot()
        if any(row['uuid'] == self.uuid for row in after['profiles']):
            raise RuntimeError('Owned profile deletion not verified')
        if after['pc_route'] != before['pc_route'] or after['default_routes'] != before['default_routes']:
            raise RuntimeError('PC/default route changed during removal')
        return dict(uuid=self.uuid, before=before, after=after, removed=True,
                    robot_commands_sent=0, can_frames_sent=0)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('operation', choices=('audit', 'create', 'activate', 'persist', 'remove'))
    parser.add_argument('--host', default='192.168.1.13')
    parser.add_argument('--user', default='root')
    parser.add_argument('--uuid', required=True, help='recorded owner UUID; generate a new UUID for create')
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--acceptance-profile', type=Path)
    args = parser.parse_args()
    if args.evidence.exists():
        parser.error('Use a new evidence path')
    accepted = None
    if args.operation == 'persist':
        if not args.acceptance_profile:
            parser.error('persist requires completed hardware acceptance')
        from e6_teleop.acceptance_profile import verify_profile
        accepted = verify_profile(args.acceptance_profile)['e6_network']
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    report = dict(operation=args.operation, uuid=args.uuid, completed=False)
    try:
        client.connect(args.host, username=args.user,
                       password=os.environ.get('E6_X5_PASSWORD') or getpass.getpass('X5 SSH password: '), timeout=8)
        def run(*arguments):
            _, out, err = client.exec_command(shlex.join(arguments), timeout=15)
            text = out.read().decode()
            if out.channel.recv_exit_status():
                raise RuntimeError(err.read().decode())
            return text
        link = DirectLink(run, args.uuid, client.get_transport().sock.getsockname()[0])
        result = link.snapshot() if args.operation == 'audit' else (
            link.persist(accepted) if args.operation == 'persist' else getattr(link, args.operation)())
        report.update(completed=True, result=result)
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        client.close()
        args.evidence.parent.mkdir(parents=True, exist_ok=True)
        with args.evidence.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report, ensure_ascii=True))


if __name__ == '__main__':
    main()
