"""Set DSCP on the accepted socket of this SSH session only.

No sshd configuration, route, interface or other session is modified.
Closing the SSH connection releases the setting. Packet marking is a request,
not a guarantee that the access point implements priority.
"""
import ipaddress
import json
import os
from pathlib import Path
import socket
import sys
import struct


def tcp_timing(sock):
    # Linux TCP_INFO starts with eight u8 fields followed by tcpi_rto in us.
    # Read-only evidence; a route's minimum is not the measured socket RTO.
    raw=sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_INFO,104)
    return dict(rto_ms=struct.unpack_from('=I',raw,8)[0]/1000,
                measured_on='X5 accepted SSH socket',route_minimum_is_not_actual_rto=True)


def session_endpoints(expected_peer,connection):
    fields=connection.split()
    if len(fields)!=4:raise ValueError('An owned SSH_CONNECTION is required')
    client,port,server,server_port=fields
    peer=[str(ipaddress.IPv4Address(client)),int(port)]
    local=[str(ipaddress.IPv4Address(server)),int(server_port)]
    if peer!=expected_peer or not 0<peer[1]<65536 or not 0<local[1]<65536:
        raise ValueError('SSH socket identity mismatch')
    return peer,local


def configure(expected_peer):
    peer,local=session_endpoints(expected_peer,os.environ.get('SSH_CONNECTION',''))
    result=dict(scope='accepted socket of this owned SSH session',requested_dscp=56,
                configured=False,wire_verified=False,peer=peer,local=local)
    if sys.platform!='linux' or not hasattr(os,'pidfd_open'):
        return dict(result,error='Linux pidfd is unavailable')
    import ctypes
    libc=ctypes.CDLL(None,use_errno=True);libc.syscall.restype=ctypes.c_long
    pid=os.getppid();errors=[]
    for _ in range(8):
        if pid<=1:break
        parent=Path('/proc')/str(pid)
        if (parent/'comm').read_text().strip()=='sshd':
            handle=os.pidfd_open(pid)
            try:
                for entry in (parent/'fd').iterdir():
                    try:
                        if not str(entry.readlink()).startswith('socket:'):continue
                        duplicate=libc.syscall(438,handle,int(entry.name),0)
                        if duplicate<0:
                            errors.append(os.strerror(ctypes.get_errno()));continue
                        with socket.socket(fileno=duplicate) as sock:
                            try:actual_peer=list(sock.getpeername());actual_local=list(sock.getsockname())
                            except OSError:continue
                            if actual_peer!=peer or actual_local!=local:continue
                            before=sock.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)
                            sock.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,56<<2)
                            sock.setsockopt(socket.IPPROTO_TCP,16,1)
                            thin=sock.getsockopt(socket.IPPROTO_TCP,16)
                            if thin!=1:raise ValueError('Owned SSH thin-stream recovery did not apply')
                            effective=sock.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)
                            return dict(result,configured=effective==224,original_ip_tos=before,
                                        effective_ip_tos=effective,owner_pid=pid,thin_linear_timeouts=thin,tcp_timing=tcp_timing(sock))
                    except OSError as exc:errors.append(str(exc))
            finally:os.close(handle)
        pid=int((parent/'stat').read_text().rsplit(')',1)[1].split()[1])
    return dict(result,error='No accessible matching ancestor SSH socket',diagnostics=sorted(set(errors)))


if __name__=='__main__':
    print(json.dumps(configure(json.loads(sys.argv[1]))))
