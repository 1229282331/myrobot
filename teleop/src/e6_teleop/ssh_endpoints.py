"""Authenticated SSH endpoint checks, including a PC behind WSL NAT."""
import ipaddress


def observed_peer(connection,destination):
    fields=connection.split()
    if len(fields)!=4:raise ValueError('Owned SSH_CONNECTION required')
    peer=[str(ipaddress.IPv4Address(fields[0])),int(fields[1])]
    server=[str(ipaddress.IPv4Address(fields[2])),int(fields[3])]
    if (server!=destination or not ipaddress.ip_address(peer[0]).is_private
            or not 0<peer[1]<65536 or not 0<server[1]<65536):
        raise ValueError('Authenticated SSH destination/peer mismatch')
    return peer
