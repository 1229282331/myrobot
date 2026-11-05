"""Priority limited to one owned control TCP socket.

IP_TOS alone isn't sufficient on Windows. No adapter, registry, routing,
firewall or system policy is changed. API success is not proof of wire QoS.
"""
import sys
import socket
import struct


def linux_low_latency_recovery(sock):
    """Linux 6.18 owned socket options; no application retransmission/lease change.

    Constants are pinned from Linux v6.18 UAPI tcp.h: MIN_US=45, DELACK_US=46.
    Unsupported kernels fail startup rather than silently keeping a slower path.
    """
    if sys.platform!='linux':raise ValueError('Owned Linux SSH socket required')
    sock.setsockopt(socket.IPPROTO_TCP,45,20_000)
    sock.setsockopt(socket.IPPROTO_TCP,46,10_000)
    sock.setsockopt(socket.IPPROTO_TCP,16,1)
    effective=dict(min_rto_us=sock.getsockopt(socket.IPPROTO_TCP,45),
                   delayed_ack_max_us=sock.getsockopt(socket.IPPROTO_TCP,46),
                   thin_linear_timeouts=sock.getsockopt(socket.IPPROTO_TCP,16))
    if effective['min_rto_us']!=20_000 or not 8_000<=effective['delayed_ack_max_us']<=12_000 or effective['thin_linear_timeouts']!=1:
        raise ValueError('Owned Linux SSH retransmission settings did not apply')
    return dict(effective,scope='one owned Linux SSH TCP socket',kernel_uapi='Linux v6.18 tcp.h',
                application_retry=False,lease_changed=False)


def linux_tcp_profile(sock):
    """Read the actual Linux socket RTO; never infer it from the route floor."""
    data=sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_INFO,104)
    return dict(rto_ms=struct.unpack_from('=I',data,8)[0]/1000,
                measured_on='PC owned SSH socket',route_minimum_is_not_actual_rto=True)


class ControlSocketQos:
    def __init__(self,sock):
        self.handle=None;self.dll=None
        self.report=dict(scope='owned control TCP socket',api='qWAVE',
                         supported=sys.platform=='win32',traffic_type='QOSTrafficTypeControl',
                         non_adaptive=True,requested_dscp=56,wire_verified=False,
                         local_endpoint=list(sock.getsockname()),remote_endpoint=list(sock.getpeername()))
        if sys.platform=='linux':
            before=sock.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)
            sock.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,224)
            effective=sock.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)
            self.report.update(api='IP_TOS',supported=True,original_ip_tos=before,
                               effective_ip_tos=effective,configured=effective==224,
                               tcp_timing=linux_tcp_profile(sock))
            if effective!=224:raise ValueError('Owned Linux SSH socket priority did not apply')
            return
        if sys.platform!='win32':return
        import ctypes
        from ctypes import wintypes as w
        class Version(ctypes.Structure):
            _fields_=[('major',w.USHORT),('minor',w.USHORT)]
        try:
            dll=ctypes.WinDLL('qwave.dll',use_last_error=True);self.dll=dll
            dll.QOSCreateHandle.argtypes=[ctypes.POINTER(Version),ctypes.POINTER(w.HANDLE)];dll.QOSCreateHandle.restype=w.BOOL
            dll.QOSAddSocketToFlow.argtypes=[w.HANDLE,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_int,w.DWORD,ctypes.POINTER(w.DWORD)];dll.QOSAddSocketToFlow.restype=w.BOOL
            dll.QOSSetFlow.argtypes=[w.HANDLE,w.DWORD,ctypes.c_int,w.ULONG,ctypes.c_void_p,w.DWORD,ctypes.c_void_p];dll.QOSSetFlow.restype=w.BOOL
            dll.QOSCloseHandle.argtypes=[w.HANDLE];dll.QOSCloseHandle.restype=w.BOOL
            handle=w.HANDLE();version=Version(1,0)
            if not dll.QOSCreateHandle(ctypes.byref(version),ctypes.byref(handle)):raise ctypes.WinError(ctypes.get_last_error())
            self.handle=handle;flow=w.DWORD(0)
            # QOS_NON_ADAPTIVE_FLOW: no LLTD probing of the Linux endpoint.
            if not dll.QOSAddSocketToFlow(handle,sock.fileno(),None,5,2,ctypes.byref(flow)):raise ctypes.WinError(ctypes.get_last_error())
            traffic=ctypes.c_int(5)
            if not dll.QOSSetFlow(handle,flow,0,ctypes.sizeof(traffic),ctypes.byref(traffic),0,None):raise ctypes.WinError(ctypes.get_last_error())
            self.report['configured']=True
        except OSError as exc:
            self.report.update(configured=False,error=str(exc))
            self.close()

    def close(self):
        if self.handle is not None:
            self.report['released']=bool(self.dll.QOSCloseHandle(self.handle))
            self.handle=None
