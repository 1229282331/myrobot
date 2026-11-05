"""Offline diagnostics only; run with the pinned dpkt 1.9.8 environment."""
import importlib.util
from pathlib import Path
import socket
import struct
import tempfile
import unittest

try:
    import dpkt
except ImportError:
    raise unittest.SkipTest('Optional offline PCAP analyzer requires dpkt==1.9.8')

spec=importlib.util.spec_from_file_location('pcap_audit',Path(__file__).parents[1]/'scripts/analyze_feedback_pcap.py')
analyzer=importlib.util.module_from_spec(spec);spec.loader.exec_module(analyzer)


class FeedbackPcapTests(unittest.TestCase):
    def analyze(self,packets):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'test.pcap'
            with path.open('wb') as stream:
                writer=dpkt.pcap.Writer(stream)
                for n,(seq,payload) in enumerate(packets):
                    tcp=dpkt.tcp.TCP(sport=30004,dport=40000,seq=seq,flags=dpkt.tcp.TH_ACK,data=payload)
                    ip=dpkt.ip.IP(src=socket.inet_aton('192.168.1.4'),dst=socket.inet_aton('192.168.1.14'),p=6,data=tcp)
                    writer.writepkt(bytes(dpkt.ethernet.Ethernet(type=0x800,data=ip)),ts=1000+n*.008)
            return analyzer.analyze(path)['flows'][0]

    @staticmethod
    def frame():
        data=bytearray(1440);struct.pack_into('<Q',data,0,1440)
        struct.pack_into('<Q',data,48,0x0123456789abcdef)
        return bytes(data)

    def test_fragmentation_reordering_and_identical_retransmission(self):
        data=self.frame()*3
        result=self.analyze([(100,data[:800]),(2200,data[2100:]),(900,data[800:2100]),(900,data[800:2100])])
        self.assertEqual(result['tcp_gaps'],[])
        self.assertEqual(result['conflicting_retransmissions'],[])
        self.assertEqual(result['runs'][0]['header_distances'],{1440:2})

    def test_gap_and_conflict_remain_evidence(self):
        data=self.frame()
        result=self.analyze([(100,data),(100,b'x'+data[1:]),(3000,data)])
        self.assertEqual(len(result['tcp_gaps']),1)
        self.assertEqual(len(result['conflicting_retransmissions']),1)
        self.assertEqual(len(result['runs']),2)

    def test_device_truncated_prefix_is_not_discarded(self):
        data=self.frame()[:80]+self.frame()*2
        result=self.analyze([(100,data)])
        unusual=result['runs'][0]['unusual']
        self.assertEqual([r['distance'] for r in unusual],[80])
        self.assertEqual(unusual[0]['prefix_hex'],data[:80].hex())
