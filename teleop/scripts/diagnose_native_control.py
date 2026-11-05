"""Read-only X5 diagnostic: native IK/FK timing and exact feedback byte stream.

Run in the deployed X5 environment. Long query deadlines are diagnostic only;
this program never enables, moves, stops, configures or transmits CAN.
"""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import threading
import time

from e6_observer.control_guardian import DashboardLink
from e6_observer.protocol import FeedbackParser


def run(ip, seconds, directory, query_tos=0xB8):
    directory.mkdir(parents=True, exist_ok=False)
    done = threading.Event()
    state = {}
    errors = []
    counts = {'feedback': 0}
    capture = subprocess.Popen(['tcpdump','-i','wlan0','-s','0','-U','-w',str(directory/'e6.pcap'),
                                'host',ip,'and','tcp','and','(','port','29999','or','port','30004',')'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def feedback():
        parser = FeedbackParser(extended=True)
        offset = 0
        with (directory/'feedback.bin').open('wb') as raw_file, (directory/'receives.jsonl').open('w') as events:
            try:
                with socket.create_connection((ip, 30004), timeout=2) as stream:
                    stream.settimeout(.1)
                    while not done.is_set():
                        try: raw = stream.recv(16384)
                        except socket.timeout: continue
                        now = time.perf_counter_ns()
                        events.write(json.dumps({'x5_receive_ns': now, 'offset': offset, 'bytes': len(raw)})+'\n')
                        raw_file.write(raw)
                        raw_file.flush()
                        events.flush()
                        offset += len(raw)
                        if not raw:
                            parser.eof()
                            raise ConnectionError('Feedback disconnected')
                        for _, decoded in parser.feed(raw):
                            counts['feedback'] += 1
                            state.update(decoded=decoded, receive_ns=now)
            except Exception as exc:
                errors.append({'kind': 'feedback', 'error': str(exc), 'offset': offset, 'buffer_hex': bytes(parser.buffer[:2880]).hex()})
                done.set()

    worker = threading.Thread(target=feedback, daemon=True)
    worker.start()
    link = None
    samples = []
    start = time.monotonic()
    try:
        link = DashboardLink(ip)
        link.sock.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,query_tos)
        while not state and not done.wait(.01):
            if time.monotonic()-start>3: raise RuntimeError('No feedback')
        deadline = time.perf_counter()
        while time.monotonic()-start<seconds and not done.is_set():
            snapshot = state.copy()
            d = snapshot['decoded']
            begin = time.perf_counter_ns()
            q = link.ik(d['controller_pose_raw_mm_deg'], d['q_actual_deg'], begin+1_000_000_000)
            middle = time.perf_counter_ns()
            p = link.fk(q, middle+1_000_000_000)
            end = time.perf_counter_ns()
            samples.append({'start_ns':begin, 'ik_ms':(middle-begin)/1e6, 'fk_ms':(end-middle)/1e6,
                            'pair_ms':(end-begin)/1e6, 'q':q, 'pose':p, 'feedback_receive_ns':snapshot['receive_ns'],
                            'feedback_age_ms':(begin-snapshot['receive_ns'])/1e6,
                            'events':list(link.events)[-2:]})
            deadline += .05
            if time.perf_counter()>deadline: deadline = time.perf_counter()+.05
            done.wait(max(0, deadline-time.perf_counter()))
    except Exception as exc: errors.append({'kind':'query', 'error':str(exc)})
    finally:
        done.set()
        worker.join(3)
        if link: link.close()
        capture.terminate()
        _, capture_log = capture.communicate(timeout=5)
        (directory/'capture.txt').write_bytes(capture_log)
        result = {'duration_s':time.monotonic()-start, **counts, 'samples':samples, 'errors':errors,
                  'motion_tx':link.motion_tx if link else 0, 'stop_tx':link.stop_tx if link else 0, 'can_tx':0,
                  'query_ip_tos':query_tos,
                  'diagnostic_only':True, 'control_transaction_limit_ms':50}
        (directory/'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k!='samples'}), flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--ip', default='192.168.1.4')
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--evidence', type=Path, required=True)
    p.add_argument('--wifi-low-latency', action='store_true')
    p.add_argument('--wifi-reorder-ms',type=int)
    p.add_argument('--tcp-rto-min-ms',type=int)
    p.add_argument('--query-tos',type=int,choices=(0,184),default=184)
    a=p.parse_args()
    from e6_observer.wifi_latency import WifiLatency
    from e6_observer.tcp_latency import TcpLatency
    with WifiLatency(a.wifi_low_latency,a.wifi_reorder_ms), TcpLatency(a.ip,a.tcp_rto_min_ms) as tcp:
        run(a.ip, a.seconds, a.evidence,a.query_tos)
    (a.evidence/'transport_settings.json').write_text(json.dumps(tcp.report,indent=2),encoding='utf-8')
