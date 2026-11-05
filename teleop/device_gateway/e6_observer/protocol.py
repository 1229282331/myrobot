"""Pinned E6 feedback and observation v2, with lossless media transport."""
import base64
import math
import struct
import time
import json


MEDIA_KEYS = (('rgb','jpeg'),('depth','uint16_raw'),('rgb','preview_jpeg'),('depth','preview_jpeg'))
MAX_WIRE_BYTES = 8*1024*1024
# v3 carries bounded action/CAN event windows as well as observation v2.
# Keep the shared encoder/decoder limit explicit; 64 KiB rejected valid
# control records once request-associated gripper telemetry was present.
MAX_HEADER_BYTES = 256*1024


def encode_wire(frame, media_cache=None):
    """Small JSON header + native image bytes, lossless fast Zstandard framing."""
    import zstandard
    header=dict(frame,sources={n:dict(s) for n,s in frame['sources'].items()})
    blocks=[];media=[];references=[]
    for name,key in MEDIA_KEYS:
        encoded=header['sources'][name].pop(key,None)
        if encoded:
            data=encoded if isinstance(encoded,bytes) else base64.b64decode(encoded,validate=True)
            identity=header['sources'][name]['device_frame_id']
            if media_cache is not None and media_cache.get((name,key))==identity:
                references.append(dict(source=name,key=key,id=identity))
            else:
                media.append(dict(source=name,key=key,bytes=len(data)));blocks.append(data)
                if media_cache is not None:media_cache[(name,key)]=identity
    header['wire_media']=media
    header['wire_references']=references
    raw=json.dumps(header,allow_nan=False).encode('utf-8')
    if not 0<len(raw)<=MAX_HEADER_BYTES:raise ValueError('Invalid observation header size')
    if 4+len(raw)+sum(map(len,blocks))>MAX_WIRE_BYTES:raise ValueError('Invalid observation wire size')
    return zstandard.ZstdCompressor(level=1,write_checksum=True).compress(struct.pack('<I',len(raw))+raw+b''.join(blocks))


def decode_wire(payload, media_cache=None):
    import zstandard
    if not isinstance(payload,bytes):return json.loads(payload)
    size=zstandard.frame_content_size(payload)
    if not 0<size<=MAX_WIRE_BYTES:raise ValueError('Invalid observation wire size')
    data=zstandard.ZstdDecompressor().decompress(payload,max_output_size=MAX_WIRE_BYTES)
    if len(data)<4:raise ValueError('Truncated observation header')
    size=struct.unpack_from('<I',data)[0]
    if not 0<size<=MAX_HEADER_BYTES or len(data)<4+size:raise ValueError('Invalid observation header')
    frame=json.loads(data[4:4+size]);offset=4+size
    seen=set()
    for media in frame.pop('wire_media'):
        n,key,count=media['source'],media['key'],media['bytes']
        if (n,key) not in MEDIA_KEYS or (n,key) in seen or not isinstance(count,int) or count<0 or offset+count>len(data):raise ValueError('Invalid media block')
        seen.add((n,key))
        frame['sources'][n][key]=base64.b64encode(data[offset:offset+count]).decode()
        if media_cache is not None:media_cache[(n,key)]=(frame['sources'][n]['device_frame_id'],frame['sources'][n][key])
        offset+=count
    for reference in frame.pop('wire_references',[]):
        n,key,identity=reference['source'],reference['key'],reference['id']
        if (n,key) not in MEDIA_KEYS or (n,key) in seen or media_cache is None or media_cache.get((n,key),(None,))[0]!=identity or frame['sources'][n]['device_frame_id']!=identity:raise ValueError('Unknown media reference')
        seen.add((n,key))
        frame['sources'][n][key]=media_cache[(n,key)][1]
    if offset!=len(data):raise ValueError('Trailing observation wire data')
    return frame

SDK_COMMIT = "82bce656c6e1a1c71599901baa93ed45bb50963d"
SIZE = 1440
READ_QUERIES = ("RobotMode()", "GetAngle()", "GetPose()", "GetErrorID()")


class FeedbackParser:
    def __init__(self, extended=False, allow_length_envelope=False):
        self.buffer = bytearray()
        self.extended=extended
        self.previous_controller_time=None;self.previous_runtime=None
        self.allow_length_envelope=allow_length_envelope
        self.last_transport_prefixes=[];self.length_envelopes=0

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        self.last_transport_prefixes=[]
        while len(self.buffer) >= 2:
            if struct.unpack_from("<H", self.buffer)[0] != SIZE:
                raise ValueError(f"Unexpected E6 feedback length {struct.unpack_from('<H',self.buffer)[0]}; buffered={len(self.buffer)}; prefix={bytes(self.buffer[:16]).hex()}; no guessed resynchronization")
            if len(self.buffer) < SIZE:
                break
            prefix=b'';consumed=SIZE
            packet = bytes(self.buffer[:SIZE])
            # One captured transport variant is exactly uint64(1440) followed
            # by the unchanged 1440-byte SDK payload, which also begins with
            # uint64(1440). Normal SDK packets take precedence. No scanning,
            # arbitrary offset skipping, or permissive marker recovery.
            if (struct.unpack_from('<Q',packet,48)[0]!=0x123456789ABCDEF
                    and self.allow_length_envelope and packet[:16]==struct.pack('<QQ',SIZE,SIZE)):
                if len(self.buffer)<SIZE+8:break
                prefix=bytes(self.buffer[:8]);packet=bytes(self.buffer[8:SIZE+8]);consumed=SIZE+8
            decoded=decode(packet,self.extended)
            if self.extended:
                timestamp=decoded['controller_timestamp_raw'];runtime=decoded['controller_runtime_raw']
                if self.previous_controller_time is not None and (timestamp<self.previous_controller_time or runtime<self.previous_runtime):raise ValueError('E6 controller time reset; explicit new session required')
                self.previous_controller_time=timestamp;self.previous_runtime=runtime
            del self.buffer[:consumed]
            self.last_transport_prefixes.append(prefix)
            self.length_envelopes+=int(bool(prefix))
            result.append((packet, decoded))
        return result

    def eof(self):
        if self.buffer:
            raise ValueError("Truncated E6 feedback at disconnect")


def decode(packet,extended=False):
    if len(packet) != SIZE or struct.unpack_from("<H", packet)[0] != SIZE:
        raise ValueError("Invalid E6 feedback size")
    if struct.unpack_from("<Q", packet, 48)[0] != 0x123456789ABCDEF:
        raise ValueError("Invalid E6 test marker")
    q = list(struct.unpack_from("<6d", packet, 432))
    pose = list(struct.unpack_from("<6d", packet, 624))
    tool = list(struct.unpack_from("<6d", packet, 1248))
    load = list(struct.unpack_from("<4d", packet, 1168))
    q_target=list(struct.unpack_from('<6d',packet,192))
    q_velocity=list(struct.unpack_from('<6d',packet,480))
    user_value=list(struct.unpack_from('<6d',packet,1200))
    if not all(math.isfinite(v) for v in q + pose + tool + load + (q_target + q_velocity + user_value if extended else [])):
        raise ValueError("Non-finite E6 feedback")
    if any(abs(v) > 360 for v in q):
        raise ValueError("Invalid E6 joint angle")
    # Controller Rx/Ry/Rz are extrinsic XYZ: Rz @ Ry @ Rx; retain raw as well.
    r, p, y = [math.radians(v) / 2 for v in pose[3:]]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    quat = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
    result=dict(joints_rad=[math.radians(v) for v in q], q_actual_deg=q,
                controller_position_m=[v/1000 for v in pose[:3]], controller_quaternion_xyzw=quat,
                controller_pose_raw_mm_deg=pose, robot_mode=struct.unpack_from("<Q", packet, 24)[0],
                controller_timestamp_raw=struct.unpack_from('<Q',packet,32)[0], controller_runtime_raw=struct.unpack_from('<Q',packet,40)[0],
                user_index=packet[1012], tool_index=packet[1013], enable_status=packet[1026],
                error_status=packet[1029], payload_parameter_kg=load[0], center_parameter_mm=load[1:],
                tool_configuration_raw_mm_deg=tool, layout_source_commit=SDK_COMMIT,
                pose_reference="controller active User/Tool; not a measured finger TCP")
    if extended:result.update(q_target_deg=q_target,q_velocity_deg_s=q_velocity,user_configuration_raw_mm_deg=user_value,
                              run_queued_command=packet[1014],pause_command_flag=packet[1015],collision_state=packet[1038],
                              current_command_id=struct.unpack_from('<Q',packet,1112)[0])
    return result


def source(value, now, max_age):
    if value is None:
        return dict(valid=False, available=False, reason="尚未收到新数据", age_ms=None)
    result = dict(value)
    age = max(0, (now-value["x5_receive_monotonic_ns"])/1e6)
    result.update(age_ms=age, available=True, valid=age <= max_age and not value.get("error") and value.get('data_valid',True) and value.get('valid',True))
    result["reason"] = value.get("error") or ("深度整帧无有效值" if not value.get('data_valid',True) else (None if result["valid"] else value.get('reason') or "源数据已过期或无效"))
    return result


def assemble(latest, stats, instance, seq, now=None):
    now = now if now is not None else time.perf_counter_ns()
    sources = {name: source(latest.get(name), now, age) for name, age in (("robot",250),("rgb",500),("depth",500))}
    for name in ('robot','rgb','depth'):
        if stats.get(name+'_error'):
            sources[name].update(valid=False,reason=stats[name+'_error'])
    sources["gripper"] = dict(source='can1 interface inspection only',seq=None,x5_receive_monotonic_ns=None,valid=False, available=False, reason="当前遥测不可用：严格观测不发送CAN帧", opening=None, tx_count=0)
    associations = {}
    robot = sources["robot"]
    for name in ("rgb", "depth"):
        s = sources[name]
        delta = (s["x5_receive_monotonic_ns"]-robot["x5_receive_monotonic_ns"])/1e6 if s["available"] and robot["available"] else None
        associations[name+"_robot"] = dict(receive_delta_ms=delta, valid=delta is not None and abs(delta)<=100 and s["valid"] and robot["valid"])
    decoded = robot.get("decoded", {})
    return dict(schema_version=2, backend="hardware-observe", instance=instance,
                capabilities=dict(control=False, recording=True, gripper_telemetry=False),
                input=None, effective_action=None, sources=sources, associations=associations, stats=stats,
                observation=dict(joints_rad=decoded.get("joints_rad"), gripper=None,
                                 frame_id=seq, gateway_monotonic_ns=now,
                                 valid=robot["valid"] and sources["rgb"]["valid"] and not stats.get('fatal'), is_simulated=False),
                status=dict(armed=False, reason="真实设备 · 只读观测"))
