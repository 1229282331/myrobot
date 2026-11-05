"""A failed/incomplete acceptance never unlocks physical operator control."""
import hashlib,json,math
from pathlib import Path
from .model import ROOT

REQUIRED=('offline','shadow_600s','zero_servoj','j6_return','cartesian_axes_return','frames_multi_fine',
          'gripper_15_35_15','faults','gamepad_30s','keyboard_30s','raw_media_readback')
CRITICAL=('device_gateway/e6_observer/control_core.py','device_gateway/e6_observer/control_guardian.py',
          'device_gateway/e6_observer/gripper_profile.py','src/e6_teleop/gripper_profile.py','config/gripper_profiles.json','src/e6_teleop/operator_launcher.py',
          'device_gateway/e6_observer/recovery.py',
          'device_gateway/e6_observer/control_math.py','device_gateway/e6_observer/runtime.py',
          'device_gateway/e6_observer/local_kinematics.py','device_gateway/e6_observer/kinematics_profile.py',
          'device_gateway/e6_observer/protocol.py','device_gateway/e6_observer/__main__.py',
          'device_gateway/e6_observer/feedback_diagnostics.py','device_gateway/e6_observer/wifi_latency.py',
          'device_gateway/e6_observer/camera_frames.py',
          'device_gateway/e6_observer/process_lifecycle.py',
          'device_gateway/e6_observer/ssh_socket_qos.py',
          'device_gateway/e6_observer/ssh_tcp_latency.py',
          'device_gateway/e6_observer/tcp_latency.py',
          'web/src/input.ts','web/src/HardwareControlPanel.tsx','web/src/hardwareCapabilities.ts','src/e6_teleop/observation_service.py',
          'src/e6_teleop/observation_recording.py','src/e6_teleop/service.py','src/e6_teleop/tunnel.py',
          'src/e6_teleop/control_transport.py',
          'src/e6_teleop/socket_qos.py',
          'src/e6_teleop/ssh_endpoints.py','src/e6_teleop/pc_tcp_latency.py','src/e6_teleop/observation_clock.py',
          'src/e6_teleop/wsl_tunnel.py','src/e6_teleop/ssh_runtime.py','scripts/configure_pc_peer_route.py','src/e6_teleop/kinematics.py',
          'src/e6_teleop/cli.py','src/e6_teleop/acceptance_profile.py','assets/e6/manifest.json')

def verify_profile(path):
    path=Path(path).resolve();p=json.loads(path.read_text(encoding='utf-8'))
    if p.get('schema_version')!=1 or p.get('complete') is not True:raise ValueError('Hardware acceptance is incomplete')
    scope=p.get('control_scope','complete')
    if scope not in ('complete','arm-only'):raise ValueError('Unknown hardware control scope')
    if (scope=='arm-only')!=(p.get('gripper_command_profile')=='disabled'):
        raise ValueError('Arm-only acceptance requires the disabled gripper profile')
    if p.get('recovery_policy','manual') not in ('manual','bounded'):raise ValueError('Unknown accepted recovery policy')
    if p.get('recovery_policy')=='bounded' and p.get('checks',{}).get('bounded_recovery',{}).get('passed') is not True:raise ValueError('Bounded recovery has not passed hardware acceptance')
    if p.get('experiment_only',False) or p.get('eligible_for_hardware_acceptance') is False:raise ValueError('Read-only experiments cannot unlock hardware control')
    if p.get('robot_ip')!='192.168.1.4' or p.get('camera_serial')!='AY4A1520111' or p.get('tool')!=0 or p.get('user')!=0:raise ValueError('Acceptance device/configuration mismatch')
    if p.get('kinematics_kind','controller-native') not in ('controller-native','controller-local') or p.get('feedback_framing','strict') not in ('strict','length-envelope'):raise ValueError('Unknown accepted kinematics/framing')
    if p.get('kinematics_kind')=='controller-local' and (not isinstance(p.get('kinematics_profile_sha256'),str) or len(p['kinematics_profile_sha256'])!=64):raise ValueError('Accepted local calibration hash missing')
    if not set(CRITICAL)<=set(p.get('source_sha256',{})):raise ValueError('Acceptance critical source hashes missing')
    if p.get('wifi_driver_ps_on') not in ('Y','N'):raise ValueError('Acceptance Wi-Fi driver PS parameter missing')
    if p.get('media_hz') not in (5,10,20):raise ValueError('Accepted preview rate missing')
    pc=p.get('pc_transport')
    if pc not in (dict(platform='win32',requested_min_rto_ms=None,effective_min_rto_ms=None,interface=None,preview_renderer=None),
                  dict(platform='win32-wsl-ssh',requested_min_rto_ms=20,effective_min_rto_ms=20,interface='eth0',preview_renderer=None)):
        raise ValueError('Accepted PC TCP configuration missing or unsupported')
    network=p.get('e6_network') or {}
    if (network.get('robot_ip')!='192.168.1.4' or network.get('interface')!='eth0'
            or network.get('source')!='192.168.1.14' or network.get('gateway') is not None
            or network.get('direct_ethernet') is not True):
        raise ValueError('Accepted direct E6 Ethernet route missing')
    for relative,expected in p['source_sha256'].items():
        source=(ROOT/relative).resolve()
        if ROOT not in source.parents or not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()!=expected:raise ValueError('Acceptance source changed: '+relative)
    can_driver=ROOT.parent/'omnipicker/tools/verify_can_control.py'
    if hashlib.sha256(can_driver.read_bytes()).hexdigest()!=p.get('can_driver_sha256'):raise ValueError('Acceptance CAN driver changed or hash missing')
    required=tuple(k for k in REQUIRED if scope!='arm-only' or k!='gripper_15_35_15')
    if scope=='arm-only':required+=('gripper_disabled',)
    for name in required+(('bounded_recovery',) if p.get('recovery_policy')=='bounded' else ()):
        check=p.get('checks',{}).get(name,{})
        if check.get('passed') is not True:raise ValueError('Acceptance check has not passed: '+name)
        evidence=(path.parent/check['file']).resolve()
        if not evidence.is_file() or hashlib.sha256(evidence.read_bytes()).hexdigest()!=check['sha256']:raise ValueError('Acceptance evidence changed: '+name)
    limits=p.get('limits',{})
    from .gripper_profile import gripper_profile
    settings=gripper_profile(p.get('gripper_command_profile','plan32'))
    if limits.get('gripper_s',float('inf'))>settings['gripper_s']:raise ValueError('Accepted rate exceeds fixed gripper profile')
    for name,ceiling in dict(translation_m_s=.005,rotation_rad_s=math.radians(2),gripper_s=.1,radius_m=.05,orientation_rad=math.radians(10),joint_window_rad=math.radians(10),joint_velocity_rad_s=math.radians(2),joint_accel_rad_s2=math.radians(4)).items():
        value=limits.get(name)
        if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=ceiling:raise ValueError('Untested control limit: '+name)
    adjustable={'translation_m_s','rotation_rad_s','gripper_s','radius_m','orientation_rad','joint_window_rad','joint_velocity_rad_s','joint_accel_rad_s2'}
    if set(limits)!=adjustable:raise ValueError('Profile may not alter leases, transaction budgets or immutable protection configuration')
    return p
