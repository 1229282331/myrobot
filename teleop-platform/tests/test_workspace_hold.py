"""A held outward input must not restart motion after a workspace Stop."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_gateway import setup
from test_moveit_bridge import Bridge, row


@pytest.mark.parametrize('speed',[.15707963267948966,.04])
def test_boundary_stays_stopped_for_held_direction_and_speed_change(speed):
    e,d,send,step,now=setup(moveit_bundle='fixture',motion_profile='normal',workspace_config={'rotation_deg':.65})
    e.moveit=Bridge()
    # Already near the workspace edge; first physical write is a small step.
    d.data['status']['joint_state']['q_actual'][5]=.48
    d.data['status']['joint_state']['q_target'][5]=.48
    def target(angle):
        r=row(now[0],e.session.sequence)
        r['quaternion_xyzw']=Rotation.from_euler('x',angle,degrees=True).as_quat().tolist()
        r['joints_rad'][5]=np.deg2rad(angle)
        e.moveit.row=r
    send(deadman=True,angular_rad_s=[.15707963267948966,0,0]);target(.5);step()
    send(deadman=True,angular_rad_s=[.15707963267948966,0,0]);target(.9);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop']
    d.data['status']['tool_vector_actual']['rx']=.3
    d.data['status']['joint_state']['q_actual'][5]=.3
    d.data['status']['joint_state']['q_target'][5]=.3
    step()  # Stop reply; actual settles inside the boundary.
    submits=sum(isinstance(c,tuple) for c in e.moveit.calls)
    for _ in range(20):
        send(deadman=True,angular_rad_s=[speed,0,0]);target(.5);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop']
    assert sum(isinstance(c,tuple) for c in e.moveit.calls)==submits
    assert e.session.ready and e.stop_acknowledged and '边界' in e.reason
    # Reversing is a new direction, still checked against the original range.
    # A confirmed settled Stop restores the first-step bound (0.095 deg).
    send(deadman=True,angular_rad_s=[-.04,0,0]);target(.22);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop','ServoJ']
    assert e.reason is None


@pytest.mark.parametrize('neutral',[{'deadman':False},{'deadman':True}])
def test_centering_releases_directional_block_without_repreparing(neutral):
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0])
    r=row(now[0]);r['position_m'][0]=.051;e.moveit.row=r;step()
    assert not d.calls and e.blocked_signature is not None
    send(**neutral);step()
    assert e.blocked_signature is None and e.session.ready
    send(deadman=True,linear_m_s=[-.002,0,0]);r=row(now[0]);r['position_m'][0]=.001
    e.moveit.row=r;step()
    assert d.calls[-1][0]=='ServoJ'
