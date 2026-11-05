import numpy as np
from studio.model import Assembly, stl_vertices


def test_official_meshes_and_tree():
    a=Assembly()
    assert len(a.links)==11 and len(a.joints)==10 and len(a.visuals)==9
    assert len(a.transforms({}))==11
    for _,path,_ in a.visuals:
        v=stl_vertices(path)
        assert len(v)>0 and np.isfinite(v).all()


def test_closure_and_continuity_entire_display_range():
    a=Assembly();zero=a.transforms({});previous=None
    for p in np.linspace(0,100,1001):
        angles=a.nominal_angles(p);now=a.transforms(angles)
        for side in ['narrow','wide']:
            loop='hand_'+side+'_loop_Link';tip='hand_'+side+'4_Link'
            endpoint=np.linalg.inv(zero[loop])@zero[tip][:,3]
            closed=now[loop]@endpoint
            # Official rounded origins leave sub-micrometre out-of-plane error.
            assert np.linalg.norm(closed[:3]-now[tip][:3,3])<1e-6
        if previous:
            assert max(abs(angles[k]-previous[k]) for k in angles)<.02
        previous=angles
