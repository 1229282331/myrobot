from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from fastapi import APIRouter, FastAPI
from fastapi.testclient import TestClient
from e6_phosphobot.workspace import TestWorkspace
from e6_phosphobot.gateway import evaluate_workspace, NativeExecutor
from e6_phosphobot.assembly_model import ASSETS, verified_assets, attach
from e6_phosphobot.workspace_api import install_workspace_api
from e6_phosphobot.tool_frame import get_tool


def test_legacy_default_and_model_mode_do_not_create_a_new_radius():
    old=TestWorkspace.from_config(dict(position_m=.05,rotation_deg=None,joint_deg=None))
    model=TestWorkspace.from_config(dict(mode='model',position_m=.05,rotation_deg=None,joint_deg=None))
    assert old.description()==dict(position_m=.05,rotation_deg=None,joint_deg=None)
    base=(np.zeros(3),Rotation.identity(),np.zeros(6))
    far=(np.array([.25,0,0]),Rotation.identity(),np.zeros(6))
    assert not evaluate_workspace(base,base,far,old)['allowed']
    assert evaluate_workspace(base,base,far,model)['allowed']
    assert model.uses_model_range and model.velocity_boundary([0,0,0]) is None
    assert old.velocity_boundary([1,2,3])==dict(origin_m=[1,2,3],radius_m=.05)


@pytest.mark.parametrize('mode',['unbounded',None,True,0])
def test_unknown_mode_rejected(mode):
    with pytest.raises(ValueError,match='workspace'):TestWorkspace(mode=mode)


def test_model_mode_cannot_bypass_moveit_or_use_another_tool():
    with pytest.raises(ValueError,match=r'C\+\+ MoveIt'):
        NativeExecutor('unused','shadow',None,workspace_config={'mode':'model'})


def test_assembly_is_byte_verified_and_keeps_actual_mounts_only():
    info=verified_assets()
    assert info['environment_modeled'] is False
    assert not any('Link2' in pair or 'Link3' in pair or 'Link4' in pair or 'Link5' in pair for pair in info['allowed_contacts'])
    assert info['rigid_collision']==('source_hulls_component_boxes_and_source_triangle_decomposition'
        if info.get('body_refinement') else 'source_convex_hulls_and_component_boxes')
    source=Path(__file__).resolve().parents[1]/'config/e6-controller-model'
    original=get_tool('omnipicker-r4').attach(dict(robot_description=(source/'e6.urdf').read_text(),robot_description_semantic=(source/'e6.srdf').read_text()))
    saved=deepcopy(original);model,identity=attach(original,'omnipicker-r4')
    assert original==saved and identity==info['model_id']
    tree=ET.fromstring(model['robot_description'])
    display=ET.parse(ASSETS/'robot.urdf').getroot()
    for link in tree.findall('link'):
        reference=display.find(f"link[@name='{link.get('name')}']")
        for a,b in zip(link.findall('collision/geometry/mesh'),reference.findall('collision/geometry/mesh')):
            assert Path(a.get('filename')).name==Path(b.get('filename')).name
        assert len(link.findall('collision'))==len(reference.findall('collision'))
        for a,b in zip(link.findall('collision'),reference.findall('collision')):
            if a.find('geometry/box') is not None:
                assert ET.tostring(a)==ET.tostring(b)
    assert len(tree.findall("joint[@name='grasp_tcp_fixed']"))==1
    if info.get('dynamic_gripper'):
        assert len(tree.findall("link[@name='omnipicker']/collision/geometry/box"))==3
        moving=info['dynamic_gripper']['movable_links']
        assert len(moving)==len(set(moving))==8
        for name in moving:
            assert len(tree.findall(f"link[@name='{name}']/collision/geometry/box"))==1
            joint=next(j for j in tree.findall('joint') if j.find('child').get('link')==name)
            assert joint.get('type') in ('revolute','continuous')
        fallback=info['dynamic_gripper']['fallback_link']
        assert tree.findall(f"link[@name='{fallback}']/collision/geometry/box")
    else:
        assert len(tree.findall("link[@name='omnipicker']/collision/geometry/box"))==11
    assert len(tree.findall("link[@name='r4_plate']/collision/geometry/box"))==2
    assert len(tree.findall("link[@name='gemini_e']/collision/geometry/box"))==1
    assert not tree.findall("link[@name='omnipicker']/collision/geometry/mesh")
    assert info['gripper_collision']['supplier_dimensions_mm']==[60.5,125,153]
    with pytest.raises(ValueError,match='already'):attach(model,'omnipicker-r4')


def test_sweep_is_independent_of_display_aperture_and_preserves_nominal_limits():
    info=verified_assets()['gripper_collision']
    assert info['method']=='per_cad_link_nested_oriented_boxes'
    assert info['driver_range_deg']==[-50,5]
    assert info['reserve_m']==.005
    assert info['qualification']=='nominal_free_open_close_not_measured_contact_deformation'
    assert info['passive_fingertip_deg']==0
    assert len(info['pieces'])==8
    for side in ('narrow','wide'):
        rows=[r for r in info['pieces'] if r['side']==side]
        assert {r['section'] for r in rows}=={'root','tip'}
        for row in rows:
            np.testing.assert_allclose(np.array(row['size_m'])-row['source_box_size_m'],.01,atol=1e-10)
            r=np.array(row['box_to_link'])[:3,:3]
            np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-10)
            assert np.linalg.det(r)>0.999999
            # The new box cannot expand any corner beyond its former parent.
            import itertools
            vertices=np.array(list(itertools.product((-1,1),repeat=3)))*np.array(row['size_m'])/2
            pose=np.array(row['box_to_link']);parent=np.array(row['parent_box_to_link'])
            local=(vertices@pose[:3,:3].T+pose[:3,3]-parent[:3,3])@parent[:3,:3]
            assert np.max(np.abs(local)-np.array(row['parent_box_size_m'])/2)<1e-9
    # The previously fictitious wall at y=77.5, z=45.3 mm must not survive on
    # proximal fingers or the body just because distal fingers open wider.
    body=verified_assets()['box_collision_proofs']['gripper_body']
    assert len(body)==3 and body[0]['slab_m']==[None,.0285]
    assert np.diff(np.array(body[0]['bounds_m']),axis=0)[0,1]<.060
    assert max(np.abs(np.array(p['bounds_m'])[:,1]).max() for p in body)<.041


def test_display_keeps_source_cad_and_collision_keeps_verified_outer_geometry():
    import struct
    def faces(name):return struct.unpack('<I',(ASSETS/name).read_bytes()[80:84])[0]
    assert faces('e6-Link2-cad.stl')==48690
    assert faces('e6-Link2.stl')==faces('e6-Link2-cad.stl')
    assert faces('gripper-visual.stl')==sum(faces('gripper-'+name+'.stl') for name in (
        'base_link','hand_narrow1_Link','hand_narrow2_Link','hand_narrow3_Link','hand_narrow_loop_Link',
        'hand_wide1_Link','hand_wide2_Link','hand_wide3_Link','hand_wide_loop_Link'))
    assert (ASSETS/'r4-plate.stl').read_bytes()==(ASSETS/'r4-plate-collision.stl').read_bytes()
    proofs=verified_assets()['rigid_collision_proofs']
    assert proofs['e6-Link2']['original_faces']==48690
    for proof in proofs.values():
        assert proof['max_outside_m']<-.00009
        assert proof['hull_faces']<proof['original_faces']
    # Parent hull proofs are retained for provenance. When refined, the active
    # URDF must reference ONLY the explicitly covered source-triangle parts.
    tree=ET.parse(ASSETS/'robot.urdf')
    for name,proof in verified_assets().get('body_refinement',{}).items():
        assert proof['method']=='CoACD_then_original_whole_triangle_exact_hull_cube_offset'
        assert proof['source_triangles']==faces(proof['source_file'])
        assert sum(p['source_triangles'] for p in proof['parts'])==proof['source_triangles']
        assert all(p['max_vertex_excess_m']<-.00009 for p in proof['parts'])
        assert [p.get('filename') for p in tree.findall(f"link[@name='{name}']/collision/geometry/mesh")]==[p['file'] for p in proof['parts']]


def test_readonly_assets_are_allowlisted():
    app=FastAPI();router=APIRouter();install_workspace_api(router);app.include_router(router)
    with TestClient(app) as client:
        assert client.get('/e6/workspace').json()['model_id']==verified_assets()['model_id']
        assert client.get('/e6/workspace/assets/robot.urdf').status_code==200
        assert client.get('/e6/workspace/assets/manifest.json').status_code==404
        assert client.get('/e6/workspace/assets/..%5cAGENTS.md').status_code==404
        assert client.post('/e6/workspace').status_code==405


def test_map_version_mismatch_does_not_claim_reachability(tmp_path,monkeypatch):
    import e6_phosphobot.workspace_api as api
    (tmp_path/'reachability.json').write_text(json.dumps({'model_id':'stale'}))
    monkeypatch.setattr(api,'ASSETS',tmp_path)
    app=FastAPI();router=APIRouter();api.install_workspace_api(router);app.include_router(router)
    with TestClient(app) as client:assert client.get('/e6/workspace/map').status_code==409
