"""Offline CAD generation; no robot, USB, network, or machine control.

Run from the project with .venv-cad/Scripts/python.exe. The STEP uses tap-drill
bores, not helical threads: the drawing's M3x0.5-6H THRU requirement is mandatory.
"""
from pathlib import Path
import csv
import json
import math
import zipfile

import cadquery as cq
import ezdxf
from OCP.BRepAdaptor import BRepAdaptor_Surface

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/cad/E6_OmniPicker_adapter_R2'
OUT.mkdir(parents=True, exist_ok=True)
REF = ROOT / 'research/grippers/omnipicker_mount_2026-09-23/official_cover.step'
P = dict(revision='R2', units='mm', material='6061-T6', diameter=65.0,
         thickness=7.0, edge_chamfer=0.3, m6_pcd=50.0, m6_clearance=6.6,
         countersink_diameter=12.4, countersink_angle=90.0,
         m3_pcd=25.0, m3_tap_drill=2.5, m3_thread='M3x0.5-6H THRU',
         outer_m3_pcd=56.0, outer_m3_clearance=3.4,
         outer_m3_csink_diameter=6.4, outer_m3_csink_angle=90.0,
         outer_m3_first_angle_deg=22.5, outer_m3_spacing_deg=45.0,
         relief_diameter=34.0, relief_depth=1.0, tap_entry_diameter=3.4)
M6 = [(25*math.cos(math.radians(a)),25*math.sin(math.radians(a)))
      for a in (45,135,225,315)]
M3 = [(12.5,0),(0,12.5),(-12.5,0),(0,-12.5)]
OUTER_M3 = [(28*math.cos(math.radians(22.5+45*i)),
             28*math.sin(math.radians(22.5+45*i))) for i in range(8)]
P['m6_xy'] = M6
P['m3_xy'] = M3
P['outer_m3_xy'] = OUTER_M3
P['csink_depth_reference'] = (P['countersink_diameter']-P['m6_clearance'])/2

def cylinder(r, h, xyz):
    return cq.Solid.makeCylinder(r,h,cq.Vector(*xyz))

def cone(r1,r2,h,xyz):
    return cq.Solid.makeCone(r1,r2,h,cq.Vector(*xyz))

def build_plate():
    s=cq.Workplane('XY').circle(32.5).extrude(7).edges('%Circle').chamfer(0.3).val()
    # A-face relief is an allowance, NOT a measured E6 recess or locating fit.
    s=s.cut(cylinder(17,1.1,(0,0,-0.1)))
    for x,y in M6:
        s=s.cut(cylinder(3.3,9,(x,y,-1)))
        s=s.cut(cone(3.3,6.2,2.9,(x,y,4.1)))
        # Small A-face edge break at clearance-hole exits.
        s=s.cut(cone(3.5,3.3,0.2,(x,y,0)))
    for x,y in M3:
        s=s.cut(cylinder(1.25,9,(x,y,-1)))
        s=s.cut(cone(1.25,1.7,0.45,(x,y,6.55)))
        s=s.cut(cone(1.7,1.25,0.45,(x,y,1.0)))
    # Optional E6 PCD56 attachment: clearance holes, NOT a second set of threads.
    for x,y in OUTER_M3:
        s=s.cut(cylinder(1.7,9,(x,y,-1)))
        s=s.cut(cone(1.7,3.2,1.5,(x,y,5.5)))
        s=s.cut(cone(1.9,1.7,0.2,(x,y,0)))
    return s.clean()

plate=build_plate()
assert plate.isValid() and len(plate.Solids())==1
part_path=OUT/'E6_OMNI_ADAPTER_R2.step'
cq.exporters.export(plate,str(part_path))
roundtrip=cq.importers.importStep(str(part_path)).val()
assert roundtrip.isValid() and len(roundtrip.Solids())==1
assert abs(roundtrip.Volume()-plate.Volume()) < 1e-5
b=roundtrip.BoundingBox()
assert all(abs(a-e)<1e-6 for a,e in [(b.xlen,65),(b.ylen,65),(b.zlen,7)])

# Independently extract cylindrical face locations from the exported STEP.
def bore_centres(shape,r):
    pts=set()
    for f in shape.Faces():
        if f.geomType()!='CYLINDER': continue
        cy=BRepAdaptor_Surface(f.wrapped).Cylinder()
        ax=cy.Axis();d=ax.Direction();o=ax.Location()
        if abs(d.Z())>0.999 and abs(cy.Radius()-r)<1e-5:
            pts.add((round(o.X(),4),round(o.Y(),4)))
    return sorted(pts)

assert bore_centres(roundtrip,3.3)==sorted((round(x,4),round(y,4)) for x,y in M6)
assert bore_centres(roundtrip,1.25)==sorted((round(x,4),round(y,4)) for x,y in M3)
assert bore_centres(roundtrip,1.7)==sorted((round(x,4),round(y,4)) for x,y in OUTER_M3)
# Compare all adjacent hole envelopes and a conservative tolerance allowance.
nearest_m6_outer_centre=min(math.hypot(x-a,y-b) for x,y in M6 for a,b in OUTER_M3)
nominal_ligament=nearest_m6_outer_centre-6.2-3.2
worst_ligament=nominal_ligament-2*math.sqrt(2)*0.05-0.1
assert nominal_ligament>1.3 and worst_ligament>1.0
assert 32.5-0.3-28-3.2>0.99

cover=cq.importers.importStep(str(REF)).val()
# Normalise by the actual rear-most planar datum, not an inferred drawing depth.
back_z=min(f.Center().z for f in cover.Faces()
           if f.geomType()=='PLANE' and f.normalAt().z < -0.99)
cover=cover.translate((0,0,7-back_z))
cb_seat=min(f.Center().z for f in cover.Faces()
            if f.geomType()=='PLANE' and f.normalAt().z>0.99
            and 19<f.Area()<21)
clamp=cb_seat-7
overlap=plate.intersect(cover).Volume()
assert overlap<1e-5
actual_holes=bore_centres(cover,1.6)
for x,y in M3:
    assert any(math.hypot(x-a,y-b)<0.001 for a,b in actual_holes)

# Envelope screws only: no helical threads, sockets, underhead fillets or coatings.
m3s=[];m6s=[];outer_m3s=[]
for x,y in M3:
    # Verify the maximum allowed generic replacement head diameter, 5.5 mm.
    s=cylinder(1.5,8,(x,y,cb_seat-8)).fuse(cylinder(2.75,1.3,(x,y,cb_seat)))
    assert s.intersect(cover).Volume()<1e-5
    assert cb_seat-8 > P['relief_depth']
    m3s.append(s)
for x,y in M6:
    s=cylinder(3,9,(x,y,-5.2)).fuse(cone(3,6,3,(x,y,3.8)))
    assert s.intersect(plate).Volume()<1e-5
    assert s.intersect(cover).Volume()<1e-5
    m6s.append(s)
for x,y in OUTER_M3:
    s=cylinder(1.5,10.5,(x,y,-5.2)).fuse(cone(1.5,3.0,1.5,(x,y,5.3)))
    assert s.intersect(plate).Volume()<1e-5
    assert s.intersect(cover).Volume()<1e-5
    assert all(s.intersect(t).Volume()<1e-5 for t in m6s+m3s+outer_m3s)
    outer_m3s.append(s)

assy=cq.Assembly(name='REFERENCE_ASSEMBLY_NOT_FOR_MACHINING')
assy.add(plate,name='MACHINE_THIS_ADAPTER_ONLY',color=cq.Color(0.2,0.55,0.75))
assy.add(cover,name='OFFICIAL_COVER_REFERENCE',color=cq.Color(0.4,0.42,0.46))
for i,s in enumerate(m3s):assy.add(s,name=f'M3x8_ENVELOPE_{i+1}',color=cq.Color(0.8,0.66,0.25))
for i,s in enumerate(m6s):assy.add(s,name=f'M6x12_ENVELOPE_{i+1}',color=cq.Color(0.65,0.68,0.7))
for i,s in enumerate(outer_m3s):
    assy.add(s,name=f'OPTIONAL_OUTER_M3x12_ENVELOPE_{i+1}',color=cq.Color(0.88,0.42,0.18))
assy.save(str(OUT/'REFERENCE_assembly_R2.step'))

P['computed_mass_g']=plate.Volume()*0.0027
P['checks']={
    'step_valid_single_solid':True,'roundtrip_volume_mm3':roundtrip.Volume(),
    'step_size_mm':[b.xlen,b.ylen,b.zlen],
    'official_cover_interference_mm3':overlap,
    'm6_bore_centres':bore_centres(roundtrip,3.3),
    'm3_tap_drill_centres':bore_centres(roundtrip,1.25),
    'outer_m3_clearance_centres':bore_centres(roundtrip,1.7),
    'outer_m3_hole_count':len(OUTER_M3),
    'total_attachment_hole_count':len(M6)+len(M3)+len(OUTER_M3),
    'm6_to_outer_m3_centre_distance_min_mm':nearest_m6_outer_centre,
    'm6_to_outer_m3_csink_ligament_nominal_mm':nominal_ligament,
    'm6_to_outer_m3_csink_ligament_conservative_with_tolerances_mm':worst_ligament,
    'outer_m3_edge_ligament_at_B_nominal_mm':32.5-0.3-28-3.2,
    'cover_step_clamp_thickness_mm':clamp,
    'cover_drawing_clamp_thickness_mm':2.5,
    'm3x8_nominal_insertion_mm':[8-clamp,8-2.5],
    'm3x8_tip_clearance_to_relief_floor_mm':[cb_seat-8-1,7+2.5-8-1],
    'm3_head_envelope':'generic diameter 5.5 max, height 1.3 max; counterbore diameter 6, depth 1.5',
    'outer_m3_screw_envelope':'DIN7991 M3x12, head diameter6, simplified 90deg cone, nominal insertion5.2; optional',
    'outer_m3_fastener_interferences_mm3':0.0,
    'm6_nominal_insertion_reference_mm':5.2,
    'csink_edge_ligament_before_chamfer_mm':32.5-25-6.2,
    'minimum_edge_ligament_at_B_face_mm':32.5-0.3-25-6.2,
    'm3_nominal_threaded_material_thickness_mm':6.0,
    'm3_straight_tap_drill_length_after_entries_mm':5.1,
    'limits':'Cover and simplified fastener envelopes only; no E6 solid, PCB, full gripper, plug/cable swept volume, strength or physical fit validation.',
}
(OUT/'design_parameters_and_checks.json').write_text(json.dumps(P,ensure_ascii=False,indent=2),encoding='utf-8')

# 1:1 editable DXF: geometrical front/back views, clear layer names and hole table.
doc=ezdxf.new('R2010');doc.units=4
for name,col,lt in [('OUTLINE',7,'CONTINUOUS'),('HOLES',7,'CONTINUOUS'),('THREAD_MAJOR',3,'CONTINUOUS'),('HIDDEN',8,'DASHED'),('CENTRE',4,'CENTER'),('NOTES',7,'CONTINUOUS')]:
    if lt not in doc.linetypes:
        doc.linetypes.new(lt,dxfattribs={'pattern':[2,1,-0.5,0.2,-0.3] if lt=='CENTER' else [1,0.5,-0.5]})
    doc.layers.new(name,dxfattribs={'color':col,'linetype':lt})
m=doc.modelspace()
def line(a,b,layer='OUTLINE'):m.add_line(a,b,dxfattribs={'layer':layer})
def circ(c,r,layer='OUTLINE'):m.add_circle(c,r,dxfattribs={'layer':layer})
def txt(s,x,y,h=2.2):m.add_text(s,dxfattribs={'height':h,'layer':'NOTES','insert':(x,y)})
for ox,label in [(0,'B FACE / GRIPPER SIDE / +Z VIEW'),(100,'A FACE / ROBOT SIDE / -Z VIEW')]:
    circ((ox,0),32.5);circ((ox,0),32.2)
    for rad in (25,12.5,28):circ((ox,0),rad,'CENTRE')
    for x,y in M6:
        circ((x+ox,y),3.3,'HOLES')
        circ((x+ox,y),6.2,'HOLES' if ox==0 else 'HIDDEN')
    for x,y in M3:
        circ((x+ox,y),1.25,'HOLES');circ((x+ox,y),1.5,'THREAD_MAJOR');circ((x+ox,y),1.7,'HOLES')
    for x,y in OUTER_M3:
        circ((x+ox,y),1.7,'HOLES')
        circ((x+ox,y),3.2,'HOLES' if ox==0 else 'HIDDEN')
    circ((ox,0),17,'HIDDEN' if ox==0 else 'OUTLINE')
    line((ox-36,0),(ox+36,0),'CENTRE');line((ox,-36),(ox,36),'CENTRE')
    txt(label,ox-32.5,43,2.1)
    txt('OD 65 +/-0.10; T 7 +/-0.05',ox-32.5,-41)
txt('E6-OMNI-ADP-01  REV R2   6061-T6   UNITS mm   FIRST ARTICLE / FIT CHECK REQUIRED',-32.5,55,2.4)
notes=[
    'H1-H4: DIA 6.6 +0.15/0 THRU; B FACE CSK DIA 12.4 +/-0.10 x 90 deg +/-0.5 deg.',
    'PCD 50: 45/135/225/315 deg. XY=(+/-17.6777,+/-17.6777). XY tolerance +/-0.05.',
    'H5-H8: M3x0.5-6H THRU TO A RELIEF, PCD25, 0/90/180/270 deg.',
    'XY=(12.5,0),(0,12.5),(-12.5,0),(0,-12.5). XY tolerance +/-0.05.',
    'STEP shows DIA2.5 TAP DRILL, NOT FINISHED SMOOTH HOLES. CUT M3 THREADS.',
    'H9-H16: 8x DIA3.4 +0.10/0 THRU; B CSK DIA6.4 +/-0.10 x90deg +/-0.5deg.',
    'H9-H16 PCD56; first angle22.5deg from +X CCW, spacing45deg. NO THREADS.',
    'Optional E6 DIN7991 M3x12; head included in length. Existing M6 remains default.',
    'A FACE: central DIA34 +/-0.10 pocket, depth1 +/-0.10. No locating boss or pin.',
    'OD edges C0.3. M3 entries DIA3.4 x90deg; M6 A exits C0.2. Deburr others <=0.1.',
    'A annulus and B mounting face: Ra1.6, flatness0.05; parallelism0.05 to A.',
    'No anodising on first article. General linear tolerance +/-0.10 mm.',
    'Use PDF drawing for section, assembly and fastener purchasing. DXF views are 1:1.',
]
for i,s in enumerate(notes):txt(s,-32.5,-52-5*i,2.0)
# Meridional section through M3 holes, s=X, height=Z, 1:1.
oy=-137
pts=[(-32.2,0),(-17,0),(-17,1),(17,1),(17,0),(32.2,0),(32.5,0.3),(32.5,6.7),(32.2,7),(-32.2,7),(-32.5,6.7),(-32.5,0.3)]
m.add_lwpolyline([(x,y+oy) for x,y in pts],close=True,dxfattribs={'layer':'OUTLINE'})
for x in (-12.5,12.5):
    for dx in (-1.25,1.25):line((x+dx,oy+1),(x+dx,oy+7),'HOLES')
    for dx in (-1.5,1.5):line((x+dx,oy+1),(x+dx,oy+7),'THREAD_MAJOR')
txt('SECTION THROUGH H5/H7 (THREADS SYMBOLIC); A z=0, B z=7',-32.5,-145,2)
doc.saveas(OUT/'E6_OMNI_ADAPTER_R2.dxf')
check=ezdxf.readfile(OUT/'E6_OMNI_ADAPTER_R2.dxf')
assert not check.audit().has_errors

# A reproducible preview of actual BREP tessellation, not a generated illustration.
import vtk
def actor(shape,color,translation=(0,0,0)):
    verts,tris=shape.tessellate(0.05,0.1)
    points=vtk.vtkPoints()
    for v in verts:points.InsertNextPoint(v.x+translation[0],v.y+translation[1],v.z+translation[2])
    cells=vtk.vtkCellArray()
    for t in tris:
        cells.InsertNextCell(3)
        for j in t:cells.InsertCellPoint(j)
    poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(cells)
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly)
    a=vtk.vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*color)
    a.GetProperty().SetInterpolationToPhong()
    a.GetProperty().SetSpecular(0.18);a.GetProperty().SetSpecularPower(25)
    return a
def render(path,items,eye):
    ren=vtk.vtkRenderer();ren.SetBackground(0.97,0.98,0.99)
    for s,c,t in items:ren.AddActor(actor(s,c,t))
    win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.AddRenderer(ren);win.SetSize(1300,950)
    cam=ren.GetActiveCamera();cam.SetPosition(*eye);cam.SetFocalPoint(0,0,6);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn()
    ren.ResetCamera();cam.Zoom(1.15);win.Render()
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(win);capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(path));writer.SetInputConnection(capture.GetOutputPort());writer.Write();win.Finalize()
render(OUT/'adapter_top.png',[(plate,(0.25,0.57,0.73),(0,0,0))],(90,-110,130))
render(OUT/'adapter_bottom.png',[(plate,(0.25,0.57,0.73),(0,0,0))],(90,-110,-130))
render(OUT/'assembly_exploded.png',[(plate,(0.25,0.57,0.73),(0,0,0)),(cover,(0.38,0.4,0.44),(0,0,17))]+[(s,(0.82,0.69,0.32),(0,0,25)) for s in m3s]+[(s,(0.72,0.73,0.75),(0,0,3)) for s in m6s],(110,-140,125))
print(json.dumps({'output':str(OUT),'mass_g':P['computed_mass_g'],'checks':P['checks']},ensure_ascii=False,indent=2))
