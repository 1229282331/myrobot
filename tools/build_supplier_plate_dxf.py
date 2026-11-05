"""Export the existing R2 plate as a neutral, editable supplier drawing."""
from pathlib import Path
import json
import math
import re

import ezdxf
from ezdxf.enums import TextEntityAlignment
from ezdxf.addons.drawing import matplotlib

ROOT = Path(__file__).resolve().parents[1]
PARAMS = ROOT / 'output/cad/E6_OmniPicker_adapter_R2/design_parameters_and_checks.json'
OUT = ROOT / 'output/manufacturer/PD-01_R2/PD-01_R2.dxf'
PREVIEW = ROOT / 'tmp/cad_mount/PD-01_R2_dxf.png'
P = json.loads(PARAMS.read_text(encoding='utf-8'))
assert P['revision'] == 'R2' and P['units'] == 'mm'

doc = ezdxf.new('R2010', setup=True)
doc.units = 4
doc.header['$MEASUREMENT'] = 1
doc.header['$LUNITS'] = 2
doc.header['$LUPREC'] = 4
doc.header['$INSBASE'] = (0, 0, 0)
doc.header['$LTSCALE'] = 1.0
doc.linetypes.get('CENTER').setup_pattern([6.5, 4.0, -1.0, 0.5, -1.0])
doc.linetypes.get('DASHED').setup_pattern([2.5, 1.5, -1.0])
doc.styles.get('Standard').dxf.font = 'arial.ttf'
for name, weight, lt in [
    ('OUTLINE', 30, 'CONTINUOUS'), ('HOLES', 25, 'CONTINUOUS'),
    ('THREAD', 13, 'CONTINUOUS'), ('HIDDEN', 13, 'DASHED'),
    ('CENTRE', 13, 'CENTER'), ('DIMENSIONS', 13, 'CONTINUOUS'),
    ('NOTES', 18, 'CONTINUOUS'), ('HATCH', 9, 'CONTINUOUS'),
    ('FRAME', 18, 'CONTINUOUS'),
]:
    doc.layers.new(name, dxfattribs={'color': 7, 'lineweight': weight, 'linetype': lt})
m = doc.modelspace()


def line(a, b, layer='OUTLINE'):
    return m.add_line(a, b, dxfattribs={'layer': layer,
                      'linetype': doc.layers.get(layer).dxf.linetype})


def circle(c, r, layer='OUTLINE'):
    return m.add_circle(c, r, dxfattribs={'layer': layer,
                        'linetype': doc.layers.get(layer).dxf.linetype})


def text(s, x, y, height=2, centre=False):
    e = m.add_text(s, dxfattribs={'height': height, 'layer': 'NOTES', 'style': 'Standard'})
    e.set_placement((x, y), align=TextEntityAlignment.MIDDLE_CENTER if centre else TextEntityAlignment.LEFT)
    return e


DIM = dict(dimtxt=1.9, dimasz=1.3, dimgap=0.65, dimexo=0.6,
           dimexe=1, dimdec=2, dimzin=8, dimtad=1, dimclrd=0, dimclre=0, dimclrt=0,
           dimlfac=1, dimscale=1, dimblk='', dimtxsty='Standard')


def dimension(base, p1, p2, label, angle=0):
    d = m.add_linear_dim(base=base, p1=p1, p2=p2, angle=angle, text=label,
                         dimstyle='EZDXF', override=DIM,
                         dxfattribs={'layer': 'DIMENSIONS'})
    d.render()


def polygon(points, hatch=True):
    m.add_lwpolyline(points, close=True, dxfattribs={'layer': 'OUTLINE'})
    if hatch:
        h = m.add_hatch(color=7, dxfattribs={'layer': 'HATCH'})
        h.set_pattern_fill('ANSI31', scale=0.4)
        h.paths.add_polyline_path(points, is_closed=True)


# All part geometry in model space is at its real size, in millimetres.
m.add_lwpolyline([(-45, -150), (240, -150), (240, 65), (-45, 65)],
                 close=True, dxfattribs={'layer': 'FRAME'})
text('PD-01 / R2', -39, 55, 4)
text('6061-T6   |   QTY 1   |   UNITS mm   |   VIEWS 1:1', 34, 56, 2.5)
line((-45, 49), (240, 49), 'FRAME')
text('B FACE / VIEW FROM +Z', 0, 44, 2.3, True)
text('A FACE / VIEW FROM -Z', 91, 44, 2.3, True)

groups = [(1, P['m6_xy'], P['m6_clearance']/2, P['countersink_diameter']/2),
          (5, P['m3_xy'], P['m3_tap_drill']/2, P['tap_entry_diameter']/2),
          (9, P['outer_m3_xy'], P['outer_m3_clearance']/2, P['outer_m3_csink_diameter']/2)]
for ox, face in [(0, 'B'), (91, 'A')]:
    sign = 1 if face == 'B' else -1  # Back view is mirrored about the vertical axis.
    circle((ox, 0), P['diameter']/2)
    circle((ox, 0), P['diameter']/2-P['edge_chamfer'])
    for radius in (P['m6_pcd']/2, P['m3_pcd']/2, P['outer_m3_pcd']/2):
        circle((ox, 0), radius, 'CENTRE')
    circle((ox, 0), P['relief_diameter']/2, 'HIDDEN' if face == 'B' else 'OUTLINE')
    line((ox-35, 0), (ox+35, 0), 'CENTRE')
    line((ox, -35), (ox, 35), 'CENTRE')
    for start, coords, bore, mouth in groups:
        for i, (x, y) in enumerate(coords, start):
            c = (ox+sign*x, y)
            circle(c, bore, 'HOLES')
            if start == 5:
                m.add_arc(c, 1.5, 35, 305, dxfattribs={'layer': 'THREAD'})
                circle(c, mouth, 'HOLES')
            else:
                circle(c, mouth, 'HOLES' if face == 'B' else 'HIDDEN')
                if face == 'A':
                    circle(c, bore+0.2, 'HOLES')
            if face != 'B':
                continue
            if start == 5:
                positions = [(8, 3), (4, 12.5), (-8, 3), (4, -12.5)]
                tx, ty = positions[i-5]
                text(f'H{i}', tx, ty, 1.7, True)
            else:
                radius = math.hypot(x, y)
                rr = 40 if start == 1 else 38.5
                tip = (x*(radius+mouth)/radius, y*(radius+mouth)/radius)
                end = (x*(rr-1.5)/radius, y*(rr-1.5)/radius)
                line(tip, end, 'DIMENSIONS')
                text(f'H{i}', x*rr/radius, y*rr/radius, 1.7, True)
text('+X', 34.5, 1.2, 1.7)
text('+Y', 1.2, 35, 1.7)
text('O', 1.5, 1.5, 1.7)
dimension((0, -44), (-32.5, 0), (32.5, 0), '%%c<> %%p0.10')
text('A: CENTRAL POCKET %%c34 %%p0.10', 91, -40, 2, True)
text('DEPTH 1 %%p0.10 FROM A', 91, -45, 2, True)

# Coordinate table, referenced only to the B view.
text('HOLE CENTRES / B VIEW', 146, 42, 2.3)
text('X/Y: %%p0.05  |  ANGLES: +X, CCW', 146, 37, 1.9)
for s, x in [('ID', 148), ('X', 167), ('Y', 196), ('TYPE', 224)]:
    text(s, x, 30.8, 1.9)
line((145, 29), (235, 29), 'FRAME')
all_rows = []
for start, coords, _, _ in groups:
    kind = 'A' if start == 1 else 'B' if start == 5 else 'C'
    for i, (x, y) in enumerate(coords, start):
        all_rows.append((i, x, y, kind))
for row, (i, x, y, kind) in enumerate(all_rows):
    yy = 24.5-row*4.5
    for val, xx in [(f'H{i}', 148), (f'{x:+.4f}', 164), (f'{y:+.4f}', 192), (kind, 226)]:
        text(val, xx, yy, 1.9)
    line((145, yy-1.5), (235, yy-1.5), 'FRAME')
text('A = %%c6.6 THRU + CSK', 146, -54, 1.9)
text('B = M3 THREAD; C = %%c3.4 THRU + CSK', 146, -59, 1.9)

# Exact meridional section through the two axial tapped holes.
text('SECTION Y=0 / THROUGH H5, H7', -37, -59, 2.2)
sy = -82
left = [(-32.2, 7), (-14.2, 7), (-13.75, 6.55), (-13.75, 1.45),
        (-14.2, 1), (-17, 1), (-17, 0), (-32.2, 0), (-32.5, .3), (-32.5, 6.7)]
mid = [(-10.8, 7), (10.8, 7), (11.25, 6.55), (11.25, 1.45),
       (10.8, 1), (-10.8, 1), (-11.25, 1.45), (-11.25, 6.55)]
for profile in [left, mid, [(-x, z) for x, z in reversed(left)]]:
    polygon([(x, sy+z) for x, z in profile])
for x in (-12.5, 12.5):
    for dx in (-1.5, 1.5):
        line((x+dx, sy+1.45), (x+dx, sy+6.55), 'THREAD')
    line((x, sy-2), (x, sy+10), 'CENTRE')
text('B', -38, sy+6, 2)
text('A', -38, sy-1, 2)
dimension((41, sy), (32.2, sy), (32.2, sy+7), '<> %%p0.05', 90)
dimension((0, sy-7), (-17, sy+1), (17, sy+1), '%%c<> %%p0.10')
text('THREADS SYMBOLIC; LOCAL THICKNESS 6', -37, -96, 1.9)

# A real-size countersink section. Opposite side is mirrored, no line crosses voids.
text('H1-H4 / COUNTERSINK SECTION', 62, -59, 2.2)
cx = 89
for sign in (-1, 1):
    profile = [(sign*8, 0), (sign*3.5, 0), (sign*3.3, .2),
               (sign*3.3, 4.1), (sign*6.2, 7), (sign*8, 7)]
    polygon([(cx+x, sy+z) for x, z in profile])
line((cx, sy-3), (cx, sy+10), 'CENTRE')
dimension((cx, -68), (cx-6.2, sy+7), (cx+6.2, sy+7), '%%c<> %%p0.10')
text('B', 100, sy+6, 2)
text('A', 100, sy-1, 2)
text('90%%d %%p0.5%%d; CONE DEPTH 2.9 REF', 62, -89, 1.9)
text('H9-H16: SAME FORM, CONE DEPTH 1.5 REF', 62, -96, 1.9)

line((-45, -101), (240, -101), 'FRAME')
left_notes = [
    'H1-H4: 4x %%c6.6 +0.15/0 THRU; PCD50, 45%%d + 90%%d STEPS.',
    'B COUNTERSINK %%c12.4 %%p0.10 x 90%%d %%p0.5%%d; A EXIT C0.2.',
    'H5-H8: 4x M3x0.5-6H THRU TO A POCKET; PCD25, 0%%d + 90%%d STEPS.',
    'BOTH ENTRIES %%c3.4 x 90%%d. %%c2.5 IS TAP DRILL: CUT M3 THREADS.',
    'H9-H16: 8x %%c3.4 +0.10/0 THRU, NO THREADS; PCD56, 22.5%%d + 45%%d STEPS.',
    'B COUNTERSINK %%c6.4 %%p0.10 x 90%%d %%p0.5%%d; A EXIT C0.2.',
]
right_notes = [
    'MATERIAL 6061-T6; QTY 1; NO ANODISING.',
    'OUTER RIM C0.3 BOTH SIDES; OTHER SHARP EDGES <=0.1.',
    'A ANNULUS AND B PLANE: Ra1.6, FLATNESS 0.05.',
    'B PARALLELISM 0.05 TO A ANNULUS.',
    'GENERAL LINEAR TOLERANCE %%p0.10.',
    'NO LOCATING PIN HOLES OR CENTRING BOSS.',
]
for i, s in enumerate(left_notes):
    text(s, -39, -108-i*5.5, 1.85)
for i, s in enumerate(right_notes):
    text(s, 119, -108-i*5.5, 1.85)
line((-45, -141), (240, -141), 'FRAME')
text('PD-01_R2  |  2D MANUFACTURING DRAWING  |  2026-09-24', -39, -147, 2)
text('1 / 1', 226, -147, 2)

doc.set_modelspace_vport(height=230, center=(97.5, -42.5))
OUT.parent.mkdir(parents=True, exist_ok=True)
PREVIEW.parent.mkdir(parents=True, exist_ok=True)
doc.saveas(OUT)

# Verify the actual saved file, including editable geometry and dimensions.
check = ezdxf.readfile(OUT)
audit = check.audit()
assert not audit.has_errors and not audit.has_fixes
assert check.units == 4
cm = check.modelspace()
assert len(list(cm.query('DIMENSION'))) == 4
for d, expected in zip(cm.query('DIMENSION'), (65, 7, 34, 12.4)):
    assert abs(d.get_measurement()-expected) < 1e-9
    assert d.override().get('dimlfac') == 1
    rendered = [e.text for e in check.blocks[d.dxf.geometry].query('MTEXT')]
    assert len(rendered) == 1 and f'{expected:g}' in rendered[0], rendered
front_holes = [e for e in cm.query('CIRCLE[layer=="HOLES"]') if e.dxf.center.x < 45]
for start, coords, bore, _ in groups:
    matched = [e for e in front_holes if abs(e.dxf.radius-bore) < 1e-9
               and any(math.hypot(e.dxf.center.x-x, e.dxf.center.y-y) < 1e-9 for x, y in coords)]
    assert len(matched) == len(coords), (start, len(matched))
assert len(list(cm.query('CIRCLE[layer=="OUTLINE"]'))) >= 4
semantic = []
for entity in check.entitydb.values():
    if not entity.is_alive:
        continue
    for key, val in entity.dxfattribs().items():
        if key not in {'handle', 'owner'} and isinstance(val, str):
            semantic.append(val)
identity = re.compile(r'omnipicker|dobot|智元|越疆|夹爪|机械臂|lixiaoyu|vscode_files|\bE6\b', re.I)
assert not any(identity.search(s) for s in semantic)
matplotlib.qsave(cm, PREVIEW, bg='#FFFFFF', fg='#152E42', dpi=180, size_inches=(16, 12))
print(json.dumps({'file': str(OUT), 'bytes': OUT.stat().st_size,
                  'preview': str(PREVIEW), 'audit': 'clean',
                  'hole_centres_verified': 16, 'dimension_entities': 4,
                  'units': 'mm', 'identity_check': 'clean'}, ensure_ascii=False))
