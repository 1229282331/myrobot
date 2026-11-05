"""Dimension drawing + concise purchasing/assembly sheet, entirely offline."""
from pathlib import Path
import json
import math
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.colors import HexColor, white

R=Path(__file__).resolve().parents[1]
D=R/'output/cad/E6_OmniPicker_adapter_R2'
P=json.loads((D/'design_parameters_and_checks.json').read_text(encoding='utf-8'))
F=R/'output/pdf/E6_OmniPicker_转接盘加工图与采购单_R2.pdf'
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/msyh.ttc',subfontIndex=0))
pdfmetrics.registerFont(TTFont('CB','C:/Windows/Fonts/msyhbd.ttc',subfontIndex=0))
c=canvas.Canvas(str(F),pagesize=(420*mm,297*mm))
c.setTitle('E6 - OmniPicker 转接盘 | R2 加工图及螺钉采购单')
c.setAuthor('E6 项目 | 自定义转接件')
c.setSubject('官方接口尺寸建模；首件试装版，未做实物与承载验证')
INK='#152E42';MUT='#526879';BLUE='#087A98';LIGHT='#EAF2F6';GOLD='#876623'
def T(s,x,y,size=9,col=INK,b=False,align='left'):
    c.setFillColor(HexColor(col));c.setFont('CB' if b else 'CN',size)
    fn={'left':c.drawString,'center':c.drawCentredString,'right':c.drawRightString}[align]
    fn(x*mm,y*mm,s)
def L(a,b,col=INK,w=0.6,dash=None):
    c.setStrokeColor(HexColor(col));c.setLineWidth(w);c.setDash(dash or [])
    c.line(a[0]*mm,a[1]*mm,b[0]*mm,b[1]*mm);c.setDash([])
def C(x,y,r,col=INK,w=0.7,dash=None):
    c.setStrokeColor(HexColor(col));c.setLineWidth(w);c.setDash(dash or [])
    c.circle(x*mm,y*mm,r*mm,stroke=1,fill=0);c.setDash([])
def Rct(x,y,w,h,fill=LIGHT,stroke=None):
    c.setFillColor(HexColor(fill));c.setStrokeColor(HexColor(stroke or fill))
    c.rect(x*mm,y*mm,w*mm,h*mm,stroke=int(stroke is not None),fill=1)
def arrow(tip,direction,col=INK):
    x,y=tip;dx,dy=direction;ll=math.hypot(dx,dy);dx/=ll;dy/=ll
    path=c.beginPath();path.moveTo(x*mm,y*mm)
    path.lineTo((x+dx*2-dy*0.55)*mm,(y+dy*2+dx*0.55)*mm)
    path.lineTo((x+dx*2+dy*0.55)*mm,(y+dy*2-dx*0.55)*mm);path.close()
    c.setFillColor(HexColor(col));c.drawPath(path,stroke=0,fill=1)
def dh(x1,x2,y,orig,label):
    for x in(x1,x2):L((x,orig),(x,y+1),MUT,0.35)
    L((x1,y),(x2,y),MUT,0.4);arrow((x1,y),(1,0),MUT);arrow((x2,y),(-1,0),MUT)
    T(label,(x1+x2)/2,y+2,9,align='center')
def leader(start,end,label,dy=1):
    L(start,end,MUT,0.4);arrow(start,(end[0]-start[0],end[1]-start[1]),MUT)
    T(label,end[0],end[1]+dy,8.5,MUT)
def image(name,x,y,w,h):c.drawImage(str(D/name),x*mm,y*mm,w*mm,h*mm,preserveAspectRatio=True,anchor='c')

# Page 1: dimensioned manufacturing sheet.
T('转接盘加工图',14,282,21,b=True)
T('E6-OMNI-ADP-01  /  R2',405,284,12,BLUE,True,'right')
T('R2 新增 E6 外圈 8 个 M3 备用安装孔 · 首件实物适配尚未验证',14,274,10,MUT)
L((14,269),(406,269),BLUE,1.1)
T('单位 mm   |   主视图 2:1（打印缩放后勿量图）   |   螺纹、公差及表面要求以本页标注为准',14,261,9)

for ox,label,face in [(82,'B 面：朝夹爪，从 +Z 正视','B'),(237,'A 面：朝机械臂，从 -Z 正视','A')]:
    oy=177;sc=2
    T(label,ox,246,11,b=True,align='center')
    C(ox,oy,65);C(ox,oy,64.4,MUT,0.4)
    for rr in(50,25,56):C(ox,oy,rr,BLUE,0.35,[7,2,1,2])
    L((ox-69,oy),(ox+69,oy),BLUE,0.35,[7,2,1,2])
    L((ox,oy-69),(ox,oy+69),BLUE,0.35,[7,2,1,2])
    for i,(x,y) in enumerate(P['m6_xy'],1):
        if face=='A':x=-x
        cx,cy=ox+x*sc,oy+y*sc
        C(cx,cy,6.6);C(cx,cy,12.4,INK if face=='B' else MUT,0.6,None if face=='B' else [2,2])
        if face=='B':T(f'H{i}',cx+6,cy-10,8.2,MUT)
    for i,(x,y) in enumerate(P['m3_xy'],5):
        if face=='A':x=-x
        cx,cy=ox+x*sc,oy+y*sc
        C(cx,cy,2.5,w=0.65);C(cx,cy,3,MUT,0.35);C(cx,cy,3.4,MUT,0.35)
        if face=='B':T(f'H{i}',cx+4,cy+4,8.2,MUT)
    for i,(x,y) in enumerate(P['outer_m3_xy'],9):
        if face=='A':x=-x
        cx,cy=ox+x*sc,oy+y*sc
        C(cx,cy,3.4,BLUE,0.7)
        C(cx,cy,6.4,BLUE,0.7,None if face=='B' else [2,2])
        if face=='B':
            T(f'H{i}',ox+x/28*70,oy+y/28*70-1,7.5,BLUE,align='center')
    C(ox,oy,34,MUT,0.5,[2,2] if face=='B' else None)
    if face=='B':
        T('+X',ox+70,oy+2,8,BLUE);T('+Y',ox+2,oy+57,8,BLUE)
        T('O',ox+2,oy+2,8,BLUE)
        dh(ox-65,ox+65,102,177,'Φ65 ±0.10')
    else:
        Rct(ox-14,oy+12,28,4.5,fill='#FFFFFF')
        Rct(ox-14,oy-14,28,4.5,fill='#FFFFFF')
        T('Φ34 ±0.10',ox,oy+13,9,MUT,align='center')
        T('深 1 ±0.10',ox,oy-13,9,MUT,align='center')
        T('中央浅槽仅用于避让；不作止口定位',ox,103,8.5,MUT,align='center')

# Right column: unambiguous hole table and notes.
T('孔位表（按 B 面坐标）',321,244,10,b=True)
cols=[321,342,365,389,406]
Rct(321,232,85,8)
for s,x in [('孔号',324),('X',345),('Y',369),('类型',392)]:T(s,x,235,8.4,b=True)
rows=[('H1','17.6777','17.6777','M6过孔'),('H2','-17.6777','17.6777','M6过孔'),('H3','-17.6777','-17.6777','M6过孔'),('H4','17.6777','-17.6777','M6过孔'),('H5','12.5','0','M3牙'),('H6','0','12.5','M3牙'),('H7','-12.5','0','M3牙'),('H8','0','-12.5','M3牙')]
rows.extend((f'H{i}',f'{x:.4f}',f'{y:.4f}','M3过孔') for i,(x,y) in enumerate(P['outer_m3_xy'],9))
for i,row in enumerate(rows):
    y=227-i*5
    for s,x in zip(row,[324,343,367,390]):T(s,x,y,7.6,BLUE if i>=8 else INK)
    L((321,y-2),(406,y-2),'#CED9DF',0.3)
T('孔心 X/Y 公差 ±0.05；角度自 +X 逆时针。',321,144,8.1)
T('H1-H4：PCD Φ50，45°起，每90°。',321,134,8.3,b=True)
T('4×Φ6.6 +0.15/0 通孔；配 DIN7991 M6×12。',321,128,8.2)
T('B 面沉头 Φ12.4 ±0.10 × 90° ±0.5°。',321,122,8.3)
T('H5-H8：PCD Φ25，0°起，每90°。',321,112,8.3,b=True)
T('4×M3×0.5-6H 攻通；底孔Φ2.5。',321,106,8.3)
T('两端入口Φ3.4×90°；STEP底孔必须攻牙。',321,100,8.1)
T('H9-H16：PCD Φ56，22.5°起，每45°。',321,90,8.2,BLUE,True)
T('8×Φ3.4 +0.10/0 通孔，不攻牙。',321,84,8.3,BLUE)
T('B 面沉头 Φ6.4 ±0.10 × 90° ±0.5°。',321,78,8.3,BLUE)
T('A 口 C0.2；备用 DIN7991 M3×12 沉头。',321,72,8.1,BLUE)
T('两种沉头均朝 B 面；长度均含头。',321,65,8.1)

# Section through the two axial M3 bores at Y=0.
T('C-C 剖面（Y=0，穿 H5/H7；螺纹简化）',16,87,9.5,b=True)
sx,sy,sc=82,63,2
outline=[(-32.2,0),(-17,0),(-17,1),(17,1),(17,0),(32.2,0),(32.5,0.3),(32.5,6.7),(32.2,7),(-32.2,7),(-32.5,6.7),(-32.5,0.3)]
p=c.beginPath()
for i,(x,z) in enumerate(outline):
    (p.moveTo if i==0 else p.lineTo)((sx+x*sc)*mm,(sy+z*sc)*mm)
p.close();c.setFillColor(HexColor('#E5ECF0'));c.setStrokeColor(HexColor(INK));c.setLineWidth(0.8);c.drawPath(p,fill=1,stroke=1)
for x in(-12.5,12.5):
    pts=[(x-1.7,1),(x-1.25,1.45),(x-1.25,6.55),(x-1.7,7),(x+1.7,7),(x+1.25,6.55),(x+1.25,1.45),(x+1.7,1)]
    q=c.beginPath()
    for i,(xx,z) in enumerate(pts):(q.moveTo if i==0 else q.lineTo)((sx+xx*sc)*mm,(sy+z*sc)*mm)
    q.close();c.setFillColor(white);c.drawPath(q,fill=1,stroke=1)
    for xx in(x-1.5,x+1.5):L((sx+xx*sc,sy+2.7),(sx+xx*sc,sy+11.3),MUT,0.3)
T('B',151,76,9,b=True);T('A',151,63,9,b=True)
L((157,63),(157,77),MUT,0.4);arrow((157,63),(0,1),MUT);arrow((157,77),(0,-1),MUT)
T('7 ±0.05',160,68,9)
leader((104,65),(115,54),'A 槽深 1 ±0.10')
T('A 槽处余厚 6；M3 螺纹穿至浅槽。',17,47,8.5,MUT)

# Countersink detail at enlarged 4:1 scale.
T('沉头剖面：M6 示意（4:1）',206,87,9.5,b=True)
sx2,sy2,s2=242,56,4
for side in(-1,1):
    pts=[(side*8,0),(side*3.5,0),(side*3.3,.2),(side*3.3,4.1),(side*6.2,7),(side*8,7)]
    q=c.beginPath()
    for i,(x,z) in enumerate(pts):(q.moveTo if i==0 else q.lineTo)((sx2+x*s2)*mm,(sy2+z*s2)*mm)
    q.close();c.setFillColor(HexColor('#E5ECF0'));c.drawPath(q,fill=1,stroke=1)
dh(sx2-24.8,sx2+24.8,91,84,'Φ12.4 ±0.10')
T('90°',sx2,76,9,align='center');T('Φ6.6 通孔',sx2,51,8.5,align='center')
T('M6：锥深 2.9参考；外圈M3：Φ6.4/Φ3.4，锥深1.5参考。',190,45,7.7,MUT)

notes=['6061-T6；1件；约54.4g；本色不氧化。',
       '外圆C0.3；余锐边≤0.1；A/B面Ra1.6。',
       'A/B平面度0.05；B对A平行度0.05。',
       '未注线性±0.10；不做定位销孔或止口。']
for i,s in enumerate(notes):T(s,321,58-i*6,8.2)
Rct(14,12,392,23,fill='#F5F7F8',stroke='#B5C6D0')
T('图号 E6-OMNI-ADP-01  |  R2  |  2026-09-23',18,26,10,b=True)
T('仅加工 E6_OMNI_ADAPTER_R2.step；REFERENCE_assembly 为装配参考，不加工。',18,18,8.5)
T('首件下料前核对第 2 页实物条件',402,26,9,BLUE,True,'right');T('1 / 2',402,18,8.5,align='right')
c.showPage()

# Page 2: concise purchasing and fit conditions.
c.setPageSize((210*mm,297*mm))
T('螺钉采购与装配',14,281,20,b=True)
T('配套 E6-OMNI-ADP-01 / R2；新增外圈孔可暂不使用',14,272,10,MUT)
L((14,267),(196,267),BLUE,1)
T('默认用 M6 固定到 E6；外圈 M3 为备用安装位',14,256,11,BLUE,True)

Rct(14,209,182,39,fill=LIGHT)
T('① 转接盘到 E6：M6 默认方案，4 颗（建议买10颗）',18,240,10.2,b=True)
T('DIN7991 M6×12，90°沉头内六角，10.9级。',18,233,9.6)
T('头径最大12；12为含头总长；4mm内六角；不加垫圈。',18,226,9)
T('勿混发 ISO10642 大头件；装盘后穿出4.8-5.5。',18,219,9)
T('实际孔可用深度须比穿出量至少多0.5，防止顶底。',18,212,9)

Rct(14,163,182,40,fill=LIGHT)
T('② 后盖到转接盘：超薄圆头 M3×8，4 颗（买10颗）',18,195,10.1,b=True)
T('M3×0.5；头下杆长8；头径5-5.5；头高≤1.3。',18,188,9.5)
T('头底面平，非沉头；驱动槽型不限；不加垫圈。',18,181,9.2)
T('普通品牌可用，核对尺寸；强度等级参考A2-50或以上。',18,174,9)
T('NBK SSHS-M3-8-SD仅为尺寸参考，不限定采购品牌。',18,167,9)

Rct(14,130,182,27,fill=LIGHT)
T('③ 后盖装回本体：M2.5×8 沉头，4 颗（备10颗）',18,149,10,b=True)
T('按官方装配示例备购；核对原螺钉头径、头型及总长。',18,142,9)
T('只用侧面原孔；小螺钉不套用通用大扭矩。',18,135,9)

Rct(14,83,182,41,fill='#E8F4F5')
T('④ 新增外圈备用孔：可暂不采购螺钉',18,116,10.3,BLUE,True)
T('需要使用时：DIN7991 M3×12，90°沉头内六角。',18,109,9.3)
T('8颗一套（建议买10颗）；10.9级；头径最大6；2mm扳手。',18,102,8.8)
T('12为含头总长；装盘后穿出4.8-5.5，距孔底至少0.5。',18,95,9)
T('外圈用沉头；夹爪后盖②用超薄圆头，勿混用。',18,88,9,BLUE)

T('安装顺序',14,73,11,b=True)
T('断电并支撑 → 拆后盖 → 固定盘到E6 → 内侧M3装后盖',14,66,9.2)
T('→ 装回本体、恢复侧螺钉；备用螺钉也在装后盖前安装。',14,59,9.2)

T('首件下料前核对实物；首装验收螺钉深度',14,47,10.3,b=True)
T('1. 后盖PCD25、Φ6×1.5沉孔；夹持厚2.5-2.9，贴合无遮挡。',14,40,8.7)
T('2. E6中央凸出件在Φ34内且高≤0.8；各安装孔深度足够。',14,33,8.7)
T('3. 外圈8个M3是PCD56、22.5°错位；不拆E6中央结构螺钉。',14,26,8.7)
T('已核对官方后盖模型；整机、PCB、插头净空及承载未实测。',14,17,8.5,MUT)
T('2026-09-23  |  R2替代R1用于本次加工；新增孔不构成承载升级。',14,9,7.5,MUT)
T('2 / 2',196,9,8,align='right')
c.save()
print(F)
