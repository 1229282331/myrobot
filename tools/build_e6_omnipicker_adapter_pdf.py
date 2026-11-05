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
D=R/'output/cad/E6_OmniPicker_adapter_R1'
P=json.loads((D/'design_parameters_and_checks.json').read_text(encoding='utf-8'))
F=R/'output/pdf/E6_OmniPicker_转接盘加工图与采购单_R1.pdf'
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/msyh.ttc',subfontIndex=0))
pdfmetrics.registerFont(TTFont('CB','C:/Windows/Fonts/msyhbd.ttc',subfontIndex=0))
c=canvas.Canvas(str(F),pagesize=(420*mm,297*mm))
c.setTitle('E6 - OmniPicker 转接盘 | R1 加工图及螺钉采购单')
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
T('E6-OMNI-ADP-01  /  R1',405,284,12,BLUE,True,'right')
T('首件试装版 · 按官方接口资料设计 · 实物适配尚未验证',14,274,10,MUT)
L((14,269),(406,269),BLUE,1.1)
T('单位 mm   |   主视图 2:1（打印缩放后勿量图）   |   螺纹、公差及表面要求以本页标注为准',14,261,9)

for ox,label,face in [(82,'B 面：朝夹爪，从 +Z 正视','B'),(237,'A 面：朝机械臂，从 -Z 正视','A')]:
    oy=177;sc=2
    T(label,ox,246,11,b=True,align='center')
    C(ox,oy,65);C(ox,oy,64.4,MUT,0.4)
    for rr in(50,25):C(ox,oy,rr,BLUE,0.35,[7,2,1,2])
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
    C(ox,oy,34,MUT,0.5,[2,2] if face=='B' else None)
    if face=='B':
        T('+X',ox+70,oy+2,8,BLUE);T('+Y',ox+2,oy+57,8,BLUE)
        T('O',ox+2,oy+2,8,BLUE)
        dh(ox-65,ox+65,102,177,'Φ65 ±0.10')
    else:
        T('Φ34 ±0.10',ox,oy+13,9,MUT,align='center')
        T('深 1 ±0.10',ox,oy-13,9,MUT,align='center')
        T('中央浅槽仅用于避让；不作止口定位',ox,103,8.5,MUT,align='center')

# Right column: unambiguous hole table and notes.
T('孔位表（按 B 面坐标）',321,244,10,b=True)
cols=[321,342,365,389,406]
Rct(321,232,85,8)
for s,x in [('孔号',324),('X',345),('Y',369),('类型',392)]:T(s,x,235,8.4,b=True)
rows=[('H1','17.6777','17.6777','M6过孔'),('H2','-17.6777','17.6777','M6过孔'),('H3','-17.6777','-17.6777','M6过孔'),('H4','17.6777','-17.6777','M6过孔'),('H5','12.5','0','M3牙'),('H6','0','12.5','M3牙'),('H7','-12.5','0','M3牙'),('H8','0','-12.5','M3牙')]
for i,row in enumerate(rows):
    y=227-i*6
    for s,x in zip(row,[324,343,367,390]):T(s,x,y,7.6)
    L((321,y-2),(406,y-2),'#CED9DF',0.3)
T('孔心 X/Y 公差 ±0.05，原点为盘中心。',321,174,8.1)
T('H1-H4：PCD Φ50，45°起均布。',321,167,8.3,b=True)
T('4×Φ6.6 +0.15/0 通孔。',321,161,8.5)
T('B 面沉头 Φ12.4 ±0.10 × 90° ±0.5°。',321,155,8.3)
T('配 DIN 7991 M6×12，勿换大头 ISO 件。',321,149,8.1)
T('H5-H8：PCD Φ25，0°起均布。',321,138,8.3,b=True)
T('4×M3×0.5-6H，攻通至 A 面浅槽。',321,132,8.3)
T('底孔 Φ2.5；两端入口 Φ3.4 × 90°。',321,126,8.3)
T('STEP 仅示底孔，必须攻 M3 螺纹！',321,119,8.7,BLUE,True)

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
T('单个 M6 孔剖面示意（4:1）',206,87,9.5,b=True)
sx2,sy2,s2=242,56,4
for side in(-1,1):
    pts=[(side*8,0),(side*3.5,0),(side*3.3,.2),(side*3.3,4.1),(side*6.2,7),(side*8,7)]
    q=c.beginPath()
    for i,(x,z) in enumerate(pts):(q.moveTo if i==0 else q.lineTo)((sx2+x*s2)*mm,(sy2+z*s2)*mm)
    q.close();c.setFillColor(HexColor('#E5ECF0'));c.drawPath(q,fill=1,stroke=1)
dh(sx2-24.8,sx2+24.8,91,84,'Φ12.4 ±0.10')
T('90°',sx2,76,9,align='center');T('Φ6.6 通孔',sx2,51,8.5,align='center')
T('锥深 2.9 为参考值；A 口 C0.2',206,45,8.5,MUT)

notes=['材料：6061-T6 铝；数量 1；理论重量约 56 g。',
       '外圆双边 C0.3；其余锐边去毛刺 ≤0.1。',
       'A 环形面及 B 面 Ra1.6、平面度 0.05。',
       'B 对 A 平行度 0.05；未注线性公差 ±0.10。',
       '首件本色，不阳极氧化；安装面不得有毛刺。',
       '不加工定位销孔或中心止口。']
for i,s in enumerate(notes):T(s,321,103-i*6,8.2)
Rct(14,12,392,23,fill='#F5F7F8',stroke='#B5C6D0')
T('图号 E6-OMNI-ADP-01  |  R1  |  2026-09-23',18,26,10,b=True)
T('仅加工 E6_OMNI_ADAPTER_R1.step；REFERENCE_assembly 为装配参考，不加工。',18,18,8.5)
T('首件下料前核对第 2 页实物条件',402,26,9,BLUE,True,'right');T('1 / 2',402,18,8.5,align='right')
c.showPage()

# Page 2: A4 portrait, readable purchasing and installation guide.
c.setPageSize((210*mm,297*mm))
T('螺钉采购与装配',14,281,20,b=True)
T('配套 E6-OMNI-ADP-01 / R1；尺寸单位 mm',14,272,10,MUT)
L((14,267),(196,267),BLUE,1)
T('买这三种，每种 10 颗即可留足备件',14,255,12,BLUE,True)

Rct(14,190,182,58,fill=LIGHT)
T('① 转接盘固定到 E6：4 颗',18,240,10.5,b=True)
T('DIN 7991 · M6×12 · 90°沉头内六角 · 10.9 级',18,233,10)
T('头径上限 12；12 是含头总长；使用 4 mm 内六角。',18,226,9.2)
T('搜索词：DIN7991 M6×12 沉头内六角 10.9级',18,219,9.2)
T('需按 DIN 7991 头部尺寸供货；不要混发 ISO 10642。',18,210,9.2)
T('不加平垫/弹垫；装盘后量穿出量，接受范围 4.8-5.5。',18,203,9.2)
T('M6 实物孔可用深度须比穿出量至少多 0.5，不能顶底。',18,196,9.0)

Rct(14,127,182,57,fill=LIGHT)
T('② 原后盖固定到转接盘：4 颗',18,176,10.5,b=True)
T('NBK SSHS-M3-8-SD · M3×0.5 · 头下长度 8',18,169,10,b=True)
T('超短小头：头径 5、头高 1.3；1.5 mm 内六角；A2-50。',18,162,9.2)
T('搜索词：NBK SSHS-M3-8-SD（10 颗/包）',18,155,9.2)
T('可买同尺寸同等级替代件；不能只凭“低头 M3”下单。',18,148,9.0)
T('对应后盖 Φ6×1.5 沉孔；不得用普通圆柱头或加垫圈。',18,141,9.0)
T('相比初选 M3×6，改用 8 长以增加铝螺纹啮合长度。',18,134,9.0)

Rct(14,89,182,32,fill=LIGHT)
T('③ 后盖装回夹爪本体：4 颗',18,113,10.5,b=True)
T('M2.5×8 · 90°沉头内六角（按官方装配示例备购）',18,106,9.6)
T('只用于侧面原孔；采购时核对原螺钉头径、头型及总长。',18,99,9.0)
T('另备 4 mm、1.5 mm 内六角；小螺钉不套用通用大扭矩。',18,92,9.0)

T('安装顺序',14,79,11,b=True)
T('断电并支撑本体 → 拆后盖侧螺钉 → M6 锁盘到 E6',14,72,9.3)
T('→ 后盖内侧装 4 颗 M3 → 装回本体、恢复侧面 M2.5。',14,65,9.3)
T('先锁 M6，再装后盖；两贴合面不可夹线或遗留隔柱。',14,58,9.3,BLUE)

T('首件下料前，核对三项实物条件',14,46,10.5,b=True)
T('1. 后盖外围四孔对孔距 25；M3 沉孔 Φ6、深 1.5。',14,39,9)
T('2. 后盖贴合凸台无遮挡；沉孔底到贴合面厚 2.5-2.9。',14,32,9)
T('3. E6 中央凸出件落在 Φ34 内且高 ≤0.8；M6 孔深足够。',14,25,9)
T('不满足时修改 CAD 再加工；整只夹爪、PCB 和插头空间未实测。',14,17,8.6,MUT)
T('2026-09-23  |  官方图/STEP + 紧固件厂家尺寸核查；非实机验收',14,9,7.4,MUT)
T('2 / 2',196,9,8,align='right')
c.save()
print(F)
