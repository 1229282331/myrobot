"""Create the neutral, one-page supplier drawing. No device names or photos."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Table, TableStyle, Paragraph
from reportlab.lib.styles import ParagraphStyle

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'output/pdf/GX转GH1.25_80mm_定制图_R2.pdf'
OUT.parent.mkdir(parents=True, exist_ok=True)
pdfmetrics.registerFont(TTFont('SheetCN','C:/Windows/Fonts/msyh.ttc',subfontIndex=0))
pdfmetrics.registerFont(TTFont('SheetBold','C:/Windows/Fonts/msyhbd.ttc',subfontIndex=0))
pdfmetrics.registerFontFamily('SheetCN',normal='SheetCN',bold='SheetBold')
W,H=A4
M=38
CW=W-2*M
INK=colors.HexColor('#203A4B')
GRAY=colors.HexColor('#697983')
TEAL=colors.HexColor('#137A79')
LIGHT=colors.HexColor('#EDF4F6')
LINE=colors.HexColor('#CDDAE1')
RED=colors.HexColor('#C73930')
BLACK=colors.HexColor('#242B32')
ORANGE=colors.HexColor('#BF621E')
YELLOW=colors.HexColor('#977300')
GREEN=colors.HexColor('#248051')
BLUE=colors.HexColor('#3169AA')
C=canvas.Canvas(str(OUT),pagesize=A4)
C.setTitle('GX转GH1.25短线束定制图 R2')
C.setAuthor('线束定制')
C.setSubject('三种GX六孔母头各1根；总长80mm；2芯连接，4芯独立引出')


def text(s,x,y,size=10,col=INK,bold=False,center=False):
    C.setFillColor(col)
    C.setFont('SheetBold' if bold else 'SheetCN',size)
    (C.drawCentredString if center else C.drawString)(x,y,s)


def para(s,x,y,width,size=9.2,leading=14):
    st=ParagraphStyle('body',fontName='SheetCN',fontSize=size,leading=leading,textColor=INK,wordWrap='CJK')
    q=Paragraph(s,st)
    _,hh=q.wrap(width,300)
    assert y-hh>44
    q.drawOn(C,x,y-hh)
    return y-hh


def dim(x1,x2,y,label):
    C.setStrokeColor(GRAY); C.setLineWidth(.6)
    C.line(x1,y,x2,y)
    for x,d in ((x1,1),(x2,-1)):
        C.line(x,y,x+d*6,y+3); C.line(x,y,x+d*6,y-3)
        C.line(x,y-25,x,y+7)
    text(label,(x1+x2)/2,y+9,10,TEAL,True,True)


text('GX 转 GH1.25 短线束定制图',M,791,22,bold=True)
text('共3根｜三种规格，各1根｜主线总长80 mm（含两端接头）',M,766,10,GRAY)
text('R2',W-M-18,796,9,GRAY)

# Compact order strip.
C.setFillColor(LIGHT); C.roundRect(M,690,CW,58,5,fill=1,stroke=0)
for cx,s in [(M+86,'GX12-6 × 1根'),(M+260,'GX16-6 × 1根'),(M+432,'GX20-6 × 1根')]:
    text(s,cx,727,11,INK,True,True)
text('GX端：六孔母头＋内螺纹锁紧环     GH端：1.25 mm、6P、带锁扣',M+13,706,9.5)

# Mechanical harness sketch: neutral jacket paths, no electrical junction claim.
text('线束外形与长度',M,666,12,INK,True)
x1,x2=60,W-60
dim(x1,x2,638,'80 ± 5 mm：从两端插接端最外端量取')
cy=591
C.setStrokeColor(INK); C.setFillColor(LIGHT); C.setLineWidth(1)
C.roundRect(x1,cy-20,99,40,4,fill=1,stroke=1)
C.roundRect(x2-65,cy-20,65,40,4,fill=1,stroke=1)
text('GX母头',x1+49.5,cy-3,11,INK,True,True)
text('GH 6P',x2-32.5,cy-3,10,INK,True,True)
bx=x1+117
C.setStrokeColor(GRAY); C.setLineWidth(4); C.line(x1+99,cy,bx,cy)
C.setLineWidth(8); C.line(bx,cy,x2-65,cy)
text('2芯',x1+103,cy+16,8,GRAY)
text('6芯',334,cy+16,9,GRAY,center=True)
C.setStrokeColor(TEAL); C.setLineWidth(5)
C.line(bx,cy,bx+30,cy-36); C.line(bx+30,cy-36,bx+142,cy-36)
for i,(name,col) in enumerate([('T2',ORANGE),('T3',YELLOW),('T4',GREEN),('T5',BLUE)]):
    yy=cy-41-i*11
    C.setStrokeColor(col); C.setLineWidth(1.8)
    C.line(bx+142,cy-36,bx+161,yy)
    C.line(bx+161,yy,bx+185,yy)
    text(name,bx+192,yy-3,8.2,col,True)
text('四条预留线各30 ± 5 mm；分叉靠近GX尾部，朝GH端收纳',M+12,493,9,TEAL)
text('外形示意，不按比例；分叉不表示导线并接。',M+12,477,8.5,GRAY)

# GX mating face, with custom A/B names to avoid vendor pin-number ambiguity.
text('GX孔位：正面看插孔',M,446,11,INK,True)
cx,fy,r=135,355,53
C.setStrokeColor(INK); C.setLineWidth(1.2)
C.circle(cx,fy,r,stroke=1,fill=0)
C.circle(cx,fy,r+6,stroke=1,fill=0)
C.setFillColor(colors.white); C.rect(cx-9,fy-r-7,18,17,stroke=0,fill=1)
C.setStrokeColor(TEAL)
C.line(cx-8,fy-r-6,cx-8,fy-r+7)
C.line(cx-8,fy-r+7,cx+8,fy-r+7)
C.line(cx+8,fy-r+7,cx+8,fy-r-6)
for dx,dy in [(-25,-27),(-32,14),(0,35),(32,14)]:
    C.setStrokeColor(LINE); C.setFillColor(colors.white)
    C.circle(cx+dx,fy+dy,7,stroke=1,fill=1)
    text('×',cx+dx,fy+dy-3,8,GRAY,center=True)
for s,dx,dy,col in [('A',25,-27,RED),('B',0,0,BLACK)]:
    C.setLineWidth(2); C.setStrokeColor(col); C.setFillColor(colors.white)
    C.circle(cx+dx,fy+dy,9,stroke=1,fill=1)
    text(s,cx+dx,fy+dy-3,10,col,True,True)
text('定位键／槽朝下',cx,fy-80,9,GRAY,center=True)
text('A：右下孔，接红线',M+15,249,9.5,RED,True)
text('B：中心孔，接黑线',M+15,232,9.5,BLACK,True)
text('其余4孔不焊线、不跳接',M+15,215,9,GRAY)

# Netlist; wire colors are assigned for this order only.
tx,tw=250,W-M-250
text('逐针接线（三根相同）',tx,446,11,INK,True)
st=ParagraphStyle('cell',fontName='SheetCN',fontSize=9,leading=13,textColor=INK,wordWrap='CJK')
hs=ParagraphStyle('head',parent=st,fontName='SheetBold',textColor=colors.white)
raw=[['GH脚号','线色','另一端'],
     ['1','红','接GX图示 A孔（24V）'],
     ['2','橙','独立引出 T2（VIN）'],
     ['3','黄','独立引出 T3（通信）'],
     ['4','绿','独立引出 T4（通信）'],
     ['5','蓝','独立引出 T5（GND）'],
     ['6','黑','接GX图示 B孔（GND）']]
data=[[Paragraph(x,hs if i==0 else st) for x in row] for i,row in enumerate(raw)]
t=Table(data,colWidths=[55,37,tw-92],rowHeights=[27]*7)
t.setStyle(TableStyle([
    ('BACKGROUND',(0,0),(-1,0),INK),
    ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,LIGHT]),
    ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
    ('LEFTPADDING',(0,0),(-1,-1),7),
    ('RIGHTPADDING',(0,0),(-1,-1),6),
    ('LINEBELOW',(0,1),(-1,-1),.4,LINE),
]))
t.wrap(tw,300); t.drawOn(C,tx,430-189)
text('GH端6脚全部装线；四条预留线不接GX。',tx,224,8.8,TEAL,True)

C.setStrokeColor(LINE); C.setLineWidth(.7); C.line(M,197,W-M,197)
text('加工要求',M,177,11,INK,True)
y=164
for s in [
    '1. 26 AWG多股软铜线，匹配端子压接；六条线各自独立，不并接；金属外壳不接线。',
    '2. 预留线末端不剥皮，分别绝缘并标T2/T3/T4/T5；线色不足时用对应色标。',
    '3. GX按图示A/B孔位接线（非厂家刻号）；GH胶壳、锁扣及针1方向按样线核对。',
    '4. 交货前逐线测导通、排除短路。选短款GX壳体；若做不到80 mm，先反馈最短尺寸。',
]:
    y=para(s,M,y,CW,size=9.1,leading=14)-4
C.setStrokeColor(LINE); C.line(M,52,W-M,52)
text('GX-GH-80 / R2',M,36,8,GRAY)
text('单位：mm   |   1 / 1',W-M-112,36,8,GRAY)
C.save()
print(OUT)
