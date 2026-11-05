"""Neutral supplier sheet. GH views reconstructed from JST eGH pp. 1 and 3.
The JST housing convention does not establish the user's sample wiring.
"""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/pdf/GX转GH1.25_80mm_定制图_R6.pdf'
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/msyh.ttc',subfontIndex=0))
pdfmetrics.registerFont(TTFont('Bold','C:/Windows/Fonts/msyhbd.ttc',subfontIndex=0))
W,H=A4
M=38
INK='#203A4B'; GRAY='#697983'; TEAL='#137A79'; LIGHT='#EDF4F6'; LINE='#CDDAE1'
PIN={1:('红','#C73930'),2:('橙','#BF621E'),3:('黄','#977300'),4:('绿','#248051'),5:('蓝','#3169AA'),6:('黑','#242B32')}
c=canvas.Canvas(str(OUT),pagesize=A4)
c.setTitle('GX转GH1.25短线束定制图 R6')
c.setAuthor('线束定制')
c.setSubject('三种GX六孔母头各1根；总长80mm；2芯连接，4芯独立引出；含GH双面孔位图')
def txt(s,x,y,size=10,col=INK,bold=False,center=False):
    c.setFillColor(colors.HexColor(col)); c.setFont('Bold' if bold else 'CN',size)
    (c.drawCentredString if center else c.drawString)(x,y,s)
def line(x1,y1,x2,y2,col=INK,width=1):
    c.setStrokeColor(colors.HexColor(col)); c.setLineWidth(width); c.line(x1,y1,x2,y2)
def rect(x,y,w,h,col=INK,fill='#FFFFFF',radius=2,width=1):
    c.setStrokeColor(colors.HexColor(col)); c.setFillColor(colors.HexColor(fill)); c.setLineWidth(width)
    c.roundRect(x,y,w,h,radius,fill=1,stroke=1)
def circle(x,y,r,col=INK,width=1):
    c.setStrokeColor(colors.HexColor(col)); c.setFillColor(colors.white); c.setLineWidth(width); c.circle(x,y,r,stroke=1,fill=1)

txt('GX 转 GH1.25 短线束定制图',M,793,21,bold=True)
txt('共3根｜主线总长80 mm（含两端接头）｜2芯供电＋4芯独立引出',M,770,9.8,GRAY)
txt('R6',W-M-17,798,9,GRAY)
rect(M,705,W-2*M,49,col=LIGHT,fill=LIGHT,radius=5)
for cx,s in [(124,'GX12-6 × 1根'),(298,'GX16-6 × 1根'),(470,'GX20-6 × 1根')]: txt(s,cx,734,11,bold=True,center=True)
txt('GX：六孔母头＋内螺纹锁紧环       GH：1.25 mm、6P、带锁扣',M+13,716,9.5)

# Mechanical outline represents bundles, not connected electrical nets.
txt('① 外形与长度',M,683,11,bold=True)
x1,x2,cy=60,W-60,623
line(x1,657,x2,657,GRAY,.6)
for x,d in [(x1,1),(x2,-1)]:
    line(x,657,x+6*d,660,GRAY,.6); line(x,657,x+6*d,654,GRAY,.6); line(x,632,x,664,GRAY,.6)
txt('80 ± 5 mm：两端插接端最外端之间',(x1+x2)/2,666,10,TEAL,True,True)
rect(x1,cy-17,87,34,fill=LIGHT,radius=4); rect(x2-61,cy-17,61,34,fill=LIGHT,radius=4)
txt('GX母头',x1+43.5,cy-3,10,bold=True,center=True); txt('GH 6P',x2-30.5,cy-3,10,bold=True,center=True)
bx=x1+111
line(x1+87,cy,bx,cy,GRAY,4); line(bx,cy,x2-61,cy,GRAY,7)
txt('2芯',x1+87,cy+12,8,GRAY); txt('6芯',350,cy+12,8,GRAY,center=True)
line(bx,cy,bx+28,cy-24,TEAL,4); line(bx+28,cy-24,bx+117,cy-24,TEAL,4)
for i,pin in enumerate([2,3,4,5]):
    yy=611-i*12; col=PIN[pin][1]
    line(bx+117,cy-24,bx+146,yy,col,1.7); line(bx+146,yy,bx+176,yy,col,1.7)
    txt(f'T{pin}',bx+183,yy-3,8.5,col,True)
txt('预留线各30 ± 5 mm；分叉靠近GX尾部，向GH端收纳。示意不按比例。',M+9,556,8.8,TEAL)

# The assembly illustration fixes the mating direction for the No.1 housing view.
txt('② GH孔位：同一线端插头，锁扣均朝上',M,533,11,bold=True)
def gh(cx,pins,title,rear=False):
    txt(title,cx,511,10,bold=True,center=True)
    rect(cx-96,444,192,34,fill=LIGHT,radius=3)
    rect(cx-31,478,62,11,radius=2)
    txt('锁扣',cx,492,8.4,TEAL,True,True)
    for i,p in enumerate(pins):
        xx=cx-75+i*30; name,col=PIN[p]
        rect(xx-10,452,20,18,col=col,width=1.6 if p==1 else .85)
        txt(str(p),xx,456,10,col,True,True)
        if rear: line(xx,444,xx,434,col,2)
        txt(name,xx,421,8.5,col,center=True)
    txt('从电线侧看进线孔' if rear else '正对插孔看，电线在背后',cx,405,8.5,GRAY,center=True)
gh(162,[6,5,4,3,2,1],'插孔面（插接端）')
gh(435,[1,2,3,4,5,6],'出线面（压接穿线端）',True)
txt('GH图按JST GHR-06V-S；开工前核对样线胶壳及针1方向，一致后加工。',M,385,8.8,TEAL,True)

# GX custom A/B positions, not vendor stamped pin numbers.
txt('③ GX母头：正面看插孔',M,362,10.5,bold=True)
txt('插接端朝自己，电线在背后',M,346,8.5,TEAL,True)
cx,fy,r=122,284,46
circle(cx,fy,r+5); circle(cx,fy,r)
c.setFillColor(colors.white); c.rect(cx-8,fy-r-6,16,15,stroke=0,fill=1)
line(cx-7,fy-r-5,cx-7,fy-r+6,TEAL); line(cx-7,fy-r+6,cx+7,fy-r+6,TEAL); line(cx+7,fy-r+6,cx+7,fy-r-5,TEAL)
for dx,dy in [(-22,-24),(-29,12),(0,32),(29,12)]:
    circle(cx+dx,fy+dy,6.5,LINE); txt('×',cx+dx,fy+dy-3,8,GRAY,center=True)
for s,dx,dy,col in [('A',22,-24,PIN[1][1]),('B',0,0,PIN[6][1])]:
    circle(cx+dx,fy+dy,8,col,1.6); txt(s,cx+dx,fy+dy-3,9,col,True,True)
txt('定位键／槽朝下',cx,218,8.7,GRAY,center=True)
txt('A右下孔：红线   B中心孔：黑线',M,197,8.6,bold=True)
txt('其余4孔及金属壳不接线',M,181,8.5,GRAY)
txt('A/B为本图孔位名，非厂家刻号',M,166,8.3,GRAY)

# Neutral six-net mapping.
tx,tw=250,W-M-250
txt('④ 逐针接线（三根相同）',tx,362,10.5,bold=True)
rows=[['GH脚号','线色','另一端'],['1','红','GX图示 A孔（24V）'],['2','橙','独立引出 T2（VIN）'],['3','黄','独立引出 T3（通信）'],['4','绿','独立引出 T4（通信）'],['5','蓝','独立引出 T5（GND）'],['6','黑','GX图示 B孔（GND）']]
for i,row in enumerate(rows):
    yy=348-(i+1)*24
    fill=INK if i==0 else (LIGHT if i%2==0 else '#FFFFFF')
    c.setFillColor(colors.HexColor(fill)); c.rect(tx,yy,tw,24,fill=1,stroke=0)
    if i: line(tx,yy,tx+tw,yy,LINE,.4)
    for xx,s in zip([tx+7,tx+61,tx+97],row): txt(s,xx,yy+8,8.7,'#FFFFFF' if i==0 else INK,i==0)
txt('GH六脚全部装线；T2-T5不接GX。',tx,163,8.5,TEAL,True)
txt('图示为母头插孔正面，与配对公头正面左右镜像；不是背面的焊接面。',M,137,9,TEAL,True)
line(M,52,W-M,52,LINE,.7)
txt('GX-GH-80 / R6',M,36,8,GRAY)
txt('孔位示意不按比例｜单位：mm｜1 / 1',W-M-179,36,8,GRAY)
c.save()
print(OUT)
