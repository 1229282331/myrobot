from pathlib import Path
import math
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.platypus import Paragraph
from reportlab.lib.styles import ParagraphStyle
R=Path(__file__).resolve().parents[1]
p=R/'output/pdf/E6_OmniPicker_安装转接方案_2026-09-23.pdf'
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/msyh.ttc',subfontIndex=0))
pdfmetrics.registerFont(TTFont('CB','C:/Windows/Fonts/msyhbd.ttc',subfontIndex=0))
pdfmetrics.registerFontFamily('CN',normal='CN',bold='CB')
W,H=A4;M=37
I='#223C4C';G='#607481';T='#167B7A';L='#EDF4F6';B='#347DC0';A='#A06225'
c=canvas.Canvas(str(p),pagesize=A4)
c.setTitle('E6与OmniPicker安装转接方案 | 2026-09-23')
c.setAuthor('机械转接方案')
c.setSubject('单片6061-T6铝盘；保留原后盖；方案级参考，非生产加工图')
def text(s,x,y,size=10,col=I,bold=False,center=False):
 c.setFillColor(colors.HexColor(col));c.setFont('CB' if bold else 'CN',size)
 (c.drawCentredString if center else c.drawString)(x,y,s)
def line(x1,y1,x2,y2,col=G,w=1):
 c.setStrokeColor(colors.HexColor(col));c.setLineWidth(w);c.line(x1,y1,x2,y2)
def box(x,y,w,h,fill=L,stroke=L):
 c.setFillColor(colors.HexColor(fill));c.setStrokeColor(colors.HexColor(stroke));c.setLineWidth(1);c.roundRect(x,y,w,h,4,fill=1,stroke=1)
def para(s,x,y,w,size=9.3,leading=14):
 q=Paragraph(s,ParagraphStyle('p',fontName='CN',fontSize=size,leading=leading,textColor=colors.HexColor(I),wordWrap='CJK'))
 _,h=q.wrap(w,300);q.drawOn(c,x,y-h);return y-h

def arrow(x1,y,x2):
 line(x1,y,x2,y,T,1.3);line(x2,y,x2-5,y+3,T,1.3);line(x2,y,x2-5,y-3,T,1.3)
text('E6 × OmniPicker 安装转接方案',M,793,20,bold=True)
text('推荐：一块薄铝盘＋保留原后盖',M,768,11,T,True)
text('2026-09-23',W-M-64,770,8.5,G)
box(M,709,W-2*M,43)
para('<b>初设方案，非生产加工图。</b>两侧孔圈尺寸来自官方资料；圆盘厚度、螺钉及重量为设计建议。实物后盖和螺钉净空复核后定稿。',M+12,742,W-2*M-24,9.4,14)

text('连接结构与螺钉方向',M,688,11,bold=True)
for x,w,label,detail in [(45,94,'E6 末端法兰','4×M6螺纹孔'),(180,82,'定制铝盘','Φ65 × 7 初设'),(309,63,'原后盖','内侧装M3'),(421,128,'夹爪本体','侧向4×M2.5')]:
 box(x,625,w,45,fill=L,stroke=G);text(label,x+w/2,650,10,bold=True,center=True);text(detail,x+w/2,633,8.2,G,center=True)
for x1,x2 in [(142,176),(266,305),(376,417)]:arrow(x1,647,x2)
text('M6先锁盘；再从后盖内侧装M3；最后从侧面装回本体。',M,609,9,T,True)

text('转接盘孔圈示意',M,585,11,bold=True)
cx,cy,scale=151,474,2.62
c.setStrokeColor(colors.HexColor(I));c.setLineWidth(1.2);c.circle(cx,cy,32.5*scale,fill=0,stroke=1)
c.saveState();c.setDash(3,3)
for r,col in [(25,B),(12.5,T)]:
 c.setStrokeColor(colors.HexColor(col));c.circle(cx,cy,r*scale,fill=0,stroke=1)
c.restoreState()
line(cx-92,cy,cx+92,cy,'#C8D7DF',.4);line(cx,cy-92,cx,cy+92,'#C8D7DF',.4)
for deg in [45,135,225,315]:
 rad=math.radians(deg);x=cx+25*scale*math.cos(rad);y=cy+25*scale*math.sin(rad)
 c.setStrokeColor(colors.HexColor(B));c.setLineWidth(1.2);c.circle(x,y,6*scale,fill=0,stroke=1);c.circle(x,y,3.3*scale,fill=0,stroke=1)
for deg in [0,90,180,270]:
 rad=math.radians(deg);x=cx+12.5*scale*math.cos(rad);y=cy+12.5*scale*math.sin(rad)
 c.setStrokeColor(colors.HexColor(T));c.setLineWidth(1.5);c.circle(x,y,1.5*scale,fill=0,stroke=1)
text('蓝：4个M6沉头通孔 / PCD 50',M,367,8.8,B,True)
text('绿：4个M3螺纹孔 / PCD 25',M,351,8.8,T,True)
text('周向角度与定位结构按实物确认。',M,335,8.2,G)

x=282;ww=W-M-x;y=583
for title,body in [
 ('材料与外形','6061-T6铝；外径65、面间厚7 mm。保留原后盖，不改动电路板。'),
 ('E6侧','PCD50的4×Φ6.6通孔＋90°沉头座，螺钉头不高出贴合面。初选M6×12：名义穿出5 mm，原孔深6 mm。'),
 ('夹爪侧','外围PCD25的4×M3螺纹孔。超短头M3×6为候选；适配原盖Φ3.2通孔及Φ6×1.5沉孔，核对PCB净空。'),
 ('定位与出线','可利用E6的Φ6定位孔，销插入端建议≤3 mm。保留Type-C、六芯插头和侧向螺钉的操作空间。'),
 ('质量预算','铝盘实心毛坯约62.7 g，开孔后略轻；盘＋紧固件先按约70 g预算。')]:
 text(title,x,y,9.7,bold=True);y=para(body,x,y-7,ww,9,13)-11

line(M,316,W-M,316,'#CEDCE2',.7)
text('装配顺序',M,296,11,bold=True)
y=280
for n,s in [
 ('1','断电并支撑夹爪，拆后盖侧面4颗M2.5螺钉，取下后盖。'),
 ('2','先把铝盘用4颗M6沉头螺钉固定到E6，确认不顶孔底。'),
 ('3','后盖贴上铝盘，从盖内侧装M3螺钉，拧入铝盘的M3孔。'),
 ('4','夹爪本体装回后盖，用侧向M2.5螺钉固定；检查线缆与净空。')]:
 text(n,M,y-11,10,T,True);y=para(s,M+20,y,W-2*M-20,9.5,14)-7

box(M,119,W-2*M,62)
para('<b>下单前核对：</b>后盖实际孔距、孔型、两处金色紧固件用途；M3头部与PCB净空；E6实际孔深、定位和插头避让。旧照片不能替代实物尺寸。',M+11,171,W-2*M-22,9,13)
para('<b>负载：</b>430 g夹爪＋约70 g转接件≈500 g；相机和物体另计，并按E6合成质心负载曲线降额。装配时支撑本体，既有J4/J5保持问题需处理。',M,104,W-2*M,8.8,12)
text('依据：E6手册V1.4 §5.4/7.3.2；OmniPicker官方手册 §3.2/3.3及后盖工程图。',M,62,8,G)
c.linkURL('https://www.agibot.com/filepage/265.html',(M,58,W-M,72),relative=0)
line(M,48,W-M,48,'#CEDCE2',.7)
text('结构方案 / 未完成实物装配验证',M,32,8,G);text('1 / 1',W-M-25,32,8,G)
c.save();print(p)
