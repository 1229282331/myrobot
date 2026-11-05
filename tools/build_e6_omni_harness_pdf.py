"""Offline, deterministic build of the three-variant E6/OmniPicker harness order."""
from pathlib import Path
from math import sin, cos, radians
from xml.sax.saxutils import escape

from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, Table, TableStyle

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/pdf/E6_OmniPicker_三规格短线束定制单_R1.pdf'
OUT.parent.mkdir(parents=True, exist_ok=True)
pdfmetrics.registerFont(TTFont('CN', 'C:/Windows/Fonts/msyh.ttc', subfontIndex=0))
pdfmetrics.registerFont(TTFont('CNB', 'C:/Windows/Fonts/msyhbd.ttc', subfontIndex=0))
pdfmetrics.registerFontFamily('CN', normal='CN', bold='CNB')
W, H = A4
M, CW = 42, W - 84
NAVY = colors.HexColor('#19384E')
TEAL = colors.HexColor('#146C72')
INK = colors.HexColor('#243849')
GRAY = colors.HexColor('#62717C')
LINE = colors.HexColor('#D4DEE4')
PALE = colors.HexColor('#EFF5F8')
AMBER = colors.HexColor('#FFF4DB')
RED = colors.HexColor('#C03830')
BLACK = colors.HexColor('#26303A')
C = canvas.Canvas(str(OUT), pagesize=A4)
C.setTitle('E6 与 OmniPicker 三规格短线束定制单 R1')
C.setAuthor('机器人项目 - 线束加工需求')
C.setSubject('GX12-6 / GX16-6 / GX20-6，各1根；总长80mm；2芯供电，4芯独立预留')


def p(txt, x, top, width=CW, size=9.3, leading=14, color=INK, bold=False):
    style = ParagraphStyle('p', fontName='CNB' if bold else 'CN', fontSize=size,
                           leading=leading, textColor=color, wordWrap='CJK')
    q = Paragraph(txt, style)
    _, height = q.wrap(width, H)
    assert top-height > 39, (txt[:50], top, height)
    q.drawOn(C, x, top-height)
    return top-height


def label(txt, x, y, size=9, color=INK, bold=False, center=False):
    C.setFillColor(color)
    C.setFont('CNB' if bold else 'CN', size)
    (C.drawCentredString if center else C.drawString)(x, y, txt)


def section(num, title, y):
    label(num, M, y, 10, TEAL, True)
    label(title, M+27, y, 12, NAVY, True)
    return y-15


def box(txt, y, fill=PALE, size=9.3):
    style = ParagraphStyle('box', fontName='CN', fontSize=size, leading=14,
                           textColor=INK, wordWrap='CJK')
    q = Paragraph(txt, style)
    _, height = q.wrap(CW-22, 200)
    total = height+18
    C.setFillColor(fill)
    C.roundRect(M, y-total, CW, total, 5, fill=1, stroke=0)
    q.drawOn(C, M+11, y-9-height)
    return y-total-12


def table(headers, rows, widths, y, size=9.0):
    sty = ParagraphStyle('cell', fontName='CN', fontSize=size, leading=13,
                         textColor=INK, wordWrap='CJK')
    hsty = ParagraphStyle('head', parent=sty, fontName='CNB', textColor=colors.white)
    data = [[Paragraph(t, hsty) for t in headers]]
    data += [[Paragraph(str(t), sty) for t in row] for row in rows]
    t = Table(data, colWidths=widths, hAlign='LEFT')
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, PALE]),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
        ('RIGHTPADDING', (0,0), (-1,-1), 8),
        ('TOPPADDING', (0,0), (-1,-1), 7),
        ('BOTTOMPADDING', (0,0), (-1,-1), 7),
        ('LINEBELOW', (0,0), (-1,0), .6, NAVY),
        ('LINEBELOW', (0,1), (-1,-1), .35, LINE),
    ]))
    _, hh = t.wrap(CW, H)
    assert y-hh > 43, ('table overflow', y, hh)
    t.drawOn(C, M, y-hh)
    return y-hh-13


def page(n, title, sub):
    C.setFillColor(TEAL)
    C.rect(M, H-37, 24, 3, fill=1, stroke=0)
    label('线束加工单  /  E6-OP-GX-80-R1', M+34, H-39, 8.5, GRAY)
    label(title, M, H-75, 20, NAVY, True)
    label(sub, M, H-98, 9.2, GRAY)
    C.setStrokeColor(LINE)
    C.line(M, 37, W-M, 37)
    label('2026-09-21  |  R1  |  尺寸单位：mm  |  本单共3页，请整份交付厂家', M, 23, 7.5, GRAY)
    label(f'{n:02d} / 03', W-M-35, 23, 8, GRAY)
    return H-119


def face(cx, cy, mirror=False):
    r = 49
    C.setStrokeColor(NAVY)
    C.setLineWidth(1.2)
    C.circle(cx, cy, r, fill=0, stroke=1)
    C.circle(cx, cy, r+6, fill=0, stroke=1)
    # The key is a position reference, not a mechanical cross-section.
    C.setFillColor(colors.white)
    C.rect(cx-9, cy-r-7, 18, 16, fill=1, stroke=0)
    C.setStrokeColor(TEAL)
    C.line(cx-8, cy-r-6, cx-8, cy-r+6)
    C.line(cx-8, cy-r+6, cx+8, cy-r+6)
    C.line(cx+8, cy-r+6, cx+8, cy-r-6)
    pos = {1:(-23,-25), 2:(23,-25), 3:(30,13), 4:(0,33), 5:(-30,13), 6:(0,0)}
    for pin,(dx,dy) in pos.items():
        dx = -dx if mirror else dx
        col = RED if pin==1 else BLACK if pin==6 else colors.HexColor('#AEBCC5')
        C.setFillColor(colors.white if mirror else col)
        C.setStrokeColor(col)
        C.setLineWidth(2 if pin in (1,6) else 1)
        C.circle(cx+dx,cy+dy,7,fill=1,stroke=1)
        text = f'E{pin}' if mirror else str(pin)
        label(text, cx+dx, cy+dy-2.8, 7.5, col if mirror else colors.white, True, True)
    label('定位键 / 槽位置朝下', cx, cy-76, 8, GRAY, center=True)


# PAGE 1: the unambiguous netlist is the primary manufacturing requirement.
y = page(1, 'GX 航插转 OmniPicker 六芯线束', '三种规格，各1根  |  单对供电 + 四芯独立引出  |  总长80 mm，含两端接头')
y = box('<b>订单核心：</b>J1（GX）仅接2个供电触点；J2（夹爪端）6个触点全部装线。'
        'J2 的2、3、4、5脚各自引出，不接J1，也不在本线束内相互并接。', y)
y = section('01', '加工数量与接头', y-4)
y = table(['线束编号', 'J1：GX线端接头', 'J2：夹爪线端接头', '数量'], [
    ['H12', 'GX12-6，六孔母触点<br/>带内螺纹锁紧环', 'GH1.25 6P（拟用）<br/>按原装六芯线确认配合', '1根'],
    ['H16', 'GX16-6，六孔母触点<br/>带内螺纹锁紧环', '同上，六个孔位均装端子', '1根'],
    ['H20', 'GX20-6，六孔母触点<br/>带内螺纹锁紧环', '同上，六个孔位均装端子', '1根'],
], [56,168,243-20,64], y)
y = p('三种GX按试配样件制作，不表示三种都适配E6。J2已知为1.25 mm间距、6P、带锁扣；'
      '<b>GH系列和具体料号须与原线/夹爪插座核对</b>，不得仅凭“1.25 mm”替代。', M, y, leading=14)
y = section('02', '逐针对应关系（三根完全相同）', y-25)
y = table(['J2脚号', '功能', '加工线色', '另一端连接', '成品标识'], [
    ['1', 'VIN主供电', '红', 'J1-E1（与E6针1相配）', '24V / J2-1'],
    ['2', 'VIN预留', '橙', 'T2，独立引出；不接J1', 'VIN / J2-2'],
    ['3', '通信脚3', '黄', 'T3，独立引出；不接J1', 'COMM3 / J2-3'],
    ['4', '通信脚4', '绿', 'T4，独立引出；不接J1', 'COMM4 / J2-4'],
    ['5', 'GND预留', '蓝', 'T5，独立引出；不接J1', 'GND / J2-5'],
    ['6', 'GND主回路', '黑', 'J1-E6（与E6针6相配）', 'GND / J2-6'],
], [51,81,63,186,130], y, 8.7)
y = box('<b>J1-E1 / J1-E6 是功能孔位名：</b>分别表示与E6针1（24V）、针6（GND）配对的孔。'
        '它们不等同于任意GX产品自带的刻号；插接面位置见第2页。', y, AMBER, 9)
y = p('<b>J1其余4孔（对应E6针2、3、4、5）：不焊线、不跳接，焊杯留空。</b>'
      '固定式触点无需拆除。金属外壳不与任一导线相接。线色为本次加工约定，针号/导通关系优先。', M, y, leading=14)
C.showPage()

# PAGE 2: orientation, source illustration, version-independent breakout labels.
y = page(2, '针位与观察方向', '加工与验收均按“插接面”核对；焊接面需要镜像转换')
y = section('03', 'E6公座与J1母头的配合面', y)
label('E6设备公座：正面看针', 169, y-8, 11, NAVY, True, True)
label('J1线端母头：正面看孔', 423, y-8, 11, NAVY, True, True)
face(169,y-84,False)
face(423,y-84,True)
y -= 181
y = box('<b>左图依据E6手册§6.2重绘；右图是相同定位方向下的镜像配合关系。</b>'
        '右图仅E1、E6装线；E2-E5不接。此图不按比例，不代替GX厂家的机械尺寸图。'
        '若孔位或定位结构不同，需先查明配合关系，不能硬套刻号。', y, AMBER, 9)
y = section('04', 'OmniPicker针号依据', y-1)
img = ROOT/'research/grippers/omnipicker_wiring_2026-09-21/omnipicker_official_20_pinout.png'
iw, ih = 511, 210
C.drawImage(str(img), M, y-ih, width=iw, height=ih, preserveAspectRatio=True, anchor='c', mask='auto')
y -= ih+4
y = p('图源：智元官方OmniPicker手册§5.1，20 PCBA接口图。左图PCB标出1-6脚；右图标出本体出线针号。'
      '本图的原装线色不替代第1页加工线色。<b>J2须按与设备相配的脚号装线，不能自行假定“锁扣朝上、左起为1”。</b>', M, y, size=8.7, leading=13)
y = section('05', '预留通信线的标签规则', y-23)
y = table(['预留端', '20 PCBA（默认CAN）', '30 PCBA（默认CAN）'], [
    ['T3 / J2-3', 'CAN_H', 'CAN_L'],
    ['T4 / J2-4', 'CAN_L', 'CAN_H'],
], [151,180,180], y, 8.8)
y = p('制作时统一标COMM3、COMM4，不凭待确认板型固定H/L。T3、T4成对轻绞；本线不加终端电阻或转换模块。'
      'T2、T5分别为备用VIN/GND；插上夹爪后可能经内部电路与电源相连，预留端必须分别绝缘。', M, y, size=9, leading=13)
C.showPage()

# PAGE 3: short overall length, fabrication, inspection, concise references.
y = page(3, '尺寸、工艺与出厂验收', '主线总长80 mm；预留尾线每根约30 mm，朝夹爪端收纳')
y = section('06', '长度定义与分叉位置', y)
yt = y-53
x1,x2 = M+22,W-M-22
# Connector housings and central harness: schematic dimensions, not to scale.
C.setFillColor(PALE); C.setStrokeColor(NAVY); C.setLineWidth(1)
C.roundRect(x1,yt-17,95,34,4,fill=1,stroke=1)
C.roundRect(x2-53,yt-14,53,28,3,fill=1,stroke=1)
label('J1  GX母头',x1+47,yt-3,9,NAVY,True,True)
label('J2  6P',x2-26,yt-3,9,NAVY,True,True)
bx=x1+111
# These are jacket/bundle paths, not electrical circuit wires.
C.setStrokeColor(GRAY); C.setLineWidth(4); C.line(x1+95,yt,bx,yt)
C.setLineWidth(8); C.line(bx,yt,x2-53,yt)
C.setStrokeColor(TEAL); C.setLineWidth(4)
C.line(bx,yt,bx+28,yt-44); C.line(bx+28,yt-44,bx+132,yt-44)
label('6芯线束段', (bx+x2-53)/2, yt+13, 8, GRAY, center=True)
label('GX内部仅接2芯', x1+47, yt-32, 8, GRAY, center=True)
C.setLineWidth(1.5)
for k in range(4):
    yy=yt-53-7*k
    C.line(bx+132,yt-44,bx+156,yy); C.line(bx+156,yy,bx+183,yy)
label('T2 / T3 / T4 / T5',bx+199,yt-64,8,TEAL)
label('分叉设在GX尾套出线口附近；尾线朝J2收纳',M+38,yt-94,8.5,GRAY)
label('图为线束外形示意，分叉不表示导线电气并接；接线以第1页为准。',M+38,yt-108,8.1,TEAL)
# Overall dimension excludes no housing: endpoints shown explicitly.
dy=yt+42
C.setStrokeColor(GRAY); C.setLineWidth(.6)
for xx in (x1,x2): C.line(xx,yt+20,xx,dy+7)
C.line(x1,dy,x2,dy)
for xx,sign in ((x1,1),(x2,-1)):
    C.line(xx,dy,xx+sign*6,dy+3); C.line(xx,dy,xx+sign*6,dy-3)
label('80 ± 5（含两端接头；不含预留支线）', (x1+x2)/2,dy+7,9,TEAL,True,True)
y = yt-120
y = p('长度沿自然拉直的主线中心路径计，端点为GX接头插接端最外端和J2壳体插接端最外端。'
      '预留线从分叉点至绝缘末端各30±5 mm（设计建议），末端朝J2方向收纳，尽量不超出主线包络。'
      '<b>优先短款接头；若壳体和尾套使80 mm无法实现，厂家需报最短可做尺寸，不得省略应力释放结构。</b>', M, y, size=9, leading=13)
y = section('07', '材料与加工要求', y-23)
y = table(['项目', '要求'], [
    ['导线与端子', '优先26 AWG多股软铜线（约0.13 mm²）；J2用匹配端子压接，不塞入超规格粗线。若确为JST GH，26 AWG绝缘外径须符合0.76-1.0 mm。'],
    ['分叉与预留端', 'T2-T5每根独立绝缘、标号；交货末端不剥皮、不上锡。分叉包覆柔软热缩/护套，不用硬胶把短线段整体固定。'],
    ['机械处理', 'GX焊点独立绝缘；尾套夹紧线束护套，避免直接压导体。J2端子锁止到位。三根分别贴H12/H16/H20，独立包装。'],
], [78,433], y, 8.4)
y = section('08', '厂家100%导通检查（线束脱离设备）', y-1)
y = p('<b>仅允许以下6组导通：</b>J1-E1与J2-1；J1-E6与J2-6；'
      'T2与J2-2；T3与J2-3；T4与J2-4；T5与J2-5。'
      '每组扣除表笔电阻后建议≤1 Ω；轻弯线束时不得间歇断路。', M, y, size=8.9, leading=13)
y = p('六组网络彼此之间、各组与J1空置触点及金属壳之间均不导通（万用表电阻档显示OL）。'
      '注意：J2-1与J2-2、J2-5与J2-6在独立线束内也必须不导通。留存三根线的检测结果和插接面照片。', M, y-5, size=8.9, leading=13)
y = box('<b>本单是线束加工要求，不是供电能力认证。</b>JST GH原厂额定电流为1 A/触点（26 AWG条件）；'
        '若采用其他端子，厂家须标明对应额定值。用途电压为24V DC，图中板上30V/5A标记不作为本线额定值。'
        'E6腕部电源容量与夹爪峰值电流仍待核实。'
        '出厂只做脱机检查，不连接机械臂或夹爪上电。', y-8, AMBER, 8.5)
y = p('<b>厂家随货填写：</b>H12/H16/H20各自GX料号及螺纹；J2胶壳/端子料号；'
      'J1厂刻号与E1/E6的映射；J2针1识别方式；实测总长；导通/短路检查结论。', M, y, size=8.5, leading=12)
y = p('<b>依据：</b>[1] Dobot Magician E6用户手册V1.4，§6.2（印刷页22）；'
      '[2] 智元OmniPicker官方手册，§5.1；[3] JST GH原厂目录。<br/>'
      '<link href="https://www.agibot.com/filepage/265.html" color="#146C72">agibot.com/filepage/265.html</link>'
      '　|　<link href="https://www.jst-mfg.com/product/index.php?series=105&amp;lang=2" color="#146C72">JST GH规格</link>'
      '　|　<link href="https://download.dobot.cc/2024/04/Dobot%20Magician%20E6%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8CV1.4_20240226_cn.pdf" color="#146C72">E6官方手册</link>',
      M,y-11,size=7.7,leading=11)
C.showPage()
C.save()
print(OUT)
