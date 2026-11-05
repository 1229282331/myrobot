"""Create a two-file, application-neutral manufacturing handoff from R2.

Uses the verified R2 geometry verbatim. Internal project/assembly information
is kept out of the delivered PDF metadata, STEP filename and archive.
"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import zipfile

from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'output/manufacturer/PD-01_R2'
DEST.mkdir(parents=True, exist_ok=True)
PDF = ROOT / 'output/pdf/PD-01_R2_加工图.pdf'
STEP_NAME = 'PD-01_R2.step'
ZIP = ROOT / 'output/manufacturer/圆盘_R2_厂家加工包.zip'
SOURCE_STEP = ROOT / 'output/cad/E6_OmniPicker_adapter_R2/E6_OMNI_ADAPTER_R2.step'
SOURCE_DRAWING = ROOT / 'tools/build_e6_omnipicker_adapter_pdf_r2.py'

source = SOURCE_DRAWING.read_text(encoding='utf-8').split('c.showPage()', 1)[0]
replacements = {
    "F=R/'output/pdf/E6_OmniPicker_转接盘加工图与采购单_R2.pdf'": "F=R/'output/pdf/PD-01_R2_加工图.pdf'",
    "c.setTitle('E6 - OmniPicker 转接盘 | R2 加工图及螺钉采购单')": "c.setTitle('PD-01 / R2 圆盘加工图')",
    "c.setAuthor('E6 项目 | 自定义转接件')": "c.setAuthor('')",
    "c.setSubject('官方接口尺寸建模；首件试装版，未做实物与承载验证')": "c.setSubject('6061-T6; 1 piece; dimensions in mm')",
    "T('转接盘加工图',14,282,21,b=True)": "T('圆盘加工图',14,282,21,b=True)",
    "T('E6-OMNI-ADP-01  /  R2',405,284,12,BLUE,True,'right')": "T('PD-01  /  R2',405,284,12,BLUE,True,'right')",
    "T('R2 新增 E6 外圈 8 个 M3 备用安装孔 · 首件实物适配尚未验证',14,274,10,MUT)": "T('材料：6061-T6 铝合金    数量：1 件    表面：本色，不氧化',14,274,10,MUT)",
    "'B 面：朝夹爪，从 +Z 正视'": "'B 面：沉头面，正视（+Z）'",
    "'A 面：朝机械臂，从 -Z 正视'": "'A 面：浅槽面，正视（-Z）'",
    "T('中央浅槽仅用于避让；不作止口定位',ox,103,8.5,MUT,align='center')": "T('中心浅槽 Φ34，深 1',ox,103,8.5,MUT,align='center')",
    "T('4×Φ6.6 +0.15/0 通孔；配 DIN7991 M6×12。',321,128,8.2)": "T('4×Φ6.6 +0.15/0 通孔，不攻牙。',321,128,8.2)",
    "T('A 口 C0.2；备用 DIN7991 M3×12 沉头。',321,72,8.1,BLUE)": "T('共8个；孔位按上表坐标加工。',321,72,8.1,BLUE)",
    "T('两种沉头均朝 B 面；长度均含头。',321,65,8.1)": "T('两组沉头朝 B 面；A 面孔口均 C0.2。',321,65,8.1)",
    "notes=['6061-T6；1件；约54.4g；本色不氧化。',": "notes=['6061-T6；1件；安装面不得有毛刺。',",
    "T('图号 E6-OMNI-ADP-01  |  R2  |  2026-09-23',18,26,10,b=True)": "T('图号 PD-01  |  R2  |  单位 mm',18,26,10,b=True)",
    "T('仅加工 E6_OMNI_ADAPTER_R2.step；REFERENCE_assembly 为装配参考，不加工。',18,18,8.5)": "T('模型：PD-01_R2.step；H5-H8必须攻M3牙；H1-H4及H9-H16为光通孔。',18,18,8.5)",
    "T('首件下料前核对第 2 页实物条件',402,26,9,BLUE,True,'right');T('1 / 2',402,18,8.5,align='right')": "T('模型与本图配套；螺纹及公差以图纸标注为准',402,26,9,BLUE,True,'right');T('1 / 1',402,18,8.5,align='right')",
}
for old, new in replacements.items():
    if old not in source:
        raise ValueError('Source drawing changed; missing replacement: '+old)
    source = source.replace(old, new)
source += '\nc.save()\n'
# Only reuse trusted local drawing code. No hardware or network access.
exec(compile(source, str(SOURCE_DRAWING), 'exec'), {'__file__': str(SOURCE_DRAWING), '__name__': '__supplier_drawing__'})

shutil.copy2(SOURCE_STEP, DEST / STEP_NAME)
shutil.copy2(PDF, DEST / PDF.name)
assert hashlib.sha256(SOURCE_STEP.read_bytes()).digest() == hashlib.sha256((DEST / STEP_NAME).read_bytes()).digest()

reader = PdfReader(PDF)
assert len(reader.pages) == 1
body = reader.pages[0].extract_text() or ''
metadata = str(reader.metadata)
for token in ('H16', '6H', '22.5', '56', '12.4', '6.4', '6061', '必须攻M3牙'):
    assert token in body, token
for text in (body + metadata, (DEST / STEP_NAME).read_text(encoding='ascii')):
    assert not re.search(r'\bE6\b|OmniPicker|DOBOT|智元|越疆|夹爪|机械臂|采购|装配|Lixiaoyu|vscode', text, re.I)
assert '\ufffd' not in body

with zipfile.ZipFile(ZIP, 'w', zipfile.ZIP_DEFLATED) as z:
    # Explicit allowlist: exactly two core files, no research or auxiliary files.
    z.write(DEST / STEP_NAME, STEP_NAME)
    z.write(DEST / PDF.name, PDF.name)
with zipfile.ZipFile(ZIP) as z:
    assert z.namelist() == [STEP_NAME, PDF.name]
    assert z.testzip() is None

print(json.dumps({'package': str(ZIP), 'files': [STEP_NAME, PDF.name],
                  'pdf_pages': 1, 'geometry_byte_identical_to_R2': True,
                  'application_labels_removed': True}, ensure_ascii=False))
