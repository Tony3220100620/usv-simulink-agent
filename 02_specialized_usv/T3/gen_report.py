# -*- coding: utf-8 -*-
"""
根据 MATLAB 脚本生成的 report_data.json 排版输出 docx 仿真实验报告。
用法：python gen_report.py <report_data.json> <output.docx>
"""
import sys
import json

from docx import Document
from docx.shared import Pt, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn


def set_ea(run, name):
    """设置中英文字体（含 EastAsia）。"""
    run.font.name = name
    run._element.rPr.rFonts.set(qn('w:eastAsia'), name)


def add_title(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run(text)
    r.bold = True
    r.font.size = Pt(20)
    set_ea(r, '黑体')


def add_h1(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(14)
    p.paragraph_format.space_after = Pt(6)
    r = p.add_run(text)
    r.bold = True
    r.font.size = Pt(14)
    set_ea(r, '黑体')


def add_p(doc, text, indent=True, size=10.5):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.line_spacing = 1.4
    pf.space_after = Pt(3)
    if indent:
        pf.first_line_indent = Pt(21)
    r = p.add_run(text)
    r.font.size = Pt(size)
    set_ea(r, '宋体')


def add_caption(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run(text)
    r.font.size = Pt(9)
    set_ea(r, '宋体')


def add_table(doc, header, rows, fs=9):
    table = doc.add_table(rows=1, cols=len(header))
    table.style = 'Table Grid'
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, h in enumerate(header):
        p = table.rows[0].cells[i].paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(str(h))
        r.bold = True
        r.font.size = Pt(fs)
        set_ea(r, '宋体')
    for row in rows:
        cells = table.add_row().cells
        for i, v in enumerate(row):
            p = cells[i].paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            r = p.add_run(str(v))
            r.font.size = Pt(fs)
            set_ea(r, '宋体')
    doc.add_paragraph()


def add_table_block(doc, spec, caption):
    add_table(doc, spec['header'], spec['rows'])
    add_caption(doc, caption)


def main():
    json_path, out_path = sys.argv[1], sys.argv[2]
    with open(json_path, 'r', encoding='utf-8') as f:
        d = json.load(f)

    doc = Document()
    sec = doc.sections[0]
    sec.left_margin = Inches(1.05)
    sec.right_margin = Inches(1.05)
    sec.top_margin = Inches(0.95)
    sec.bottom_margin = Inches(0.95)

    style = doc.styles['Normal']
    style.font.name = 'Times New Roman'
    style.font.size = Pt(10.5)
    rpr = style.element.get_or_add_rPr()
    rpr.get_or_add_rFonts().set(qn('w:eastAsia'), '宋体')

    add_title(doc, d['meta']['title'])
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run('模型：%s　｜　脚本：%s　｜　日期：%s' %
                  (d['meta']['model'], d['meta']['script'], d['meta']['date']))
    r.font.size = Pt(9)
    set_ea(r, '宋体')

    # 1 结构
    add_h1(doc, '1  实验目的与模型结构')
    for t in d['structure']['text']:
        add_p(doc, t)
    add_table_block(doc, d['structure'], '表 1  模型模块清单')

    fig_no = 1
    model_fig = d['structure'].get('figure')
    if model_fig:
        doc.add_picture(model_fig['path'], width=Inches(6.1))
        doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
        add_caption(doc, '图 %d  %s' % (fig_no, model_fig['caption']))
        fig_no += 1

    # 2 参数
    add_h1(doc, '2  仿真参数设置')
    add_table_block(doc, d['params_ship'], '表 2  无人船运动模型参数（usv_params.m）')
    add_table_block(doc, d['params_pid'], '表 3  航向 PID 控制器参数')
    add_table_block(doc, d['params_sim'], '表 4  仿真配置')
    for t in d['calibration']['text']:
        add_p(doc, t)

    # 3 曲线
    add_h1(doc, '3  控制响应曲线')
    for fig in d['figures']:
        doc.add_picture(fig['path'], width=Inches(6.1))
        doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
        add_caption(doc, '图 %d  %s' % (fig_no, fig['caption']))
        fig_no += 1

    # 4 数据
    add_h1(doc, '4  各阶段收敛时间与指标数据')
    add_table_block(doc, d['metrics'], '表 5  三阶段控制性能指标')
    add_table_block(doc, d['checks'], '表 6  切换时刻稳定性校验')

    # 5 分析
    add_h1(doc, '5  控制效果分析')
    for t in d['analysis']:
        add_p(doc, t)

    doc.core_properties.title = d['meta']['title']
    doc.core_properties.author = 'USV Simulink 建模助手'
    doc.save(out_path)
    print('[gen_report] 报告已保存: %s' % out_path)


if __name__ == '__main__':
    main()
