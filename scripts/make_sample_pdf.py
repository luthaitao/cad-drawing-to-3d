#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成标准测试零件图 PDF（方板 100x60x10，4xØ12 孔，PyMuPDF 矢量绘制）。

与 make_sample_dxf.py 的区别：
  - 走 PDF 矢量分支（get_drawings / get_text）
  - 无线层信息，用线宽区分：轮廓线 1.0pt、中心线/尺寸线 0.3pt
  - 按 1:2 比例打印（mm -> pt 乘 2.8346/2），验证 scale 反推

预期重建结果：extrude 方板 100(X) x 10(Y) x 60(Z)，深度来自俯视图对位宽度。
"""
from __future__ import annotations

import math
import os
import sys

try:
    import fitz  # PyMuPDF
except ImportError:
    sys.stderr.write("缺少依赖：pip install pymupdf\n")
    sys.exit(1)

MM_TO_PT = 72.0 / 25.4
SCALE = 0.5  # 1:2 打印比例（图纸 mm -> 物理 mm），PDF 坐标按物理 mm 画
PT_PER_MM = MM_TO_PT * SCALE
# A3 竖版物理尺寸（mm）
PAGE_W_MM = 420 * SCALE
PAGE_H_MM = 594 * SCALE
# 图纸 y=0 映射到距页顶 200mm 处（保证 y<0 的内容不出页）
Y_BASE_MM = 200.0

# 主视图（左上）：矩形 100 x 10，y_pdf 100..110
# 俯视图（正下方）：矩形 100 x 60（y_pdf -30..30）+ 4xØ12 孔


def mm(x: float, y: float) -> tuple[float, float]:
    """图纸 mm 坐标 -> PDF pt 坐标（y 翻转为 PDF 的 y 向下，偏移保证在页面内）。"""
    return (x * PT_PER_MM, (Y_BASE_MM - y * SCALE) * MM_TO_PT)


def rect_mm(x0: float, y0: float, x1: float, y1: float) -> fitz.Rect:
    """图纸 mm 矩形 -> fitz.Rect。

    必须用 4 参数构造：Rect(Point, Point) 两点构造不规范化 y
    （图纸上方 y 大 -> pt y 小，y0>y1 会得到"无限矩形"，实测被压扁成线）。
    """
    a = mm(x0, y0)
    b = mm(x1, y1)
    return fitz.Rect(min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]))


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "sample-plate.pdf"
    doc = fitz.open()
    page = doc.new_page(width=PAGE_W_MM * MM_TO_PT, height=PAGE_H_MM * MM_TO_PT)
    W_OUT = 1.0  # 轮廓线宽 pt
    W_THIN = 0.3  # 细线 pt

    # ---- 主视图：矩形 x -50..50, y 100..110 ----
    page.draw_rect(rect_mm(-50, 100, 50, 110), color=(0, 0, 0), width=W_OUT)
    # 主视图中心线
    a, b = mm(-65, 105), mm(65, 105)
    page.draw_line(a, b, color=(0.5, 0.5, 0.5), width=W_THIN)
    a, b = mm(0, 92), mm(0, 118)
    page.draw_line(a, b, color=(0.5, 0.5, 0.5), width=W_THIN)

    # ---- 俯视图：矩形 x -50..50, y -30..30 + 4xØ12 孔 @ (±35, ±15) ----
    page.draw_rect(rect_mm(-50, -30, 50, 30), color=(0, 0, 0), width=W_OUT)
    for cx, cy in ((35, 15), (-35, 15), (35, -15), (-35, -15)):
        c = mm(cx, cy)
        r = 6 * PT_PER_MM
        page.draw_circle(c, r, color=(0, 0, 0), width=W_OUT)
    a, b = mm(-65, 0), mm(65, 0)
    page.draw_line(a, b, color=(0.5, 0.5, 0.5), width=W_THIN)
    a, b = mm(0, -45), mm(0, 45)
    page.draw_line(a, b, color=(0.5, 0.5, 0.5), width=W_THIN)

    # ---- 尺寸层：文字 + 标注线段（供 scale 反推） ----
    def dim_text(x, y, s):
        page.insert_text(mm(x, y), s, fontsize=5 * PT_PER_MM * 1.4, color=(0, 0, 0))

    def dim_line(x0, y0, x1, y1):
        a, b = mm(x0, y0), mm(x1, y1)
        page.draw_line(a, b, color=(0, 0, 0), width=W_THIN)

    dim_text(60, 105, "100")     # 主视图宽
    dim_line(50, 96, -50, 96)    # 100mm 标注线段
    dim_text(60, 20, "60")       # 俯视图宽
    dim_line(50, 36, -50, 36)    # 60mm 标注线段
    dim_text(5, 122, "10")       # 厚度（x 取正，PyMuPDF 负 x 文字会丢）
    dim_line(-56, 110, -56, 100)  # 10mm 标注线段
    dim_text(20, -38, "4xØ12")
    dim_line(35, -15, 35, 15)    # Ø12 孔直径标注线段(24mm)

    doc.save(out)
    print("[make_sample_pdf] -> %s (%d bytes)" % (os.path.abspath(out), os.path.getsize(out)))
    print("[make_sample_pdf] 打印比例 1:2，预期 scale≈%.4f pt/mm；预期 extrude 方板 100x10x60" % (1 / PT_PER_MM))
    return 0


if __name__ == "__main__":
    sys.exit(main())
