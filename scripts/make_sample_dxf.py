#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成标准测试零件图（平面法兰 PN16 DN100，等厚度简化）。

用途：
  1. skill 交付示例素材：用户装好环境后跑本脚本即可验证整条管线
  2. 开发期 smoke test：analyze_views / rebuild3d 的确定性输入

图纸布局（mm，y 向上，第一角投影）：
  主视图（左上 y=100..120）：外矩形 220x20 + 内孔矩形 100x20 + 中心线
  俯视图（正下方 y=-110..110）：同心圆 Ø220 / Ø100 + 4xØ18 螺栓孔 + 中心线
  尺寸层 DIM：Ø220 / Ø100 / 4×Ø18 / 20（厚度，线性标注实体）
  图层：0=轮廓 / CENTER=中心线 / DIM=尺寸（GB 约定）

预期重建结果：revolve，法兰盘 Ø220 x 20，带 Ø100 内孔。
"""
from __future__ import annotations

import os
import sys

try:
    import ezdxf
except ImportError:
    sys.stderr.write("缺少依赖：pip install ezdxf\n")
    sys.exit(1)


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "sample-flange.dxf"
    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = 4  # mm
    msp = doc.modelspace()

    # 图层（GB 机械制图约定）
    for name, color in (("0", 7), ("CENTER", 1), ("DIM", 3)):
        if name not in doc.layers:
            doc.layers.add(name, color=color)

    # ---- 主视图：外矩形 220 x 20（y=100..120） ----
    for a, b in (((-110, 100), (110, 100)), ((110, 100), (110, 120)),
                 ((110, 120), (-110, 120)), ((-110, 120), (-110, 100))):
        msp.add_line(a, b, dxfattribs={"layer": "0"})
    # ---- 主视图：内孔矩形 100 x 20（x=-50..50） ----
    for a, b in (((-50, 100), (50, 100)), ((50, 100), (50, 120)),
                 ((50, 120), (-50, 120)), ((-50, 120), (-50, 100))):
        msp.add_line(a, b, dxfattribs={"layer": "0"})
    # ---- 主视图中心线 ----
    msp.add_line((-130, 110), (130, 110), dxfattribs={"layer": "CENTER"})
    msp.add_line((0, 90), (0, 130), dxfattribs={"layer": "CENTER"})

    # ---- 俯视图：同心圆 Ø220 / Ø100（圆心下移，与主视图 Y 分离，符合投影规范） ----
    CY = -80  # 俯视图圆心 y
    msp.add_circle((0, CY), 110, dxfattribs={"layer": "0"})
    msp.add_circle((0, CY), 50, dxfattribs={"layer": "0"})
    # ---- 4x Ø18 螺栓孔（PCD 180） ----
    for cx, cy in ((90, CY), (-90, CY), (0, CY + 90), (0, CY - 90)):
        msp.add_circle((cx, cy), 9, dxfattribs={"layer": "0"})
    # ---- 俯视图中心线 ----
    msp.add_line((-130, CY), (130, CY), dxfattribs={"layer": "CENTER"})
    msp.add_line((0, CY - 130), (0, CY + 130), dxfattribs={"layer": "CENTER"})

    # ---- 尺寸层 ----
    msp.add_text("Ø220", dxfattribs={"layer": "DIM", "height": 5}).set_placement((120, 115))
    msp.add_text("Ø100", dxfattribs={"layer": "DIM", "height": 5}).set_placement((55, 115))
    msp.add_text("4×Ø18", dxfattribs={"layer": "DIM", "height": 5}).set_placement((95, CY + 20))
    msp.add_text("20", dxfattribs={"layer": "DIM", "height": 5}).set_placement((118, 110))
    # 线性尺寸实体（厚度 20）：从 (-110,100) 到 (110,100) 的 Y 向尺寸改为直接标厚度
    dim = msp.add_linear_dim(
        base=(140, 100), p1=(110, 100), p2=(110, 120),
        dimstyle="EZDXF", dxfattribs={"layer": "DIM"},
    )
    dim.render()

    doc.saveas(out)
    print("[make_sample] -> %s (%d bytes)" % (os.path.abspath(out), os.path.getsize(out)))
    print("[make_sample] 预期：revolve 法兰盘 Ø220x20 带 Ø100 孔；4×Ø18 孔需人工补")
    return 0


if __name__ == "__main__":
    sys.exit(main())
