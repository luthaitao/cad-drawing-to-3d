#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drawing.json 中间格式的公共定义与工具。

统一管线（所有输入格式最终归一化成同一种 drawing.json）：

  DWG --dwg2dxf.py--> DXF --dxf_extract.py--\
                                              >--> drawing.json --analyze_views.py--> rebuild_plan.json
  DXF ------------------ dxf_extract.py ----/
  PDF（矢量/位图） ---- pdf_extract.py -----/

drawing.json 结构：
{
  "source": "E-0101009.dwg",
  "source_format": "dwg|dxf|pdf",
  "units": "mm",
  "layers": {"0": {"kind": "outline", "color": 7, "linetype": "Continuous", "n": 42}, ...},
  "views": [                       # PDF 多页/ DXF 多布局 时一个 source 一个 view
    {
      "name": "Model",
      "bbox": [xmin, ymin, xmax, ymax],
      "polylines": [{"id": 1, "layer": "0", "closed": true, "points": [[x,y],...],
                     "bbox": [...], "area": 12.5}],
      "circles":   [{"id": 2, "layer": "0", "center": [x,y], "radius": 25.0}],
      "arcs":      [{"id": 3, "layer": "0", "center": [x,y], "radius": 25.0,
                     "start_angle": 0.0, "end_angle": 90.0}],
      "lines":     [{"id": 4, "layer": "0", "start": [x,y], "end": [x,y]}],
      "texts":     [{"id": 5, "layer": "DIM", "text": "Ø50", "position": [x,y], "height": 3.5}],
      "dims":      [{"id": 6, "layer": "DIM", "text": "50", "measurement": 50.0,
                     "position": [x,y], "dimtype": "linear"}]
    }
  ]
}
"""
from __future__ import annotations

import json
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# 单位
# ---------------------------------------------------------------------------

# DXF $INSUNITS / IGES 单位码 -> 名称
INSUNITS = {
    0: "unitless", 1: "inch", 2: "ft", 3: "mile", 4: "mm", 5: "cm", 6: "m",
    7: "km", 8: "microinch", 9: "mil", 10: "yard", 11: "angstrom",
    12: "nanometer", 13: "micron", 14: "dm", 15: "dam", 16: "hm",
}

# 非 mm 单位 -> mm 换算（drawing.json 内部一律换算成 mm）
UNIT_TO_MM = {
    "unitless": 1.0,  # 视为 1:1（无单位信息时按 mm 名义值处理）
    "inch": 25.4, "ft": 304.8, "mile": 1609344.0, "mil": 0.0254,
    "mm": 1.0, "cm": 10.0, "m": 1000.0, "km": 1e6, "micron": 0.001,
    "dm": 100.0, "dam": 10000.0, "hm": 100000.0, "angstrom": 1e-7,
    "nanometer": 1e-6, "microinch": 2.54e-5, "yard": 914.4,
}

# ---------------------------------------------------------------------------
# 图层分类（GB 机械制图 + 国内外主流 CAD 常见图层名，匹配大小写不敏感）
# ---------------------------------------------------------------------------

# 顺序即优先级：靠前的规则先命中
LAYER_KIND_RULES = [
    ("frame",  ["DEFPOINTS", "BORDER", "FRAME", "TITLE", "TUAN", "图框", "图纸", "SHEET", "A3", "A4"]),
    ("text",   ["TEXT", "WENZI", "WZ", "NOTE", "ANNOT", "文字", "说明", "TECH", "REQUIRE", "TABLE", "BIAOGE", "BGTK", "GONGYI"]),
    ("dim",    ["DIM", "BZHU", "标注", "尺寸", "DIMENSION", "GD&T", "TOLERANCE"]),
    ("center", ["CENTER", "CENTRE", "CENTERLINE", "CENT", "ZHONGXIN", "中心线", "轴线", "轴心", "PHI", "AXIS", "CEN"]),
    ("hidden", ["HID", "HIDDEN", "YIN", "虚线", "隐藏", "不可见", "DASH"]),
    ("hatch",  ["HATCH", "剖面", "POMIAN", "POM", "SECTION", "SECTIONLINE", "截交"]),
    ("outline", ["OUTLINE", "粗实线", "轮廓", "OBJECT", "SOLID", "VISIBLE", "CONTOUR", "ENTITY", "0", "DEFAULT", "MAIN", "PART", "COMPONENT"]),
]

# 视图分析时默认剔除的图层类型（不参与 3D 重建）
EXCLUDE_KINDS_DEFAULT = {"frame", "text", "dim", "center", "hidden", "hatch"}


def layer_kind(name: str) -> str:
    """按 GB/常见 CAD 约定把图层名映射到分类。"""
    if name is None:
        return "unknown"
    n = name.strip().upper()
    if not n:
        return "unknown"
    for kind, keys in LAYER_KIND_RULES:
        for k in keys:
            if n == k.upper() or n.startswith(k.upper()) or n.endswith(k.upper()):
                return kind
    return "unknown"


# ---------------------------------------------------------------------------
# 尺寸文字解析（GB 标注样式）
# ---------------------------------------------------------------------------

# DXF TEXT 的 %% 转义
_DXF_ESCAPES = [
    ("%%c", "Ø"), ("%%C", "Ø"), ("%%d", "°"), ("%%D", "°"),
    ("%%p", "±"), ("%%P", "±"), ("%%%", "%"),
    ("\\P", " "), ("\\X", " "), ("\\~", " "),
]


def decode_dxf_text(t: str | None) -> str:
    if not t:
        return ""
    s = str(t)
    for a, b in _DXF_ESCAPES:
        s = s.replace(a, b)
    return s.strip()


_NUM = r"([0-9]+(?:\.[0-9]+)?)"


def parse_dimension_text(text: str) -> list[dict]:
    """解析尺寸/技术要求文字，返回识别结果列表。

    识别：Ø/φ/⌀ 直径、R 半径、2×/2- 数量前缀、± 公差、M 螺纹、° 角度。
    例：
      "Ø50"        -> [{"kind": "diameter", "value": 50.0}]
      "2×Ø8"       -> [{"kind": "diameter", "value": 8.0, "count": 2}]
      "R20"        -> [{"kind": "radius", "value": 20.0}]
      "M10-6H"     -> [{"kind": "thread", "value": 10.0}]
      "50±0.1"     -> [{"kind": "linear", "value": 50.0, "tolerance": 0.1}]
    """
    out: list[dict] = []
    if not text:
        return out
    s = decode_dxf_text(text)
    # 数量前缀：2×Ø8 / 2-Ø8 / 4孔Ø8
    count = None
    m = re.match(r"^\s*(\d+)\s*[×xX\*\-]\s*(.*)$", s)
    if m:
        count = int(m.group(1))
        s = m.group(2).strip()
    m = re.match(r"^\s*(\d+)\s*[孔孔位]\s*(.*)$", s)
    if m:
        count = int(m.group(1))
        s = m.group(2).strip()

    def add(kind: str, value: float, **extra):
        item = {"kind": kind, "value": round(float(value), 6)}
        if count is not None:
            item["count"] = count
        item.update(extra)
        out.append(item)

    # 直径 Ø50 / ⌀50 / φ50 / 50%%c（已解码）
    for m in re.finditer(r"[Øφ⌀]\s*" + _NUM, s):
        add("diameter", m.group(1))
    # 半径 R20
    for m in re.finditer(r"[Rr]\s*" + _NUM, s):
        if not re.match(r"^[Rr]\d", s[: m.start()]):  # 避免把 "R20" 前面的字母吞掉
            pass
        add("radius", m.group(1))
    # 螺纹 M10-6H / M10
    for m in re.finditer(r"[Mm]\s*(\d+(?:\.\d+)?)\s*(?:[xX*]\s*(\d+(?:\.\d+)?))?\s*-\s*6[HGg]", s):
        item = {"kind": "thread", "value": round(float(m.group(1)), 6)}
        if m.group(2):
            item["pitch"] = float(m.group(2))
        if count is not None:
            item["count"] = count
        out.append(item)
    for m in re.finditer(r"[Mm]\s*(\d+(?:\.\d+)?)\s*-\s*6[HGg]", s):
        add("thread", m.group(1))
    # 线性 50±0.1 / 50 +0.2/-0.1
    m = re.match(r"^\s*" + _NUM + r"\s*(?:±|\+-)\s*" + _NUM, s)
    if m:
        add("linear", m.group(1), tolerance=float(m.group(2)))
    else:
        m = re.match(r"^\s*" + _NUM + r"\s*\+\s*" + _NUM + r"\s*/\s*-\s*" + _NUM, s)
        if m:
            add("linear", m.group(1), tol_plus=float(m.group(2)), tol_minus=float(m.group(3)))
        else:
            m = re.match(r"^\s*" + _NUM + r"\s*$", s)
            if m:
                add("linear", m.group(1))
    # 角度 30°
    for m in re.finditer(_NUM + r"\s*°", s):
        add("angle", m.group(1))
    return out


# ---------------------------------------------------------------------------
# 几何小工具
# ---------------------------------------------------------------------------

def bbox_of_points(pts) -> list[float]:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return [min(xs), min(ys), max(xs), max(ys)]


def bbox_overlap(a, b) -> float:
    """两个 bbox 的 X 向重叠比例（相对较小的宽度）。"""
    ox = min(a[2], b[2]) - max(a[0], b[0])
    if ox <= 0:
        return 0.0
    w = min(a[2] - a[0], b[2] - b[0])
    return ox / w if w > 1e-9 else 0.0


def bbox_gap_y(a, b) -> float:
    """两个 bbox 的 Y 向间隙（负值=重叠）。"""
    if a[3] < b[1]:
        return b[1] - a[3]
    if b[3] < a[1]:
        return a[1] - b[3]
    return -min(a[3] - a[1], b[3] - b[1])


def polygon_area(pts) -> float:
    """鞋带公式，pts 为闭合点列（首末可重复）。"""
    p = pts if pts[0] == pts[-1] else list(pts) + [pts[0]]
    s = 0.0
    for i in range(len(p) - 1):
        s += p[i][0] * p[i + 1][1] - p[i + 1][0] * p[i][1]
    return abs(s) / 2.0


def dist(p, q) -> float:
    return math.hypot(p[0] - q[0], p[1] - q[1])


# ---------------------------------------------------------------------------
# JSON IO
# ---------------------------------------------------------------------------

def save_json(obj, path: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    return path


def load_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def new_drawing(source: str, source_format: str, units: str = "mm") -> dict:
    return {
        "source": source,
        "source_format": source_format,
        "units": units,
        "layers": {},
        "views": [],
    }


def layer_stat(doc: dict, name: str, kind: str | None = None) -> dict:
    st = doc["layers"].setdefault(name, {"kind": kind or layer_kind(name), "n": 0})
    st["n"] += 1
    return st
