#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DXF -> drawing.json（ezdxf 解析，含 DWG 转出的 DXF）。

特点：
  - recover 模式读取（LibreDWG/ODA 转出的 DXF 常带小瑕疵，普通模式会整文件拒绝）
  - 编码自动探测：$DWGCODEPAGE=ANSI_936 时按 GBK 解文字，避免中文乱码
  - bulge 圆弧段自动离散化（弦高 0.02mm），粗实线层不丢弧
  - 单位统一换算为 mm
  - 尺寸实体（DIMENSION）提取测量值 + 覆盖文本

用法：
  python dxf_extract.py input.dxf [-o drawing.json] [--flatten 0.02]
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drawing_common import (  # noqa: E402
    INSUNITS, UNIT_TO_MM, bbox_of_points, decode_dxf_text, layer_stat,
    layer_kind, new_drawing, polygon_area, save_json,
)

try:
    import ezdxf
    from ezdxf import path as ezpath
    from ezdxf import recover
except ImportError:
    sys.stderr.write("缺少依赖：pip install ezdxf\n")
    sys.exit(1)

# LibreDWG 转出的 DXF 中 XRECORD（扩展记录）常带非法 XDATA（混入 <1000 组码），
# ezdxf 校验器会整文件拒收。XRECORD 对几何提取无用，跳过其结构校验。
if hasattr(recover, "EXCLUDE_STRUCTURE_CHECK"):
    recover.EXCLUDE_STRUCTURE_CHECK = set(recover.EXCLUDE_STRUCTURE_CHECK) | {"XRECORD"}


def _patch_lenient_entity_linker() -> None:
    """LibreDWG 转出的 DXF 常见 INSERT/POLYLINE 子实体链断裂
    （attribs_follow=1 但无 ATTRIB、POLYLINE 后无 VERTEX），
    ezdxf 严格 linker 会整文件拒收。替换为容错版：链断裂时断开主实体，
    当前实体按独立实体继续处理。对结构正常的文件行为与原版完全一致。"""
    try:
        from ezdxf.sections import entities as _ent_mod
        from ezdxf.entities import subentity as _sub_mod
    except Exception:  # noqa: BLE001
        return
    linked = getattr(_sub_mod, "LINKED_ENTITIES", None) or {"POLYLINE": "VERTEX", "INSERT": "ATTRIB"}

    def lenient_factory():
        state = {"main": None, "expected": ""}

        def linker_(entity):
            dxftype = entity.dxftype()
            if state["main"] is not None:
                if dxftype == "SEQEND":
                    state["main"].link_seqend(entity)
                    state["main"] = None
                    return True
                if dxftype == state["expected"]:
                    state["main"].link_entity(entity)
                    return True
                # 链断裂：放弃该主实体的子实体链接，继续评估当前实体
                state["main"] = None
                state["expected"] = ""
            if dxftype in linked:
                if dxftype == "INSERT" and not entity.dxf.get("attribs_follow", 0):
                    return False
                state["main"] = entity
                state["expected"] = linked[dxftype]
                return False
            return False

        return linker_

    _ent_mod.entity_linker = lenient_factory
    _sub_mod.entity_linker = lenient_factory


_patch_lenient_entity_linker()


def detect_encoding(path: str) -> str | None:
    """预读 header 判断文字编码。中文 CAD 图纸常见 ANSI_936(GBK)。"""
    try:
        with open(path, "rb") as f:
            head = f.read(1 << 20)
        for enc in ("utf-8", "latin-1"):
            try:
                txt = head.decode(enc, errors="ignore")
            except Exception:
                continue
            if "$DWGCODEPAGE" in txt:
                idx = txt.index("$DWGCODEPAGE")
                seg = txt[idx: idx + 120]
                if "936" in seg or "GB" in seg.upper():
                    return "gbk"
                return None
    except Exception:
        pass
    return None


def open_doc(path: str, encoding: str | None):
    """先标准模式（ezdxf 自动探测 $DWGCODEPAGE 编码），失败再 recover 恢复模式。

    注意：ezdxf 1.4+ 的 recover.readfile 不接受 encoding 参数（内部自行处理），
    中文 DXF 若走 recover 路径且乱码，尺寸数字（ASCII）不受影响，可降级使用。
    """
    first_err = None
    try:
        return ezdxf.readfile(path, encoding=encoding), None
    except Exception as exc:  # noqa: BLE001
        first_err = exc
    try:
        return recover.readfile(path)
    except Exception:  # noqa: BLE001
        raise first_err


def flatten_entity(e, flatten: float = 0.02):
    """把 ezdxf 实体离散成点列（支持 LWPOLYLINE/POLYLINE/ARC/CIRCLE/SPLINE/ELLIPSE/LINE）。"""
    try:
        p = ezpath.make_path(e)
        pts = [(round(x, 4), round(y, 4)) for x, y in p.flattening(flatten)]
        # 去重相邻点
        out = [pts[0]] if pts else []
        for q in pts[1:]:
            if abs(q[0] - out[-1][0]) > 1e-6 or abs(q[1] - out[-1][1]) > 1e-6:
                out.append(q)
        return out
    except Exception:
        return []


def scale_pts(pts, k: float):
    return [(round(x * k, 5), round(y * k, 5)) for x, y in pts]


def extract(dxf_path: str, out_path: str, flatten: float = 0.02) -> dict:
    src = os.path.abspath(dxf_path)
    enc = detect_encoding(src)
    doc, auditor = open_doc(src, enc)

    # 单位
    insunits = int(doc.header.get("$INSUNITS", 4) or 4)
    units = INSUNITS.get(insunits, "unitless")
    k = UNIT_TO_MM.get(units, 1.0)
    if enc:
        doc_encoding = enc

    drawing = new_drawing(os.path.basename(src), "dxf", "mm")
    drawing["dxf_units"] = units
    drawing["dxf_encoding"] = enc or doc.encoding if hasattr(doc, "encoding") else enc
    drawing["scale_to_mm"] = k
    drawing["dxf_version"] = str(doc.dxfversion)

    view = {"name": "Model", "bbox": None, "polylines": [], "circles": [],
            "arcs": [], "lines": [], "texts": [], "dims": [], "blocks": []}

    msp = doc.modelspace()
    for e in msp:
        dxftype = e.dxftype()
        layer = e.dxf.layer if e.dxf.hasattr("layer") else "0"
        kind = layer_kind(layer)
        layer_stat(drawing, layer, kind)

        if dxftype in ("LWPOLYLINE", "POLYLINE"):
            pts = flatten_entity(e, flatten)
            if len(pts) < 2:
                continue
            pts = scale_pts(pts, k)
            closed = bool(e.closed) if e.dxf.hasattr("closed") else False
            # LibreDWG 转出的闭合标志偶尔丢失：用首末距离兜底
            if not closed and len(pts) > 2:
                d = ((pts[0][0] - pts[-1][0]) ** 2 + (pts[0][1] - pts[-1][1]) ** 2) ** 0.5
                closed = d < max(0.05, (bbox_of_points(pts)[2] - bbox_of_points(pts)[0]) * 1e-4)
            view["polylines"].append({
                "id": len(view["polylines"]) + 1,
                "layer": layer, "layer_kind": kind, "closed": closed,
                "points": pts, "bbox": bbox_of_points(pts),
                "area": round(polygon_area(pts), 5) if closed else None,
                "entity": dxftype,
            })
        elif dxftype == "CIRCLE":
            c = e.dxf.center
            r = float(e.dxf.radius) * k
            view["circles"].append({
                "id": len(view["circles"]) + 1,
                "layer": layer, "layer_kind": kind,
                "center": [round(c.x * k, 5), round(c.y * k, 5)], "radius": round(r, 5),
            })
        elif dxftype == "ARC":
            c = e.dxf.center
            view["arcs"].append({
                "id": len(view["arcs"]) + 1,
                "layer": layer, "layer_kind": kind,
                "center": [round(c.x * k, 5), round(c.y * k, 5)],
                "radius": round(float(e.dxf.radius) * k, 5),
                "start_angle": round(float(e.dxf.start_angle), 4),
                "end_angle": round(float(e.dxf.end_angle), 4),
            })
        elif dxftype == "LINE":
            s, t = e.dxf.start, e.dxf.end
            view["lines"].append({
                "id": len(view["lines"]) + 1,
                "layer": layer, "layer_kind": kind,
                "start": [round(s.x * k, 5), round(s.y * k, 5)],
                "end": [round(t.x * k, 5), round(t.y * k, 5)],
            })
        elif dxftype in ("TEXT", "MTEXT"):
            if dxftype == "TEXT":
                raw = e.dxf.text or ""
                pos = e.dxf.insert
                height = float(getattr(e.dxf, "height", 3.5) or 3.5)
            else:
                raw = e.plain_text() if hasattr(e, "plain_text") else (e.text or "")
                pos = e.dxf.insert
                height = float(getattr(e.dxf, "char_height", 3.5) or 3.5)
            view["texts"].append({
                "id": len(view["texts"]) + 1,
                "layer": layer, "layer_kind": kind,
                "text": decode_dxf_text(raw), "raw": raw,
                "position": [round(pos.x * k, 5), round(pos.y * k, 5)],
                "height": round(height * k, 4),
            })
        elif dxftype == "DIMENSION":
            raw = ""
            if e.dxf.hasattr("text"):
                raw = e.dxf.text or ""
            meas = None
            try:
                meas = float(e.get_measurement())
            except Exception:
                meas = None
            dimtype = e.dimtype if hasattr(e, "dimtype") else 0
            pos = None
            try:
                # DIMENSION 文字位置
                if e.dxf.hasattr("text_midpoint"):
                    p = e.dxf.text_midpoint
                    pos = [round(p.x * k, 5), round(p.y * k, 5)]
            except Exception:
                pos = None
            view["dims"].append({
                "id": len(view["dims"]) + 1,
                "layer": layer, "layer_kind": kind,
                "text": decode_dxf_text(raw) if raw else "",
                "measurement": round(meas * k, 5) if meas is not None else None,
                "dimtype": int(dimtype),
                "position": pos,
            })
        elif dxftype == "INSERT":
            view["blocks"].append({
                "id": len(view["blocks"]) + 1,
                "layer": layer, "name": e.dxf.name,
                "insert": [round(e.dxf.insert.x * k, 5), round(e.dxf.insert.y * k, 5)],
                "scale": [float(getattr(e.dxf, "xscale", 1) or 1), float(getattr(e.dxf, "yscale", 1) or 1)],
                "rotation": float(getattr(e.dxf, "rotation", 0) or 0),
            })
        # HATCH/其它实体只统计图层，不取边界

    all_pts = []
    for key in ("polylines", "circles", "arcs", "lines"):
        for it in view[key]:
            if key == "polylines":
                all_pts.extend(it["points"])
            elif key == "circles":
                c, r = it["center"], it["radius"]
                all_pts.extend([[c[0] - r, c[1] - r], [c[0] + r, c[1] + r]])
            elif key == "arcs":
                c, r = it["center"], it["radius"]
                all_pts.extend([[c[0] - r, c[1] - r], [c[0] + r, c[1] + r]])
            else:
                all_pts.extend([it["start"], it["end"]])
    if all_pts:
        view["bbox"] = [round(v, 4) for v in bbox_of_points(all_pts)]

    drawing["views"].append(view)
    if auditor is not None:
        drawing["dxf_recover_errors"] = len(auditor.errors)
        drawing["dxf_recover_fixes"] = len(auditor.fixes)

    save_json(drawing, out_path)
    n_pl = len(view["polylines"])
    n_cl = sum(1 for p in view["polylines"] if p["closed"])
    print("[dxf_extract] %s: 版本=%s 单位=%s(->mm x%g) 编码=%s" % (
        os.path.basename(src), drawing["dxf_version"], units, k, enc or "默认"))
    print("[dxf_extract] 多段线=%d(闭合%d) 圆=%d 弧=%d 线=%d 文字=%d 尺寸=%d 块=%d" % (
        n_pl, n_cl, len(view["circles"]), len(view["arcs"]), len(view["lines"]),
        len(view["texts"]), len(view["dims"]), len(view["blocks"])))
    print("[dxf_extract] bbox=%s" % view["bbox"])
    print("[dxf_extract] -> %s" % os.path.abspath(out_path))
    return drawing


def main() -> int:
    ap = argparse.ArgumentParser(description="DXF -> drawing.json")
    ap.add_argument("input", help="输入 .dxf 文件")
    ap.add_argument("-o", "--output", help="输出 drawing.json（默认 <输入>.drawing.json）")
    ap.add_argument("--flatten", type=float, default=0.02, help="弧离散化弦高(mm)，默认 0.02")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        sys.stderr.write("输入文件不存在: %s\n" % src)
        return 1
    out = args.output or os.path.splitext(src)[0] + ".drawing.json"
    try:
        extract(src, out, args.flatten)
        return 0
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write("[dxf_extract] 失败: %s\n" % exc)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
