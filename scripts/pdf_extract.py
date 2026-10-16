#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PDF -> drawing.json（PyMuPDF；矢量路径 + 位图两条分支）。

矢量分支（CAD 直接导出的 PDF）：
  - page.get_drawings() 提取全部路径，bezier 展平成点列
  - 闭合路径 -> polylines；圆度检测（半径方差 < 5%）-> circles
  - page.get_text("dict") 提取文字（含位置/字号）
  - 无线层信息，用「线宽 + 虚线短段」启发式分类图层（见 references/cad-layers-and-dims.md）

位图分支（扫描件/照片 PDF，--bitmap 强制或矢量极少时自动进入）：
  - get_pixmap(dpi) -> cv2 自适应阈值 -> findContours -> approxPolyDP -> 闭合轮廓
  - OCR（可选依赖 rapidocr）识别尺寸数字；无 OCR 时仅出轮廓，尺寸靠人工

单位：PDF 坐标是 pt 且 CAD 可能按任意比例打印，坐标 ≠ mm。
  用「尺寸文字数值 / 页面中最近的标注线段长度」反推全局比例 scale（中位数），
  成功后坐标全部换算成 mm；推不出时 scale=1 并写 scale_warning，交由人工指定。

用法：
  python pdf_extract.py input.pdf [-o drawing.json] [--dpi 300] [--bitmap]
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drawing_common import (  # noqa: E402
    bbox_of_points, bbox_overlap, decode_dxf_text, layer_stat, new_drawing,
    parse_dimension_text, polygon_area, save_json,
)

try:
    import pymupdf as fitz  # PyMuPDF 1.24+ 推荐新 API 名
except ImportError:
    try:
        import fitz  # 旧版回退
    except ImportError:
        sys.stderr.write("缺少依赖：pip install pymupdf\n")
        sys.exit(1)

# 线宽阈值（pt）：CAD 打印轮廓线一般 >= 0.6pt，中心线/尺寸线更细
_WIDTH_OUTLINE = 0.6
# 虚线短段判定（pt）：短于此长度且成群出现 -> center/hidden 虚线
_DASH_MAX = 10.0


def _bezier_flat(p0, p1, p2, p3, n=12):
    """三次贝塞尔展平采样。"""
    pts = []
    for i in range(n + 1):
        t = i / n
        mt = 1 - t
        x = mt ** 3 * p0[0] + 3 * mt * mt * t * p1[0] + 3 * mt * t * t * p2[0] + t ** 3 * p3[0]
        y = mt ** 3 * p0[1] + 3 * mt * mt * t * p1[1] + 3 * mt * t * t * p2[1] + t ** 3 * p3[1]
        pts.append((x, y))
    return pts


def path_to_points(items, close_path: bool):
    """把 PDF path items 展平为点列。"""
    pts: list[tuple[float, float]] = []
    for it in items:
        kind = it[0]
        if kind == "l":
            _, p1, p2 = it
            if not pts:
                pts.append((p1.x, p1.y))
            pts.append((p2.x, p2.y))
        elif kind == "c":
            _, p1, p2, p3, p4 = it
            if not pts:
                pts.append((p1.x, p1.y))
            pts.extend(_bezier_flat((p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y), (p4.x, p4.y)))
        elif kind == "re":
            _, rect, _orient = it
            # Rect 可能未规范化（y0>y1 的"无限矩形"），统一 min/max
            x0, x1 = min(rect.x0, rect.x1), max(rect.x0, rect.x1)
            y0, y1 = min(rect.y0, rect.y1), max(rect.y0, rect.y1)
            # 4 角 + 首点重复成闭合点列（否则 closed 判定失败，矩形走链分组时
            # 单 seg 会出自相交环）
            for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)):
                if not pts or (abs(pts[-1][0] - p[0]) > 1e-6 or abs(pts[-1][1] - p[1]) > 1e-6):
                    pts.append(p)
        elif kind == "qu":
            _, quad = it
            q = [(quad.ul.x, quad.ul.y), (quad.ur.x, quad.ur.y), (quad.lr.x, quad.lr.y),
                 (quad.ll.x, quad.ll.y), (quad.ul.x, quad.ul.y)]
            for p in q:
                if not pts or (abs(pts[-1][0] - p[0]) > 1e-6 or abs(pts[-1][1] - p[1]) > 1e-6):
                    pts.append(p)
    # 去重相邻
    out = []
    for p in pts:
        if not out or abs(out[-1][0] - p[0]) > 1e-6 or abs(out[-1][1] - p[1]) > 1e-6:
            out.append(p)
    return out


def circle_fit(pts, tol=0.05):
    """圆度检测：返回 (center, radius) 或 None。判据：半径相对标准差 < tol 且点数 >= 10。"""
    n = len(pts)
    if n < 10:
        return None
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    rs = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
    r_avg = sum(rs) / n
    if r_avg < 1e-6:
        return None
    sd = (sum((r - r_avg) ** 2 for r in rs) / n) ** 0.5
    return ((cx, cy), r_avg) if sd / r_avg < tol else None


# 标准出图比例（1:N 与 N:1）。几何反推时只在这些离散比例上投票。
_STD_RATIOS = [1 / 100, 1 / 50, 1 / 25, 1 / 20, 1 / 10, 1 / 5, 1 / 2.5, 1 / 2, 1.0,
               2.0, 2.5, 5.0, 10.0]
# 1 pt = 25.4/72 mm；图纸比例 1:N 时 pdf单位->mm 系数 s = (25.4/72)*N
_MM_PER_PT = 25.4 / 72.0
_SCALE_TEXT_RE = re.compile(r"(?<!\d)(\d+(?:\.\d+)?)\s*[:：]\s*(\d+(?:\.\d+)?)(?!\d)")


def scale_from_text(texts):
    """从标题栏/注释文字里读显式比例（如 "1:10"、"比例 1:2"）。

    返回 (scale, method)；scale = pdf 单位->mm 系数。找不到返回 (None, reason)。
    优先取「整条文本就是比例 token」或带 比例/SCALE 字样的，避免把 BOM 行
    或公差里的冒号误判成比例。多条时取众数比例。
    """
    strict, loose = [], []
    for t in texts:
        s = (t.get("text") or "").strip()
        for m in _SCALE_TEXT_RE.finditer(s):
            a, b = float(m.group(1)), float(m.group(2))
            if a <= 0 or b <= 0:
                continue
            ratio = b / a  # 1:N -> N
            if not (0.05 <= ratio <= 500):
                continue
            if s == m.group(0).strip() or re.search(r"比例|SCALE|scale", s):
                strict.append(ratio)
            else:
                loose.append(ratio)
    pick = strict or loose
    if not pick:
        return None, "no-text-scale"
    from collections import Counter
    ratio = Counter(pick).most_common(1)[0][0]
    return _MM_PER_PT * ratio, "text-scale(1:%.4g)" % ratio


def estimate_scale(texts, segments):
    """几何反推 pdf单位->mm 比例（无标题栏比例时的兜底）。

    旧做法「线段长≈尺寸值」只对 1:1 出图成立：1:10 图纸上 l = d*(25.4/72)/10
    ≈ 0.283*d，真实配对全落在 0.5d~2d 窗口之外，密度聚类只能锁到噪声
    （实测该鞍座图误得 1.1759 = 真值 3.5278 的 1/3）。
    改为「按标准比例投票」：对每个候选 s=25.4/72*N，统计有多少个 distinct
    尺寸值 d 能在页面线段里找到 l ≈ d/s 的佐证，取支持数最多者。物理约束强、抗噪。
    """
    dim_values = []
    for t in texts:
        for item in parse_dimension_text(t["text"]):
            if item["kind"] in ("diameter", "radius", "linear", "thread") and item["value"] > 1.0:
                dim_values.append(item["value"])
    if not dim_values or not segments:
        return None, "no-dim-or-segment"
    lens = sorted(l for l in segments if l > 1.0)
    if not lens:
        return None, "no-segment"

    best_s, best_support = None, set()
    for N in _STD_RATIOS:
        s = _MM_PER_PT * N
        support = set()
        for d in dim_values:
            l_pred = d / s
            if l_pred <= 0.5:
                continue
            for l in lens:  # lens 升序，超窗即止
                if abs(l - l_pred) / l_pred < 0.02:
                    support.add(round(d, 3))
                    break
                if l > l_pred * 1.02:
                    break
        if len(support) > len(best_support):
            best_s, best_support = s, support
    if best_s is not None and len(best_support) >= 3:
        return best_s, "ratio-vote(distinct-dim %d)" % len(best_support)
    return None, "no-ratio-support"


def extract_vector_page(page, view_name, scale, flip_y):
    """提取单页矢量路径。scale: pdf单位->mm 系数。"""
    view = {"name": view_name, "bbox": None, "polylines": [], "circles": [],
            "arcs": [], "lines": [], "texts": [], "dims": [], "blocks": []}
    drawings = page.get_drawings()
    dash_paths = 0
    for d in drawings:
        items = d.get("items") or []
        if not items:
            continue
        width = float(d.get("width") or 0.0)
        r = d.get("rect")
        short = bool(r) and (r.width < _DASH_MAX or r.height < _DASH_MAX)
        if width < _WIDTH_OUTLINE and short:
            dash_paths += 1
            layer = "PDF-CENTER"
        elif width < _WIDTH_OUTLINE:
            layer = "PDF-THIN"
        else:
            layer = "PDF-OUTLINE"
        pts = path_to_points(items, bool(d.get("closePath")))
        if len(pts) < 2:
            continue
        if flip_y:
            H = page.rect.height
            pts = [(p[0], H - p[1]) for p in pts]
        if scale != 1.0:
            pts = [(round(x * scale, 4), round(y * scale, 4)) for x, y in pts]
        circ = circle_fit(pts)
        # 注意：draw_circle 的 path closePath=False，圆判定不能依赖 closePath，
        # circle_fit 成功（半径方差 < 5%）即判为圆。
        # pts 在此之前已做过 flip/scale，圆心坐标直接用，不要二次翻转。
        if circ:
            (cx, cy), rad = circ
            view["circles"].append({
                "id": len(view["circles"]) + 1,
                "layer": layer, "layer_kind": "center" if "CENTER" in layer else "outline",
                "center": [round(cx, 4), round(cy, 4)], "radius": round(rad, 4),
            })
            continue
        # 首尾闭合判定（同一 path 的 closePath 或首末点接近）
        closed = bool(d.get("closePath"))
        if not closed and len(pts) > 2:
            dd = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
            closed = dd < 1.0
        view["polylines"].append({
            "id": len(view["polylines"]) + 1,
            "layer": layer,
            "layer_kind": "center" if "CENTER" in layer else ("dim" if "THIN" in layer else "outline"),
            "closed": closed,
            "points": [list(p) for p in pts],
            "bbox": bbox_of_points(pts),
            "area": round(polygon_area(pts), 5) if closed else None,
            "entity": "pdfpath",
        })
        # 单段直线（细线）也镜像到 lines，便于尺寸线段配对
        if len(pts) == 2 and "OUTLINE" not in layer:
            view["lines"].append({
                "id": len(view["lines"]) + 1,
                "layer": layer, "layer_kind": view["polylines"][-1]["layer_kind"],
                "start": list(pts[0]), "end": list(pts[1]),
            })
    return view, dash_paths


def extract_texts_page(page, scale, flip_y):
    texts = []
    d = page.get_text("dict")
    for block in d.get("blocks", []):
        if block.get("type") != 0:
            continue
        for line in block.get("lines", []):
            for span in line.get("spans", []):
                txt = (span.get("text") or "").strip()
                if not txt:
                    continue
                bbox = span.get("bbox") or (0, 0, 0, 0)
                x, y = bbox[0], bbox[1]
                if flip_y:
                    y = page.rect.height - y
                if scale != 1.0:
                    x, y = x * scale, y * scale
                texts.append({
                    "text": txt,
                    "position": [round(x, 4), round(y, 4)],
                    "height": round(float(span.get("size", 3.5)) * scale, 4),
                    "font": span.get("font", ""),
                })
    return texts


def extract_bitmap_page(page, view_name, dpi, ocr=False):
    """位图分支：光栅 -> 轮廓。cv2 可选；OCR(rapidocr) 可选。"""
    try:
        import numpy as np  # noqa: F401
        import cv2
    except ImportError:
        raise RuntimeError("位图分支需要 opencv-python：pip install opencv-python")
    pix = page.get_pixmap(dpi=dpi)
    img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
    if pix.n >= 3:
        gray = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2GRAY)
    else:
        gray = img[:, :, 0]
    bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                               cv2.THRESH_BINARY_INV, 25, 10)
    # 轻度闭运算连接断线
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, hierarchy = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    mm_per_px = 25.4 / dpi
    view = {"name": view_name, "bbox": None, "polylines": [], "circles": [],
            "arcs": [], "lines": [], "texts": [], "dims": [], "blocks": []}
    page_area = pix.width * pix.height
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < page_area * 1e-6:  # 过滤噪点
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.004 * peri, True)
        pts = [(float(p[0][0]) * mm_per_px, float(pix.height - p[0][1]) * mm_per_px) for p in approx]
        circ = circle_fit([(x, y) for x, y in pts], tol=0.08)
        if circ and len(pts) >= 10:
            (cx, cy), rad = circ
            view["circles"].append({
                "id": len(view["circles"]) + 1,
                "layer": "PDF-BITMAP", "layer_kind": "outline",
                "center": [round(cx, 3), round(cy, 3)], "radius": round(rad, 3),
            })
            continue
        view["polylines"].append({
            "id": len(view["polylines"]) + 1,
            "layer": "PDF-BITMAP", "layer_kind": "outline", "closed": True,
            "points": [[round(x, 3), round(y, 3)] for x, y in pts],
            "bbox": bbox_of_points(pts),
            "area": round(polygon_area(pts), 5),
            "entity": "bitmap-contour",
        })

    if ocr:
        try:
            from rapidocr_onnxruntime import RapidOCR
            engine = RapidOCR()
            result, _ = engine(img[:, :, :3] if pix.n >= 3 else gray)
            if result:
                for box, txt, conf in result:
                    xs = [p[0] for p in box]
                    ys = [p[1] for p in box]
                    view["texts"].append({
                        "text": str(txt).strip(),
                        "position": [round(sum(xs) / len(xs) * mm_per_px, 3),
                                     round((pix.height - sum(ys) / len(ys)) * mm_per_px, 3)],
                        "height": round((max(ys) - min(ys)) * mm_per_px, 3),
                        "font": "ocr",
                    })
        except ImportError:
            view.setdefault("warnings", []).append("OCR 未安装（pip install rapidocr-onnxruntime），尺寸文字缺失")
    return view


def extract(pdf_path: str, out_path: str, dpi: int = 300, bitmap: bool = False):
    src = os.path.abspath(pdf_path)
    doc = fitz.open(src)
    drawing = new_drawing(os.path.basename(src), "pdf", "mm")

    # 先粗提第一页文字与线段，推断比例：标题栏明文比例优先，几何投票兜底
    first = doc[0]
    tmp_texts = extract_texts_page(first, 1.0, True)
    scale, method = scale_from_text(tmp_texts)
    if scale is None:
        seg_lens = []
        for d in first.get_drawings():
            pts = path_to_points(d.get("items") or [], False)
            if len(pts) >= 2:
                for i in range(len(pts) - 1):
                    seg_lens.append(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]))
        scale, method = estimate_scale(tmp_texts, seg_lens)
    drawing["scale_hint"] = {
        "pdf_units_per_mm": round(scale, 6) if scale else None,
        "method": method,
        "warning": None if scale else "无法从尺寸标注反推比例，坐标按 1pt=1 名义单位处理，请在 rebuild_plan.json 中人工核对",
    }

    all_pts = []
    for pno in range(len(doc)):
        page = doc[pno]
        vname = "page-%d" % (pno + 1)
        n_draw = len(page.get_drawings())
        use_bitmap = bitmap or (n_draw < 5 and len(page.get_images()) > 0)
        if use_bitmap:
            view = extract_bitmap_page(page, vname, dpi, ocr=True)
        else:
            view, dash = extract_vector_page(page, vname, scale or 1.0, True)
            view["texts"] = extract_texts_page(page, scale or 1.0, True)
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
        drawing["views"].append(view)

    if all_pts:
        for v in drawing["views"]:
            vpts = []
            for key in ("polylines", "circles", "arcs", "lines"):
                for it in v[key]:
                    if key == "polylines":
                        vpts.extend(it["points"])
                    elif key == "circles":
                        c, r = it["center"], it["radius"]
                        vpts.extend([[c[0] - r, c[1] - r], [c[0] + r, c[1] + r]])
                    else:
                        vpts.extend([it["start"], it["end"]])
            if vpts:
                v["bbox"] = [round(x, 4) for x in bbox_of_points(vpts)]

    # 图层统计（PDF 无图层，用启发式层名统计；texts 无 layer 键）
    for v in drawing["views"]:
        for key in ("polylines", "circles", "arcs", "lines"):
            for it in v[key]:
                layer_stat(drawing, it.get("layer", "PDF-UNKNOWN"))
        if v.get("texts"):
            layer_stat(drawing, "PDF-TEXT")

    save_json(drawing, out_path)
    print("[pdf_extract] %s: 页数=%d 比例系数=%s(%s)" % (
        os.path.basename(src), len(doc), scale, method))
    for v in drawing["views"]:
        print("[pdf_extract] %s: 多段线=%d 圆=%d 线=%d 文字=%d" % (
            v["name"], len(v["polylines"]), len(v["circles"]),
            len(v["lines"]), len(v["texts"])))
    print("[pdf_extract] -> %s" % os.path.abspath(out_path))
    return drawing


def main() -> int:
    ap = argparse.ArgumentParser(description="PDF -> drawing.json")
    ap.add_argument("input", help="输入 .pdf 文件")
    ap.add_argument("-o", "--output", help="输出 drawing.json")
    ap.add_argument("--dpi", type=int, default=300, help="位图分支渲染 DPI，默认 300")
    ap.add_argument("--bitmap", action="store_true", help="强制走位图分支（扫描件）")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        sys.stderr.write("输入文件不存在: %s\n" % src)
        return 1
    out = args.output or os.path.splitext(src)[0] + ".drawing.json"
    try:
        extract(src, out, args.dpi, args.bitmap)
        return 0
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write("[pdf_extract] 失败: %s\n" % exc)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
