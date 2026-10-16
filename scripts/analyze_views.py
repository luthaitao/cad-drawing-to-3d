#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drawing.json -> rebuild_plan.json（机器初判 + 半自动确认清单）。

流程：
  1. 图层过滤：剔除 frame/text/dim/center/hidden/hatch 层（GB 图层约定，见
     references/cad-layers-and-dims.md）
  2. 轮廓提取：闭合多段线；LINE/ARC 端点相连用 shapely polygonize 组装成环；
     CIRCLE 单独保留（回转特征判据）
  3. 孔洞识别：完全被另一轮廓包含的小轮廓 -> hole
  4. 视图分组：bbox X 重叠聚类成列（左/中/右视图），列内 Y 间隙分行
     （第一角投影：俯视图在主视图正下方，左视图在正右方）
  5. 特征初判：
       - >=2 个同心圆（圆心距<0.5mm 且半径不同）-> revolve 候选（法兰/端盖/轴套）
       - 闭合多边形 -> extrude 候选（板/块/支架）
  6. 深度推断（extrude）：俯视图对位宽度 > 尺寸标注匹配 > 人工（记入 issues）
  7. 输出 rebuild_plan.json：parts[] + issues[] + views[]

半自动约定：本脚本只做「初判」，issues 里每条都必须人工确认或改写后
再执行 rebuild3d.py。plan.json 里每个字段都可手工编辑。

用法：
  python analyze_views.py drawing.json [-o rebuild_plan.json] [--exclude-layer 层1,层2]
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drawing_common import (  # noqa: E402
    EXCLUDE_KINDS_DEFAULT, bbox_gap_y, bbox_of_points, bbox_overlap, dist,
    load_json, parse_dimension_text, save_json,
)

try:
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import polygonize, unary_union
except ImportError:
    sys.stderr.write("缺少依赖：pip install shapely\n")
    sys.exit(1)

# 轮廓最小面积（mm^2）：小于此视为噪点/填充
MIN_AREA = 1.0
# 弧离散化段数
ARC_SEG = 8


def log(msg: str) -> None:
    print("[analyze] %s" % msg)


# ---------------------------------------------------------------------------
# 轮廓提取
# ---------------------------------------------------------------------------

def collect_outline_items(view, exclude_kinds):
    """从 view 中取出参与重建的图元（已按图层过滤）。"""
    polylines, circles, arcs, lines = [], [], [], []
    for p in view.get("polylines", []):
        if p.get("layer_kind") in exclude_kinds:
            continue
        polylines.append(p)
    for c in view.get("circles", []):
        if c.get("layer_kind") in exclude_kinds:
            continue
        circles.append(c)
    for a in view.get("arcs", []):
        if a.get("layer_kind") in exclude_kinds:
            continue
        arcs.append(a)
    for l in view.get("lines", []):
        if l.get("layer_kind") in exclude_kinds:
            continue
        lines.append(l)
    return polylines, circles, arcs, lines


def arc_points(a):
    """ARC 实体离散成点列（含端点）。"""
    cx, cy = a["center"]
    r = a["radius"]
    s = math.radians(a["start_angle"])
    e = math.radians(a["end_angle"])
    if e <= s:
        e += 2 * math.pi
    pts = []
    for i in range(ARC_SEG + 1):
        t = s + (e - s) * i / ARC_SEG
        pts.append((round(cx + r * math.cos(t), 4), round(cy + r * math.sin(t), 4)))
    return pts


# ---------------------------------------------------------------------------
# 端点链分组：CAD 语义下「外轮廓」与「内孔边」是各自闭合的独立线环，
# 不能混进同一次 polygonize（会被当剖分线切开，孔变实心）。
# ---------------------------------------------------------------------------

def _order_ring(segs, tol=0.05):
    """把端点相连的线段排成闭合环。segs = [(p0, p1, full_pts)]。
    返回完整点列（含中间顶点）或 None（不成环）。"""
    if not segs:
        return None
    remaining = list(range(len(segs)))
    chain = [remaining.pop(0)]  # seg 索引链
    head_pts = list(segs[chain[0]][2])
    tail_pts = list(segs[chain[-1]][2])
    progress = True
    while remaining and progress:
        progress = False
        for k, idx in enumerate(remaining):
            p0, p1, full = segs[idx]
            fwd = list(full)
            rev = list(reversed(full))
            if dist(rev[0], tail_pts[-1]) < tol:
                tail_pts.extend(fwd[1:])
                remaining.pop(k)
                progress = True
                break
            if dist(fwd[0], tail_pts[-1]) < tol:
                tail_pts.extend(rev[1:])
                remaining.pop(k)
                progress = True
                break
            if dist(fwd[-1], head_pts[0]) < tol:
                head_pts = fwd[:-1] + head_pts
                remaining.pop(k)
                progress = True
                break
            if dist(rev[-1], head_pts[0]) < tol:
                head_pts = rev[:-1] + head_pts
                remaining.pop(k)
                progress = True
                break
    if remaining or len(tail_pts) < 3:
        return None
    pts = head_pts + tail_pts[1:]
    # 闭合判据：所有 seg 都已排入（remaining 空）。首点与尾点正是首条 seg 的
    # 两端（首尾相邻靠这条边闭合），不要求空间距离 < tol。
    if len(pts) < 3:
        return None
    return pts


def chain_assembled_polylines(polylines, arcs, lines, tol=0.05):
    """端点链分组：返回 [points] 闭合环列表（不含圆，圆单独处理）。"""
    segs = []  # (p0, p1, full_pts)
    for p in polylines:
        if p.get("closed") or len(p.get("points", [])) < 2:
            continue
        pts = [tuple(q) for q in p["points"]]
        segs.append((pts[0], pts[-1], pts))
    for a in arcs:
        pts = [tuple(q) for q in arc_points(a)]
        segs.append((pts[0], pts[-1], pts))
    for l in lines:
        pts = [tuple(l["start"]), tuple(l["end"])]
        segs.append((pts[0], pts[-1], pts))
    if not segs:
        return []
    if len(segs) > 4000:
        log("线段数 %d 超过 4000，端点链分组可能较慢" % len(segs))
    n = len(segs)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            if find(i) == find(j):
                continue
            hit = any(dist(segs[i][a], segs[j][b]) < tol for a in (0, 1) for b in (0, 1))
            if hit:
                parent[find(j)] = find(i)
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(segs[i])
    rings = []
    for g in groups.values():
        ring = _order_ring(g, tol)
        if ring:
            rings.append(ring)
    return rings


def build_profiles(view, exclude_kinds):
    """返回 (profiles, circles)。profile = {"poly": shapely Polygon, "bbox": [...], "layer": str}"""
    polylines, circles, arcs, lines = collect_outline_items(view, exclude_kinds)
    profiles = []

    # 1) 闭合多段线直接成面
    for p in polylines:
        if p.get("closed") and len(p.get("points", [])) >= 3:
            try:
                poly = Polygon(p["points"])
                if not poly.is_valid:
                    poly = poly.buffer(0)
            except Exception:
                continue
            if poly.is_valid and poly.area >= MIN_AREA:
                profiles.append({"poly": poly, "bbox": list(poly.bounds),
                                 "layer": p.get("layer", ""), "n_pts": len(p["points"])})

    # 2) 未闭合的线/弧：端点链分组成环（外轮廓与内孔边各自独立成环）
    rings = chain_assembled_polylines(polylines, arcs, lines)
    for ring in rings:
        try:
            poly = Polygon(ring)
            if not poly.is_valid:
                poly = poly.buffer(0)
        except Exception:
            continue
        if poly.is_valid and poly.area >= MIN_AREA:
            profiles.append({"poly": poly, "bbox": list(poly.bounds),
                             "layer": "assembled", "n_pts": len(ring)})

    # 3) 圆 -> 也作为 profile（可能整体是回转体轮廓/圆柱孔），同时单独留 circles
    for c in circles:
        poly = Point(c["center"]).buffer(c["radius"], resolution=64)
        profiles.append({"poly": poly, "bbox": list(poly.bounds),
                         "layer": c.get("layer", ""), "circle": True,
                         "center": c["center"], "radius": c["radius"]})

    # 3) 圆 -> 也作为 profile（可能整体是回转体轮廓/圆柱孔），同时单独留 circles
    for c in circles:
        poly = Point(c["center"]).buffer(c["radius"], resolution=64)
        profiles.append({"poly": poly, "bbox": list(poly.bounds),
                         "layer": c.get("layer", ""), "circle": True,
                         "center": c["center"], "radius": c["radius"]})

    # 去重（面积与 bbox 都一致的重复轮廓）
    uniq = []
    for pf in profiles:
        dup = False
        for u in uniq:
            if abs(u["poly"].area - pf["poly"].area) < 1e-6 and \
               all(abs(a - b) < 1e-6 for a, b in zip(u["bbox"], pf["bbox"])):
                dup = True
                break
        if not dup:
            uniq.append(pf)
    return uniq, circles


# ---------------------------------------------------------------------------
# 视图分组
# ---------------------------------------------------------------------------

def group_views(profiles):
    """bbox X 重叠 -> 同列；列内 Y 间隙 -> 分行。返回 [[profile, ...], ...]"""
    if not profiles:
        return []
    n = len(profiles)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        a, b = find(i), find(j)
        if a != b:
            parent[b] = a

    for i in range(n):
        for j in range(i + 1, n):
            if bbox_overlap(profiles[i]["bbox"], profiles[j]["bbox"]) > 0.3:
                union(i, j)
    cols = {}
    for i in range(n):
        cols.setdefault(find(i), []).append(profiles[i])

    views = []
    for col in cols.values():
        col.sort(key=lambda p: p["bbox"][1])
        max_dim = max(max(p["bbox"][2] - p["bbox"][0], p["bbox"][3] - p["bbox"][1]) for p in col)
        rows, cur, cur_bbox = [], [], None
        for pf in col:
            if cur and cur_bbox is not None:
                h = max(cur_bbox[3] - cur_bbox[1], pf["bbox"][3] - pf["bbox"][1])
                # 行间距阈值：参考较小视图高度的比例 + 全局尺度下限。
                # 不能用 0.25*max(h)：大俯视图会把阈值抬到 50mm+，
                # 导致与主视图仅隔 10mm 也不拆行（实测法兰图踩过）。
                thr = max(0.15 * h, 0.01 * max_dim)
                if bbox_gap_y(cur_bbox, pf["bbox"]) > thr:
                    rows.append(cur)
                    cur, cur_bbox = [], None
            cur.append(pf)
            cur_bbox = pf["bbox"] if cur_bbox is None else [
                min(cur_bbox[0], pf["bbox"][0]), min(cur_bbox[1], pf["bbox"][1]),
                max(cur_bbox[2], pf["bbox"][2]), max(cur_bbox[3], pf["bbox"][3]),
            ]
        if cur:
            rows.append(cur)
        views.extend(rows)
    return views


def view_bbox(v):
    return [
        min(p["bbox"][0] for p in v), min(p["bbox"][1] for p in v),
        max(p["bbox"][2] for p in v), max(p["bbox"][3] for p in v),
    ]


# ---------------------------------------------------------------------------
# 特征判定
# ---------------------------------------------------------------------------

def concentric_circles(circles, tol=0.5):
    """同心圆分组：圆心距 < tol 的圆归组。返回 [(center, [r1, r2, ...]), ...]"""
    groups = []
    for c in circles:
        placed = False
        for g in groups:
            if math.hypot(c["center"][0] - g["center"][0], c["center"][1] - g["center"][1]) < tol:
                g["radii"].append(c["radius"])
                placed = True
                break
        if not placed:
            groups.append({"center": c["center"], "radii": [c["radius"]]})
    return [g for g in groups if len(g["radii"]) >= 2]


def classify_view(view, circles):
    """返回 (op, confidence, reason)。op in {'revolve', 'extrude', 'unknown'}"""
    groups = concentric_circles(circles)
    if groups:
        return "revolve", "high", "发现 %d 组同心圆（法兰/端盖/轴套类回转特征）" % len(groups)
    n_poly = sum(1 for p in view if not p.get("circle"))
    if n_poly >= 1:
        return "extrude", "medium", "闭合多边形轮廓（板/块/支架类拉伸特征）"
    return "unknown", "low", "无闭合轮廓"


# ---------------------------------------------------------------------------
# 深度推断
# ---------------------------------------------------------------------------

def find_top_view(main_bbox, views):
    """第一角投影：俯视图在主视图正下方（X 大幅重叠且整体 y 更低）。"""
    best, best_ov = None, 0.0
    for v in views:
        vb = view_bbox(v)
        ov = bbox_overlap(main_bbox, vb)
        if ov > 0.55 and vb[3] <= main_bbox[1] + 1e-6:
            if ov > best_ov:
                best, best_ov = v, ov
    return best


def dim_values(drawing):
    """收集 drawing 中全部可解析的尺寸数值（含 DIMENSION measurement 与文字解析）。"""
    vals = []
    for v in drawing.get("views", []):
        for d in v.get("dims", []):
            if d.get("measurement"):
                vals.append(("dim", float(d["measurement"])))
            for it in parse_dimension_text(d.get("text", "")):
                if it["kind"] in ("diameter", "radius", "linear", "thread"):
                    vals.append(("dimtext", float(it["value"])))
        for t in v.get("texts", []):
            for it in parse_dimension_text(t.get("text", "")):
                if it["kind"] in ("diameter", "radius", "linear", "thread"):
                    vals.append(("text", float(it["value"])))
    return vals


def pick_main_view(views):
    """第一角投影选主视图：主视图上方没有其它视图组（俯视图在其下方）。

    不能用「面积最大的组」——法兰/端盖类俯视图是整圆，面积远大于
    矩形主视图（实测法兰图因此选错，shell 变成了圆）。
    规则：
      1. 只有一组 -> 就是它
      2. 候选 = 上方无组的组（列的顶部组）
      3. 多候选取最左列（第一角投影主视图在最左），同列取面积最大
    """
    if len(views) == 1:
        return views[0], []
    tops = []
    for v in views:
        vb = view_bbox(v)
        has_above = any(
            o is not v and view_bbox(o)[1] > vb[3] - 1e-6 and bbox_overlap(vb, view_bbox(o)) > 0.3
            for o in views
        )
        if not has_above:
            tops.append(v)
    if not tops:  # 兜底：都有上方组（异常布局），取最左
        tops = [min(views, key=lambda v: view_bbox(v)[0])]
    main = min(tops, key=lambda v: (round(view_bbox(v)[0]), -sum(p["poly"].area for p in v)))
    return main, [v for v in views if v is not main]


def infer_depth(main_view, main_bbox, other_views, dims):
    """返回 (depth, source, confidence)。depth=None 表示必须人工。"""
    # 1) 俯视图对位宽度
    top = find_top_view(main_bbox, other_views)
    if top is not None:
        tb = view_bbox(top)
        w = tb[3] - tb[1]
        if w > 0.5:
            # 尺寸佐证：找与 w 接近（5%）的标注
            for kind, val in dims:
                if val > 0.5 and abs(val - w) / w < 0.05:
                    return w, "top-view-width(dim=%.3g佐证)" % val, "medium"
            return w, "top-view-width", "medium"
    # 2) 尺寸匹配：俯视图缺失时，主视图若近似等宽等高（方形板）取主视图高度做候选
    mw = main_bbox[2] - main_bbox[0]
    mh = main_bbox[3] - main_bbox[1]
    if mw > 0.5 and abs(mw - mh) / max(mw, mh) < 0.08:
        return mh, "main-view-height(近似等宽高，需人工确认)", "low"
    return None, "manual", "low"


# ---------------------------------------------------------------------------
# profile 组装（shell + holes）
# ---------------------------------------------------------------------------

def split_shell_holes(profiles):
    """按面积/包含关系分 shell 与 holes。返回 (shell_profiles, hole_profiles)。"""
    shells, holes = [], []
    for pf in profiles:
        is_hole = False
        for other in profiles:
            if other is pf:
                continue
            if other["poly"].contains(pf["poly"].buffer(-1e-9)) and other["poly"].area > pf["poly"].area * 1.5:
                is_hole = True
                break
        if is_hole:
            holes.append(pf)
        else:
            shells.append(pf)
    return shells, holes


def poly_to_xy(poly):
    """shapely Polygon -> 外环点列（去闭合点）。

    真实图纸里自交轮廓 buffer(0) 修复后可能返回 MultiPolygon / GeometryCollection
    （样例图未暴露），此处兜底取面积最大的子面，避免 AttributeError: exterior。
    """
    if poly is None or poly.is_empty:
        return []
    if poly.geom_type == "MultiPolygon":
        subs = [g for g in poly.geoms if g.geom_type == "Polygon" and g.area > 0]
        if not subs:
            return []
        poly = max(subs, key=lambda g: g.area)
    if poly.geom_type != "Polygon":
        return []
    shell = [list(p) for p in poly.exterior.coords]
    if len(shell) > 1 and shell[0] == shell[-1]:
        shell = shell[:-1]
    return shell


def main() -> int:
    ap = argparse.ArgumentParser(description="drawing.json -> rebuild_plan.json（机器初判）")
    ap.add_argument("input", help="drawing.json")
    ap.add_argument("-o", "--output", help="rebuild_plan.json（默认 <输入>.plan.json）")
    ap.add_argument("--exclude-layer", help="额外排除的图层名（逗号分隔）")
    ap.add_argument("--keep-kinds", help="覆盖默认排除策略：保留的图层类型（逗号分隔，如 outline,unknown）")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        sys.stderr.write("输入不存在: %s\n" % src)
        return 1
    drawing = load_json(src)
    out = args.output or os.path.splitext(src)[0] + ".plan.json"

    exclude_kinds = set(EXCLUDE_KINDS_DEFAULT)
    if args.keep_kinds:
        exclude_kinds = {"frame", "text", "dim", "center", "hidden", "hatch"} - set(
            k.strip() for k in args.keep_kinds.split(",") if k.strip())
    exclude_layers = set(k.strip() for k in (args.exclude_layer or "").split(",") if k.strip())

    dims = dim_values(drawing)
    plan = {
        "source": drawing.get("source"),
        "units": "mm",
        "generated_by": "analyze_views.py",
        "parts": [],
        "issues": [],
        "views": [],
    }

    for view in drawing.get("views", []):
        if exclude_layers:
            view = dict(view)
            for key in ("polylines", "circles", "arcs", "lines", "texts"):
                view[key] = [it for it in view.get(key, []) if it.get("layer") not in exclude_layers]
        profiles, circles = build_profiles(view, exclude_kinds)
        if not profiles:
            plan["issues"].append({"view": view.get("name"), "code": "no-profile",
                                   "msg": "该视图未提取到闭合轮廓（检查图层过滤是否过严/位图分辨率）"})
            continue
        views = group_views(profiles)
        vb_all = view_bbox(profiles)
        plan["views"].append({
            "view": view.get("name"),
            "n_profiles": len(profiles),
            "n_groups": len(views),
            "bbox": [round(x, 4) for x in vb_all],
        })

        if len(views) <= 1:
            # 单视图零件图：直接按该视图建
            op, conf, reason = classify_view(views[0], circles)
            main_v, others = views[0], []
        else:
            # 主视图 = 顶部组（上方无其它组，第一角投影），最左列优先
            main_v, others = pick_main_view(views)
            op, conf, reason = classify_view(main_v, circles)

        shells, holes = split_shell_holes(main_v)
        main_bbox = view_bbox(main_v)
        if not shells:
            plan["issues"].append({"view": view.get("name"), "code": "no-shell",
                                   "msg": "主视图组没有外壳轮廓（全部被判定为孔洞？）"})
            continue

        if op == "revolve":
            # 回转体：plan 里保存完整截面（shell + holes 均不切半），
            # 切右半与布尔差集由 rebuild3d.py 统一处理。
            # 切半必须与 holes 同步，否则跨轴孔有一半落在截面外，
            # Polygon(shell, holes) 直接无效（实测踩过）。
            shell = max(shells, key=lambda p: p["poly"].area)
            hole_polys = [h["poly"] for h in holes]
            part = {
                "name": "revolve-%d" % (len(plan["parts"]) + 1),
                "op": "revolve",
                "profile": {"shell": poly_to_xy(shell["poly"]),
                            "holes": [poly_to_xy(h) for h in hole_polys]},
                "axis": "Y",
                "angle": 360.0,
                "depth": None,
                "depth_source": "revolve(轮廓即全高)",
                "confidence": conf,
                "reason": reason,
                "source_view": view.get("name"),
                "layer": shell.get("layer", ""),
                "color": None,
            }
        else:
            shell = max(shells, key=lambda p: p["poly"].area)
            depth, dsrc, dconf = infer_depth(main_v, main_bbox, others, dims)
            hole_polys = [h["poly"] for h in holes]
            part = {
                "name": "extrude-%d" % (len(plan["parts"]) + 1),
                "op": "extrude",
                "profile": {"shell": poly_to_xy(shell["poly"]),
                            "holes": [poly_to_xy(h) for h in hole_polys]},
                "axis": "Z",
                "depth": round(depth, 4) if depth else None,
                "depth_source": dsrc,
                "confidence": dconf if op == "extrude" else "low",
                "reason": reason,
                "source_view": view.get("name"),
                "layer": shell.get("layer", ""),
                "color": None,
            }
            if depth is None:
                plan["issues"].append({
                    "view": view.get("name"), "code": "depth-missing",
                    "msg": "拉伸深度无来源：无俯视图对位、无匹配尺寸标注。请在 plan.json 中手工填写 depth"})
            elif dconf == "low":
                plan["issues"].append({
                    "view": view.get("name"), "code": "depth-low-confidence",
                    "msg": "深度来源「%s」置信度低，请人工核对 depth=%.4g" % (dsrc, depth)})
            elif dsrc == "top-view-width":
                plan["issues"].append({
                    "view": view.get("name"), "code": "depth-check-halfview",
                    "msg": "深度取自俯视图宽度 %.4g。若俯视图为对称简化半视图需 ×2，请核对" % depth})
        part["profile"]["holes"] = [poly_to_xy(h) for h in hole_polys]
        plan["parts"].append(part)

    save_json(plan, out)
    log("parts=%d issues=%d" % (len(plan["parts"]), len(plan["issues"])))
    for it in plan["issues"]:
        log("  ! [%s] %s" % (it["code"], it["msg"]))
    log("-> %s" % os.path.abspath(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
