#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rebuild_plan.json -> GLB + STL（trimesh 重建，含校验）。

每个 part：
  extrude: shapely Polygon(shell, holes) -> trimesh.creation.extrude_polygon(沿 +Z)
  revolve : 截面取 x>=0 半轮廓 -> trimesh.creation.revolve(绕 Y 轴, 360°)

输出：
  <name>.glb   主格式（three.js 双击加载，带颜色，单位 mm，导入时乘 0.001）
  <name>.stl   兼容格式（3D 打印 / 所有 CAD 软件，无颜色）
  <name>.report.json  校验报告（水密性/绕序/包围盒/三角面数/问题清单）

用法：
  python rebuild3d.py rebuild_plan.json [-o out.glb] [--stl out.stl] [--report report.json]
退出码：0 成功（允许带 warning）；1 失败
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drawing_common import load_json, save_json  # noqa: E402

try:
    import numpy as np
    import trimesh
    from shapely.geometry import Polygon
except ImportError:
    sys.stderr.write("缺少依赖：pip install trimesh shapely numpy\n")
    sys.exit(1)

# 默认色板（按 part 顺序循环），GB 常见铸钢/不锈钢色感
DEFAULT_COLORS = ["#c0c0c0", "#c8a165", "#8fa3b8", "#a8b5a0", "#b89ac0", "#d0b8a0"]

# 最小三角面数：低于此视为空几何
MIN_FACES = 4


def log(msg: str) -> None:
    print("[rebuild3d] %s" % msg)


def hex_to_rgba(hex_color):
    if not hex_color:
        return None
    h = hex_color.lstrip("#")
    if len(h) == 6:
        r, g, b = (int(h[i:i + 2], 16) for i in (0, 2, 4))
        return [r, g, b, 255]
    return None


def build_extrude(part):
    prof = part["profile"]
    shell = prof["shell"]
    holes = prof.get("holes") or []
    depth = part.get("depth")
    if not depth or depth <= 0:
        raise ValueError("extrude 缺少有效 depth（当前 %r）" % depth)
    poly = Polygon(shell, holes)
    if not poly.is_valid:
        poly = poly.buffer(0)
    # buffer(0) 修复自交后可能返回 MultiPolygon（真实图纸易触发），
    # 取面积最大子面，避免 extrude_polygon 内部访问 .exterior 抛 AttributeError。
    if poly.geom_type == "MultiPolygon":
        subs = [g for g in poly.geoms if g.geom_type == "Polygon" and g.area > 0]
        if not subs:
            raise ValueError("extrude 轮廓无效或面积为 0")
        poly = max(subs, key=lambda g: g.area)
    if poly.is_empty or poly.area <= 0:
        raise ValueError("extrude 轮廓无效或面积为 0")
    mesh = trimesh.creation.extrude_polygon(poly, float(depth))
    # extrude_polygon 恒沿 +Z 拉伸。axis 指定实际拉伸方向：
    #   Z(默认) 截面在 X-Y；Y 截面在 X-Z（板的平面轮廓，垂直孔天然成圆）；
    #   X 截面在 Y-Z。旋转后截面第二坐标按右手系映射。
    axis = (part.get("axis") or "Z").upper()
    if axis == "Y":
        mesh.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))
    elif axis == "X":
        mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    elif axis != "Z":
        raise ValueError("extrude 不支持 axis=%r" % axis)
    return mesh


def _ring_points(ring):
    """shapely LinearRing -> 去闭合点/连续重复点/x=0 微移的点列（revolve 安全）。"""
    pts = [list(p) for p in ring.coords]
    if pts and pts[0] == pts[-1]:
        pts = pts[:-1]
    dedup = []
    for q in pts:
        if not dedup or abs(q[0] - dedup[-1][0]) > 1e-9 or abs(q[1] - dedup[-1][1]) > 1e-9:
            dedup.append(q)
    # x=0 的顶点微移：trimesh revolve 对零半径顶点产生退化三角形，毁掉水密性
    return [[(x if x > 1e-6 else 1e-6), y] for x, y in dedup]


def _signed_area(pts):
    """鞋带公式带符号：>0 逆时针，<0 顺时针。"""
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def _ensure_ccw(pts):
    """trimesh revolve 要求闭合母线逆时针（否则法线朝内、体积为负）。"""
    return pts if _signed_area(pts) >= 0 else list(reversed(pts))


def build_revolve(part):
    from shapely.geometry import box as sbox

    prof = part["profile"]
    shell = prof["shell"]
    holes = prof.get("holes") or []
    angle = math.radians(float(part.get("angle") or 360.0))
    sections = int(part.get("sections") or 128)

    # 1) 组装完整截面（shell + holes，此时仍是全轮廓，未切半）
    poly = Polygon(shell, holes)
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty or poly.area <= 0:
        raise ValueError("revolve 截面无效或面积为 0")

    # 2) 切右半（revolve 绕 Y 轴，截面必须 x>=0）。
    #    holes 随 poly 一起切，跨轴孔的右半成为截面的 interiors。
    minx, miny, maxx, maxy = poly.bounds
    if maxx <= 0:
        raise ValueError("revolve 截面全部位于 x<=0，无法绕 Y 轴旋转（检查轴线方向）")
    half = poly.intersection(sbox(0.0, miny - 1.0, maxx + 1.0, maxy + 1.0))
    geoms = list(half.geoms) if half.geom_type in ("MultiPolygon", "GeometryCollection") else [half]
    polys = [g for g in geoms if g.geom_type == "Polygon" and g.area > 0]
    if not polys:
        raise ValueError("revolve 半截面为空")
    half = max(polys, key=lambda g: g.area)

    # 3) revolve 外轮廓。
    #    注意：trimesh 的 cap=True 只在部分旋转(0<angle<2pi)时封盖，整圆旋转
    #    时端盖逻辑被跳过。整圆的水密性来自「闭合母线」——各边（底/顶/侧/轴）
    #    的扫掠面天然拼成封闭曲面，所以必须把截面闭合成环（首尾点相同）。
    ext = _ensure_ccw(_ring_points(half.exterior))
    if len(ext) < 3:
        raise ValueError("revolve 半截面点数不足 3")
    if ext[0] != ext[-1]:
        ext = ext + [ext[0]]
    mesh = trimesh.creation.revolve(np.asarray(ext, dtype=np.float64), angle,
                                    cap=True, sections=sections)

    # 4) interiors（切半后的孔）逐个布尔差集；manifold3d 后端
    for interior in half.interiors:
        ring = _ring_points(interior)
        if len(ring) < 3:
            continue
        if ring[0] != ring[-1]:
            ring = ring + [ring[0]]
        hole_mesh = trimesh.creation.revolve(np.asarray(ring, dtype=np.float64), angle,
                                             cap=True, sections=sections)
        try:
            mesh = mesh.difference(hole_mesh)
        except Exception as exc:  # noqa: BLE001
            raise ValueError("布尔差集失败（孔洞）: %s" % exc)
    # 布尔差集后兜底：volume<0 说明法线朝内，翻转
    if mesh.is_watertight and mesh.volume < 0:
        mesh.invert()
    return mesh


def check_mesh(mesh, name, report):
    problems = []
    info = {
        "name": name,
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "volume": float(mesh.volume) if mesh.is_watertight else None,
        "bbox": [round(float(v), 4) for v in mesh.bounds.reshape(-1)],
    }
    if len(mesh.faces) < MIN_FACES:
        problems.append("三角面数 %d < %d（空几何）" % (len(mesh.faces), MIN_FACES))
    if not np.isfinite(mesh.vertices).all():
        problems.append("顶点含 NaN/Inf")
    b = mesh.bounds.reshape(-1)
    if np.isfinite(b).all() and (np.abs(b) > 1e6).any():
        problems.append("包围盒出现飞点（>1e6）: %s" % b.tolist())
    if not mesh.is_watertight:
        problems.append("网格不水密（孔洞/开裂，三维打印或布尔会出问题）")
    info["problems"] = problems
    report["meshes"].append(info)
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description="rebuild_plan.json -> GLB + STL")
    ap.add_argument("input", help="rebuild_plan.json")
    ap.add_argument("-o", "--output", help="输出 .glb（默认 <输入>.glb）")
    ap.add_argument("--stl", help="输出 .stl（默认与 glb 同名；--no-stl 跳过）")
    ap.add_argument("--no-stl", action="store_true", help="只出 GLB")
    ap.add_argument("--report", help="校验报告 JSON（默认 <输入>.report.json）")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        sys.stderr.write("输入不存在: %s\n" % src)
        return 1
    plan = load_json(src)
    stem = os.path.splitext(src)[0]
    glb_path = os.path.abspath(args.output or (stem + ".glb"))
    stl_path = os.path.abspath(args.stl or (stem + ".stl")) if not args.no_stl else None
    report_path = os.path.abspath(args.report or (stem + ".report.json"))

    report = {"source": plan.get("source"), "meshes": [], "plan_issues": plan.get("issues", [])}
    scene = trimesh.Scene()
    meshes = []
    n_fail = 0

    for i, part in enumerate(plan.get("parts", [])):
        name = part.get("name") or "part-%d" % (i + 1)
        op = part.get("op")
        color = hex_to_rgba(part.get("color")) or hex_to_rgba(DEFAULT_COLORS[i % len(DEFAULT_COLORS)])
        try:
            if op == "extrude":
                mesh = build_extrude(part)
            elif op == "revolve":
                mesh = build_revolve(part)
            else:
                raise ValueError("未知 op: %r" % op)
        except Exception as exc:  # noqa: BLE001
            log("!! %s 重建失败: %s" % (name, exc))
            report["meshes"].append({"name": name, "op": op, "fatal": str(exc), "problems": ["重建失败"]})
            n_fail += 1
            continue
        mesh.metadata["part_name"] = name
        mesh.metadata["op"] = op
        # 可选 per-part 平移（多零件装配时每个件在 Z 上的定位不同；
        # extrude 恒从 Z=0 起拉伸，需要靠 translate 摆位）。
        tr = part.get("translate")
        if tr:
            mesh.apply_translation([float(tr[0]), float(tr[1]), float(tr[2])])
        mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=np.tile(
            np.asarray(color, dtype=np.uint8), (len(mesh.faces), 1)))
        problems = check_mesh(mesh, name, report)
        for p in problems:
            log("  ! %s: %s" % (name, p))
        scene.add_geometry(mesh, geom_name=name)
        meshes.append(mesh)

    if not meshes:
        log("没有任何可导出的网格（parts=%d）" % len(plan.get("parts", [])))
        save_json(report, report_path)
        return 1

    # 归一化：图纸坐标可能带任意偏移（PDF flip 常数、DXF 布局坐标）。
    # 默认把模型平移为 XZ 居中、Y 最小贴 0（站在地面上），幂等。
    # plan["normalize"] = "none" 可关闭。
    norm = plan.get("normalize", "center_xz_ground_y")
    if norm != "none" and meshes:
        # mesh.bounds 是 (2,3)：[[xmin,ymin,zmin],[xmax,ymax,zmax]]
        all_bounds = np.vstack([m.bounds for m in meshes]).reshape(-1, 2, 3)
        mins = all_bounds[:, 0, :].min(axis=0)
        maxs = all_bounds[:, 1, :].max(axis=0)
        cx = (mins[0] + maxs[0]) / 2.0
        cz = (mins[2] + maxs[2]) / 2.0
        offset = np.array([-cx, -mins[1], -cz])
        for m in meshes:
            m.apply_translation(offset)
        log("归一化：平移 %s（XZ 居中，Y 贴地）" % np.round(offset, 3).tolist())

    # GLB（保留多 mesh + 名称 + 颜色）
    glb_bytes = scene.export(file_type="glb")
    with open(glb_path, "wb") as f:
        f.write(glb_bytes)
    log("GLB -> %s (%d bytes)" % (glb_path, len(glb_bytes)))

    # STL（concatenate 成单网格；STL 无颜色）
    if stl_path:
        if len(meshes) == 1:
            merged = meshes[0]
        else:
            merged = trimesh.util.concatenate(meshes)
        stl_bytes = merged.export(file_type="stl")
        with open(stl_path, "wb") as f:
            f.write(stl_bytes)
        log("STL -> %s (%d bytes)" % (stl_path, len(stl_bytes)))

    # 汇总
    total_faces = sum(len(m.faces) for m in meshes)
    total_verts = sum(len(m.vertices) for m in meshes)
    watertight = all(m.is_watertight for m in meshes)
    all_bounds = np.vstack([m.bounds for m in meshes])
    report["summary"] = {
        "n_parts": len(meshes),
        "n_failed": n_fail,
        "total_faces": int(total_faces),
        "total_vertices": int(total_verts),
        "all_watertight": bool(watertight),
        "scene_bbox": [round(float(v), 4) for v in all_bounds.reshape(-1)],
        "glb": glb_path,
        "stl": stl_path,
    }
    save_json(report, report_path)

    log("parts=%d 失败=%d 三角面=%d 顶点=%d 水密=%s" % (
        len(meshes), n_fail, total_faces, total_verts, watertight))
    log("场景包围盒(mm)=%s" % report["summary"]["scene_bbox"])
    log("report -> %s" % report_path)
    if plan.get("issues"):
        log("注意：plan 中仍有 %d 条未处理 issues（半自动确认项）" % len(plan["issues"]))
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
