#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""export_cad.py —— rebuild_plan.json -> 机械 CAD 实体格式（STEP / ISO-10303-21 AP214）

为什么需要单独一条线
--------------------
rebuild3d.py 出的是**三角网格**（GLB/STL）。STEP 这类机械格式要求**解析 B-rep**
（平面/圆柱等解析曲面 + 直线/圆弧边），所以本脚本另走一条：

    plan.json ─解析化(圆弧拟合)─> B-rep(顶点/边/环/面) ──> .stp  ISO-10303-21 AP214

★ 2026-10-16 起：机械实体**只输出 STEP**（纯 Python、零外部依赖、可离线）。
  历史上还做过 SAT / DGN / 含 3DSOLID 的 DXF，但那些必须借本机已授权的 CAD 内核
  （COM 自动化），发布给没有 CAD 的用户时完全不可用 —— 相关实现保留在
  §五之二 与 §七（**已停用存档**，不再由 CLI 暴露）。

方向约定（推导过程，改动前务必核对）
--------------------------------
局部标架 (u,v,w)，w = 拉伸方向，u×v=w 右手系。与 rebuild3d 的旋转严格等价：
    Z: (u,v,w)->( u, v, w)     Y: (u,v,w)->( u, w,-v)     X: (u,v,w)->( w, v,-u)

* 外轮廓规范成 CCW、孔规范成 CW —— 于是「材料恒在走向左侧」。
* 侧面法线 n = 走向 × e_w（材料左侧 → 外侧在右）。外轮廓、孔一律成立。
* 平面面：平面法线即面法线，same_sense=.T.。
* 圆柱面：CYLINDRICAL_SURFACE 的面法线恒为「离心向外」，与轴方向无关；
  n·r̂>0 取 same_sense=.T.，否则 .F.（凹弧、孔壁属于后者）。
* 端盖：顶面法线 +e_w（外环正向、孔环正向）；底面 -e_w（外环反向、孔环反向）。
* 弧一律拆到 ≤180°，避免出现「闭合边」与「大于半圆的弧」带来的歧义。

自校验（不依赖任何外部内核）
--------------------------
1) 拓扑：V - E + Σ(1-k)=2-2g，g = 贯通孔数；每条边恰好被 2 个环以相反方向引用。
2) 几何：散度定理 V=(1/3)∮r·n dA 逐面解析积分（平面用一阶矩、圆柱数值∫r̂du），
   与 rebuild3d 的网格体积/包围盒比对。**面朝向只要错一个，体积就偏**。

用法
----
  python export_cad.py saddle.plan.json --outdir . --step saddle.stp
  python export_cad.py plan.json                      # 默认写 <outdir>/<stem>.stp
退出码：0 成功；1 有失败
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drawing_common import load_json, save_json  # noqa: E402

EPS = 1e-9
MAX_ARC_DEG = 180.0

FRAMES = {
    "Z": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    "Y": ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
    "X": ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
}


def log(msg: str) -> None:
    print("[export_cad] %s" % msg)


# ==========================================================================
# 向量工具
# ==========================================================================
def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def v_len(a):
    return math.sqrt(v_dot(a, a))


def v_norm(a):
    n = v_len(a)
    if n < 1e-12:
        raise ValueError("零向量无法归一化")
    return (a[0] / n, a[1] / n, a[2] / n)


def _perp(a):
    a = v_norm(a)
    ref = (0.0, 0.0, 1.0) if abs(a[2]) < 0.9 else (1.0, 0.0, 0.0)
    return v_norm(v_cross(a, ref))


# ==========================================================================
# 一、2D 截面解析化：折线 -> 直线/圆弧
# ==========================================================================
def _fit_circle(p0, p1, p2):
    ax, ay = p0
    bx, by = p1
    cx, cy = p2
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    if r < 1e-9:
        return None
    return (ux, uy), r


def _signed_area(pts):
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def _dedup_closed(pts, tol=1e-7):
    out = []
    for p in pts:
        q = (float(p[0]), float(p[1]))
        if not out or math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > tol:
            out.append(q)
    while len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= tol:
        out.pop()
    return out


def _turn_angle(pts, i):
    n = len(pts)
    p0, p1, p2 = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
    d1 = (p1[0] - p0[0], p1[1] - p0[1])
    d2 = (p2[0] - p1[0], p2[1] - p1[1])
    l1 = math.hypot(*d1)
    l2 = math.hypot(*d2)
    if l1 < EPS or l2 < EPS:
        return 0.0
    c = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (l1 * l2)))
    return math.acos(c)


def _max_dev(run, c):
    cx, cy = c[0], c[1]
    r = c[2]
    return max(abs(math.hypot(p[0] - cx, p[1] - cy) - r) for p in run)


def _arc_sweep(c, a, b, ccw):
    """弧段扫角（弧度、带符号）。ccw=True 表示相对 +Z 逆时针。"""
    a0 = math.atan2(a[1] - c[1], a[0] - c[0])
    a1 = math.atan2(b[1] - c[1], b[0] - c[0])
    if ccw:
        return (a1 - a0) % (2 * math.pi)
    return -((a0 - a1) % (2 * math.pi))


def _split_long_arcs(segs, max_deg=MAX_ARC_DEG):
    out = []
    for s in segs:
        if s[0] == "line":
            out.append(s)
            continue
        _, c, r, a, b, ccw = s
        sweep = _arc_sweep(c, a, b, ccw)
        deg = abs(math.degrees(sweep))
        if deg <= max_deg + 1e-6:
            out.append(s)
            continue
        k = int(math.ceil(deg / max_deg))
        a0 = math.atan2(a[1] - c[1], a[0] - c[0])
        prev = a
        for i in range(1, k + 1):
            t = a0 + sweep * i / k
            p = (c[0] + r * math.cos(t), c[1] + r * math.sin(t))
            out.append(("arc", c, r, prev, p, ccw))
            prev = p
    return out


def _merge_collinear(segs):
    out = []
    for s in segs:
        if out and s[0] == "line" and out[-1][0] == "line":
            p0, p1, p2 = out[-1][1], out[-1][2], s[2]
            d1 = (p1[0] - p0[0], p1[1] - p0[1])
            d2 = (p2[0] - p1[0], p2[1] - p1[1])
            l1, l2 = math.hypot(*d1), math.hypot(*d2)
            if l1 > EPS and l2 > EPS:
                cr = abs(d1[0] * d2[1] - d1[1] * d2[0]) / (l1 * l2)
                if cr < 1e-6 and (d1[0] * d2[0] + d1[1] * d2[1]) > 0:
                    out[-1] = ("line", p0, p2)
                    continue
        out.append(s)
    return out


def _arc_dir(run, c, max_step_deg=60.0):
    """判断点列是否真为一段圆弧（并给出扫角与走向）。

    关键：仅靠「到圆心距离一致」不够 —— **矩形的 4 个角天然共圆**，会被误判成
    339.8° 的大圆弧（实测踩坑）。因此追加两条约束：
      1) 相邻采样点的扫角必须足够小（≤60°）——多边形顶点之间的弦不会满足；
      2) 逐步扫角之和必须等于首尾直接扫角（一致闭合）。
    返回 (sweep, ccw) 或 None。
    """
    angs = [math.atan2(p[1] - c[1], p[0] - c[0]) for p in run]
    two = 2 * math.pi
    for ccw in (True, False):
        steps = []
        for k in range(len(angs) - 1):
            if ccw:
                steps.append((angs[k + 1] - angs[k]) % two)
            else:
                steps.append(-((angs[k] - angs[k + 1]) % two))
        if max(abs(s) for s in steps) > math.radians(max_step_deg):
            continue
        sweep = sum(steps)
        direct = (((angs[-1] - angs[0]) % two) if ccw
                  else -((angs[0] - angs[-1]) % two))
        if abs(sweep - direct) > 1e-9:
            continue
        if abs(sweep) > math.pi + 1e-9:
            continue
        return sweep, ccw
    return None


def wire_from_points(pts, tol=None):
    """闭合折线 -> [seg]，seg=('line',p0,p1) | ('arc',c,r,p0,p1,ccw)。"""
    pts = _dedup_closed(pts)
    n = len(pts)
    if n < 3:
        raise ValueError("截面点数不足（%d）" % n)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    scale = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    if tol is None:
        tol = max(1e-3, scale * 5e-4)

    # 整圆检测
    if n >= 6:
        f = _fit_circle(pts[0], pts[n // 3], pts[(2 * n) // 3])
        if f:
            cc, rr = f
            if _max_dev(pts, (cc[0], cc[1], rr)) <= max(tol, rr * 2e-3):
                a, b = pts[0], pts[n // 2]
                return [("arc", cc, rr, a, b, True), ("arc", cc, rr, b, a, True)]

    start = 0
    if n > 4:
        start = max(range(n), key=lambda i: _turn_angle(pts, i))
    seq = [pts[(start + k) % n] for k in range(n)]

    segs = []
    i = 0
    while i < n:
        best_j, best_fit = None, None
        j = i + 3
        while j < n:
            run = seq[i:j + 1]
            f = _fit_circle(run[0], run[len(run) // 2], run[-1])
            if not f:
                break
            cc, rr = f
            if rr > 1e5 or _max_dev(run, (cc[0], cc[1], rr)) > tol:
                break
            res = _arc_dir(run, cc)
            if res is None:
                break
            best_j, best_fit = j, (cc, rr, res)
            j += 1
        if best_j is not None:
            run = seq[i:best_j + 1]
            cc, rr, (_sw, ccw) = best_fit
            a, b = run[0], run[-1]
            segs.append(("arc", cc, rr, a, b, ccw))
            i = best_j
        else:
            segs.append(("line", seq[i], seq[(i + 1) % n]))
            i += 1
    segs = [s for s in segs if not (s[0] == "line" and math.hypot(
        s[2][0] - s[1][0], s[2][1] - s[1][1]) < 1e-7)]
    return _split_long_arcs(_merge_collinear(segs))


def _wire_points(segs, n_arc=16):
    pts = []
    for s in segs:
        if s[0] == "line":
            pts.append(s[1])
        else:
            _, c, r, a, b, ccw = s
            sweep = _arc_sweep(c, a, b, ccw)
            a0 = math.atan2(a[1] - c[1], a[0] - c[0])
            for k in range(n_arc):
                t = a0 + sweep * k / float(n_arc)
                pts.append((c[0] + r * math.cos(t), c[1] + r * math.sin(t)))
    return pts


def _reverse_wire(segs):
    out = []
    for s in reversed(segs):
        if s[0] == "line":
            out.append(("line", s[2], s[1]))
        else:
            _, c, r, a, b, ccw = s
            out.append(("arc", c, r, b, a, not ccw))
    return out


# ==========================================================================
# 二、零件实体（局部标架 + 2D 截面 + 拉伸）
# ==========================================================================
class PartSolid(object):
    def __init__(self, name, axis, depth, outer, holes, offset, color=None, note=None):
        self.name = name
        self.axis = axis
        self.eu, self.ev, self.ew = FRAMES[axis]
        self.depth = float(depth)
        self.offset = tuple(float(x) for x in offset)
        self.color = color
        self.note = note
        if _signed_area(_wire_points(outer)) < 0:
            outer = _reverse_wire(outer)
        self.outer = outer
        self.holes = []
        for h in holes:
            if _signed_area(_wire_points(h)) > 0:
                h = _reverse_wire(h)
            self.holes.append(h)
        self.brep = None

    def to3d(self, u, v, w=0.0):
        o = self.offset
        return (o[0] + u * self.eu[0] + v * self.ev[0] + w * self.ew[0],
                o[1] + u * self.eu[1] + v * self.ev[1] + w * self.ew[1],
                o[2] + u * self.eu[2] + v * self.ev[2] + w * self.ew[2])

    def uv_bounds(self, n=48):
        us, vs = [], []
        for wire in [self.outer] + self.holes:
            for s in wire:
                if s[0] == "line":
                    us += [s[1][0], s[2][0]]
                    vs += [s[1][1], s[2][1]]
                else:
                    _, c, r, a, b, ccw = s
                    sweep = _arc_sweep(c, a, b, ccw)
                    a0 = math.atan2(a[1] - c[1], a[0] - c[0])
                    for k in range(n + 1):
                        t = a0 + sweep * k / float(n)
                        us.append(c[0] + r * math.cos(t))
                        vs.append(c[1] + r * math.sin(t))
        return min(us), max(us), min(vs), max(vs)

    def corners(self):
        u0, u1, v0, v1 = self.uv_bounds(24)
        return [self.to3d(u, v, w) for u in (u0, u1) for v in (v0, v1) for w in (0.0, self.depth)]

    def shift(self, off):
        self.offset = (self.offset[0] + off[0], self.offset[1] + off[1], self.offset[2] + off[2])


# ==========================================================================
# 三、B-rep 构建
# ==========================================================================
def _tangent2(a2, c, ccw):
    d = (a2[0] - c[0], a2[1] - c[1])
    l = math.hypot(*d)
    d = (d[0] / l, d[1] / l)
    return (-d[1], d[0]) if ccw else (d[1], -d[0])


def _tangent3(a2, c, ccw, P):
    t = _tangent2(a2, c, ccw)
    return v_norm((t[0] * P.eu[0] + t[1] * P.ev[0],
                   t[0] * P.eu[1] + t[1] * P.ev[1],
                   t[0] * P.eu[2] + t[1] * P.ev[2]))


def _arc_mid2(c, r, a, b, ccw):
    """弧段几何中点（与走向无关）。"""
    a0 = math.atan2(a[1] - c[1], a[0] - c[0])
    t = a0 + _arc_sweep(c, a, b, ccw) / 2.0
    return (c[0] + r * math.cos(t), c[1] + r * math.sin(t))


class BrepBuilder(object):
    def __init__(self, part):
        self.p = part
        self.verts = {}
        self.vlist = []
        self.edges = {}
        self.elist = []
        self.faces = []

    def _vid(self, p):
        k = (round(p[0], 6), round(p[1], 6), round(p[2], 6))
        if k not in self.verts:
            self.verts[k] = len(self.vlist)
            self.vlist.append({"id": len(self.vlist), "p": k})
        return self.verts[k]

    def _edge(self, pa, pb, kind, geom):
        va, vb = self._vid(pa), self._vid(pb)
        if va == vb:
            raise ValueError("退化边（首尾同点）")
        Pa, Pb = self.vlist[va]["p"], self.vlist[vb]["p"]
        if (Pa, va) <= (Pb, vb):
            A, B, pA, pB = va, vb, Pa, Pb
            fwd = True
        else:
            A, B, pA, pB = vb, va, Pb, Pa
            fwd = False
        key = (A, B, kind, geom)
        if key not in self.edges:
            e = {"id": len(self.elist), "kind": kind, "v0": A, "v1": B,
                 "pa": pA, "pb": pB, "geom": geom}
            if kind == "arc":
                C = (geom[0], geom[1], geom[2])
                r = geom[3]
                M = (geom[4], geom[5], geom[6])
                ax = v_cross(v_sub(pA, C), v_sub(M, C))
                e["axis"] = v_norm(ax) if v_len(ax) > 1e-9 else _perp(v_sub(pA, C))
                e["center"] = C
                e["radius"] = r
            self.edges[key] = e
            self.elist.append(e)
        return self.edges[key], fwd

    def _face(self, surface, loops):
        self.faces.append({"surface": surface, "loops": loops})

    # ---------------- 弧几何元组 ----------------
    def _arc_geom(self, C3, r, M3, w):
        """弧的几何元组 (cx,cy,cz,r,mx,my,mz)，圆心/中点必须落在**该弧自身所在平面**。

        坑：若顶面弧沿用 w=0 的圆心/中点，_edge 里 cross(pA-C, M-C) 会混入沿拉伸轴的
        分量（实测把弧轴扭了 30°），顶面弧采样随即跑偏，端盖的平面体积积分整段污染
        （垫板曾因此偏 197.91%）。所以按 w 把圆心/中点搬到各自平面。
        """
        ew = self.p.ew
        Cw = (C3[0] + ew[0] * w, C3[1] + ew[1] * w, C3[2] + ew[2] * w)
        Mw = (M3[0] + ew[0] * w, M3[1] + ew[1] * w, M3[2] + ew[2] * w)
        return (round(Cw[0], 6), round(Cw[1], 6), round(Cw[2], 6), round(r, 6),
                round(Mw[0], 6), round(Mw[1], 6), round(Mw[2], 6))

    # ---------------- 侧面 ----------------
    def _side(self, s):
        P = self.p
        d = P.depth
        if s[0] == "line":
            _, a2, b2 = s
            B0, B1 = P.to3d(a2[0], a2[1], 0.0), P.to3d(b2[0], b2[1], 0.0)
            T0, T1 = P.to3d(a2[0], a2[1], d), P.to3d(b2[0], b2[1], d)
            dir3 = v_norm(v_sub(B1, B0))
            n = v_norm(v_cross(dir3, P.ew))
            eb, fb = self._edge(B0, B1, "line", None)
            et, ft = self._edge(T0, T1, "line", None)
            e0, f0 = self._edge(B0, T0, "line", None)
            e1, f1 = self._edge(B1, T1, "line", None)
            loop = [(eb, fb), (e1, f1), (et, not ft), (e0, not f0)]
            self._face({"type": "plane", "origin": B0, "axis": n, "ref": dir3,
                        "same": True}, [(loop, True)])
        else:
            _, c, r, a2, b2, ccw = s
            C3 = P.to3d(c[0], c[1], 0.0)
            B0, B1 = P.to3d(a2[0], a2[1], 0.0), P.to3d(b2[0], b2[1], 0.0)
            T0, T1 = P.to3d(a2[0], a2[1], d), P.to3d(b2[0], b2[1], d)
            mid = _arc_mid2(c, r, a2, b2, ccw)   # 必须是**弧**中点，不能用弦中点
            Pm = P.to3d(mid[0], mid[1], 0.0)
            radial = v_norm(v_sub(Pm, C3))
            tdir = _tangent3(a2, c, ccw, P)
            n_face = v_norm(v_cross(tdir, P.ew))
            same = v_dot(n_face, radial) > 0
            M3 = Pm
            m2 = mid
            gb = self._arc_geom(C3, r, M3, 0.0)
            gt = self._arc_geom(C3, r, M3, d)
            eb, fb = self._edge(B0, B1, "arc", gb)
            et, ft = self._edge(T0, T1, "arc", gt)
            e0, f0 = self._edge(B0, T0, "line", None)
            e1, f1 = self._edge(B1, T1, "line", None)
            loop = [(eb, fb), (e1, f1), (et, not ft), (e0, not f0)]
            self._face({"type": "cylinder", "origin": C3, "radius": r,
                        "axis_dir": P.ew, "same": same, "length": d}, [(loop, True)])

    # ---------------- 端盖 ----------------
    def _cap(self, is_top):
        P = self.p
        w = P.depth if is_top else 0.0
        n = P.ew if is_top else v_mul(P.ew, -1.0)
        loops = [(self._cap_loop(P.outer, w, is_top), True)]
        for h in P.holes:
            loops.append((self._cap_loop(h, w, is_top), False))
        self._face({"type": "plane", "origin": P.to3d(0.0, 0.0, w), "axis": n,
                    "ref": P.eu, "same": True}, loops)

    def _cap_loop(self, wire, w, forward):
        P = self.p
        segs = wire if forward else _reverse_wire(wire)
        out = []
        for s in segs:
            if s[0] == "line":
                _, a2, b2 = s
                e, f = self._edge(P.to3d(a2[0], a2[1], w), P.to3d(b2[0], b2[1], w),
                                  "line", None)
            else:
                _, c, r, a2, b2, ccw = s
                C3 = P.to3d(c[0], c[1], 0.0)
                m2 = _arc_mid2(c, r, a2, b2, ccw)
                M3 = P.to3d(m2[0], m2[1], 0.0)
                geom = self._arc_geom(C3, r, M3, w)
                e, f = self._edge(P.to3d(a2[0], a2[1], w), P.to3d(b2[0], b2[1], w),
                                  "arc", geom)
            out.append((e, f))
        return out

    def build(self):
        for wire in [self.p.outer] + self.p.holes:
            for s in wire:
                self._side(s)
        self._cap(False)
        self._cap(True)
        return {"verts": self.vlist, "edges": self.elist, "faces": self.faces}


# ==========================================================================
# 四、自校验
# ==========================================================================
def _edge_points(e, fwd, n_arc=16):
    pa = e["pa"] if fwd else e["pb"]
    pb = e["pb"] if fwd else e["pa"]
    if e["kind"] == "line":
        return [pa, pb]
    C, r = e["center"], e["radius"]
    u = v_norm(v_sub(pa, C))
    vv = v_cross(e["axis"], u)
    pu = v_sub(pb, C)
    ang = math.atan2(v_dot(pu, vv), v_dot(pu, u))
    out = []
    for k in range(n_arc + 1):
        t = ang * k / float(n_arc)
        out.append(v_add(C, v_add(v_mul(u, r * math.cos(t)), v_mul(vv, r * math.sin(t)))))
    out[0], out[-1] = pa, pb
    return out


def _loop_points(loop, n_arc=64):
    pts = []
    for e, fwd in loop:
        pts.extend(_edge_points(e, fwd, n_arc)[:-1])
    return pts


def _planar_contrib(face):
    """平面面对体积的贡献 (1/3)∮r·n dA。

    注意：平面一阶矩公式必须在「平面内取基准点 Q」下推导：
        ∫ r dA = (1/6)Σ(p_i+p_{i+1})Δ_i + (1/3) A Q ,  Δ_i = n·((p_i-Q)×(p_{i+1}-Q))
    少了 (1/3)AQ 这一项，只有当平面过原点时才正确 —— 实测长方体体积会偏小 1/3。
    """
    n = v_norm(face["surface"]["axis"])
    Q = face["surface"]["origin"]
    Mv = (0.0, 0.0, 0.0)
    A = 0.0
    for loop, _o in face["loops"]:
        pts = _loop_points(loop)
        m = len(pts)
        for i in range(m):
            a, b = pts[i], pts[(i + 1) % m]
            d = v_dot(v_cross(v_sub(a, Q), v_sub(b, Q)), n)
            Mv = v_add(Mv, v_mul(v_add(a, b), d / 6.0))
            A += d / 2.0
    return (v_dot(Mv, n) + A * v_dot(Q, n) / 3.0) / 3.0


def brep_volume(brep):
    """散度定理 V=(1/3)∮ r·n dA，逐面解析积分。"""
    total = 0.0
    for f in brep["faces"]:
        s = f["surface"]
        if s["type"] == "plane":
            total += _planar_contrib(f)
        else:
            C, r, L = s["origin"], s["radius"], s["length"]
            sgn = 1.0 if s["same"] else -1.0
            axis = v_norm(s["axis_dir"])
            arcs = [e for e, _f in f["loops"][0][0] if e["kind"] == "arc"]
            if not arcs:
                continue
            arc = min(arcs, key=lambda e: min(v_dot(v_sub(e["pa"], C), axis),
                                              v_dot(v_sub(e["pb"], C), axis)))
            u0 = v_norm(v_sub(arc["pa"], C))
            vv = v_cross(axis, u0)
            pu = v_sub(arc["pb"], C)
            ang = math.atan2(v_dot(pu, vv), v_dot(pu, u0))
            if ang < 0:
                vv, ang = v_mul(vv, -1.0), -ang
            # 闭式解：∮r·n_geo dA = r·L·[C·(∫r̂dt) + r·ang]，
            # ∫_0^ang r̂ dt = sin(ang)·u0 + (1-cos(ang))·vv（u0,vv,axis 右手正交）
            acc = v_add(v_mul(u0, math.sin(ang)), v_mul(vv, 1.0 - math.cos(ang)))
            total += sgn * r * L * (v_dot(C, acc) + r * ang) / 3.0
    return total


def profile_volume(part):
    """独立估算：截面面积（多边形，含孔）× 深度。与解析积分交叉验证。"""
    try:
        from shapely.geometry import Polygon
        outer = _wire_points(part.outer, 128)
        holes = [_wire_points(h, 128) for h in part.holes]
        poly = Polygon(outer, holes)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.geom_type == "MultiPolygon":
            poly = max(poly.geoms, key=lambda g: g.area)
        return abs(poly.area) * part.depth
    except Exception:  # noqa: BLE001
        return None


def brep_check(brep, part):
    problems = []
    V, E, F = len(brep["verts"]), len(brep["edges"]), len(brep["faces"])
    chi_faces = sum(1.0 - (len(f["loops"]) - 1) for f in brep["faces"])
    chi = V - E + chi_faces
    n_holes = len(part.holes)
    chi_expect = 2 - 2 * n_holes
    if abs(chi - chi_expect) > 1e-6:
        problems.append("拓扑不变量 V-E+Σ(1-k)=%.0f != 期望 %.0f（贯通孔 %d）"
                        % (chi, chi_expect, n_holes))
    use = {}
    for f in brep["faces"]:
        for loop, _o in f["loops"]:
            for e, _f in loop:
                use[e["id"]] = use.get(e["id"], 0) + 1
    bad = [i for i, c in use.items() if c != 2]
    if bad:
        problems.append("%d 条边未被恰好 2 个环引用（非流形/开裂）" % len(bad))
    vol = brep_volume(brep)
    if vol <= 0:
        problems.append("解析体积非正（%.4f）：面朝向可能整体翻转" % vol)
    vol_ref = profile_volume(part)
    if vol_ref:
        dv = abs(vol - vol_ref) / vol_ref * 100.0
        if dv > 0.5:
            problems.append("解析积分体积 %.1f 与「截面面积×深度」%.1f 相差 %.2f%%"
                            "（面/环积分有误）" % (vol, vol_ref, dv))
    pts = [v["p"] for v in brep["verts"]]
    bbox = ([min(p[i] for p in pts) for i in range(3)]
            + [max(p[i] for p in pts) for i in range(3)])
    return {"vertices": V, "edges": E, "faces": F, "chi": chi,
            "volume": vol, "volume_profile": vol_ref,
            "bbox": [round(x, 4) for x in bbox]}, problems


# ==========================================================================
# 五、STEP (ISO-10303-21 AP214)
# ==========================================================================
class StepWriter(object):
    def __init__(self):
        self.lines = []
        self.n = 100
        self._cp, self._dir, self._vp, self._line, self._circ, self._pl = {}, {}, {}, {}, {}, {}

    def add(self, text):
        self.n += 1
        self.lines.append("#%d=%s;" % (self.n, text))
        return self.n

    def point(self, p):
        k = (round(p[0], 6), round(p[1], 6), round(p[2], 6))
        if k not in self._cp:
            self._cp[k] = self.add("CARTESIAN_POINT('',(%.9f,%.9f,%.9f))" % k)
        return self._cp[k]

    def direction(self, d):
        d = v_norm(d)
        k = (round(d[0], 9), round(d[1], 9), round(d[2], 9))
        if k not in self._dir:
            self._dir[k] = self.add("DIRECTION('',(%.9f,%.9f,%.9f))" % k)
        return self._dir[k]

    def placement(self, origin, axis, ref):
        key = (tuple(round(x, 6) for x in origin), tuple(round(x, 6) for x in v_norm(axis)),
               tuple(round(x, 6) for x in v_norm(ref)))
        if key not in self._pl:
            self._pl[key] = self.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)"
                                     % (self.point(origin), self.direction(axis),
                                        self.direction(ref)))
        return self._pl[key]

    def vertex(self, p):
        k = (round(p[0], 6), round(p[1], 6), round(p[2], 6))
        if k not in self._vp:
            self._vp[k] = self.add("VERTEX_POINT('',#%d)" % self.point(p))
        return self._vp[k]

    def line(self, pa, pb):
        """LINE 必须按「一整条无限直线」唯一化，不能只拿方向做键。

        坑（2026-10-16 定位）：早期版本 key = 方向，于是**所有平行线被合并成同一条**，
        直线锚点用了第一次出现的那个点 ⇒ 后来的顶点落在错的直线上（面在、边不在线上）。
        OCC 读进来会当成坏拓扑。这里 key = (方向, 原点到直线的垂足)，锚点取垂足本身，
        保证一条直线只写一份、且锚点必然在线上。
        """
        d = v_norm(v_sub(pb, pa))
        kd = tuple(round(x, 9) for x in d)
        foot = v_sub(pa, v_mul(d, v_dot(pa, d)))     # 原点到直线的垂足（必在线上）
        key = (kd, tuple(round(x, 6) for x in foot))
        if key not in self._line:
            vec = self.add("VECTOR('',#%d,1.0)" % self.direction(d))
            self._line[key] = self.add("LINE('',#%d,#%d)" % (self.point(foot), vec))
        return self._line[key]

    def circle(self, pa, pb, C, r):
        u = v_norm(v_sub(pa, C))
        axis = v_cross(u, v_sub(pb, C))
        axis = v_norm(axis) if v_len(axis) > 1e-9 else _perp(u)
        k = (tuple(round(x, 6) for x in C), round(r, 6),
             tuple(round(x, 6) for x in axis))
        if k not in self._circ:
            pl = self.placement(C, axis, u)
            self._circ[k] = self.add("CIRCLE('',#%d,%.9f)" % (pl, r))
        return self._circ[k]

    def plane(self, origin, n, ref):
        return self.add("PLANE('',#%d)" % self.placement(origin, n, ref))

    def cylinder(self, origin, axis, r):
        return self.add("CYLINDRICAL_SURFACE('',#%d,%.9f)"
                        % (self.placement(origin, axis, _perp(axis)), r))


def write_step(parts, path, name=None):
    w = StepWriter()
    name = name or os.path.splitext(os.path.basename(path))[0]
    ctx = w.add("APPLICATION_CONTEXT('automotive design')")
    apd = w.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#%d)" % ctx)
    pc = w.add("PRODUCT_CONTEXT('',#%d,'mechanical')" % apd)
    prod = w.add("PRODUCT('%s','%s','',(#%d))" % (name, name, pc))
    pdf = w.add("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#%d,.NOT_KNOWN.)" % prod)
    pdc = w.add("PRODUCT_DEFINITION_CONTEXT('part definition',#%d,'design')" % ctx)
    pd = w.add("PRODUCT_DEFINITION('design','',#%d,#%d)" % (pdf, pdc))
    pds = w.add("PRODUCT_DEFINITION_SHAPE('','',#%d)" % pd)
    lu = w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
    au = w.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )")
    su = w.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )")
    unc = w.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-3),#%d,'distance_accuracy_value','confusion accuracy')" % lu)
    gc = w.add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#%d)) "
               "GLOBAL_UNIT_ASSIGNED_CONTEXT((#%d,#%d,#%d)) REPRESENTATION_CONTEXT('Context','3D') )"
               % (unc, lu, au, su))

    solid_ids = []
    skipped = []
    for i, part in enumerate(parts):
        if part.brep is None:
            skipped.append(part.name)
            continue
        solid_ids.append(_emit_solid(w, part))
    if skipped:
        log("STEP 跳过 %d 个 B-rep 构建失败的零件：%s" % (len(skipped), "、".join(skipped)))
    if not solid_ids:
        raise ValueError("没有任何可写出的实体（全部零件 B-rep 构建失败）")

    axis = w.placement((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    rep = w.add("ADVANCED_BREP_SHAPE_REPRESENTATION('%s',(%s,#%d),#%d)"
                % (name, ",".join("#%d" % s for s in solid_ids), axis, gc))
    w.add("SHAPE_DEFINITION_REPRESENTATION(#%d,#%d)" % (pds, rep))

    head = [
        "ISO-10303-21;",
        "HEADER;",
        "FILE_DESCRIPTION(('3D part generated by cad-drawing-to-3d skill'),'2;1');",
        "FILE_NAME('%s','%s',(''),(''),'cad-drawing-to-3d','','');"
        % (os.path.basename(path), "1970-01-01T00:00:00"),
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));",
        "ENDSEC;",
        "DATA;",
    ]
    tail = ["ENDSEC;", "END-ISO-10303-21;"]
    with open(path, "w", encoding="ascii", errors="replace", newline="\n") as f:
        f.write("\n".join(head + w.lines + tail) + "\n")
    return len(w.lines)


def _emit_solid(w, part):
    """把一个零件的 B-rep 写成 MANIFOLD_SOLID_BREP。

    ★ 关键约定（2026-10-16 修复）：**同一条几何边只能写一份 EDGE_CURVE**，
    两个相邻面各自用一个 ORIENTED_EDGE 去引用它（方向相反）。

    早期版本是「每个面各自建一份 EDGE_CURVE」，写出来的文件虽然能被自家的
    纯文本自检放过，但第三方内核（OpenCASCADE / gmsh 实测）会得到
    「N 个面 + 2N 条边 + 无壳无体」⇒ 导入后**只看到一堆散面**（用户实测就是
    「STP 显示不全」，只出来垫板一块 + 两个孔的小点）。

    另一个必须共享的是 VERTEX_POINT（同一坐标只写一份），否则面与面不共顶点。
    缓存按**零件**重置：不同零件之间绝不允许共享点/边（否则两个壳被粘成一个）。
    """
    brep = part.brep
    vp_cache = {}
    ec_cache = {}

    def vp(p):
        k = (round(p[0], 6), round(p[1], 6), round(p[2], 6))
        if k not in vp_cache:
            vp_cache[k] = w.add("VERTEX_POINT('',#%d)" % w.point(p))
        return vp_cache[k]

    def ec(e):
        """按边 id 唯一化 EDGE_CURVE；几何方向恒为该边的规范方向 pa -> pb。"""
        if e["id"] not in ec_cache:
            if e["kind"] == "line":
                cur = w.line(e["pa"], e["pb"])
            else:
                cur = w.circle(e["pa"], e["pb"], e["center"], e["radius"])
            ec_cache[e["id"]] = w.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)"
                                      % (vp(e["pa"]), vp(e["pb"]), cur))
        return ec_cache[e["id"]]

    face_ids = []
    for f in brep["faces"]:
        s = f["surface"]
        if s["type"] == "plane":
            surf = w.plane(s["origin"], s["axis"], s["ref"])
            same = ".T."
        else:
            surf = w.cylinder(s["origin"], s["axis_dir"], s["radius"])
            same = ".T." if s["same"] else ".F."
        loops = []
        for li, (loop, is_outer) in enumerate(f["loops"]):
            oes = []
            for e, fwd in loop:
                # fwd 是「本环的走法是否与边的规范方向一致」⇒ 决定 ORIENTED_EDGE 的正反
                oes.append(w.add("ORIENTED_EDGE('',*,*,#%d,%s)"
                                 % (ec(e), ".T." if fwd else ".F.")))
            el = w.add("EDGE_LOOP('',(%s))" % ",".join("#%d" % x for x in oes))
            kind = "FACE_OUTER_BOUND" if is_outer else "FACE_BOUND"
            loops.append(w.add("%s('',#%d,.T.)" % (kind, el)))
        face_ids.append(w.add("ADVANCED_FACE('',(%s),#%d,%s)"
                              % (",".join("#%d" % x for x in loops), surf, same)))
    shell = w.add("CLOSED_SHELL('',(%s))" % ",".join("#%d" % x for x in face_ids))
    return w.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (_ascii_name(part.name), shell))


def _ascii_name(name):
    """STEP 实体名要求 ASCII；非 ASCII 名退化为 PART-N。"""
    try:
        name.encode("ascii")
        return name.replace("'", "")
    except UnicodeEncodeError:
        return "PART"


# ==========================================================================
# 五之二、STEP 结构自检（不依赖任何 STEP 内核）
# ==========================================================================
def check_step_file(path):
    """解析刚写出的 .stp：引用完整性 + 拓扑引用链 + 单位/精度头。

    ISO-10303-21 是纯文本指向图，绝大多数"打不开"都是引用错（悬空 #id、
    id 重复、EDGE_CURVE 少了顶点）。这些都能静态查出来，不必装内核。
    """
    import re
    with open(path, "r", encoding="ascii", errors="replace") as f:
        text = f.read()
    problems = []
    if not text.startswith("ISO-10303-21;"):
        problems.append("缺少 ISO-10303-21; 头")
    if "END-ISO-10303-21;" not in text:
        problems.append("缺少 END-ISO-10303-21; 尾")
    for sec in ("HEADER;", "DATA;", "ENDSEC;"):
        if sec not in text:
            problems.append("缺少 %s" % sec)
    if not re.search(r"FILE_SCHEMA\(\('(AUTOMOTIVE_DESIGN|AP214|AP242)", text):
        problems.append("FILE_SCHEMA 不是 AP214/AP242（多数机械内核只认这两个）")

    inst = {}
    # 注意：STEP 允许「复合实体实例」写法 #n=( TYPE(...) TYPE(...) )，类型名前有个 '('，
    # 单纯用 r"#(\d+)\s*=\s*([A-Z_0-9]+)" 会把这类实体当成没定义（误报悬空引用）。
    for m in re.finditer(r"#(\d+)\s*=\s*(?:\(\s*)?([A-Z_0-9]+)", text):
        i, t = int(m.group(1)), m.group(2)
        if i in inst:
            problems.append("实体 #%d 重复定义" % i)
        inst[i] = t
    if not inst:
        return {"entities": 0, "by_type": {}}, ["文件里没有实体实例"]

    body = text.split("DATA;", 1)[-1]
    refs = set(int(x) for x in re.findall(r"#(\d+)", body))
    dangling = sorted(r for r in refs if r not in inst)
    if dangling:
        problems.append("%d 个悬空引用（前若干个：%s）"
                        % (len(dangling), dangling[:8]))

    # 拓扑引用链：BREP -> SHELL -> FACE -> BOUND -> LOOP -> ORIENTED_EDGE -> EDGE_CURVE
    def arg_ids(i):
        m = re.search(r"#%d\s*=\s*(?:\(\s*)?[A-Z_0-9]+\(([^;]*)\);" % i, text, re.S)
        return [int(x) for x in re.findall(r"#(\d+)", m.group(1))] if m else []

    def typed_args(i, want):
        m = re.search(r"#%d\s*=\s*(?:\(\s*)?([A-Z_0-9]+)\(([^;]*)\);" % i, text, re.S)
        if not m:
            return []
        return [int(x) for x in re.findall(r"#(\d+)", m.group(2))
                if inst.get(int(x)) == want]

    breps = [i for i, t in inst.items() if t == "MANIFOLD_SOLID_BREP"]
    if not breps:
        problems.append("没有 MANIFOLD_SOLID_BREP（不是实体模型）")
    if not any(t == "ADVANCED_BREP_SHAPE_REPRESENTATION" for t in inst.values()):
        problems.append("没有 ADVANCED_BREP_SHAPE_REPRESENTATION（B-rep 未挂到装配树）")
    n_face = n_edge = 0
    n_ec_total = 0
    split_shells = []
    for b in breps:
        shells = typed_args(b, "CLOSED_SHELL")
        if len(shells) != 1:
            problems.append("MANIFOLD_SOLID_BREP #%d 的壳数=%d（应为 1）" % (b, len(shells)))
            continue
        faces = typed_args(shells[0], "ADVANCED_FACE")
        if len(faces) < 4:
            problems.append("CLOSED_SHELL #%d 只有 %d 个面（<4 不可能是闭合体）"
                            % (shells[0], len(faces)))
        ec_use = {}
        for fc in faces:
            n_face += 1
            bounds = typed_args(fc, "FACE_OUTER_BOUND") + typed_args(fc, "FACE_BOUND")
            if not bounds:
                problems.append("ADVANCED_FACE #%d 没有 FACE_BOUND" % fc)
            if sum(1 for x in arg_ids(fc) if inst.get(x) == "FACE_OUTER_BOUND") > 1:
                problems.append("ADVANCED_FACE #%d 有多个 FACE_OUTER_BOUND" % fc)
            for bd in bounds:
                for lp in typed_args(bd, "EDGE_LOOP"):
                    oes = typed_args(lp, "ORIENTED_EDGE")
                    if not oes:
                        problems.append("EDGE_LOOP #%d 没有 ORIENTED_EDGE" % lp)
                    for oe in oes:
                        ecs = typed_args(oe, "EDGE_CURVE")
                        if len(ecs) != 1:
                            problems.append("ORIENTED_EDGE #%d 指向 %d 条 EDGE_CURVE"
                                            % (oe, len(ecs)))
                            continue
                        ec = ecs[0]
                        n_edge += 1
                        ec_use[ec] = ec_use.get(ec, 0) + 1
                        if len(typed_args(ec, "VERTEX_POINT")) != 2:
                            problems.append("EDGE_CURVE #%d 的顶点数 != 2" % ec)
        n_ec_total += len(ec_use)
        # ★ 流形共享检查（2026-10-16 新增，此前的致命盲区）：
        #   STEP 里同一个几何边**只能有一份 EDGE_CURVE**，被相邻两个面各用一个
        #   ORIENTED_EDGE 引用。若每个面各建一份，第三方内核（OpenCASCADE 实测）
        #   得不到 CLOSED_SHELL ⇒ 只导入一堆散面，表现为「模型显示不全」。
        lone = sorted(e for e, c in ec_use.items() if c != 2)
        if lone:
            split_shells.append((shells[0], len(lone), lone[:6]))
    if split_shells:
        problems.append(
            "%d 个壳里存在「未被恰好 2 个面共用」的边（如壳 #%d 有 %d 条）"
            "：同一条几何边必须只写一份 EDGE_CURVE，两个相邻面各用一个 ORIENTED_EDGE "
            "引用它；否则多数内核收不到闭合壳，只导入散面"
            % (len(split_shells), split_shells[0][0], split_shells[0][1]))
    by_type = {}
    for t in inst.values():
        by_type[t] = by_type.get(t, 0) + 1
    return {"entities": len(inst), "refs": len(refs), "solids": len(breps),
            "faces": n_face, "edge_uses": n_edge, "edge_curves": n_ec_total,
            "file_schema": "AP214" if "AUTOMOTIVE_DESIGN" in text else "?",
            "by_type": dict(sorted(by_type.items(), key=lambda kv: -kv[1]))}, problems


# ==========================================================================
# 五点五、【已停用存档】ACIS SAT 结构自检（纯文本解析，不需要 ACIS 内核）
#   仅供历史上「SAT 输出」路线做自检；当前不再输出 SAT，故 main() 也不再调用。
# ==========================================================================
# SAT 文件结构：
#   第 1 行  版本三元组 `<version> <n_entities> <n_bodies> <flags>`
#            实测 `700 870 6 0` = ACIS 7.0 / 870 个实体记录 / 6 个 body
#   第 2 行  产品串 `@33 Open Design Alliance ACIS Builder @12 ACIS 7.00 NT @24 <日期>`
#   第 3 行  单位与容差 `1 9.9999999999999995e-007 1e-010`
#   之后     逐行实体记录，行首是实体类型名；`$n` 是引用；`#` 收尾
#   末尾     `End-of-ACIS-data`
def check_sat_file(path):
    """解析 SAT 文本，返回 (info, problems)。

    能拿到的东西：header 声明的 body 数、实际 `body ` 行数、实体类型直方图、
    几何点云包围盒（用于与自研 B-rep 的包围盒交叉核对）。
    """
    problems = []
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return {"exists": False}, ["SAT 文件不存在或为空"]
    with open(path, "r", encoding="latin-1") as f:
        text = f.read()
    lines = [ln.rstrip("\r\n") for ln in text.splitlines() if ln.strip()]
    if len(lines) < 4:
        return {"exists": True, "lines": len(lines)}, ["SAT 行数过少，疑似截断"]
    hdr = lines[0].split()
    ver = hdr[0] if hdr else "?"
    n_ent = int(hdr[1]) if len(hdr) > 1 and hdr[1].isdigit() else None
    n_body_hdr = int(hdr[2]) if len(hdr) > 2 and hdr[2].isdigit() else None

    types = {}
    n_body = 0
    pts = []
    for ln in lines:
        head = ln.split(None, 1)[0]
        types[head] = types.get(head, 0) + 1
        if head == "body":
            n_body += 1
        elif head == "point":
            fp = ln.split()
            # point <s3> x y z  —— 前 3 个是 sentinel/属性
            try:
                pts.append((float(fp[4]), float(fp[5]), float(fp[6])))
            except (IndexError, ValueError):
                pass
    if not text.rstrip().endswith("End-of-ACIS-data"):
        problems.append("缺 End-of-ACIS-data 结束标记（文件可能被截断）")
    if n_body_hdr is not None and n_body != n_body_hdr:
        problems.append("header 声明 %d 个 body，实际数到 %d 个" % (n_body_hdr, n_body))
    if n_ent is not None and len(lines) - 4 != n_ent:
        # 有些版本 header 的计数含 header 行自身，这里只作提示不做失败判定
        problems.append("header 声明 %d 个实体记录，实际 %d 行（可能只是计数口径差异）"
                        % (n_ent, len(lines) - 4))
    bbox = None
    if pts:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        bbox = [round(min(xs), 4), round(min(ys), 4), round(min(zs), 4),
                round(max(xs), 4), round(max(ys), 4), round(max(zs), 4)]
    info = {"exists": True, "version": ver, "declared_entities": n_ent,
            "bodies": n_body, "declared_bodies": n_body_hdr,
            "declared_points": len(pts), "bbox": bbox,
            "bytes": os.path.getsize(path),
            "end_marker": text.rstrip().endswith("End-of-ACIS-data"),
            "by_type": dict(sorted(types.items(), key=lambda kv: -kv[1])[:14])}
    return info, problems


def sat_body_boxes(path):
    """ACIS SAT 里**逐 body 的包围盒**（纯文本解析，不需要 ACIS 内核）。

    结构：表头 3 行（版本 / 产品串 / 单位+容差），之后每行一个实体，
    实体编号 = 该行在实体流中的序号（0 起）；引用写成 `$N`；
    `point` 的坐标在 token[4..6]。于是从每个 body 出发沿 `$N` 做 BFS，
    收集可达的 point 取包围盒，就是该 body 的位姿。

    ★ 两个必须绕开的坑（2026-10-15 实测）：
      1. **表头行数不固定** → 用自校验定标：偏移量必须让「第一个 body 引用的
         实体里含 `lump`」，且「从任一 body 出发不会触达第二个 body」。
      2. **`*-adesk-attrib` 属性实体会把不同 body 串起来**（材质/颜色属性跨件
         共享），遍历时必须剪掉，否则每个 body 都等于整机。

    用途：合并 SAT 的 body 数对不上时，直接列出每个 body 的包围盒，
    就能看出多出来的是什么（实测多出的 2 个 body 是底板孔柱的残留实体）。
    返回 [{"body": i, "points": n, "bbox": [...]}, ...]，失败返回 None。
    """
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return None
    with open(path, "r", encoding="latin-1") as f:
        lines = [x.rstrip("\n") for x in f]
    raw = [(i, x.split(" ")) for i, x in enumerate(lines)]
    body_idx = [i for i, t in raw if t and t[0] == "body"]
    if not body_idx:
        return None

    def refs(toks):
        return [int(z[1:]) for z in toks
                if z.startswith("$") and z[1:].isdigit()]

    ents = None
    for start in range(0, 12):
        cand = {i - start: t for i, t in raw
                if i >= start and t and t[0] and not t[0].startswith("End-of-ACIS")}
        probe = cand.get(body_idx[0] - start)
        if not probe:
            continue
        if "lump" not in [cand.get(r, ["?"])[0] for r in refs(probe)]:
            continue
        seen, stack, leak = set(), [body_idx[0] - start], False
        while stack:
            n = stack.pop()
            if n in seen or n not in cand:
                continue
            seen.add(n)
            if n != body_idx[0] - start and cand[n] and cand[n][0] == "body":
                leak = True
                break
            stack.extend(refs(cand[n]))
        if not leak:
            ents = cand
            break
    if ents is None:
        return None

    out = []
    for bi, root in enumerate(sorted(k for k, v in ents.items()
                                     if v and v[0] == "body")):
        seen, stack, pts = set(), [root], []
        while stack:
            n = stack.pop()
            if n in seen or n not in ents:
                continue
            toks = ents[n]
            if not toks or not toks[0] or toks[0].endswith("-attrib"):
                continue
            seen.add(n)
            if toks[0] == "point" and len(toks) >= 7:
                try:
                    pts.append(tuple(float(toks[k]) for k in (4, 5, 6)))
                except ValueError:
                    pass
            stack.extend(refs(toks))
        if not pts:
            out.append({"body": bi, "points": 0, "bbox": None})
            continue
        bb = [min(p[i] for p in pts) for i in range(3)] + \
             [max(p[i] for p in pts) for i in range(3)]
        out.append({"body": bi, "points": len(pts),
                    "bbox": [round(v, 4) for v in bb]})
    return out


# ==========================================================================
# 六、plan 装载
# ==========================================================================
def _seg_from_dict(d):
    t = d.get("type")
    p0 = (float(d["p0"][0]), float(d["p0"][1]))
    p1 = (float(d["p1"][0]), float(d["p1"][1]))
    if t == "line":
        return [("line", p0, p1)]
    if t == "arc":
        c = (float(d["c"][0]), float(d["c"][1]))
        r = float(d["r"])
        ccw = bool(d.get("ccw", True))
        return [("arc", c, r, p0, p1, ccw)]
    if t == "circle":
        c = (float(d["c"][0]), float(d["c"][1]))
        r = float(d["r"])
        # 整圆拆半，避免闭合边
        return [("arc", c, r, (c[0] + r, c[1]), (c[0] - r, c[1]), True),
                ("arc", c, r, (c[0] - r, c[1]), (c[0] + r, c[1]), True)]
    raise ValueError("未知 profile 段类型 %r" % t)


def _parse_wire(spec, fallback_points):
    """spec 给解析线框（seg dict 列表）时优先；否则对折线做圆弧拟合。"""
    if spec:
        segs = []
        for d in spec:
            segs += _seg_from_dict(d)
        return _split_long_arcs(segs)
    return wire_from_points(fallback_points)


def load_parts(plan):
    """plan.json -> ([PartSolid], issues)。"""
    parts, issues = [], []
    for i, p in enumerate(plan.get("parts", [])):
        name = p.get("name") or "part-%d" % (i + 1)
        op = p.get("op")
        if op != "extrude":
            issues.append({"part": name, "code": "op-unsupported",
                           "msg": "op=%s 暂不能输出解析 B-rep（STP/SAT 需解析曲面）；"
                                  "该件请走 rebuild3d 的 GLB/STL 路线。" % op})
            continue
        depth = p.get("depth")
        try:
            depth = float(depth)
        except (TypeError, ValueError):
            depth = 0.0
        if depth <= 0:
            issues.append({"part": name, "code": "depth-missing",
                           "msg": "extrude 缺 depth，已跳过"})
            continue
        axis = (p.get("axis") or "Z").upper()
        if axis not in FRAMES:
            issues.append({"part": name, "code": "axis-bad", "msg": "未知 axis=%r" % axis})
            continue
        prof = p.get("profile") or {}
        try:
            outer = _parse_wire(prof.get("wire"), prof.get("shell") or [])
            hole_wires = prof.get("hole_wires") or []
            holes = []
            for j, hp in enumerate(prof.get("holes") or []):
                spec = hole_wires[j] if j < len(hole_wires) else None
                holes.append(_parse_wire(spec, hp))
        except Exception as exc:  # noqa: BLE001
            issues.append({"part": name, "code": "profile-bad", "msg": str(exc)})
            continue
        off = p.get("translate") or [0.0, 0.0, 0.0]
        parts.append(PartSolid(name, axis, depth, outer, holes, off,
                               p.get("color"), p.get("note")))
    return parts, issues


def normalize_parts(parts, mode="center_xz_ground_y"):
    """与 rebuild3d 同规则：XZ 居中、Y 贴地。返回平移量。"""
    if mode == "none" or not parts:
        return (0.0, 0.0, 0.0)
    pts = []
    for p in parts:
        pts += p.corners()
    mins = [min(q[i] for q in pts) for i in range(3)]
    maxs = [max(q[i] for q in pts) for i in range(3)]
    off = (-(mins[0] + maxs[0]) / 2.0, -mins[1], -(mins[2] + maxs[2]) / 2.0)
    for p in parts:
        p.shift(off)
    return off


# ==========================================================================
# 七、【已停用存档】本机 CAD 内核路线（SAT / DGN / 3D-DXF）
# ==========================================================================
#
# 机械实体输出自 2026-10-16 起**只保留 STEP**（纯 Python，无外部依赖）。
# 以下整段 —— 剖面 DXF、CAD 主程序 / ProgID 探测、AutoLISP 命令链、
# COM + .scr 双通道、进程管理与僵尸清理 —— 都是为「借本机已授权 CAD
# 出 SAT/DGN/3D-DXF」而写的，现**不再由 CLI 暴露**，仅作存档保留。
#
# 停用原因：SAT/DGN/3D-DXF 必须本机有已授权 CAD 内核，发布给没有 CAD 的
# 用户时完全不可用；而 STEP 已能直接进 SolidWorks/UG/Creo/Inventor/CATIA。
# 若要恢复：把 main() 里的 CAD 分支接回即可，本段函数仍然可用。
# 关键结论（COM 策略对照 / 剖面预置位折算表 / ERASE 清孔柱 / DXFOUT 出 DXF）
# 见 SKILL.md 的「历史路线存档」节与 .workbuddy/memory/。
# ==========================================================================
def _wire_to_lwpolyline(segs):
    """返回 [(x,y,bulge)...]：每个顶点的 bulge 描述「自该顶点出发的那一段」。"""
    pts = []
    for s in segs:
        if s[0] == "line":
            pts.append((s[1][0], s[1][1], 0.0))
        else:
            _, c, r, a, b, ccw = s
            sweep = _arc_sweep(c, a, b, ccw)
            pts.append((a[0], a[1], math.tan(sweep / 4.0)))
    return pts


def _profile_placement(part):
    """把 plan 的 `translate` **折算成剖面自己的摆放**：(du, dv, elevation)。

    ★ 为什么不在 CAD 里做平移（2026-10-15 实测，两条路都不通）：
      - `(command "_.MOVE" ss "" "0,0,0" (list x y z))` → 位移成了 **(+50,0,0)** 这种
        毫无关系的量，而且会把后续命令流带偏（之后所有 `ssget` 都选空）；
      - 改成 `vla-move`（ActiveX）同样**没生效**：正式图纸上 7 件里 6 件的
        `translate` 全丢（垫板停在 `z[0,440]` 而应 `[-220,220]`）。

      与其和这个 CAD 的提示符/ActiveX 缠斗，不如**根本不在 CAD 里平移**：
      把平移量事先折进剖面 —— 剖面在 (u,v) 平面内平移 `(du,dv)`，
      「拉伸方向分量」则变成 LWPOLYLINE 的 `elevation`（`EXTRUDE` 从剖面所在平面
      起算，标高 e 就等价于沿拉伸方向平移 e）。这样整条链里**一个平移命令都不需要**。

    折算依据（`R` = 拉伸后 `ROTATE3D` 实现的旋转映射）：
        axis Z: R(u,v,t)=(u,v,t)   → du= ox, dv= oy, e= oz
        axis Y: R(u,v,t)=(u,t,-v)  → du= ox, dv=-oz, e= oy
        axis X: R(u,v,t)=(t,v,-u)  → du=-oz, dv= oy, e= ox
    因为 `world = R(u,v,t) + offset` 等价于 `(du,dv,e) = R⁻¹(offset)`。
    （B-rep 侧语义是「先从 t=0 拉伸 depth，再整体平移 offset」，与此一致。）
    """
    ox, oy, oz = part.offset
    if part.axis == "Y":
        return ox, -oz, oy
    if part.axis == "X":
        return -oz, oy, ox
    return ox, oy, oz


def _shift_wire(pts, du, dv):
    """[(_wire_to_lwpolyline 的输出)] → 平面内整体平移 (du,dv)。"""
    return [(x + du, y + dv, b) for (x, y, b) in pts]


def write_profile_dxf(parts, path):
    """把每件的轮廓/孔写成 DXF，**每类都放独立图层**，并**预先把平移量折进去**。

    图层约定（CAD 侧全靠它做选择）：
        KHO<i>       第 i 件的外轮廓
        KHH<i>_<j>   第 i 件的第 j 个孔
        P<i>         第 i 件挤出后的实体（挤出前先 CLAYER 切到这里）
        T<i>         第 i 件的孔柱（临时层，SUBTRACT 后应当空掉）
    `P<i>` / `T<i>` 必须**预先建好**：`(setvar "CLAYER" "P0")` 在图层不存在时会失败。

    摆放见 `_profile_placement`：剖面坐标 + `elevation` 已经把 `translate` 吃掉了，
    所以 CAD 侧只需 `EXTRUDE` + `ROTATE3D`，不需要任何 `MOVE`。
    """
    import ezdxf
    doc = ezdxf.new("R2010", setup=False)
    msp = doc.modelspace()
    for i, part in enumerate(parts):
        doc.layers.add("KHO%d" % i)
        doc.layers.add("P%d" % i)
        doc.layers.add("T%d" % i)
        du, dv, elev = _profile_placement(part)
        msp.add_lwpolyline(
            _shift_wire(_wire_to_lwpolyline(part.outer), du, dv), format="xyb",
            close=True,
            dxfattribs={"layer": "KHO%d" % i, "elevation": elev})
        for j, h in enumerate(part.holes):
            lay = "KHH%d_%d" % (i, j)
            doc.layers.add(lay)
            msp.add_lwpolyline(
                _shift_wire(_wire_to_lwpolyline(h), du, dv), format="xyb",
                close=True, dxfattribs={"layer": lay, "elevation": elev})
    doc.saveas(path)
    return path


# 可 headless 跑脚本的本地 CAD（按优先级）。
#
# ★ 优先 V25（kunhengcad.exe）：本机**只有 V24/V25 有授权文件**
#   （C:\ProgramData\KunHeng\KunHengCADV24.lic / KunHengCADV25.lic），
#   V23 **没有** .lic —— 缺授权的实例里 `ACISOUT` 之类 PRO 级命令会被门禁。
#   V24 目录只剩一个空的 UserDataCache，实际不可用。
#
# ★ 启动套件的坑（导致 /b 脚本通道时好时坏）：
#   `%APPDATA%\KunHeng\KunHengCAD\V25x64\zh_CN\appload.dfs` 里挂了 4 个
#   <某行业二次开发插件> 的 **debug 构建** dll，而这 4 个路径**全部不存在**：
#       C:\Program Files\<插件目录>\<插件A>.dll
#       <用户目录>\Desktop\...\<插件B>.dll
#       <用户目录>\Desktop\...\<插件A>.dll
#       <用户目录>\Desktop\<插件目录>\...\<插件A>.dll
#   加载失败会弹模态框/卡在创建主窗口之前（实测进程存活 60s+ 但零窗口、
#   脚本第一行打点都写不出）。`/Automation`（COM）模式会绕过这些 UI，
#   因此**桥接优先走 COM**，`.scr` 只作兜底。
CAD_EXE_CANDS = [
    r"D:/Program Files/KunHeng/KunHengCAD V25 zh_CN/kunhengcad.exe",
    r"D:/Program Files/KunHeng/KunHengCAD V24 zh_CN/kunhengcad.exe",
    r"D:/Program Files/KunHeng/KunHengCAD V23 zh_CN/kcad.exe",
    r"C:/Program Files/Bricsys/BricsCAD V25 zh_CN/bricscad.exe",
]


def find_cad_exe():
    """找一个可 headless 跑脚本的本地 CAD。

    可用环境变量 `CAD_DRAW3D_EXE` 强制指定（排在最前）。
    """
    env = os.environ.get("CAD_DRAW3D_EXE")
    cands = ([env] if env else []) + CAD_EXE_CANDS
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


# COM 自动化：注册表 HKCU\Software\Classes\CLSID 里 LocalServer32 指向
#   ...KunHengCAD V25 zh_CN\kunhengcad.exe /Automation
# 注意 V23 的注册指向「V23\kunhengcad.exe」而这个文件并不存在（V23 里叫 kcad.exe），
# 直接 Dispatch("BricscadApp.AcadApplication.23.0") 会报「没有注册类」。
# 所以按版本号从高到低探测，并对每个 ProgID 做一次真实 Dispatch 验证。
CAD_PROGIDS = [
    "BricscadApp.AcadApplication.25.0",
    "BricscadApp.AcadApplication.24.0",
    "BricscadApp.AcadApplication.23.0",
    "BricscadApp.AcadApplication",
]


def find_cad_progid():
    """静态挑一个 COM ProgID：读它的 CLSID 的 LocalServer32，确认 exe 真的在磁盘上。

    不要用「Dispatch 试一次再 Quit」来验证 —— 每试一个 ProgID 就真拉起一个 CAD 实例，
    多个实例互相阻塞，实测会把一次调用拖到 6 分钟以上还不返回。
    """
    try:
        import winreg
    except ImportError:
        return None

    def _exe_from_localserver(cmd):
        cmd = (cmd or "").strip()
        low = cmd.lower()
        i = low.find(".exe")
        if i < 0:
            return None
        cand = cmd[:i + 4].strip().strip('"')
        return cand if os.path.isfile(cand) else None

    for pid in CAD_PROGIDS:
        for hive in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
            try:
                with winreg.OpenKey(hive, r"Software\Classes\%s\CLSID" % pid) as k:
                    clsid = winreg.QueryValueEx(k, "")[0]
            except OSError:
                continue
            if not clsid:
                continue
            for h2 in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
                for view in ("", r"Wow6432Node\\"):
                    try:
                        sub = r"Software\Classes\%sCLSID\%s\LocalServer32" % (view, clsid)
                        with winreg.OpenKey(h2, sub) as k:
                            cmd = winreg.QueryValueEx(k, "")[0]
                    except OSError:
                        continue
                    if _exe_from_localserver(cmd):
                        return pid
    return None


def _fwd(p):
    return p.replace("\\", "/")


def _file_kind(path):
    """按**文件头**判断真实格式，不看扩展名。

    这是被 `SAVEAS` 坑出来的：本机 CAD 的 `SAVEAS` 一律产出 DWG，
    但落盘名字是 `<目标>.dxf.dwg`，拷贝时若不校验就会「文件名叫 .dxf、
    内容其实是 DWG」地把错配交付出去（2026-10-15 实测踩到）。

    返回 'dwg' / 'dxf' / 'dgn' / 'sat' / 'stp' / 'unknown'。
    """
    try:
        with open(path, "rb") as fh:
            head = fh.read(16)
    except Exception:  # noqa: BLE001
        return "unknown"
    if head[:4] == b"AC10":                      # AC1015/AC1027/AC1032 … 都是 DWG
        return "dwg"
    if head[:4] == b"Auto":                      # "AutoCAD Binary DXF"
        return "dxf"
    if head[:4] == b"\xd0\xcf\x11\xe0":          # OLE2 复合文档（DGN V8）
        return "dgn"
    if head.lstrip()[:4] in (b"0\r\n0", b"0\n0") or b"SECTION" in head:
        return "dxf"                             # ASCII DXF 以 0/SECTION 开头
    if head.lstrip()[:2] == b"70":               # SAT 头 "700 <n> <m> 0"
        return "sat"
    if head[:12] == b"ISO-10303-21":
        return "stp"
    return "unknown"


# ---- AutoLISP 打点 ----------------------------------------------------------
# 两个踩过的坑：
# 1) **AutoLISP 符号名大小写不敏感**。曾把日志路径变量写成 `LG`、函数写成
#    `(defun lg ...)`，两者是同一个符号，defun 把路径字符串覆盖成函数对象，
#    之后 `(open LG "a")` 全部报错，日志一行都写不出来。
# 2) `(defun ...)` 在 .scr 通道里也不可靠。所以打点一律用**内联 progn + 字面量路径**，
#    这也是实测唯一稳定写盘的写法：
#       (progn (setq f (open "路径" "a")) (write-line "文本" f) (close f))
CAD_LOGPATH_SYM = "cd3logp"
CAD_CURVE_LAYER = "KHO%d"
CAD_HOLE_LAYER = "KHH%d_%d"
CAD_PART_LAYER = "P%d"


def _cad_lisp_helpers():
    """需要预定义的 AutoLISP 辅助函数（**每个必须是单行**）。

    ★ 现阶段只剩 `(vl-load-com)`：
      平移改成了「剖面预置位」（见 `_profile_placement`），
      旋转用 `_.ROTATE3D`（实测有效），所以不再需要 `vla-move` 之类的辅助函数。
      保留这个函数是为了给后续可能的 ActiveX 辅助留统一入口 ——
      注意 `defun` **必须写成整行**：命令通道一次只求值一行完整表达式，
      多行 `defun` 会静默失败（探针里踩过）。
    """
    return ['(vl-load-com)']


def _cad_header_lines(log_path=None):
    return (['(setvar "CMDECHO" 0)', '(setvar "FILEDIA" 0)',
             '(setvar "EXPERT" 5)', '(setvar "OSMODE" 0)',
             '(setvar "HIGHLIGHT" 0)'] + _cad_lisp_helpers())


def _cad_mark(text, log_path):
    if not log_path:
        return None
    return ('(progn (setq cd3fh (open "%s" "a")) (write-line "%s" cd3fh)'
            ' (close cd3fh))' % (_fwd(log_path), text))


def _cad_geometry_lines(parts, log_path=None, mark=True):
    """DXF 剖面 -> 逐件拉伸(挖孔) -> 按 axis 旋转 -> 平移到 plan 的 translate。

    ★ 实体归属怎么定（这是整条链里最容易踩错的一环，2026-10-15 用逐步诊断定死）：
      实测 **`EXTRUDE` 生成的新实体落在「当前层」(CLAYER) 上，而不是源剖面所在的层**。
      剖面在 `KHO<i>`，挤出后实体却跑到图层 `0` —— 于是 `(ssget "X" (list (cons 8 "KHO<i>")))`
      再也选不到它，`SUBTRACT` 空选失效、分件 ACISOUT 也空选。
      早期版本想用 `(command "_.CHPROP" "_LAST" "" "_LA" "P<i>" "")` 事后补层，
      实测**打层不生效**（诊断日志里 `p<i>_on_P<i>=0`），整条链因此崩掉。
      现在改成**挤出之前先切当前层**：
          `(command "_.CLAYER" "P<i>")`  →  EXTRUDE  →  实体自然就生在 `P<i>` 上。
      孔柱同理切到临时层 `T<i>`，之后 `SUBTRACT` 用两个按层取得的选集，
      完全不依赖 `_LAST` / `entlast` / `CHPROP`。

    绕轴旋转量与 FRAMES 严格对应（右手定则，已逐项验算）：
        axis Y: (x,y,z)->(x,z,-y)  <=>  绕 X 轴 -90°
        axis X: (x,y,z)->(z,y,-x)  <=>  绕 Y 轴 +90°
    """
    L = []
    for i, part in enumerate(parts):
        d = part.depth
        pl = "P%d" % i                                 # 本件实体层
        tl = "T%d" % i                                 # 本件孔柱临时层
        sel = '(ssget "X" (list (cons 8 "%s")))' % pl
        _mk(L, "p%d-start" % i, mark and log_path or None)
        # 当前层先切到 P<i>，EXTRUDE 的产物才会落在 P<i>。
        # 注意 `CLAYER` 是**系统变量**不是命令：必须 `(setvar "CLAYER" ...)`，
        # 写成 `(command "_.CLAYER" ...)` 在 AutoCAD 系里会去开图层面板。
        L.append('(setvar "CLAYER" "%s")' % pl)
        L.append('(command "_.EXTRUDE" (ssget "X" (list (cons 8 "KHO%d"))) "" %.6f)' % (i, d))
        _mk(L, "p%d-extruded" % i, mark and log_path or None)
        if part.holes:
            L.append('(setvar "CLAYER" "%s")' % tl)
            for j in range(len(part.holes)):
                L.append('(command "_.EXTRUDE"'
                         ' (ssget "X" (list (cons 8 "KHH%d_%d"))) "" %.6f)' % (i, j, d))
            # 一次性把本件所有孔柱减掉：minuend 选集 -> 回车 -> subtrahend 选集 -> 回车
            L.append('(command "_.SUBTRACT" %s "" (ssget "X" (list (cons 8 "%s"))) "")'
                     % (sel, tl))
            # ★★ SUBTRACT 在这个 CAD(ODA 内核) 上**只改形、不删源实体**：
            #    实测布尔后底板确实带孔了（SAT 里 8 面 = 6 平面 + 2 圆柱面），
            #    但 `T<i>` 图层上 2 个孔柱实体**原地保留**（逐行计数：布尔前后
            #    T0=2 / sol=3 完全没变）。后果是合并 `ACISOUT "_ALL"` 会多导出
            #    这 2 个孔柱 —— 7 件变 9 个 body（saddle.sat 实测 body=9 face=50
            #    = 正确的 44 面 + 2 个「1 圆柱 + 2 平盖」共 6 面）。
            #    这里显式把孔柱层清空；`if` 包一层是为了空集时 ERASE 不报错吞掉后续行。
            L.append('(if (ssget "X" (list (cons 8 "%s")))'
                     ' (command "_.ERASE" (ssget "X" (list (cons 8 "%s"))) ""))'
                     % (tl, tl))
            _mk(L, "p%d-holes" % i, mark and log_path or None)
        if part.axis == "Y":
            L.append('(command "_.ROTATE3D" %s "" "_X" "0,0,0" -90.0)' % sel)
        elif part.axis == "X":
            L.append('(command "_.ROTATE3D" %s "" "_Y" "0,0,0" 90.0)' % sel)
        # ★ 这里**故意没有 MOVE**：平移量已经在 `write_profile_dxf` 里折算成
        #   剖面坐标 + `elevation`（见 `_profile_placement`）。实测这个 CAD 上
        #   `_.MOVE` 与 `vla-move` 都不可靠，与其缠斗不如彻底不用。
        _mk(L, "p%d-placed" % i, mark and log_path or None)
    return L


def _mk(L, text, log_path):
    m = _cad_mark(text, log_path)
    if m:
        L.append(m)


def _cad_export_lines(sat_path, dgn_path, out_dxf_path=None, log_path=None,
                      part_sat_tmps=None):
    """导出顺序：分件 SAT -> 合并 SAT -> 3D DXF -> **DGN 最后**。

    DGN 放最后是刻意的：`DGNEXPORT` 在很多 CAD 上语法/参数各不相同，一旦它卡住，
    前面的 SAT/3D-DXF 已经落盘，损失最小。DGN 再补一组候选语法做条件重试。
    """
    L = []
    for i, p in enumerate(part_sat_tmps or []):
        f = _fwd(p)
        # 两参数（选集 + 文件名）；落不了盘再退到三参数形态。
        L.append('(command "_.ACISOUT" (ssget "X" (list (cons 8 "P%d"))) "%s")'
                 % (i, f))
        L.append('(if (not (findfile "%s"))'
                 ' (command "_.ACISOUT" (ssget "X" (list (cons 8 "P%d"))) "" "%s"))'
                 % (f, i, f))
        _mk(L, "part%d.sat" % i, log_path)
    if sat_path:
        f = _fwd(sat_path)
        L.append('(command "_.ACISOUT" "_ALL" "%s")' % f)
        L.append('(if (not (findfile "%s")) (command "_.ACISOUT" "_ALL" "" "%s"))'
                 % (f, f))
        _mk(L, "sat-done", log_path)
    if out_dxf_path:
        # ★ 2026-10-15 探针定论（`cadtest/dxfprobe.py`）：本机 CAD 上
        #   `SAVEAS` 的**所有**形态都只会退化成 DWG（`path.dwg`，AC1015/AC1027/AC1032），
        #   而真正能写出 DXF 的是 **`DXFOUT`** —— 实测 `DXFOUT <path> "16"` 得 97219 B
        #   ASCII-DXF，里面**真带 3DSOLID**（ezdxf 能读出 `{'3DSOLID': 1}`）。
        #   所以这里一律用 DXFOUT，三形态按「参数完整 → 最简」排序，落盘即止。
        p = _fwd(out_dxf_path)
        L.append('(command "_.DXFOUT" "%s" "V" "2013" "16" "_ALL" "")' % p)
        L.append('(command "_.DXFOUT" "%s" "16" "_ALL" "")' % p)
        L.append('(command "_.DXFOUT" "%s" "16")' % p)
        _mk(L, "dxf3d-done", log_path)
    if dgn_path:
        p = _fwd(dgn_path)
        # 与 COM 通道保持同一组候选（菜单宏是 `^c^c_dgnexport`，命令行形态待定）
        for _form in ('(command "_.DGNEXPORT" "%s")' % p,
                      '(command "_.-DGNEXPORT" "%s")' % p,
                      '(command "_.-DGNEXPORT" "_ALL" "" "%s")' % p,
                      '(command "_.EXPORT" "_DGN" "%s" "_ALL" "")' % p):
            L.append('(if (not (findfile "%s")) %s)' % (p, _form))
        _mk(L, "dgn-done", log_path)
    return L


def _cad_export_candidates(sat_path, dgn_path, out_dxf_path=None):
    """COM 通道用：每个目标给若干候选语法，落盘即停。

    各家 CAD 在这三条命令上的交互次数并不一致（ACISOUT 选集后是否再确认、
    DGNEXPORT 是否吃 ALL、SAVEAS 的格式/版本参数个数），所以不赌单条。
    """
    jobs = []
    if sat_path:
        p = _fwd(sat_path)
        # ★ 顺序是有实测依据的：**两参数形式必须排第一**。
        #   `(command "_.ACISOUT" "_ALL" "path")`  —— 实测产出 67519 bytes 的真 ACIS SAT；
        #   `(command "_.ACISOUT" "_ALL" "" "path")` —— 多给的那个回车会把"文件名"
        #   提示符吃掉，命令返回 nil、不落盘（曾被我误判成"没有 PRO 授权"）。
        #   即：ACISOUT 的选集提示符收到一次完整响应后即结束，不需要再补回车。
        jobs.append(("sat", sat_path, [
            '(command "_.ACISOUT" "_ALL" "%s")' % p,
            '(command "_.ACISOUT" "_ALL" "" "%s")' % p,
            '(command "_.-ACISOUT" "_ALL" "%s")' % p,
            '(command "_.EXPORT" "_SAT" "%s" "_ALL" "")' % p,
        ]))
    if dgn_path:
        p = _fwd(dgn_path)
        # `DGNEXPORT` 的菜单宏是 `^c^c_dgnexport`（单下划线，**不是**连字符变体），
        # 所以命令行形态到底是 `DGNEXPORT` 还是 `-DGNEXPORT` 得靠试。四种都摆上，
        # 落盘即停。DGN 排在 SAT 之后、DXF 之前。
        jobs.append(("dgn", dgn_path, [
            '(command "_.DGNEXPORT" "%s")' % p,
            '(command "_.-DGNEXPORT" "%s")' % p,
            '(command "_.DGNEXPORT" "%s" "")' % p,
            '(command "_.-DGNEXPORT" "_ALL" "" "%s")' % p,
            '(command "_.EXPORT" "_DGN" "%s" "_ALL" "")' % p,
            '(command "_.EXPORT" "_DGN" "%s")' % p,
        ]))
    if out_dxf_path:
        p = _fwd(out_dxf_path)
        # 放最后：即使语法不合、留下悬挂输入，也不会影响 SAT/DGN。
        # ★ 用 `DXFOUT` 不用 `SAVEAS` —— 见 `_cad_export_lines` 里的探针定论：
        #   本机 CAD 的 `SAVEAS` 只会出 DWG（`path.dwg`），DXFOUT 才出真 DXF 且含 3DSOLID。
        jobs.append(("dxf", out_dxf_path, [
            '(command "_.DXFOUT" "%s" "V" "2013" "16" "_ALL" "")' % p,
            '(command "_.DXFOUT" "%s" "16" "_ALL" "")' % p,
            '(command "_.DXFOUT" "%s" "16")' % p,
        ]))
    return jobs


def _wait_file(path, secs):
    """等文件出现，并且**尺寸稳定**（避免读到正在写一半的内容）。"""
    import time
    t = time.time() + secs
    last = -1
    stable = 0
    while time.time() < t:
        if os.path.isfile(path):
            sz = os.path.getsize(path)
            if sz > 0 and sz == last:
                stable += 1
                if stable >= 2:
                    return True
            else:
                stable = 0
            last = sz
        time.sleep(0.5)
    return os.path.isfile(path) and os.path.getsize(path) > 0


def _kill_proc_tree(proc):
    """收掉 CAD 进程。

    脚本里的 `QUIT` 在这个 CAD 上**不可靠**：实测脚本跑完几何/导出后停在
    「是否保存修改」一步不退出（普通 QUIT/N 的空脚本反而能退）。所以不依赖它，
    拿到产物后由我们主动终止；进程若是提权拉起的，taskkill 会「拒绝访问」，
    再退到 WMI 强杀。
    """
    import subprocess
    if proc is None:
        return "no-proc"
    if proc.poll() is not None:
        return "exited(rc=%s)" % proc.returncode
    try:
        proc.terminate()
        proc.wait(timeout=8)
        return "terminated"
    except Exception:  # noqa: BLE001
        pass
    try:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
        return "taskkill"
    except Exception:  # noqa: BLE001
        pass
    try:
        import win32com.client as win32
        wmi = win32.GetObject("winmgmts:")
        for p in wmi.ExecQuery("SELECT * FROM Win32_Process WHERE ProcessId=%d" % proc.pid):
            p.Terminate()
        return "wmi"
    except Exception:  # noqa: BLE001
        return "failed"


def build_cad_script(parts, dxf_path, script_path, log_path, sat_path, dgn_path,
                     out_dxf_path=None, part_sat_tmps=None):
    """生成 .scr：DXF -> 拉伸 -> 摆放 -> 分件/合并 ACISOUT + DGNEXPORT + 3D DXF。

    `.scr` 里是一行一条 `(command ...)` 的 AutoLISP（实测该 CAD 的脚本通道会正常
    求值 LISP），并全程用 `cd3mark` 打点，便于卡住时定位。
    脚本**不写 QUIT**：退出由调用方在产物就绪后主动终止进程（见 _kill_proc_tree）。
    """
    L = _cad_header_lines(log_path)
    # ★ 用 DXFIN 而**不是** `_.OPEN`。实测：脚本一行行跑，执行到 `_.OPEN` 打开另一张
    #   图纸后，后面所有行就再也不执行了（`.dwl/.dwl2` 锁文件表明文档确实打开了，
    #   但 OPEN 之后的打点一个都写不出来）。DXFIN 是把剖面**导入当前空白图**，
    #   不切换文档，脚本能一路跑到底（实测 boot/dxfin/extruded/acisout 全部落盘）。
    L.append('(command "_.DXFIN" "%s")' % _fwd(dxf_path))
    _mk(L, "dxfin", log_path)
    L += _cad_geometry_lines(parts, log_path=log_path)
    _mk(L, "geom-done", log_path)
    L += _cad_export_lines(sat_path, dgn_path, out_dxf_path, log_path=log_path,
                           part_sat_tmps=part_sat_tmps)
    _mk(L, "all-done", log_path)
    text = "\n".join(L) + "\n"
    with open(script_path, "w", encoding="gbk", errors="replace", newline="\r\n") as f:
        f.write(text)
    return script_path


def _cad_pids():
    """当前存活的 CAD 进程 PID 集合。

    用于「只清理本次新拉起的实例」——绝不能误杀用户自己开着的那个会话
    （实测用户会话常驻 CAD 主程序，标题形如 `<设计软件名> - [xxx.dwg]`）。
    """
    import subprocess
    s = set()
    for name in ("kunhengcad.exe", "kcad.exe", "bricscad.exe"):
        try:
            r = subprocess.run(["tasklist", "/FI", "IMAGENAME eq " + name,
                                "/FO", "CSV", "/NH"],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                               timeout=30)
        except Exception:  # noqa: BLE001
            continue
        out = (r.stdout or b"").decode("gbk", errors="replace")
        for line in out.splitlines():
            parts = [x.strip('"') for x in line.split('","')]
            if len(parts) >= 2 and parts[1].isdigit():
                s.add(int(parts[1]))
    return s


def _cad_visible_pids():
    """有**可见顶层窗口**的 CAD 进程 PID —— 也就是用户自己的交互会话。

    判定依据（2026-10-15 实测）：用户会话一定有可见主框架
    （标题形如 `<设计软件名> - [xxx.dwg]`）；
    而本工具经 `/Automation`(COM) 或 `/l /b` 起的实例**没有任何可见顶层窗口**。
    """
    import ctypes
    from ctypes import wintypes
    u = ctypes.windll.user32
    u.GetWindowThreadProcessId.argtypes = [wintypes.HWND,
                                           ctypes.POINTER(wintypes.DWORD)]
    u.IsWindowVisible.argtypes = [wintypes.HWND]
    u.IsWindowVisible.restype = wintypes.BOOL
    u.EnumWindows.argtypes = [ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND,
                                                 wintypes.LPARAM),
                              wintypes.LPARAM]
    vis = set()
    cad_pids = _cad_pids()

    def cb(hwnd, _):
        if not u.IsWindowVisible(hwnd):
            return True
        pid = wintypes.DWORD()
        u.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if pid.value in cad_pids:
            vis.add(pid.value)
        return True

    u.EnumWindows(ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND,
                                     wintypes.LPARAM)(cb), 0)
    return vis


def _cad_orphan_pids():
    """没有可见窗口的 CAD 实例 = 本工具此前遗留的自动化「僵尸」。

    危害很实在：它们会一直占着工作目录里的 `.dwg/.dxf/.dwl` 锁，
    下一轮 `write_profile_dxf` 直接 `PermissionError`（实测白跑 9 分钟）。
    按「无可见窗口」判定是安全的 —— 用户自己的会话永远有可见主窗口。
    """
    return _cad_pids() - _cad_visible_pids()


def _kill_pids(pids):
    """清掉指定 PID 的 CAD 实例。

    ★ 首选 Win32 `TerminateProcess`：实测这台机器上 `taskkill /F`（普通权限）会被
      「拒绝访问」打回，`Stop-Process` 静默失效，WMI `Terminate` 被安全策略拦下 ——
      只有直接调 `kernel32!TerminateProcess` 稳定有效。taskkill 作为兜底。
    """
    import ctypes
    import subprocess
    n = 0
    for pid in sorted(pids):
        done = False
        try:
            k = ctypes.windll.kernel32
            h = k.OpenProcess(0x0001, False, int(pid))     # PROCESS_TERMINATE
            if h:
                done = bool(k.TerminateProcess(h, 1))
                k.CloseHandle(h)
        except Exception:  # noqa: BLE001
            done = False
        if not done:
            try:
                r = subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=20)
                done = r.returncode == 0
            except Exception:  # noqa: BLE001
                pass
        n += 1 if done else 0
    return n


def _com_open_doc(app, path=None, tries=6, wait=4.0, diag=None):
    """拿回一个**真正能用**的 Document 对象（带重试）。

    ★★ 顺序至关重要 —— **`Documents.Item(Count-1)` 必须排第一，`Open()` 只能垫底**
       （2026-10-15 用 comitem.py 做过四策略对照，定论）：

       | 策略                          | 结果 |
       |-------------------------------|------|
       | **DispatchEx + `Item(Count-1)`** | **成功，全程 73s**：`Count=1` → `Item(0)` → `图形1.dwg` |
       | DispatchEx + `Open`           | `Open` 拿到名字后 `SendCommand` 报「远程过程调用失败」 |
       | Dispatch + `Item`             | 拿到 `图形1.dwg` 后 `SendCommand` 报「RPC 服务器不可用」 |
       | Dispatch + `Open`             | 返回空壳 IDispatch |

       新起的 `/Automation` 实例里 `Documents.Open()` 会**无限期挂住不返回**
       （COM 调用没有超时），而 `Documents.Item(Count-1)` 能直接拿到**默认空白图**。
       拿到空白图后用 `DXFIN` 把剖面导进去即可 —— 效果一样，但不碰 Open。
       老版本把 `Open` 排第一位且只认它的返回值，于是**能用的 Item 策略永远轮不到**。

    ★ 为什么还要重试：这条通道是**概率性**的（同上对照里 `DispatchEx+Item` 一次成功，
      但正式跑时同样代码出现过 `Count`/`Item` 直接抛异常）。所以给 `tries × wait`
      的重试窗口；调用方还会在外层换实例重来。每次失败原因写进 `diag`（进报告日志）。
    """
    import time
    for attempt in range(1, max(int(tries), 1) + 1):
        # —— 策略 1：默认空白图（首选）——
        try:
            docs = app.Documents
            n = None
            try:
                n = int(docs.Count)
            except Exception as exc:  # noqa: BLE001
                _diag(diag, "r%d Count失败:%s" % (attempt, type(exc).__name__))
            idxs = list(range(n - 1, -1, -1)) if n else [0, 1, 2]
            for idx in idxs:
                try:
                    c = docs.Item(idx)
                except Exception as exc:  # noqa: BLE001
                    _diag(diag, "r%d Item(%d)失败:%s"
                          % (attempt, idx, type(exc).__name__))
                    continue
                try:
                    if c is not None and c.Name:
                        _diag(diag, "r%d Item(%d)=%s" % (attempt, idx, c.Name))
                        return c
                except Exception as exc:  # noqa: BLE001
                    _diag(diag, "r%d Item(%d)空壳:%s"
                          % (attempt, idx, type(exc).__name__))
        except Exception as exc:  # noqa: BLE001
            _diag(diag, "r%d Documents失败:%s" % (attempt, type(exc).__name__))

        # —— 策略 2：ActiveDocument（这个 build 常常不暴露）——
        try:
            c = app.ActiveDocument
            if c is not None and c.Name:
                _diag(diag, "r%d ActiveDocument=%s" % (attempt, c.Name))
                return c
        except Exception:  # noqa: BLE001
            pass

        if attempt < tries:
            time.sleep(wait)

    # —— 垫底：真只有 0 张图时才碰 Open（这条路会挂，交给外层线程超时兜）——
    if path:
        try:
            d = app.Documents.Open(path)
            time.sleep(wait)
            if d is not None and getattr(d, "Name", None):
                _diag(diag, "Open=%s" % d.Name)
                return d
        except Exception:  # noqa: BLE001
            pass
    return None


def _diag(diag, msg):
    """把一条连接诊断写进日志列表（列表不存在就丢弃）。"""
    if diag is not None:
        try:
            diag.append(msg)
        except Exception:  # noqa: BLE001
            pass


def _call_with_timeout(fn, timeout):
    """在独立线程里跑 fn，超时返回 (False, None)；正常返回 (True, fn 的返回值)。

    ★ 为什么必须有这个：**COM 调用本身没有超时**。实测新起的 `/Automation` 实例上
      `Documents.Open()` 会**无限期挂住**（comdiag2.py 卡了 3 分钟仍未返回一分钱输出），
      把整条管线拖死。放进工作线程后主线程可以按 timeout 走人；
      调用方随后杀掉本次新起的 CAD 进程 —— 进程一死，挂住的 COM 调用会立刻以
      RPC 错误返回，僵尸线程也就自然结束了。
    线程是 daemon：即便还卡着，也不会拖住解释器退出。
    """
    import threading
    box = {}

    def _worker():
        try:
            box["r"] = fn()
        except Exception as exc:  # noqa: BLE001
            box["e"] = exc

    th = threading.Thread(target=_worker, daemon=True)
    th.start()
    th.join(timeout)
    if th.is_alive():
        return False, None
    if "e" in box:
        raise box["e"]
    return True, box.get("r")


def run_cad_com(parts, workdir, sat_path, dgn_path, dxf_path=None,
                progid=None, timeout=900, part_sat_tmps=None):
    """用 COM 自动化驱动本地 CAD：DXF 剖面 -> 3DSOLID -> 导出 SAT/DGN/3D-DXF。

    ★ 为什么 COM 是**首选**通道（2026-10-15 实测定论）：
      `kunhengcad.exe /l /b script.scr` 这条通道会被**启动套件**拖死 ——
      `%APPDATA%\\KunHeng\\KunHengCAD\\V25x64\\zh_CN\\appload.dfs` 里挂着 4 个
      <某行业二次开发插件> 的 debug 构建 dll，而这 4 个路径**全部不存在**：
          C:\\Program Files\\<插件目录>\\<插件A>.dll
          ...\\Desktop\\...\\<插件B>.dll
          ...\\Desktop\\...\\<插件A>.dll
          ...\\Desktop\\<插件目录>\\...\\<插件A>.dll
      加载失败会让进程卡在「创建主窗口之前」——实测存活 60s 以上却零个可见窗口、
      脚本第一行打点都写不出来（所以 .scr 通道时灵时不灵）。
      而 `/Automation`（COM）模式不加载那套 UI：实测**在用户自己的 CAD 会话开着
      的情况下**也能起独立实例并跑完全部命令、成功导出 SAT。
      代价是慢：Dispatch 约 85s、Documents.Open 约 55s，所以超时要给足。
    """
    try:
        import pythoncom
        import win32com.client as win32
    except ImportError:
        return False, ["缺少 pywin32，无法走 COM 通道"]
    import time

    os.makedirs(workdir, exist_ok=True)
    prof = os.path.join(workdir, "profiles.dxf")
    write_profile_dxf(parts, prof)

    pid = progid or find_cad_progid()
    if not pid:
        return False, ["没有可用的 CAD COM 服务（BricscadApp.AcadApplication.*）"]

    before = _cad_pids()
    logs = ["ProgID=%s" % pid, "既有 CAD 进程=%s" % sorted(before)]
    # 先清掉陈旧产物，否则「轮询落盘」会把上次的残留当成本次成功
    for p in (sat_path, dgn_path, dxf_path):
        if p:
            for stale in (p, p + ".dwl", p + ".dwl2"):
                try:
                    if os.path.isfile(stale):
                        os.remove(stale)
                except OSError:
                    pass
    pythoncom.CoInitialize()
    app = None
    ok = False
    spawned = set()
    try:
        # ★ 必须用 **DispatchEx**（强制新建独立实例），不能用 Dispatch：
        #   - 用户自己开着 CAD 时该 ProgID 在 ROT（运行对象表）里已有实例，
        #     `win32.Dispatch` 会**优先附加到那个交互会话**，随后的
        #     `SendCommand` 会抛 `com_error(-2147023174, 'RPC 服务器不可用。')`
        #     —— 因为用户的模态循环不响应我们的跨进程 RPC（cadbridge9 / comitem 实测）。
        #   - DispatchEx 走 CLSCTX_LOCAL_SERVER 新建进程，与用户会话互不相干。
        #     注册表里该 ProgID 的 `LocalServer32` 本来就是
        #     `kunhengcad.exe /Automation`，所以新实例天然是 /Automation 模式
        #     （不加载那套行业插件 UI，启动 25~40s）。
        #
        # ★ 换实例重来是必要的：这条通道**概率性**成功（comitem 对照里
        #   `DispatchEx+Item` 一次就通，正式跑同样代码却出现过 Count/Item 抛异常）。
        #   所以最多试 3 轮，每轮换一个全新实例。
        doc = None
        for attempt in range(1, 4):
            base = _cad_pids()
            t0 = time.time()
            try:
                app = win32.DispatchEx(pid)
                logs.append("DispatchEx 新建实例（第%d次）%.1fs"
                            % (attempt, time.time() - t0))
            except Exception as exc:  # noqa: BLE001
                logs.append("DispatchEx 失败(%r)，回退 Dispatch（可能附加到用户会话）"
                            % exc)
                app = win32.Dispatch(pid)
            spawned |= (_cad_pids() - base)
            try:
                app.Visible = False
            except Exception:  # noqa: BLE001
                pass
            logs.append("  app=%s %s"
                        % (getattr(app, "Name", "?"),
                           getattr(app, "Version", "?")))
            t0 = time.time()
            # 拿实例自带的默认空白图 —— 注意**不要**用 Documents.Open，
            # 那个调用在新实例上会无限期挂死（见 _com_open_doc 的顺序说明）。
            doc = _com_open_doc(app)
            if doc is not None:
                logs.append("Document %.1fs: %s" % (time.time() - t0,
                                                   getattr(doc, "Name", "?")))
                break
            logs.append("第%d次取 Document 失败 → 杀掉实例重来" % attempt)
            die = _cad_pids() - base
            if die:
                _kill_pids(sorted(die))
            spawned -= die
            time.sleep(3)
        if doc is None:
            raise RuntimeError("换 3 个实例都取不到 Document（见 _com_open_doc 注释）")
        for name, val in (("FILEDIA", 0), ("CMDECHO", 0), ("EXPERT", 5),
                          ("OSMODE", 0), ("HIGHLIGHT", 0)):
            try:
                doc.SetVariable(name, val)
            except Exception:  # noqa: BLE001
                pass
        # ★ COM 通道里剖面不是「打开」进来的，而是 **DXFIN 导进来**的：
        #   拿到的是默认空白图（实测叫 `图形1.dwg`），必须自己导入 profiles.dxf，
        #   否则后面所有 `(ssget "X" (list (cons 8 "KHO0")))` 都是空集，
        #   EXTRUDE/SUBTRACT/ACISOUT 会全线空转（只会产出空文件）。
        doc.SendCommand("(vl-load-com)\n")
        time.sleep(0.4)
        doc.SendCommand('(command "_.DXFIN" "%s")\n' % _fwd(prof))
        time.sleep(3.0)
        logs.append("DXFIN 剖面已导入")
        # COM 通道不走 build_cad_script，辅助函数要在这里单独送一遍
        for hline in _cad_lisp_helpers():
            doc.SendCommand(hline + "\n")
            time.sleep(0.25)
        logs.append("LISP 辅助函数已就绪")
        for line in _cad_geometry_lines(parts):
            if line.startswith(";"):
                continue
            doc.SendCommand(line + "\n")
            time.sleep(0.15)
        logs.append("几何命令已发送")
        # 导出：候选语法逐个试，落盘即停
        for kind, target, cands in _cad_export_candidates(sat_path, dgn_path,
                                                          dxf_path):
            hit = False
            for k, line in enumerate(cands):
                try:
                    doc.SendCommand("\x1b")      # 先取消上一条未完成的交互
                except Exception:  # noqa: BLE001
                    pass
                doc.SendCommand(line + "\n")
                if _wait_file(target, 30 if k == 0 else 20):
                    logs.append("%s 由候选#%d 落盘（%d bytes）"
                                % (kind.upper(), k + 1, os.path.getsize(target)))
                    hit = True
                    break
                logs.append("%s 候选#%d 未落盘" % (kind.upper(), k + 1))
            if not hit:
                logs.append("%s 全部候选失败" % kind.upper())
        # 分件 SAT（按图层 P<i> 逐件导出，供 BOM/装配用）
        for i, tgt in enumerate(part_sat_tmps or []):
            if not tgt:
                continue
            try:
                if os.path.isfile(tgt):
                    os.remove(tgt)
            except OSError:
                pass
            f = _fwd(tgt)
            sel = '(ssget "X" (list (cons 8 "P%d")))' % i
            for k, line in enumerate([
                    '(command "_.ACISOUT" %s "%s")' % (sel, f),
                    '(command "_.ACISOUT" %s "" "%s")' % (sel, f)]):
                try:
                    doc.SendCommand("\x1b")
                except Exception:  # noqa: BLE001
                    pass
                doc.SendCommand(line + "\n")
                if _wait_file(tgt, 20 if k == 0 else 12):
                    logs.append("分件 SAT #%d 落盘（%d bytes）"
                                % (i + 1, os.path.getsize(tgt)))
                    break
        try:
            doc.Close(False)
        except Exception:  # noqa: BLE001
            pass
        ok = bool(sat_path) and os.path.isfile(sat_path) \
            and os.path.getsize(sat_path) > 0
    except Exception as exc:  # noqa: BLE001
        logs.append("COM 异常: %r" % exc)
    try:
        if app is not None:
            app.Quit()
    except Exception:  # noqa: BLE001
        pass
    try:
        pythoncom.CoUninitialize()
    except Exception:  # noqa: BLE001
        pass
    time.sleep(2)
    new = (_cad_pids() - before) | spawned
    if new:
        logs.append("清理新实例 %s（成功 %d 个）" % (sorted(new), _kill_pids(new)))
    return ok, logs


def run_cad_script(parts, exe, workdir, sat_path, dgn_path, dxf_path=None, timeout=420,
                   part_sat_tmps=None):
    """`.scr` 通道：`cad.exe /l /b script.scr`，产物就绪即主动终止进程。

    为什么不用 `Popen(...).wait()` 等它自己退：脚本里的 QUIT 在这个 CAD 上不可靠
    （实测跑完导出后停在「是否保存修改」）。所以改成「轮询产物 -> 尺寸稳定 -> 杀进程」。
    也**不要加 `/automation`**：该开关下 /b 脚本不执行，进程纯空转。
    """
    import subprocess
    import time
    if not exe:
        return False, ["未找到可用的本地 CAD（昆仑衡CAD / BricsCAD）"]
    os.makedirs(workdir, exist_ok=True)
    prof = os.path.join(workdir, "profiles.dxf")
    scr = os.path.join(workdir, "build.scr")
    lg = os.path.join(workdir, "cad.log")
    for stale in (lg,) + tuple(part_sat_tmps or ()):
        if os.path.exists(stale):
            os.remove(stale)
    write_profile_dxf(parts, prof)
    build_cad_script(parts, prof, scr, lg, sat_path, dgn_path, dxf_path,
                     part_sat_tmps=part_sat_tmps)

    # 只在「必需产物」上设硬等待，DGN 这类易失败的项给有限宽限，避免整体拖到超时
    need = [p for p in (sat_path,) if p]
    opt = [p for p in (dxf_path, dgn_path) if p] + list(part_sat_tmps or [])
    proc = None
    try:
        proc = subprocess.Popen([exe, "/l", "/b", scr], cwd=workdir,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except Exception as exc:  # noqa: BLE001
        return False, ["调用 CAD 失败: %s" % exc]
    t0 = time.time()
    lines_abort = []
    quiet_abort = float(os.environ.get("CAD_DRAW3D_SCR_QUIET", "60"))
    while time.time() - t0 < timeout:
        if need and all(_wait_file(p, 0.6) for p in need):
            break
        if proc.poll() is not None:
            break
        # 早期放弃：脚本一直没跑起来（cad.log 一行都没写、也没有任何产物）
        # 时没必要耗满 timeout —— 实测这台机器上启动套件拖死时就是这样，
        # 白等 7 分钟对用户毫无价值。
        if quiet_abort > 0 and time.time() - t0 > quiet_abort:
            if not (os.path.isfile(lg) and os.path.getsize(lg) > 0):
                if not any(os.path.isfile(p) and os.path.getsize(p) > 0
                           for p in (list(need) + opt)):
                    lines_abort = ["quiet-abort %.0fs：脚本未产生任何输出/产物"
                                   % (time.time() - t0)]
                    break
        time.sleep(0.6)
    # 可选项统一给一次有限宽限（逐项各等 20s 会把整体从 7 分钟拖到 10 分钟）
    if opt:
        deadline = time.time() + 30
        while time.time() < deadline:
            if all(os.path.isfile(p) for p in opt):
                break
            time.sleep(0.6)
    killed = _kill_proc_tree(proc)
    lines = ["exe=%s" % os.path.basename(exe), "kill=%s" % killed,
             "elapsed=%.1fs" % (time.time() - t0)]
    lines += lines_abort
    if os.path.isfile(lg):
        with open(lg, "r", encoding="utf-8", errors="replace") as f:
            lines += [x.strip() for x in f if x.strip()]
    for p in (sat_path, dgn_path, dxf_path):
        if p:
            lines.append("%s = %s" % (os.path.basename(p),
                                      os.path.getsize(p) if os.path.isfile(p) else "MISSING"))
    ok = bool(sat_path) and os.path.isfile(sat_path) and os.path.getsize(sat_path) > 0
    return ok, lines


def _ascii_workdir(sub="cad-draw-3d"):
    """给 CAD 桥接找/建一个**纯 ASCII**、且**每次运行唯一**的工作目录。

    两个坑叠在一起：

    1. **中文路径 ⇒ 整套脚本静默卡死**。命令链在 ASCII 路径下能跑，一旦路径含中文
       （如「桌面\\CAD图生三维模型\\test-鞍座」），连脚本开头的 `(open logpath "a")`
       都写不出文件。所以 CAD 只在临时 ASCII 目录里工作，产物最后再拷回目标路径。
    2. **目录必须每次运行唯一**。踩过一次很贵的：`profiles.dxf` 被上一轮残留的 CAD
       实例锁着，`write_profile_dxf` 直接 `PermissionError`，整次运行（9 分钟）作废。
       用 `<时间戳>-<pid>` 做叶子目录就彻底不会撞锁。
    """
    import time
    leaf = "%s/%s-%d" % (sub, time.strftime("%m%d-%H%M%S"), os.getpid())
    cands = []
    for env in ("TEMP", "TMP"):
        v = os.environ.get(env)
        if v:
            cands.append(os.path.join(v, "caddraw3d", leaf))
    cands += [r"C:/Windows/Temp/caddraw3d/" + leaf, r"C:/caddraw3d/" + leaf]
    for c in cands:
        c = os.path.abspath(c)
        try:
            c.encode("ascii")
        except UnicodeEncodeError:
            continue
        try:
            os.makedirs(c, exist_ok=True)
            return c
        except OSError:
            continue
    return None


def _copy_back(src, dst):
    if not src or not dst or not os.path.isfile(src):
        return False
    import shutil
    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    shutil.copyfile(src, dst)
    return os.path.isfile(dst)


def run_cad_bridge(parts, workdir, sat_path, dgn_path, dxf_path=None, timeout=420,
                   cad_exe=None, exe_timeout=420):
    """CAD 通道总入口：在 ASCII 临时目录里跑，成功后把产物拷回目标路径。

    ★ 顺序：**先 COM，失败再退到 `.scr`**（2026-10-15 实测调换了优先级）。
      原因见 `run_cad_com` 的 docstring：`.scr` 会被启动套件里 4 个缺失的
      debug ARX 拖死（进程存活但零窗口、脚本不执行），而 `/Automation` 不受影响。
      `.scr` 保留作兜底 —— 万一换了台干净机器（没有那套行业插件），它反而更快。
      环境变量 `CAD_DRAW3D_NO_COM=1` / `CAD_DRAW3D_NO_SCR=1` 可分别禁用某一路。

    额外产出「分件 SAT」到 `<sat 同目录>/<stem>.parts/NN.sat`。
    """
    adir = _ascii_workdir() or workdir
    scratch = os.path.join(adir, "scr")
    sat_tmp = os.path.join(adir, "out.sat") if sat_path else None
    dgn_tmp = os.path.join(adir, "out.dgn") if dgn_path else None
    dxf_tmp = os.path.join(adir, "out3d.dxf") if dxf_path else None
    part_tmps = [os.path.join(adir, "part%02d.sat" % i) for i in range(len(parts))]

    # 两个通道各自 try/except：一条路抛异常（比如 DXF 被上一轮残留实例锁住导致
    # PermissionError）不该把另一条路和整个流程一起带走，日志也必须照常带回去。
    logs = []

    # ★ 先清掉「没有可见窗口」的 CAD 僵尸 —— 那是本工具前几轮留下的自动化实例
    #   （用户在界面上看不到它们，但它们握着工作目录 DXF/DWG 的锁）。
    #   不清的话下一轮 write_profile_dxf 会直接 PermissionError。
    #   用户自己的交互会话有可见主窗口，不会被误伤。
    try:
        orph = _cad_orphan_pids()
        if orph:
            logs.append("[clean] 清理无窗口 CAD 僵尸 %s（成功 %d）"
                        % (sorted(orph), _kill_pids(orph)))
    except Exception as exc:  # noqa: BLE001
        logs.append("[clean] 僵尸清理跳过: %r" % exc)

    ok = False
    if os.environ.get("CAD_DRAW3D_NO_COM") != "1":
        # COM 调用没有超时 → 用线程 join 兜住「Documents.Open 挂死」
        com_before = _cad_pids()
        done, res = _call_with_timeout(
            lambda: run_cad_com(parts, scratch, sat_tmp, dgn_tmp, dxf_tmp,
                                timeout=timeout, part_sat_tmps=part_tmps),
            timeout)
        if not done:
            ok = False
            lines = ["COM 在 %.0fs 内未返回 → 判为挂死（典型是 Documents.Open）"
                     % timeout]
            new = _cad_pids() - com_before
            if new:
                lines.append("强杀挂死实例 %s（成功 %d 个）"
                             % (sorted(new), _kill_pids(new)))
        else:
            ok, lines = res
        logs += ["[COM] " + x for x in lines]

    if not ok and os.environ.get("CAD_DRAW3D_NO_SCR") != "1":
        exe = cad_exe or find_cad_exe()
        try:
            ok2, lines2 = run_cad_script(parts, exe, scratch, sat_tmp, dgn_tmp,
                                         dxf_tmp, timeout=exe_timeout,
                                         part_sat_tmps=part_tmps)
        except Exception as exc:  # noqa: BLE001
            ok2, lines2 = False, ["SCR 通道异常: %r" % exc]
        logs += ["[SCR] " + x for x in lines2]
        ok = ok or ok2

    # CAD 侧落盘的文件名不一定等于我们给的名字（各家 CAD 会自己加/换扩展名），
    # 所以拷贝时把常见变体也认下来（附赠项，不因它失败影响 SAT/DGN）。
    def _first_existing(base, variants=()):
        for c in (base,) + tuple(base + v for v in variants):
            if c and os.path.isfile(c) and os.path.getsize(c) > 0:
                return c
        return None

    for tmp, dst, variants in ((sat_tmp, sat_path, ()),
                               (dgn_tmp, dgn_path, ()),
                               (dxf_tmp, dxf_path, (".dwg", ".bak"))):
        if not (tmp and dst):
            continue
        src = _first_existing(tmp, variants)
        if not src:
            continue
        # ★ 目标名是 .dxf 却拿到 DWG（`SAVEAS` 的退化产物）时**按真实格式改名交付**，
        #   绝不让「文件名说 DXF、内容其实是 DWG」这种错配流到用户手里。
        out_dst = dst
        want = os.path.splitext(dst)[1].lstrip(".").lower()
        got = _file_kind(src)
        if want in ("dxf", "dwg", "dgn", "sat", "stp") and got != want and got != "unknown":
            out_dst = os.path.splitext(dst)[0] + "." + got
            logs.append("[copy] !! %s 头字节显示是 %s（不是 %s），改名交付为 %s"
                        % (os.path.basename(src), got.upper(), want.upper(),
                           os.path.basename(out_dst)))
        logs.append("[copy] %s -> %s" % (os.path.basename(src),
                                         _copy_back(src, out_dst)))
    if sat_path:
        pdir = os.path.join(os.path.dirname(os.path.abspath(sat_path)),
                            os.path.splitext(os.path.basename(sat_path))[0] + ".parts")
        n = 0
        for i, tmp in enumerate(part_tmps):
            if os.path.isfile(tmp) and os.path.getsize(tmp) > 0:
                if _copy_back(tmp, os.path.join(pdir, "%02d.sat" % (i + 1))):
                    n += 1
        if n:
            logs.append("[copy] 分件 SAT %d/%d -> %s" % (n, len(parts), pdir))
    if not ok:
        ok = all(os.path.isfile(p) and os.path.getsize(p) > 0
                 for p in (sat_path, dgn_path) if p)
    return ok, logs


# ==========================================================================
# 八、主流程
# ==========================================================================
def _load_ref_report(plan_path):
    stem = os.path.splitext(plan_path)[0]
    cands = [stem + ".report.json"]
    if stem.endswith(".plan"):
        cands.append(stem[:-5] + ".report.json")
    for ref in cands:
        if os.path.isfile(ref):
            try:
                return load_json(ref)
            except Exception:  # noqa: BLE001
                return None
    return None


def main() -> int:
    ap = argparse.ArgumentParser(
        description="rebuild_plan.json -> STEP（ISO-10303-21 AP214）")
    ap.add_argument("input", help="rebuild_plan.json")
    ap.add_argument("-o", "--outdir", help="输出目录（默认与 plan 同目录）")
    ap.add_argument("--step", help=".stp 输出路径（默认 <outdir>/<stem>.stp）")
    ap.add_argument("--report", help="校验报告 JSON")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        sys.stderr.write("输入不存在: %s\n" % src)
        return 1
    plan = load_json(src)
    outdir = os.path.abspath(args.outdir or os.path.dirname(src))
    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(src))[0]
    # 机械实体只出 STEP：不给 --step 时落到 <outdir>/<stem>.stp
    step_path = (os.path.abspath(args.step) if args.step
                 else os.path.join(outdir, stem + ".stp"))
    rep_path = os.path.abspath(args.report or os.path.join(outdir, stem + ".cad.report.json"))

    parts, issues = load_parts(plan)
    if not parts:
        log("没有任何可解析输出的零件")
        save_json({"source": plan.get("source"), "parts": [], "issues": issues}, rep_path)
        return 1
    off = normalize_parts(parts, plan.get("normalize", "center_xz_ground_y"))
    log("装入 %d 件；归一化平移 %s" % (len(parts), [round(x, 3) for x in off]))

    report = {"source": plan.get("source"), "units": plan.get("units", "mm"),
              "normalize_offset": [round(x, 4) for x in off],
              "parts": [], "issues": list(issues) + list(plan.get("issues") or []),
              "outputs": {}}
    n_fail = 0
    for part in parts:
        try:
            part.brep = BrepBuilder(part).build()
        except Exception as exc:  # noqa: BLE001
            log("!! %s B-rep 构建失败: %s" % (part.name, exc))
            report["parts"].append({"name": part.name, "fatal": str(exc),
                                    "problems": ["B-rep 构建失败"]})
            n_fail += 1
            continue
        info, problems = brep_check(part.brep, part)
        info["name"] = part.name
        info["op"] = "extrude/%s" % part.axis
        info["holes"] = len(part.holes)
        info["problems"] = problems
        report["parts"].append(info)
        for p in problems:
            log("  ! %s: %s" % (part.name, p))
        log("  %s: V=%d E=%d F=%d  χ=%d  体积=%.1f mm³  包围盒=%s"
            % (part.name, info["vertices"], info["edges"], info["faces"],
               info["chi"], info["volume"], info["bbox"]))

    # 与网格路线（rebuild3d）交叉核对
    ref = _load_ref_report(src)
    if ref:
        cmps = []
        by_name = {m.get("name"): m for m in ref.get("meshes", []) if m.get("volume")}
        for info in report["parts"]:
            m = by_name.get(info.get("name"))
            if not m:
                continue
            dv = (info["volume"] - m["volume"]) / m["volume"] * 100.0 if m["volume"] else None
            cmps.append({"name": info["name"], "mesh_volume": m["volume"],
                         "brep_volume": info["volume"],
                         "volume_diff_pct": round(dv, 4) if dv is not None else None,
                         "mesh_bbox": m.get("bbox"), "brep_bbox": info["bbox"]})
        report["vs_mesh"] = cmps
        worst = max((abs(c["volume_diff_pct"]) for c in cmps if c["volume_diff_pct"] is not None),
                    default=0.0)
        log("与网格路线比对：体积最大偏差 %.4f%%（%d 件）" % (worst, len(cmps)))

    # ---- STEP ----
    ok_all = n_fail == 0
    if step_path:
        try:
            n = write_step(parts, step_path, _ascii_name(stem))
            sz = os.path.getsize(step_path)
            sinfo, sprobs = check_step_file(step_path)
            report["outputs"]["step"] = {"path": step_path, "entities": n, "bytes": sz,
                                         "check": sinfo, "problems": sprobs}
            log("STEP -> %s (%d 实体, %d bytes, %d solids/%d faces)"
                % (step_path, n, sz, sinfo["solids"], sinfo["faces"]))
            if sprobs:
                ok_all = False
                for p in sprobs:
                    log("  !! STEP 自检: %s" % p)
            else:
                log("  STEP 自检通过：引用闭合、%d 个壳、单位 mm"
                    % sinfo["solids"])
        except Exception as exc:  # noqa: BLE001
            report["outputs"]["step"] = {"path": step_path, "error": str(exc)}
            log("!! STEP 写出失败: %s" % exc)
            ok_all = False

    report["summary"] = {
        "n_parts": len(parts), "n_failed": n_fail,
        "total_faces": sum(p.get("faces", 0) for p in report["parts"]),
        "total_volume": sum(p.get("volume", 0.0) for p in report["parts"]),
        "all_clean": all(not p.get("problems") for p in report["parts"]),
    }
    save_json(report, rep_path)
    log("报告 -> %s" % rep_path)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
