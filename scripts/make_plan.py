#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一站式入口：任意输入（dwg/dxf/pdf）-> rebuild_plan.json。

管线：
  .dwg  -> dwg2dxf.py（LibreDWG/ODA 自动探测）-> .dxf
  .dxf  -> dxf_extract.py  \
  .pdf  -> pdf_extract.py    >-> drawing.json -> analyze_views.py -> rebuild_plan.json

注意本脚本**只到 plan.json**（半自动第一步）。人工确认/编辑 plan.json 后，
再执行 rebuild3d.py 生成 GLB + STL。

用法：
  python make_plan.py 图纸.dwg|dxf|pdf [--outdir DIR] [--dwg2dxf PATH] [--bitmap]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def run(cmd: list[str]) -> int:
    print("[make_plan] $ %s" % " ".join('"%s"' % c if " " in c else c for c in cmd))
    r = subprocess.run(cmd)
    return r.returncode


def main() -> int:
    ap = argparse.ArgumentParser(description="CAD 零件图 -> rebuild_plan.json（半自动第一步）")
    ap.add_argument("input", help="输入 .dwg / .dxf / .pdf")
    ap.add_argument("--outdir", help="输出目录（默认输入所在目录）")
    ap.add_argument("--dwg2dxf", help="手动指定 DWG 转换引擎（dwg2dxf.exe / ODAFileConverter.exe）")
    ap.add_argument("--bitmap", action="store_true", help="PDF 强制走位图分支（扫描件）")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        sys.stderr.write("输入不存在: %s\n" % src)
        return 1
    outdir = os.path.abspath(args.outdir or os.path.dirname(src))
    os.makedirs(outdir, exist_ok=True)
    ext = os.path.splitext(src)[1].lower()
    stem = os.path.splitext(os.path.basename(src))[0]

    # 1) DWG -> DXF
    dxf = None
    if ext == ".dwg":
        dxf = os.path.join(outdir, stem + ".dxf")
        cmd = [sys.executable, os.path.join(HERE, "dwg2dxf.py"), src, "-o", dxf]
        if args.dwg2dxf:
            cmd += ["--engine", os.path.abspath(args.dwg2dxf)]
        if run(cmd) != 0:
            sys.stderr.write("DWG->DXF 失败。安装 LibreDWG 或用 --dwg2dxf 指定引擎。\n")
            return 1
    elif ext == ".dxf":
        dxf = src
    elif ext != ".pdf":
        sys.stderr.write("不支持的输入格式: %s（仅 .dwg/.dxf/.pdf）\n" % ext)
        return 1

    # 2) -> drawing.json
    drawing_json = os.path.join(outdir, stem + ".drawing.json")
    if dxf:
        cmd = [sys.executable, os.path.join(HERE, "dxf_extract.py"), dxf, "-o", drawing_json]
    else:
        cmd = [sys.executable, os.path.join(HERE, "pdf_extract.py"), src, "-o", drawing_json]
        if args.bitmap:
            cmd += ["--bitmap"]
    if run(cmd) != 0:
        sys.stderr.write("图元提取失败。\n")
        return 1

    # 3) -> rebuild_plan.json
    plan_json = os.path.join(outdir, stem + ".plan.json")
    cmd = [sys.executable, os.path.join(HERE, "analyze_views.py"), drawing_json, "-o", plan_json]
    if run(cmd) != 0:
        sys.stderr.write("视图分析失败。\n")
        return 1

    print("")
    print("[make_plan] 完成。下一步（半自动）：")
    print("  1. 打开并核对 %s" % plan_json)
    print("     - 逐个检查 issues[] 中的确认项（深度、半视图、轴线等）")
    print("     - 按需修改 parts[]（depth / angle / 增删 part / color）")
    print("  2. 确认后执行：python %s %s -o %s" % (
        os.path.join(HERE, "rebuild3d.py"), plan_json,
        os.path.join(outdir, stem + ".glb")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
