#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DWG -> DXF 转换：自动探测本机可用的免费转换引擎。

探测顺序（先到先得）：
  1. 环境变量 DWG2DXF_PATH / LIBREDWG_BIN 指向的 dwg2dxf(.exe)
  2. PATH 中的 dwg2dxf / dxf2dwg 反查（LibreDWG 安装版）
  3. ODA File Converter（免费，官网下载，CLI 批处理）
  4. Windows 常见安装位置扫描（含 Program Files 与桌面项目 bin 目录）

引擎全部缺失时打印下载指引，退出码 2。

用法：
  python dwg2dxf.py input.dwg [-o out.dxf] [--outdir DIR] [--engine PATH]
退出码：0 成功；2 未找到转换引擎；3 转换失败
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

EXIT_OK = 0
EXIT_NO_ENGINE = 2
EXIT_FAILED = 3

# LibreDWG dwg2dxf 常见位置（Windows）
_COMMON_DIRS = [
    r"C:\Program Files\LibreDWG\bin",
    r"C:\Program Files (x86)\LibreDWG\bin",
    r"C:\ProgramData\chocolatey\bin",
]

# ODA File Converter 常见位置（免费需注册下载：https://www.opendesign.com/guestfile/oda_file_converter）
_ODA_DIRS = [
    r"C:\Program Files\ODA\ODAFileConverter",
    r"C:\Program Files (x86)\ODA\ODAFileConverter",
    r"C:\Program Files\ODA\ODA File Converter",
    r"C:\Program Files (x86)\ODA\ODA File Converter",
]


def log(msg: str) -> None:
    print("[dwg2dxf] %s" % msg)


def find_libredwg() -> str | None:
    """探测 LibreDWG dwg2dxf。返回可执行路径或 None。"""
    # 1. 环境变量显式指定
    for env in ("DWG2DXF_PATH", "LIBREDWG_BIN"):
        p = os.environ.get(env)
        if p:
            p = os.path.abspath(p)
            if os.path.isfile(p):
                return p
            if os.path.isdir(p):  # 允许指向 bin 目录
                cand = os.path.join(p, "dwg2dxf.exe" if os.name == "nt" else "dwg2dxf")
                if os.path.isfile(cand):
                    return cand
    # 2. PATH
    exe = shutil.which("dwg2dxf") or shutil.which("dwg2dxf.exe")
    if exe:
        return exe
    # 3. 常见目录
    for d in _COMMON_DIRS:
        cand = os.path.join(d, "dwg2dxf.exe" if os.name == "nt" else "dwg2dxf")
        if os.path.isfile(cand):
            return cand
    return None


def find_oda() -> str | None:
    """探测 ODA File Converter。返回可执行路径或 None。"""
    env = os.environ.get("ODA_FILE_CONVERTER")
    if env and os.path.isfile(env):
        return env
    for d in _ODA_DIRS:
        cand = os.path.join(d, "ODAFileConverter.exe")
        if os.path.isfile(cand):
            return cand
    return None


def convert_with_libredwg(exe: str, src: str, dst: str) -> bool:
    cmd = [exe, "-y", "-o", dst, src]
    log("运行: %s" % " ".join('"%s"' % c if " " in c else c for c in cmd))
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if r.stdout:
        log(r.stdout.strip()[-800:])
    if r.stderr:
        log("stderr: " + r.stderr.strip()[-800:])
    if os.path.isfile(dst) and os.path.getsize(dst) > 256:
        return True
    # dwg2dxf 有时忽略 -o 而在源目录生成同名 .dxf
    side = os.path.splitext(src)[0] + ".dxf"
    if os.path.isfile(side) and os.path.getsize(side) > 256:
        shutil.move(side, dst)
        return True
    return False


def convert_with_oda(exe: str, src: str, dst: str) -> bool:
    """ODA CLI 只接受目录级批处理：in_dir out_dir ver filetype recurse audit。"""
    with tempfile.TemporaryDirectory(prefix="oda_dwg2dxf_") as tmp:
        in_dir = os.path.join(tmp, "in")
        out_dir = os.path.join(tmp, "out")
        os.makedirs(in_dir)
        os.makedirs(out_dir)
        shutil.copy2(src, os.path.join(in_dir, os.path.basename(src)))
        # 版本用 ACAD2018（兼容性较好）；filetype=DXF；recurse=0；audit=1 修复问题
        cmd = [exe, in_dir, out_dir, "ACAD2018", "DXF", "0", "1"]
        log("运行: %s" % " ".join('"%s"' % c if " " in c else c for c in cmd))
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
            if (r.stdout or "").strip():
                log(r.stdout.strip()[-500:])
        except subprocess.TimeoutExpired:
            log("ODA 转换超时")
            return False
        produced = os.path.join(out_dir, os.path.splitext(os.path.basename(src))[0] + ".dxf")
        if os.path.isfile(produced):
            shutil.copy2(produced, dst)
            return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="DWG -> DXF（LibreDWG / ODA 自动探测）")
    ap.add_argument("input", help="输入 .dwg 文件")
    ap.add_argument("-o", "--output", help="输出 .dxf 路径（默认与输入同目录同名）")
    ap.add_argument("--outdir", help="输出目录（默认输入所在目录）")
    ap.add_argument("--engine", help="手动指定转换引擎可执行文件（dwg2dxf 或 ODAFileConverter）")
    args = ap.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        log("输入文件不存在: %s" % src)
        return EXIT_FAILED
    if not src.lower().endswith(".dwg"):
        log("输入不是 .dwg（本脚本只转 DWG；DXF 请直接用 dxf_extract.py）")
        return EXIT_FAILED

    dst = args.output or os.path.join(
        args.outdir or os.path.dirname(src),
        os.path.splitext(os.path.basename(src))[0] + ".dxf",
    )
    dst = os.path.abspath(dst)
    os.makedirs(os.path.dirname(dst), exist_ok=True)

    engine = args.engine
    if engine:
        engine = os.path.abspath(engine)
        if not os.path.isfile(engine):
            log("指定的引擎不存在: %s" % engine)
            return EXIT_NO_ENGINE
        ok = (
            convert_with_oda(engine, src, dst)
            if os.path.basename(engine).lower().startswith("oda")
            else convert_with_libredwg(engine, src, dst)
        )
        return EXIT_OK if ok else EXIT_FAILED

    lredwg = find_libredwg()
    if lredwg:
        log("使用 LibreDWG: %s" % lredwg)
        if convert_with_libredwg(lredwg, src, dst):
            log("转换成功: %s" % dst)
            return EXIT_OK
        log("LibreDWG 转换失败，尝试 ODA ...")

    oda = find_oda()
    if oda:
        log("使用 ODA File Converter: %s" % oda)
        if convert_with_oda(oda, src, dst):
            log("转换成功: %s" % dst)
            return EXIT_OK
        log("ODA 转换失败。")

    log("未找到可用的 DWG 转换引擎。请任选其一：")
    log("  A. LibreDWG（开源免费）: https://github.com/LibreDWG/libredwg/releases")
    log("     解压后用环境变量指向，如：set DWG2DXF_PATH=D:\\libredwg\\bin\\dwg2dxf.exe")
    log("  B. ODA File Converter（免费需注册）: https://www.opendesign.com/guestfile/oda_file_converter")
    log("     安装后用 --engine 指向 ODAFileConverter.exe")
    return EXIT_NO_ENGINE


if __name__ == "__main__":
    sys.exit(main())
