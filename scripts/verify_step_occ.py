# -*- coding: utf-8 -*-
"""verify_step_occ.py —— 用 OpenCASCADE（经 gmsh）独立复核 STEP 是否真被内核接受。

为什么需要它
------------
`export_cad.check_step_file()` 是**纯文本自检**，只能查「引用是否悬空」这类**语法**问题。
它查不出**拓扑健康度** —— 2026-10-16 就栽在这里：自家自检全绿，OpenCASCADE 却读出
「0 个实体 / N 个散面」，用户打开只看到一两块面（「STP 显示不全」）。

所以交付前应跑一遍本脚本：它是**独立实现**（OpenCASCADE），与自家解析器没有共同代码。

安装（可选依赖，只在本脚本用到）：
    pip install -i https://mirrors.aliyun.com/pypi/simple/ gmsh
用法：
    python verify_step_occ.py 零件图.stp [零件数]
退出码：0 = 通过（实体数吻合且每件体积为正）；2 = 未装 gmsh；1 = 不通过
"""
import os
import sys


def main():
    if len(sys.argv) < 2:
        sys.stderr.write(__doc__)
        return 1
    path = os.path.abspath(sys.argv[1])
    if not os.path.isfile(path):
        sys.stderr.write("文件不存在: %s\n" % path)
        return 1
    want = int(sys.argv[2]) if len(sys.argv) > 2 else None
    if want is None:
        # 从同目录的 cad.report.json 猜零件数
        rep = os.path.join(os.path.dirname(path),
                           os.path.splitext(os.path.basename(path))[0] + ".cad.report.json")
        if os.path.isfile(rep):
            try:
                import json
                want = json.load(open(rep, encoding="utf-8")).get("summary", {}).get("n_parts")
            except Exception:  # noqa: BLE001
                want = None

    try:
        import gmsh
    except ImportError:
        sys.stderr.write(
            "缺少 gmsh（可选依赖）。装法：\n"
            "  pip install -i https://mirrors.aliyun.com/pypi/simple/ gmsh\n"
            "不装也能继续用技能 —— 只是少一层第三方复核。\n")
        return 2

    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    try:
        gmsh.open(path)
        vols = [t for _d, t in gmsh.model.getEntities(3)]
        faces = gmsh.model.getEntities(2)
    except Exception as exc:  # noqa: BLE001
        gmsh.finalize()
        print("!! OpenCASCADE 读入失败：%r" % exc)
        return 1

    print("文件：%s（%d 字节）" % (os.path.basename(path), os.path.getsize(path)))
    print("OpenCASCADE 读到：体(dim=3) = %d ，面(dim=2) = %d" % (len(vols), len(faces)))
    ok = True
    tot = 0.0
    for t in vols:
        bb = gmsh.model.getBoundingBox(3, t)
        size = (bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
        vol = None
        for getter in ("getMass",):
            try:
                vol = getattr(gmsh.model, getter)(3, t)
            except Exception:  # noqa: BLE001
                vol = None
        if vol is None:
            try:
                vol = gmsh.model.occ.getMass(3, t)
            except Exception:  # noqa: BLE001
                vol = None
        if vol:
            tot += vol
        print("  体 %-4d 尺寸=(%9.2f,%9.2f,%9.2f)  体积=%s"
              % (t, size[0], size[1], size[2],
                 ("%14.3f" % vol) if vol else "不可用"))
        if vol is not None and vol <= 0:
            ok = False
    if tot:
        print("  体积合计 = %.3f mm^3" % tot)
    gmsh.finalize()

    if not vols:
        print("\n!! 结论：**不通过** —— 内核读到 0 个实体（只有散面）。")
        print("   典型原因：相邻面没共享 EDGE_CURVE（见 SKILL.md「STEP 拓扑铁律」）。")
        return 1
    if want is not None and len(vols) != want:
        print("\n!! 结论：**不通过** —— 实体数 %d != 期望零件数 %d。" % (len(vols), want))
        return 1
    if not ok:
        print("\n!! 结论：**不通过** —— 存在体积非正的实体。")
        return 1
    print("\n== 结论：通过 ==  OpenCASCADE 接受为 %d 个有效实体。" % len(vols))
    return 0


if __name__ == "__main__":
    sys.exit(main())
