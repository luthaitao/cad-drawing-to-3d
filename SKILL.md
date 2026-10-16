---
name: cad-drawing-to-3d
description: 从 PDF / DWG / DXF 格式的 CAD 零件图自动生成三维零件模型，同时输出网格格式（GLB 主，three.js 双击可看 + STL 辅）与机械 CAD 实体格式（**STEP / ISO-10303-21 AP214**，由自研 B-rep 写出器生成，纯 Python、零外部依赖、无 CAD 也能跑）。覆盖 DWG→DXF 引擎自动探测（LibreDWG/ODA）、ezdxf 图元提取（含 LibreDWG 坏 DXF 的容错补丁）、PDF 矢量/位图双分支（PyMuPDF + OpenCV 轮廓）、GB 图层过滤与尺寸解析、投影视图分组、拉伸/旋转特征初判、自研真 B-rep 写出器与自校验、trimesh 重建与水密校验的半自动全管线。当用户要求「零件图转三维模型」「CAD 图纸生成 3D」「dwg/pdf 建模」「法兰/端盖/轴套/板类零件三维化」「图纸出 GLB/STL」「导出 STEP/STP/实体模型/SolidWorks/UG/Creo/Inventor 能打开的格式」，或提供 .dwg/.dxf/.pdf 零件图要求三维模型时使用。机械实体格式当前覆盖拉伸件（回转件走 GLB/STL）。
agent_created: true
---

# CAD 零件图 → 三维零件模型（GLB/STL + STEP）

## 结论速览

| 项 | 结论 |
|---|---|
| 输入 | .dwg（经 LibreDWG/ODA 转 DXF）、.dxf、.pdf（矢量 CAD 导出 / 扫描位图） |
| 输出（网格） | **GLB 主**（three.js 双击加载，带颜色，单位 mm 导入乘 0.001）+ **STL 辅**（兼容兜底） |
| 输出（机械实体） | **STP**（ISO-10303-21 AP214，自研 B-rep 写出器，**纯 Python、零外部依赖、无需任何 CAD**） |
| 交互 | **半自动**：机器出 `rebuild_plan.json` 初判，人工核对 issues 后再重建 |
| 几何内核 | 网格：trimesh（extrude_polygon / revolve + manifold3d 布尔差集）<br>实体：自研 B-rep 写出器（纯 Python，含 5 层自校验） |
| 验证环境 | `python -m venv` + `pip install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt`（依赖清单同目录，含版本上限；清华镜像实测 trimesh 抽风，用阿里云）<br>**机械实体（STEP）无额外依赖** |

## 管线总览

```
.dwg ──dwg2dxf.py──> .dxf ─┐
                          ├─> drawing.json ──analyze_views.py──> rebuild_plan.json
.dxf ──dxf_extract.py──────┤   （轮廓/图层/尺寸/文字        （机器初判+issues 清单）
.pdf ──pdf_extract.py──────┘     统一为 mm 坐标）                    │
                                                               人工核对/编辑
                                                                     │
                                        ┌────────────────────────────┴────────────┐
                                        │                                         │
                              rebuild3d.py <──┘                        export_cad.py
                                        │                                         │
                    model.glb + model.stl + report.json        model.stp  自研 B-rep 写出器
                                                               （纯 Python，无外部依赖）
```

两条支路共用同一份 `plan.json`：`rebuild3d.py` 出网格，`export_cad.py` 出机械实体。

中间格式 `drawing.json` 的完整 schema 见 `scripts/drawing_common.py` 模块 docstring。

## 快速开始

```bash
# 0. 环境（venv 内）
pip install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt
#   （仅必需：ezdxf pymupdf opencv-python numpy shapely trimesh manifold3d scipy
#     可选：rapidocr_onnxruntime=位图OCR、gmsh=STEP独立复核）

# 1. DWG 用户先确认转换引擎（LibreDWG 免费；桌面上若有"DWG文件提取工具/bin/dwg2dxf.exe"可直接用）
set DWG2DXF_PATH=D:\path\to\dwg2dxf.exe        # Windows

# 2. 一条命令跑到 plan.json（半自动第一步）
python scripts/make_plan.py 零件图.dwg --outdir out/

# 3. 人工核对 out/零件图.plan.json（重点看 issues[]，改 depth/补 part）

# 4. 重建出模型
python scripts/rebuild3d.py out/零件图.plan.json -o out/零件图.glb

# 5. 看模型：把 assets/viewer-template/ 整个目录拷到 out/，glb 改名 model.glb，双击 index.html

# 6. 机械 CAD 实体格式（STEP，纯 Python，无 CAD 也能跑）
python scripts/export_cad.py out/零件图.plan.json -o out/ --step out/零件图.stp
# 省略 --step 时默认写 out/零件图.plan.stp
```

产物：`零件图.stp` + `零件图.plan.cad.report.json`
（B-rep 拓扑自检 + 散度定理体积复核 + 与网格路线交叉比对 + STEP 结构自检）。
**用装了依赖的 venv python 跑**即可；STEP 路线**不需要 pywin32、不需要任何 CAD**。

验证环境用自带示例（确定性输入，不依赖外部素材）：

```bash
python scripts/make_sample_dxf.py out/sample-flange.dxf     # 法兰（DXF/revolve 路径）
python scripts/make_sample_pdf.py out/sample-plate.pdf      # 方板（PDF/extrude 路径）
python scripts/make_plan.py out/sample-flange.dxf --outdir out/ && python scripts/rebuild3d.py out/sample-flange.plan.json -o out/sample-flange.glb
```

实测基准（2026-10-15，本机验证）：法兰 GLB 体积偏差 **0.04%**、方板体积**精确**、均水密。

## 各阶段详解

### 1. 格式归一化（dwg2dxf.py / dxf_extract.py / pdf_extract.py）

**DWG→DXF 引擎探测顺序**：`DWG2DXF_PATH/LIBREDWG_BIN` 环境变量 → PATH → ODA File Converter
常见路径。全缺失时给下载指引并以退出码 2 退出。LibreDWG 开源免费；ODA 免费需注册。
DWG 是专有二进制，**没有纯 Python 解析路线**。

**dxf_extract.py 的必修补丁**（LibreDWG 转出的 DXF 两类结构瑕疵，ezdxf 标准/recover
双模式都拒收，详见 `references/gltf-stl-notes.md` 三.2）：
- `recover.EXCLUDE_STRUCTURE_CHECK |= {"XRECORD"}`（跳过扩展记录的 XDATA 校验）
- patch `ezdxf.sections.entities.entity_linker` 为容错版（INSERT 子实体链断裂时断开继续）
- 编码探测：`$DWGCODEPAGE=ANSI_936` → GBK（中文图纸文字；尺寸数字是 ASCII 不受影响）

**pdf_extract.py 双分支**：
- 矢量分支（CAD 导出 PDF）：`get_drawings` 展平 bezier → 圆度检测（半径标准差 <5%）；
  `get_text("dict")` 取文字；线宽启发式分伪层（≥0.6pt 轮廓 / 细线尺寸 / 短 path 虚线）
- 位图分支（扫描件，`--bitmap` 强制或矢量极少自动进入）：`get_pixmap(dpi)` →
  cv2 自适应阈值 → findContours → approxPolyDP → 闭合轮廓；OCR 用 rapidocr（可选依赖）
- **比例反推**：PDF 坐标是 pt 且打印比例任意，用「尺寸值/线段长」候选 + 值域约束 +
  distinct 尺寸值聚类，实测 1:2 A3 图误差 0.01%

### 2. 视图分析与特征初判（analyze_views.py）

按 `references/view-and-feature-rules.md` 执行：图层过滤 → 端点链轮廓提取 →
投影分组 → revolve/extrude 初判 → 深度推断。**关键判据**：
- ≥2 个同心圆 → revolve（法兰/端盖/轴套）；闭合多边形 → extrude
- 主视图 = 上方无其它组的顶部组（**不是面积最大组**）
- 深度来源优先级：俯视图对位宽度 > 尺寸标注匹配 > 人工（无来源时 depth=null 并记 issue，
  **绝不拍脑袋默认值**）

### 3. 3D 重建与导出（rebuild3d.py）

- extrude：`trimesh.creation.extrude_polygon(Polygon(shell, holes), depth)`
  - `axis`（默认 `Z`）：拉伸方向。`Z` 截面在 X-Y；`Y` 截面在 X-Z（**板的平面轮廓**，
    垂直板面的孔在截面里天然是圆——底板地脚孔这类必须用 `Y`，否则孔只能表达成贯通槽）；
    `X` 截面在 Y-Z。`axis=Y` 时截面第二坐标映射到 `-Z`。
  - `translate [dx,dy,dz]`：per-part 平移。`extrude_polygon` 恒从 Z=0 起拉，多零件装配
    各件 Z 定位不同，必须靠它摆位（装配件必用）。
  - 截面 `buffer(0)` 修复自交后可能返回 MultiPolygon（真实图纸易触发），已内置取最大子面兜底。
- revolve：截面**闭合成环**（trimesh 整圆旋转的 cap 被跳过，水密性靠闭合母线）+
  **逆时针**（否则法线朝内体积为负）+ x=0 微移 + sections=128（体积偏差 0.04%）+
  孔洞 manifold3d 布尔差集
- 归一化：XZ 居中、Y 贴地（图纸坐标偏移一次性消除，幂等；`plan.normalize="none"` 关闭）
- 校验：水密 / 绕序 / NaN / 飞点 / 三角面数下限 → `*.report.json`
- 导出：GLB（多 mesh+名称+COLOR_0 顶点色，需 scipy）+ STL（concatenate 单网格）

## 机械格式导出（export_cad.py）

网格格式（GLB/STL）能看、能打，但**下游机械 CAD（SolidWorks/UG/Creo/Inventor/CATIA）不认**。
`export_cad.py` 把同一份 `plan.json` 再写成 **STEP（ISO-10303-21 AP214）** ——
上述软件都能直接打开的通用实体格式。

| 产物 | 路线 | 依赖 | 实测（鞍座 7 件） |
|---|---|---|---|
| `.stp` | **自研 B-rep 写出器**（`export_cad.py` 内，纯 Python 手写 ISO-10303-21 AP214） | **无** | 39 297 B / 945 实体 / **7 solid / 46 face** / `problems=[]` |

> **★ STEP 拓扑铁律（2026-10-16 血泪教训，改写出器前必读）**
> 同一条几何边在文件里**只能有一份 `EDGE_CURVE`**，被相邻两个面各用一个
> `ORIENTED_EDGE` 引用（一侧 `.T.`、另一侧 `.F.`）。
> 早期版本「每个面各写一份 `EDGE_CURVE`」时：自家纯文本自检照样通过、
> `MANIFOLD_SOLID_BREP` 数也对，但**第三方内核（OpenCASCADE）读出来是
> 「N 个面 / 2N 条边 / 0 个实体」**，用户那边表现为「**STP 显示不全**」
> （只出来垫板一块弧面 + 两个孔的小点）。
> 健康文件的特征比值：`ORIENTED_EDGE : EDGE_CURVE = 2 : 1`
> （真实 SolidWorks/UG 导出件实测也是 2:1）。
> `check_step_file()` 已把这条加为硬性自检（`edge_curves` / `edge_uses` 两个字段）。
> 同理 `VERTEX_POINT` 也要按坐标唯一化；两组缓存都必须**按零件**重置，
> 不同零件之间绝不共享点/边（否则两个壳被粘成一个）。
> 另有一条同类坑：`StepWriter.line()` 的缓存键**不能只用方向**，否则平行的不同直线
> 会被合并、锚点取到别的线上（顶点落不到直线上）——键要用 `(方向, 原点到直线的垂足)`。

> **机械实体只出 STEP**（2026-10-16 起）。历史上还做过 SAT / DGN / 含 3DSOLID 的 DXF，
> 但那三者必须**借本机已授权的 CAD 内核**，发布给没有 CAD 的用户时完全不可用，故移除。
> 关键经验以存档形式保留在 §4.2。

> **覆盖范围（发布须知）**：只支持 **`op=extrude` 的拉伸件**（板类 / 鞍座类 / 箱体类）。
> 回转件（法兰 / 端盖 / 轴套 / 管板，`op=revolve`）的 B-rep 需要解析曲面（圆柱/圆锥/圆环），
> 当前不在写出器范围内 —— 这类件请走 `rebuild3d.py` 的 **GLB/STL** 路线。
> `load_parts()` 会为每件记一条 `op-unsupported` issue，不会静默丢弃。

### 4.1 自研 STEP 写出器（为什么不用现成库）

免费路线全部试过并否决（详见项目文档 §2）：`pythonocc`（conda 依赖 + 200MB）、`cadquery`（同上）、
`steputils`（只读不写）、`OCP`（同上）、FreeCAD CLI（需装 FreeCAD）、`Open Cascade` 编译（数小时）。
结论：**自研**。B-rep 是我们在 §3 重建阶段就有的中间产物，直接吐 STEP 只需 400 行。

**方向约定**（局部标架 `(u,v,w)`，`w` = 拉伸方向，`u×v=w`）：

| `axis` | 局部 → 世界 | 等价变换 |
|---|---|---|
| `Z` | 恒等 | 无 |
| `Y` | `(u,v,w) → (u,w,−v)` | 绕 X 轴 −90° |
| `X` | `(u,v,w) → (w,v,−u)` | 绕 Y 轴 +90° |

**自校验 5 层**（`brep_check()`，任一不过就进 `problems[]`）：欧拉公式 `V−E+Σ(1−k)=2−2g`、
散度定理复算体积、面法线一致性、边界环闭合性、STEP 文本结构自检。
7 件实测：与 trimesh 网格体积最大偏差 **0.0146%**（垫板），其余 <0.005%。

#### 第三方复核（强烈建议，别只信自己的解析器）

自研自检只能查「引用是否悬空」这类**语法**问题；**拓扑是否被内核接受是另一回事**
（2026-10-16 就是栽在这里：自家自检全绿，OpenCASCADE 却读出 0 个实体）。

一条命令即可让 OCC（与自研解析器完全独立的内核）复核：

```bash
pip install -i https://mirrors.aliyun.com/pypi/simple/ gmsh   # 自带 OCC，约 42 MB
python scripts/verify_step_occ.py out/零件图.stp              # 退出码 0 = 通过
```

脚本判据：`OpenCASCADE 实体数 == 零件数` 且每件体积为正。
若打印「读到 0 个实体（只有散面）」，就是**壳没被内核接受**（先查边共享）。
手工版等价于：

### 4.2 历史路线存档：SAT / DGN / 3D-DXF（已停用）

这三者都得**借本机已授权的 CAD 内核**（COM 自动化：把剖面导进去 → 拉伸 → 布尔 → 导出），
没有 CAD 的机器上完全不可用，因此在发布版本里移除。实现代码作为存档留在
`export_cad.py` 的 §七；下面是踩过的关键坑，将来若要恢复必读：

| 主题 | 结论 |
|---|---|
| 版本判定 | 只认「有 `.lic` 的那个」版本。本机唯一可用的是 KunHengCAD **V25**（`kunhengcad.exe` / `KunHengCADV25.lic`），ProgID `BricscadApp.AcadApplication.25.0`；V23 无授权 |
| COM 通道 | **`DispatchEx` + `Documents.Item(Count-1)`（默认空白图）+ `DXFIN` 导剖面**是唯一可用组合。`Documents.Open` 无限期挂死；`Dispatch` 附加用户会话后 RPC 不可用 |
| 自动化面 | 只有 `Name` / `Version` / `Documents` / `Documents.Count` / `Documents.Item(i)` / `SendCommand` / `SetVariable` 可用；`Documents.Add()`、`ActiveDocument`、`Quit`、`Visible` 不存在 |
| 几何命令链 | `DXFIN 剖面 → (setvar CLAYER) → EXTRUDE → SUBTRACT → ERASE 清孔柱 → ROTATE3D → ACISOUT`。`CLAYER` 是系统变量不是命令 |
| 平移 | **不用 `_.MOVE` / `vla-move`**（实测都不可靠）→ 改「**剖面预置位**」：`(du,dv,e) = R⁻¹(offset)`，剖面 2D 平移 + LWPOLYLINE `elevation`。折算：`Z→(ox,oy,oz)`；`Y→(ox,−oz,oy)`；`X→(−oz,oy,ox)`。实测 gap 由 220/85/7 mm 降到 **0.00** |
| 布尔 | 本机 `SUBTRACT` **只改形不删源实体** ⇒ 必须 `ERASE` 清空孔柱层，否则合并 `ACISOUT "_ALL"` 会多导 body（实测 9 body / 50 面 → 修后 7 / 44） |
| 出 DXF | **只能用 `DXFOUT`**（`(command "_.DXFOUT" "p" "V" "2013" "16" "_ALL" "")`）；`SAVEAS` 的任何形态都退化成 DWG |
| LISP | 命令通道**一次只求值一行** ⇒ 多行 `defun` 静默失败（日志是空文件），必须写 `.lsp` 文件 + 单行 `(load "…")` |
| 进程管理 | 清 CAD 进程只有 `kernel32!TerminateProcess` 有效；僵尸 = **无可见顶层窗口**的 CAD 进程（有窗口的一律不动） |
| 闭环验证 | `ACISIN` 回读体积偏差 **+0.0019%**、`DGNIMPORT` 回读 7 个 3DSOLID、`DXFIN` 与 SAT 逐位一致（回读脚本未随本技能分发）；**读日志必须轮询等 `TOTALVOL=`，不能固定 sleep** |

## 半自动流程（核心约定，不要改成全自动）

1. `make_plan.py` 产出 `plan.json`：`parts[]`（op/profile/depth/angle/color/sections/…）
   + `issues[]`（待确认项）+ `views[]`
2. **人工逐条处理 issues**（代码表见 view-and-feature-rules.md 第七节），编辑 plan.json：
   填 depth、删/加 part、调 angle、给 color
3. `rebuild3d.py` 重建 + 校验；report.json 里 `problems` 非空必须回头改 plan

## 验收标准（交付前逐项过）

- [ ] `report.json` 所有 mesh `watertight=true`、`winding_consistent=true`、`problems=[]`
- [ ] 体积与手算/图纸理论值对比：revolve 件 <0.1%，extrude 件应精确
- [ ] `scene_bbox` 与图纸尺寸一致（注意归一化后是相对尺寸）
- [ ] GLB 能被 three.js 加载：viewer 模板打开有模型、HUD 统计正确
- [ ] **机械格式**：`cad.report.json` 中 `outputs.step.problems=[]`、
      STEP `check` 全过（引用闭合 / `solids` == 零件数 / `faces` 合理 / 单位 mm）
- [ ] **STEP 流形共享**：`check.edge_uses == 2 × check.edge_curves`
      （不做这条，文件可能「自家自检通过、第三方内核只读到散面」）
- [ ] 有条件时用**独立内核**复核一次（见 §4.1「第三方复核」），别只信自己的解析器
- [ ] 半自动确认项全部关闭（issues 已处理或在交付说明中明示）

## 已知限制与失败诊断

| 现象 | 诊断 | 处置 |
|---|---|---|
| drawing.json 块多线少（n_blocks≈300, n_polylines≈0） | DWG 带私有扩展对象（天正类），LibreDWG 解析损坏（dwgread 报大量 ERROR） | 换 ODA File Converter 或让委托方导标准 DXF；**不要调参** |
| `no-profile` issue | 图层过滤过严 / 位图分辨率低 | `--exclude-layer` 反查、`dwglayers.exe` 看全部层名、DPI 提到 400+ |
| `depth-missing` | 无俯视图且无尺寸 | 人工填 depth（半自动设计如此，不是 bug） |
| revolve 体积为负/不水密 | 母线未闭合/顺时针/贴轴顶点 | 已由 rebuild3d 内置处理；仍有问题查 gltf-stl-notes.md 二 |
| 钣金折弯/曲面件/装配图 | 特征规则不覆盖 | 在交付说明中明示；复杂件人工补 part |
| **焊接组件**（底板+腹板+筋板+垫板…） | 自动管线按「单零件」假设，只会抓到一个壳体/碎片 | 明确告知「需人工按 BOM 逐件补 `parts[]`」；用 `axis`/`translate` 摆位，每件独立 extrude |
| PDF 比例反推错（如 1:10 图误得 1/3 值） | 旧「线段长≈尺寸值」只对 1:1 成立 | 已改为「标题栏明文比例优先 → 标准比例投票兜底」；仍失败时人工填 `scale_hint` |
| `MultiPolygon object has no attribute 'exterior'` | 自交轮廓 `buffer(0)` 返回多面 | 已在 `poly_to_xy` / `build_extrude` 内置取最大子面兜底 |
| 清华镜像装不上 trimesh | 镜像临时抽风 | 换 `-i https://mirrors.aliyun.com/pypi/simple/` |
| `export_cad.py` 报 `No module named ezdxf` 等 | 用了系统 python | 用装了依赖的 **venv python** 跑（STEP 路线**不需要 pywin32 / 不需要 CAD**） |
| 无 CAD 的机器上能不能跑 | **能** —— 机械实体只出 STEP，纯 Python | 直接跑即可，不用装 CAD、不用任何开关 |
| 回转件（法兰/端盖/轴套）出不了 STEP | `load_parts()` 只认 `op=extrude`，`op=revolve` 记 `op-unsupported` | 设计如此（解析曲面不在 B-rep 写出器范围）；该件走 GLB/STL，或在人工 CAD 里建模 |
| **STEP 打开后「显示不全」**：只出来一两块面，其余零件不见 | **相邻面没有共享 `EDGE_CURVE`** ⇒ 内核收不到闭合壳，只导入散面（自家纯文本自检**查不出来**，因为它只数引用是否悬空） | 已修（每边一份 `EDGE_CURVE` + 两面反向 `ORIENTED_EDGE`）；自检加 `edge_uses == 2×edge_curves`。老文件用新自检一跑就现形 |
| STEP 打开后「某个面的形状不对 / 边不在面上」 | `StepWriter.line()` 的缓存键只用了方向 ⇒ 平行的不同直线被合并，锚点串了 | 已修：键 = `(方向, 原点到直线的垂足)`，锚点取垂足 |

## 文件清单

```
scripts/
  drawing_common.py     drawing.json schema、图层分类、GB 尺寸文字解析、几何工具
  dwg2dxf.py            DWG→DXF 引擎自动探测（LibreDWG/ODA）
  dxf_extract.py        DXF→drawing.json（ezdxf + 两个容错补丁 + 编码探测）
  pdf_extract.py        PDF→drawing.json（矢量/位图双分支 + 比例反推）
  analyze_views.py      drawing.json→rebuild_plan.json（视图分组+特征初判+深度推断）
  rebuild3d.py          plan.json→GLB/STL/report.json（trimesh 重建+校验+归一化）
  export_cad.py         plan.json→STEP 机械实体格式（自研纯 Python B-rep 写出器 + 5 层自校验；
                        §七 保留已停用的 CAD 内核路线作存档，见 §4.2）
  verify_step_occ.py    【建议】用 OpenCASCADE（gmsh）独立复核 STEP 是否真被内核接受
                        （可选依赖 `pip install gmsh`；自家文本自检查不出拓扑缺陷）
  make_plan.py          一站式 CLI（任意输入→plan.json）
  make_sample_dxf.py    示例素材：法兰零件图 DXF（环境自检用）
  make_sample_pdf.py    示例素材：方板零件图 PDF（1:2 打印，验证比例反推）
references/
  cad-layers-and-dims.md    GB 图层约定、常见 CAD 图层名映射、尺寸文字解析规则
  view-and-feature-rules.md 轮廓提取/视图分组/特征判定/深度推断全部规则与实测修正
  gltf-stl-notes.md         格式选型结论 + trimesh/ezdxf/PyMuPDF 实测坑速查
assets/viewer-template/   three.js r128 UMD 离线查看器（双击 index.html，支持拖放 GLB/STL）
assets/icon.png           技能图标 512×512（等轴测实体 + 顶面网格 + 尺寸标注）
requirements.txt          依赖清单（必需/可选分组 + 版本上限）
LICENSE.md                MIT 许可（标准正文，便于平台自动识别）
NOTICE.md                 适用范围 / 第三方组件（three.js 系）/ 外部工具 / 使用须知
```
