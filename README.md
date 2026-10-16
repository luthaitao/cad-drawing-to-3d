# CAD 零件图 → 三维模型（GLB / STL / STEP）

> 丢一张 PDF / DWG / DXF 零件图进去，自动产出可在浏览器里转着看的三维模型，以及能直接被
> SolidWorks / UG / Creo / Inventor 打开的三维实体（STEP AP214）。
> **纯 Python，不依赖任何 CAD 软件** —— 装完 pip 依赖就能跑。

[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE.md)
[![Format](https://img.shields.io/badge/output%20STEP-AP214-orange)](#-输出格式)

---

## ✨ 特性

- **两种输出，各管一头**
  - **网格格式**：`GLB`（主，自带离线 three.js 查看器，双击 `index.html` 即可看，支持拖放）+ `STL`（辅，3D 打印 / 通用 CAD 兼容）
  - **机械实体格式**：`STEP`（ISO-10303-21 AP214）—— 自研纯 Python B-rep 写出器生成，**无需任何 CAD 内核**
- **三种输入**
  - `.dxf` 直接读（ezdxf，含对损坏文件的容错补丁与编码探测）
  - `.dwg` 经 LibreDWG / ODA File Converter 自动转换（引擎自动探测）
  - `.pdf` 走矢量 / 位图双分支：矢量兜底解析 + OpenCV 轮廓提取，位图可选的 OCR 文字识别
- **会读图，不只读线**：GB 图层约定过滤、尺寸标注文字解析、标题栏比例反推（标题栏 `1:N` 优先，
  标准比例投票兜底）、投影视图分组、拉伸/旋转特征初判、深度推断
- **自校验很硬**：B-rep 层做欧拉公式 + 散度定理体积 + 面法线一致性 + 边界环闭合性 + STEP 结构共 5 层校验；
  另附 `verify_step_occ.py` 用 **OpenCASCADE 独立内核**复核导出的 STEP 是否真被接受
- **半自动而非全自动**：机器给出初判 `plan.json` + `issues[]` 待确认清单，人工核对后再建模。
  工程图纸千图千面，"自动出图"不如"自动到 90% 再让人点两下"靠谱

## 🚀 快速开始

```bash
# 1. 装依赖（清华镜像实测 trimesh 会抽风，用阿里云）
python -m venv .venv && . .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt

# 2. 任意输入 → plan.json（半自动第一步）
python scripts/make_plan.py 零件图.pdf --outdir out/

# 3. 人工核对 out/零件图.plan.json —— 重点看 issues[]，填 depth、补/删 part
#    （半自动设计如此，这一步不能省，详见「半自动流程」）

# 4. 出网格模型
python scripts/rebuild3d.py out/零件图.plan.json -o out/零件图.glb

# 5. 出机械实体（STEP）
python scripts/export_cad.py out/零件图.plan.json -o out/ --step out/零件图.stp

# 6. 看模型：把 assets/viewer-template/ 拷到 out/，glb 改名 model.glb，双击 index.html
```

### 没有现成图纸？跑示例

```bash
mkdir -p out

# 拉伸件示例（方板 100×10×60，1:2 打印）—— 可一路出到 STEP
python scripts/make_sample_pdf.py out/plate.pdf
python scripts/make_plan.py out/plate.pdf --outdir out/
python scripts/export_cad.py out/plate.plan.json -o out/ --step out/plate.stp

# 回转件示例（法兰 Ø220×20）—— 只走 GLB/STL（STEP 不支持回转件，见「已知限制」）
python scripts/make_sample_dxf.py out/flange.dxf
python scripts/make_plan.py out/flange.dxf --outdir out/
python scripts/rebuild3d.py out/flange.plan.json -o out/flange.glb
```

> 两个 `make_sample_*.py` 的参数是**输出的文件路径**，不是目录。

## 📁 目录结构

```
cad-drawing-to-3d/
├── SKILL.md                 技能说明书：完整管线、验收标准、踩坑表（最有价值的是这张表）
├── requirements.txt         依赖清单（必需 / 可选分组，带版本上限）
├── LICENSE.md               MIT 许可（标准正文）
├── NOTICE.md                适用范围 / 第三方组件 / 外部工具 / 使用须知
├── scripts/
│   ├── make_plan.py         一站式 CLI：任意输入 → plan.json
│   ├── dwg2dxf.py           DWG→DXF 引擎自动探测（LibreDWG / ODA）
│   ├── dxf_extract.py       DXF → drawing.json（图元提取 + 容错补丁）
│   ├── pdf_extract.py       PDF → drawing.json（矢量/位图双分支 + 比例反推）
│   ├── analyze_views.py     drawing.json → rebuild_plan.json（视图分组 + 特征初判）
│   ├── rebuild3d.py         plan.json → GLB / STL（trimesh 重建 + 水密校验）
│   ├── export_cad.py        plan.json → STEP（自研 B-rep 写出器 + 5 层自校验）
│   ├── verify_step_occ.py   用 OpenCASCADE 独立复核 STEP（强烈建议跑）
│   └── drawing_common.py    drawing.json schema、图层分类、GB 尺寸文字解析
├── references/
│   ├── view-and-feature-rules.md   轮廓/视图/特征/深度的全部判定规则与实测修正
│   ├── cad-layers-and-dims.md      GB 图层约定与尺寸文字解析规则
│   └── gltf-stl-notes.md           格式选型结论 + 依赖库实测坑速查
└── assets/
    ├── viewer-template/     three.js 离线查看器（双击 index.html）
    └── icon.png             技能图标
```

## 📖 使用说明

### 半自动流程（核心约定，别改成全自动）

```
图纸 ──► drawing.json ──► plan.json ──人工核对──► GLB/STL
                       （机器初判+issues）  └──► STEP
```

1. `make_plan.py` 产出 `plan.json`：`parts[]`（op/profile/depth/angle/color/…）+ `issues[]`（待确认项）
2. **逐条处理 `issues[]`**，手工编辑 `plan.json`：填 `depth`、补/删 `part`、调 `angle`、给 `color`
3. `rebuild3d.py` / `export_cad.py` 重建；report 里 `problems` 非空就回头改 plan

`plan.json` 里的 `extrude` 支持 `axis`（Z/Y/X）与 `translate`；底板类的**垂直孔必须用 `axis: "Y"`** 才是圆孔。

### 焊接组件要人工拆件

底板 + 腹板 + 筋板 + 垫板这类**焊接组件**，自动管线按"单零件"假设只会抓到一个壳体。
必须按 BOM 逐件补 `parts[]`（用 `axis` / `translate` 摆位，每件独立 `extrude`）。

### 交付前必跑的两道校验

```bash
# ① 自校验（report 里 problems 必须为 []）
python scripts/rebuild3d.py out/零件图.plan.json -o out/零件图.glb

# ② 独立内核复核 STEP（判据：实体数 == 零件数）
pip install gmsh
python scripts/verify_step_occ.py out/零件图.stp --parts 7
```

> **为什么要第二道**：自家解析器只能查**语法**（悬空引用、类型不匹配），查不出**拓扑健康度**。
> 我们就被这个坑过一次 —— 每条几何边各写一份 `EDGE_CURVE`（而不是相邻两面共享一份并反向引用），
> 语法完全合法、自家自检全绿，但 OpenCASCADE 只读到一堆散面，用户看到的是「STEP 打开后显示不全」。
> 详见 `SKILL.md` §4 的「STEP 拓扑铁律」。

## 🔧 环境依赖

必需（全管线）：

```
ezdxf  pymupdf  opencv-python  numpy  shapely  trimesh  manifold3d  scipy
```

可选：

```
rapidocr_onnxruntime   # 位图 PDF 的 OCR 文字识别兜底
gmsh                   # STEP 独立复核（自带 OpenCASCADE）
```

外部工具（**不打包**，按需自行安装）：

- **DWG 输入**需要 DWG→DXF 引擎：LibreDWG 或 ODA File Converter，两者均免费。
  两者都缺时 `dwg2dxf.py` 以退出码 2 退出并给出下载指引。
- 机械实体（STEP）**不需要**任何外部工具。

## ⚠️ 已知限制

| 限制 | 说明 |
|---|---|
| 机械实体只覆盖**拉伸件** | 回转件（法兰/端盖/轴套）记 `op-unsupported` 并跳过，只走 GLB/STL |
| 部分 DWG 解析不了 | 带私有扩展对象（天正类）的 DWG 会被 LibreDWG 解析损坏 → 需 ODA File Converter，或委托方导标准 DXF |
| 钣金折弯件 / 曲面件 / 装配图 | 不在当前特征规则覆盖范围内 |
| 焊接组件 | 必须人工按 BOM 逐件补 `parts[]`（见上） |
| 输出性质 | 三维模型是**几何近似重建**，仅供设计参考；**不得**直接作为制造、检验或安全评定的依据 |

## 📄 License

[MIT](LICENSE.md)。`assets/viewer-template/vendor/` 下的 three.js 系组件遵循其原始 MIT 许可，版权归原作者所有。
适用范围、第三方组件明细、外部工具（LibreDWG / ODA / gmsh）与使用须知见 [NOTICE.md](NOTICE.md)。

## 👤 作者

**设备人** — luthaitao@qq.com

---

<sub>本仓库同时是 [WorkBuddy](https://www.workbuddy.cn) 技能包：把整个目录放进 `~/.workbuddy/skills/` 即可被 Agent 直接调用；
`SKILL.md` 的 frontmatter 为技能元数据。</sub>
