# 输出格式选型与 trimesh/PyMuPDF/ezdxf 实测坑

## 一、输出格式选型结论：GLB 主 + STL 辅

| 格式 | 角色 | 理由 |
|---|---|---|
| **GLB**（glTF 2.0 二进制） | **主输出** | ① three.js 一等公民（`GLTFLoader` 直接加载，双击 index.html 即可看）② 二进制紧凑、带 COLOR_0 顶点色（按 part 着色）③ 保留多 mesh 与名称（模型树可读）④ trimesh 一行导出 |
| **STL** | 兼容输出 | 3D 打印、所有 CAD 软件、MeshLab/Blender。无颜色无语义，仅几何兜底 |

**为什么不选 STEP/IGES**：需要 OCP/OCCT（~350MB 依赖），且本 skill 的输入是 2D 图纸轮廓，
产出的是网格几何而非参数化 B-rep，STEP 的语义优势用不上。
**为什么不选 OBJ**：文本体积大、无动画/材质扩展需求，GLB 全面覆盖。

### 单位约定（极易出错）

- 管线内部**一律 mm**（drawing.json / plan.json / trimesh 场景单位）
- GLB/glTF 规范无单位，three.js 约定 1 单位 = 1 m → **导入时必须 scale 0.001**（viewer 模板已内置）
- STL 同样无单位，下游按 mm 处理；打印切片软件（Cura/PrusaSlicer）导入 STL 时选 mm

### GLB 颜色

trimesh 的 `ColorVisuals(mesh, face_colors=...)` 导出 GLB 时转 COLOR_0 顶点色。
**需要 scipy**（`face_to_vertex_color` 内部用 `scipy.sparse`）——不装 scipy 导出直接抛
`ModuleNotFoundError`。venv 必须装：`trimesh shapely numpy scipy manifold3d`。

## 二、trimesh 5.1.1 实测坑（全部踩过）

### 1. revolve 的 cap 语义：整圆旋转不封盖

```python
trimesh.creation.revolve(linestring, angle=2*pi, cap=True)  # cap 被忽略！
```

源码逻辑：`if not closed and cap:` —— `angle=2π` 时 `closed=True`，**cap 分支整个跳过**。
开放母线（如矩形 4 点）revolve 得到的是"管"，两端开口，`is_watertight=False`。

**修法**：把截面**闭合成环**（首尾点相同）再 revolve。闭合母线的各边（底/顶/侧/轴）
扫掠面天然拼成封闭曲面（如矩形闭合环 → 完整圆柱含上下底）。
注意闭合环最后一段是零长（首尾重复），trimesh 内部按零面积三角形过滤，不影响。

### 2. revolve 要求母线逆时针

trimesh 文档原话："if your linestring is closed, it needs to be counterclockwise if you
would like face winding and normals facing outwards"。
shapely `exterior.coords` 的绕序不保证（实测给出顺时针 → `mesh.volume` 为负、法线朝内）。
**修法**：revolve 前用鞋带公式检查，顺时针则 `reversed`；布尔差集后兜底
`if mesh.is_watertight and mesh.volume < 0: mesh.invert()`。

### 3. 圆周逼近误差与 sections

revolve 默认 `sections=32`，半径 110mm 的圆体积偏差 0.64%（不可接受）。
**修法**：`sections=128` → 偏差 0.04%。plan.json 可用 `sections` 字段覆盖。

### 4. 布尔差集需要 manifold3d

带孔回转体（法兰）：外轮廓 revolve 得实体，孔环 revolve 得实体，
`mesh.difference(hole_mesh)`。trimesh 5.x 布尔默认走 manifold3d 后端（pip 装
`manifold3d`，有 cp313 wheel）。不装则布尔抛 `MeshError`。

### 5. revolve 对 x=0 顶点敏感

贴轴线的顶点（x=0）会产生退化三角形，毁掉水密性。**修法**：x=0 微移到 1e-6。

### 6. Scene 导出 STL 要 concatenate

`scene.export('stl')` 不支持多 mesh 场景，先
`trimesh.util.concatenate([m1, m2, ...])` 再导出（GLB 无此问题，保留多 mesh+名称+颜色）。

## 三、ezdxf 1.4.4 实测坑（LibreDWG 转出的 DXF）

### 1. `recover.readfile` 没有 encoding 参数

ezdxf 1.4.4 签名：`recover.readfile(filename, errors='surrogateescape')`；
`ezdxf.readfile(filename, encoding=None, errors=...)` 才有。
中文 DXF（`$DWGCODEPAGE=ANSI_936`）走 recover 路径会丢编码，**数字尺寸不受影响**（ASCII），
中文文字可能乱码。策略：先 `ezdxf.readfile(encoding=探测值)`，失败再 `recover.readfile()`。

### 2. LibreDWG 输出的 DXF 有两类结构瑕疵，ezdxf 双模式都拒收

- **XRECORD 的 XDATA 混入 <1000 组码** → `entity_structure_validator` 抛 `DXFXDataError`
- **INSERT 子实体链断裂**（`attribs_follow=1` 但无 ATTRIB）→ `entity_linker_` 抛
  `DXFStructureError: Expected DXF entity INSERT or SEQEND`

**修法**（`dxf_extract.py` 顶部两个 patch）：
```python
# (a) 跳过 XRECORD 结构校验（XRECORD 是扩展记录，几何提取不需要）
recover.EXCLUDE_STRUCTURE_CHECK |= {"XRECORD"}

# (b) 替换 entity_linker 为容错版：链断裂时断开主实体，当前实体按独立实体继续处理。
#     对结构正常的文件行为与原版完全一致。注意 patch 点是
#     ezdxf.sections.entities.entity_linker（_build 在函数内每次新建 linker，
#     引用的是模块级名字）。
```
注意 `_build` 里 `linked_entities = entity_linker()` 是**函数内调用**，
所以 patch `ezdxf.entities.subentity.entity_linker` 无效，必须 patch
`ezdxf.sections.entities.entity_linker`。

### 3. LibreDWG dwg2dxf 对非标准 DWG 输出坏 INSERT（未解决，已知限制）

桌面实测两份国产静设备软件 DWG（AC1018，2.1MB/1MB）：`dwgread -v2` 报
**282+ 个 ERROR**（`unknown EED code 58`、`bit_read_RD buffer overflow`、
`bit_read_BL: unexpected 2-bit code`），转出的 DXF 中 INSERT 实体块名为空、
插入点 (1,1)、scale 含 NaN，且全部几何都在 336 个块参照里 → **几何全丢**。
`--as r2000` 输出同样坏（R2000 路径的 INSERT bug 更明显）。
**结论**：这类带私有扩展对象（天正类）的 DWG，LibreDWG 0.14 解析不了。
替代：ODA File Converter（免费需注册）或 AutoCAD/浩森导一次标准 DXF。

### 4. bulge 弧离散化

LWPOLYLINE 的 bulge 段必须离散化，否则弧轮廓变直线。用
`ezdxf.path.make_path(e).flattening(0.02)`（弦高 0.02mm）。
dxf_extract 对无 bulge 的多段线直接取原始顶点（保精度）。

## 四、PyMuPDF 实测坑（PDF 矢量分支）

### 1. `Rect(Point, Point)` 两点构造不规范化 y

`Rect((-70, 700), (70, 686))` 得到 `Rect(-70, 700, 70, 686)`（y0>y1，"无限矩形"），
`get_drawings()` 读回时被压扁成**零高度线**（实测矩形变 3 点退化路径）。
**只有 4 参数构造 `Rect(x0, y0, x1, y1)` 才规范化**。
画矩形一律用 4 参数；`path_to_points` 解析 're' 项时也做了 min/max 防御。

### 2. `draw_circle` 的 path `closePath=False`

圆判定**不能依赖 closePath**。`circle_fit`（半径相对标准差 < 5%）成功即判圆。
副作用：正六边形等多边形不会误判（半径方差 >5%）。

### 3. 页外内容：文字丢、几何回

`insert_text` 画到页面外（负 x 或 y>页高）→ `get_text` **不返回**；
`draw_line/draw_rect/draw_circle` 画到页外 → `get_drawings` **仍返回**。
生成测试图/处理扫描件时保证内容在页面内；pdf_extract 的 flip 用
`page.rect.height`，内容纵坐标会被翻转成"y 向上"（与 DXF 一致），
翻转带来的常数偏移由 rebuild3d 的归一化消除。

### 4. 比例反推（estimate_scale）

PDF 坐标是 pt 且 CAD 可按任意比例打印，坐标 ≠ mm。用「尺寸文字值 / 页面线段长」
做候选 scale，**值域约束**（线段长 ∈ [d/2, 2d]，防长中心线混入）+
**按 distinct 尺寸值最多的连续簇**聚类（防单一尺寸值的噪声簇刷数）。
实测 1:2 打印的 A3 图反推 scale=0.70556（理论 0.70562，误差 0.01%）。

### 5. get_drawings 的线宽启发式

PDF 无线层。轮廓线一般 ≥0.6pt（CAD 打印笔宽），中心线/尺寸线更细；
虚线（中心线/隐藏线）表现为大量短 path（rect 宽或高 <10pt）。
据此给 `PDF-OUTLINE / PDF-CENTER / PDF-THIN` 伪层名，
analyze 阶段按图层类型过滤（center/dim 类剔除）。

## 五、环境依赖（venv 一键）

```bash
python -m venv .venv && .venv/bin/pip install -i https://mirrors.aliyun.com/pypi/simple/ \
    ezdxf pymupdf trimesh shapely numpy scipy manifold3d
```

**镜像注意**：清华镜像实测 trimesh `from versions: none`（临时抽风），阿里云正常。
位图分支另需 `opencv-python`，OCR 另需 `rapidocr-onnxruntime`。

## 六、three.js 接入要点（viewer 模板约定）

- r128 UMD（`three.min.js` + `examples/js/` 非 module loader）——**file:// 双击可用**；
  ESM + importmap 在 file:// 下被 Chrome CORS 拦截，勿用
- mm→m 统一 scale 0.001；归一化幂等（按当前实际尺寸反算，不累积）
- 零法线修复：STL 常见 `normal` 全 0 → `computeVertexNormals()`
- 材质统一：`MeshBasicMaterial` 无 emissive 无法高亮 → 转 `MeshStandardMaterial`
- 水密性：three 侧不检查，以 rebuild3d 的 `*.report.json` 为准
