# GB 机械制图图层约定与尺寸标注解析

本文件是 `drawing_common.py` 中 `layer_kind()` 与 `parse_dimension_text()` 的规则来源，
也是**人工核对 drawing.json 时的速查表**。规则匹配大小写不敏感，支持中英文图层名。

## 一、图层分类规则（顺序即优先级）

| 分类 | 含义 | 匹配关键词（前缀/后缀/全等） | 重建时处理 |
|---|---|---|---|
| `frame` | 图框/标题栏/定义点 | DEFPOINTS, BORDER, FRAME, TITLE, 图框, 图纸, SHEET, A3, A4 | **剔除**（含大量边框线与标题栏文字，混入会毁掉轮廓） |
| `text` | 文字/技术要求/表格 | TEXT, WZ, NOTE, 文字, 说明, TECH, REQUIRE, TABLE | **剔除**（技术要求常含尺寸数字，留待尺寸解析） |
| `dim` | 尺寸标注 | DIM, BZ, 标注, 尺寸, DIMENSION, GD&T | **剔除**（但 DIMENSION 实体/标注文字的数值要抽出来做深度推断） |
| `center` | 中心线/轴线 | CENTER, CENTRE, ZHONGXIN, 中心线, 轴线, AXIS, PHI, CEN | **剔除**（虚线短段；若混入会被 polygonize 组装成垃圾环） |
| `hidden` | 隐藏线 | HID, HIDDEN, YIN, 虚线, 隐藏 | **剔除**（不可见棱边，建进去会多出内部结构） |
| `hatch` | 剖面线 | HATCH, 剖面, POMIAN, SECTION | **剔除**（剖面线是密集短斜线，是 polygonize 的最大污染源） |
| `outline` | 轮廓实线 | OUTLINE, 粗实线, 轮廓, OBJECT, SOLID, VISIBLE, **0**, DEFAULT, MAIN, PART | **保留**（3D 重建的唯一几何来源） |
| `unknown` | 未识别 | 其它 | **保留但降权**（提示人工确认；常见于自定义图层名） |

### 常见 CAD 软件图层名实例

| 来源 | 轮廓层 | 中心线层 | 尺寸层 |
|---|---|---|---|
| AutoCAD 机械模板（GB） | 0 / 粗实线 | 中心线 / CENTER | 标注 / DIM |
| 中望/浩辰机械 | 0 / 粗实线 | 中心线 | 尺寸标注 |
| SolidWorks 出图 | 0 | 中心线 | 标注 |
| 天喻/开目等国产 CAD | 01 / 实体 | 中心线 | 尺寸 |

### 陷阱

1. **`0` 层万金油**：大量图纸把轮廓、中心线甚至图框全画在 `0` 层。此时图层分类失效，
   必须靠**线型/线宽**过滤：中心线是 `CENTER/DASHED` 线型，剖面线在 `0` 层但成密集斜线群。
   `dxf_extract.py` 已输出 `linetype` 供人工核对；`analyze_views.py` 的 `--exclude-layer`
   可追加排除。
2. **自定义图层名**（如 `L-01`、`Layer3`）→ `unknown` 默认保留。若图纸全画在自定义层且混有
   剖面线，用 `--exclude-layer` 显式排除，或先用 `dwglayers.exe`（LibreDWG 自带）查全部图层名。
3. **DEFPOINTS 层**：AutoCAD 放标注定义点的层，不可打印但含坐标点，必须剔除。

## 二、尺寸文字解析规则（GB/T 4458.4 样式）

`parse_dimension_text()` 识别以下模式，返回 `[{kind, value, count?, tolerance?}]`：

| 图纸写法 | kind | 解析 | 示例输出 |
|---|---|---|---|
| `Ø50` / `φ50` / `⌀50` / DXF 内 `%%c50` | diameter | 直径 | `{"kind":"diameter","value":50}` |
| `2×Ø8` / `2-Ø8` / `4孔Ø8` | diameter + count | 数量×直径 | `{"kind":"diameter","value":8,"count":2}` |
| `R20` | radius | 半径 | `{"kind":"radius","value":20}` |
| `M10-6H` / `M10` | thread | 螺纹公称直径 | `{"kind":"thread","value":10}` |
| `50±0.1` | linear + tolerance | 线性+公差 | `{"kind":"linear","value":50,"tolerance":0.1}` |
| `50 +0.2/-0.1` | linear + tol_plus/minus | 单向公差 | `{"kind":"linear","value":50,"tol_plus":0.2,"tol_minus":0.1}` |
| `30°` | angle | 角度 | `{"kind":"angle","value":30}` |
| `50`（纯数字） | linear | 线性 | `{"kind":"linear","value":50}` |

### DXF 文本转义（`decode_dxf_text` 已处理）

| DXF 原文 | 解码后 |
|---|---|
| `%%c` / `%%C` | Ø |
| `%%d` / `%%D` | ° |
| `%%p` / `%%P` | ± |
| `\P`（MTEXT 分段） | 空格 |

### 编码陷阱（中文图纸必踩）

- 中文 AutoCAD 图纸 DXF 头 `$DWGCODEPAGE = ANSI_936`，文字是 **GBK** 字节。
  ezdxf 默认按 cp1252 解 → 尺寸中文部分乱码，但**数字仍可解析**。
  `dxf_extract.py` 的 `detect_encoding()` 会自动探测并切 GBK。
- LibreDWG 转出的 DXF 偶见 `$DWGCODEPAGE` 丢失 → 乱码。用 `--flatten` 重跑无效，
  只能接受数字解析结果（尺寸数字是 ASCII，不受影响）。

## 三、尺寸数值的三种用途（按优先级）

1. **extrude 深度**：俯视图对位宽度 > 标注匹配 > 人工。见 `analyze_views.py::infer_depth`。
2. **比例反推**（PDF 专用）：`pdf_extract.py::estimate_scale` 用「标注数值 / 页面中最近的
   线段长度」取中位数作为 pdf单位→mm 比例。CAD 打印比例任意（1:1、1:2、A3 缩放到 A4）
   时这是唯一可靠的恢复手段。
3. **孔位/螺纹规格**：`count` 字段标记均布孔（`4孔Ø8`），当前版本仅记录，未自动布孔——
   半自动确认时在 plan.json 中手工补 `holes`。
