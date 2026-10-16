# 视图分组、特征判定与深度推断规则

本文件是 `analyze_views.py` 全部启发式规则的说明与调参依据。
**这些规则是"机器初判"**，输出一律进 `rebuild_plan.json` 的 `issues[]`，
必须经人工确认后才执行 `rebuild3d.py`——不要试图把半自动改成全自动。

## 一、管线总览

```
drawing.json
  │
  ├─ 1. 图层过滤（剔除 frame/text/dim/center/hidden/hatch，见 cad-layers-and-dims.md）
  ├─ 2. 轮廓提取（闭合多段线 / polygonize 组装 / 圆检测）
  ├─ 3. 孔洞识别（包含关系 -> holes）
  ├─ 4. 视图分组（bbox X 重叠聚类 -> 列；列内 Y 间隙 -> 行）
  ├─ 5. 特征判定（revolve / extrude）
  └─ 6. 深度推断（俯视图对位 > 尺寸匹配 > 人工）
  ↓
rebuild_plan.json（parts[] + issues[]，人工确认后进 rebuild3d.py）
```

## 二、轮廓提取的三种来源

| 来源 | 处理 | 风险 |
|---|---|---|
| 闭合多段线 | 直接用顶点成面 | LibreDWG 转出的闭合标志偶丢，用首末点距离 < max(0.05, 宽×1e-4) mm 兜底 |
| LINE + ARC 散件 | **端点链分组**（见下） | 端点容差 0.05mm；>4000 线段会慢（O(n²) 并查集） |
| CIRCLE | `Point.buffer(r, 64)` 成正 64 边形面，同时单独保留 | 圆 = 回转特征判据，绝不能漏；64 段对半径 <5mm 的圆偏粗 |

去重规则：面积与 bbox 都一致（1e-6 容差）的重复轮廓只留一个
（LibreDWG 有时把同一实体导出两遍）。

### 端点链分组 vs polygonize（重要修正）

**不要用 `shapely.ops.polygonize` 一次处理所有线**。polygonize 输出的是平面剖分的
「最小面片」：外矩形被内矩形（孔）的边切成左/中/右三条带，`unary_union` 合并后
**孔变成实心区域**（实测法兰图 Ø100 孔丢失，面积 4400 而非 2400）。
CAD 语义下「外轮廓」与「内孔边」是各自闭合的独立线环。

正确做法（`chain_assembled_polylines`）：
1. 每条线/弧/多段线取其**完整路径点列**（中间顶点不能丢）
2. 端点距离 < 0.05mm → 并查集归组（同一闭合环）
3. 组内按端点连接排序成环（`_order_ring`），环上点列含原始中间顶点
4. 闭合判据 = **所有线段都排入环**；首点与尾点正是首条线段的两端
   （首尾相邻靠这条边闭合），**不要求空间距离 < tol**（此 bug 曾导致所有矩形组装失败）
5. 每个环独立成 Polygon，包含关系判 shell/hole

## 三、视图分组（第一角投影，GB 国标）

算法（`group_views`）：

1. **列聚类**：两个轮廓 bbox 的 **X 向重叠比例 > 0.3**（相对较小宽度）→ 并查集归同列。
   典型三视图：主视图（左列）+ 俯视图（中列，正下方）+ 左视图（右列）。
2. **行聚类**：同列内按 ymin 排序，**Y 向间隙 > max(0.15 × 高度, 0.01 × 全局尺度)** → 拆成新行。
   阈值**不能只用 0.25×max(高度)**——大俯视图（Ø220 圆）会把阈值抬到 50mm+，
   导致与主视图仅隔 10mm 也不拆行（实测法兰图踩过：主/俯视图混成一组）。
3. **主视图判定**（`pick_main_view`）：**不能用「面积最大的组」**——法兰/端盖类俯视图是整圆，
   面积远大于矩形主视图（实测因此选错，shell 变成圆）。规则：
   1. 只有一组 → 就是它
   2. 候选 = **上方无其它组**的组（第一角投影中俯视图在主视图下方）
   3. 多候选取最左列，同列取面积最大

### 陷阱

- **局部视图/剖视图混入**：一个零件图常带 A-A 剖视、局部放大图，它们与主视图 X 不重叠
  → 自成一列，默认**不参与**特征判定（只有面积最大的主视图进 parts）。
  这是保守设计：剖视图是主视图的另一种表达，建进去会重复。
- **斜视图/向视图**：X/Y 均不重叠 → 单独成组，被忽略。需要时人工在 plan.json 加 part。
- **单视图零件图**（法兰、垫片最常见）：只有一行 → 直接按该视图判定，不报错。
- **输入文件本身损坏**（LibreDWG 报 282 个 ERROR、INSERT 块名为空/scale=NaN）：
  这不是算法问题，是 DWG 带私有扩展对象（天正类），LibreDWG 0.14 解析不了。
  判据：drawing.json 里 `n_blocks` 很多但 `n_polylines≈0`。换 ODA File Converter
  或让委托方导标准 DXF（见 gltf-stl-notes.md 三.3）。

## 四、特征判定（`classify_view`）

| 判据 | op | confidence | 说明 |
|---|---|---|---|
| ≥2 个同心圆（圆心距 < 0.5 mm 且半径不同） | `revolve` | high | 法兰/端盖/轴套/环件的强特征。同心圆组数也暗示"回转体 + 内孔"层数 |
| ≥1 个闭合多边形 | `extrude` | medium | 板/块/支架类。**默认** extrude（保守：深度无来源时记 issue 而不是瞎猜） |
| 都没有 | `unknown` | low | 记 issue `no-profile` |

**为什么不自动 revolve 所有圆**：单个圆可能是圆柱孔的俯视、也可能是球/回转体端面，
没有第二信息源时 revolve 会造出错误实体。两圆同心才有把握。

## 五、revolve 截面提取

trimesh `revolve` 绕 **Y 轴**，输入截面必须是 **x ≥ 0 的半轮廓**：

```
截面与 box(0, ymin-1, xmax+1, ymax+1) 求交 -> 取面积最大的多边形 -> exterior 点列
x=0 的顶点微移到 1e-6（trimesh revolve 对零半径顶点会产生退化三角形，毁掉水密性）
```

主轴约定：**图纸主视图的 X = 回转半径，Y = 回转体轴向**。模型绕 Y 轴旋转，
因此法兰/端盖类零件建模后"平放"（轴向 = 世界 Y）。若业务需要轴向 = Z，
在后处理统一 `rotateX(-90°)`，或在 plan.json 手工加 `post_rotate` 字段。

跨轴轮廓（如完整圆）：右半 ∩ 轮廓 = 右半圆 → revolve 得球，正确。
轮廓完全在 x<0（轴线画反）：自动取左半并记 issue `revolve-half` 要求人工核对。

## 六、extrude 深度推断（`infer_depth`）

按优先级：

1. **俯视图对位宽度**（`top-view-width`）：主视图正下方（X 重叠 > 0.55 且整体 y 更低）
   的视图，其 **Y 向宽度** = 零件深度。confidence=medium，必记 issue `depth-check-halfview`
   ——因为 GB 允许俯视图画半视图（对称简化），半视图要 ×2。
2. **主视图高度兜底**（`main-view-height`）：主视图近似等宽高（差 < 8%）时取高度作候选，
   confidence=low，记 issue 人工确认。**只适用于方板类**，圆盘不适用。
3. **都没有** → `depth=null`，记 issue `depth-missing`，**必须人工填**。
   绝不默认给 10mm 之类的拍脑袋值——半自动的底线是不猜。

尺寸佐证：俯视图宽度若与某标注数值差 < 5%，depth_source 追加 `(dim=xx佐证)`，
confidence 保持 medium。

## 七、issues 代码表（人工核对清单）

| code | 含义 | 人工动作 |
|---|---|---|
| `no-profile` | 视图无闭合轮廓 | 检查图层过滤/位图 DPI/线型 |
| `no-shell` | 全部轮廓被判为孔洞 | 检查 `--exclude-layer` 是否排除过度 |
| `depth-missing` | extrude 深度无来源 | **必填**：编辑 plan.json 的 `parts[].depth` |
| `depth-low-confidence` | 深度来源置信度低 | 核对 depth 值 |
| `depth-check-halfview` | 俯视图宽度取深度 | 确认俯视图是否半视图（半视图 ×2） |
| `revolve-half` | 轴线方向存疑 | 核对 revolve 截面方向 |

## 八、rebuild_plan.json 字段约定（人工可编辑）

```json
{
  "op": "extrude | revolve",
  "profile": {"shell": [[x,y],...], "holes": [[[x,y],...], ...]},
  "axis": "Z（extrude）/ Y（revolve）",
  "angle": 360.0,
  "depth": 20.0,
  "color": "#c8a165（十六进制或 null 用默认色板）",
  "confidence": "high | medium | low"
}
```

`shell`/`holes` 是 shapely 多边形点列（首末不重复）。删除一个 hole = 从 `holes` 数组移除；
加深一个不通孔成通孔 = 把 hole 加入 `holes`。

## 九、已知能力边界（不要承诺用户做不到的事）

| 场景 | 当前状态 |
|---|---|
| 法兰/端盖/轴套/环件（同心圆 + 矩形半剖） | revolve 规则覆盖，成功率高 |
| 板/块/支架（单闭合多边形） | extrude 覆盖，深度靠俯视图/标注/人工 |
| 多视图复杂机加件（键槽、斜孔、螺纹） | 轮廓可提，**特征不自动识别**，靠人工补 part |
| 钣金折弯件 | 不支持（折弯线无法从轮廓推断） |
| 曲面件（叶轮、螺旋) | 不支持 |
| 装配图（多零件） | 单零件优先；装配图需按图块拆分，当前只记 INSERT 不展开 |
