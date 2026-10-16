# 声明与第三方组件（NOTICE）

本文件是 [LICENSE.md](LICENSE.md)（MIT License）的补充说明。
许可范围与第三方组件的完整信息集中在此，以便 `LICENSE.md` 保持标准 MIT 正文。

## 适用范围

MIT License 适用于本技能的**代码与文档**：`scripts/`、`references/`、`SKILL.md`、
`README.md`、`assets/viewer-template/index.html`、`assets/viewer-template/js/`。

## 第三方组件（随附打包）

`assets/viewer-template/vendor/` 下打包了第三方前端库。它们各自遵循其**原始许可**，
版权归原作者所有；此处仅为**离线可用**而原样随附，**未作任何修改**：

| 文件 | 组件 | 许可 |
|---|---|---|
| `vendor/three.min.js` | three.js | MIT License，© three.js authors |
| `vendor/loaders/GLTFLoader.js` | three.js 官方示例 | MIT License |
| `vendor/loaders/STLLoader.js` | three.js 官方示例 | MIT License |
| `vendor/controls/OrbitControls.js` | three.js 官方示例 | MIT License |

MIT 全文见 `LICENSE.md`。

## 外部工具（不打包，仅运行时探测）

下列工具**不包含**在本技能内，只在运行时探测本机是否已安装。
是否安装、如何获取、以及遵守何种许可，均由使用者自行决定：

- **LibreDWG** / **ODA File Converter** —— 用于 DWG → DXF 转换
  （分别为 GPL-3.0 与专有免费版）。未安装时 `scripts/dwg2dxf.py` 以退出码 2 退出并给出指引。
- **gmsh** —— 用于 `scripts/verify_step_occ.py` 的 STEP 独立复核（GPL，可选）。
  未安装时该脚本以退出码 2 退出，不影响主流程。

## 使用须知

- 技能的输出**完全来源于使用者自己提供的图纸文件**；其可用范围由使用者依其对
  该图纸所持有的授权自行判断。
- 本技能**不附带**任何标准正文、图纸样本或第三方数据集。
- 生成的三维模型（GLB / STL / STEP）为**几何近似重建**，仅供设计参考，
  **不得直接作为制造、检验或安全评定的依据**。
