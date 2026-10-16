MIT License

Copyright (c) 2026 设备人 (luthaitao@qq.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

## 适用范围

本许可适用于本技能的**代码与文档**（`scripts/`、`references/`、`SKILL.md`）。

## 第三方组件

`assets/viewer-template/vendor/` 下打包了第三方前端库，各自遵循其原始许可，
版权归原作者所有，此处仅为离线可用而**原样随附**，未作修改：

- `three.min.js`、`loaders/GLTFLoader.js`、`loaders/STLLoader.js`、
  `controls/OrbitControls.js` —— three.js（MIT License，© three.js authors）

`assets/viewer-template/index.html`、`js/main.js` 为本技能原创。

## 外部工具

本技能**不打包**下列外部工具，仅在运行时探测本机是否已安装：

- LibreDWG / ODA File Converter —— DWG→DXF 转换（GPL-3.0 / 专有免费版，由使用者自行获取并遵守其许可）
- gmsh —— STEP 独立复核（可选，GPL）

## 使用须知

- 技能的输出完全来源于**使用者自己提供的图纸文件**，其可用范围由使用者依其对
  该图纸的授权自行判断。
- 技能不附带任何标准正文、图纸样本或第三方数据。
- 生成的三维模型（GLB/STL/STEP）为**几何近似重建**，仅供设计参考，
  不得直接作为制造、检验或安全评定的依据。
