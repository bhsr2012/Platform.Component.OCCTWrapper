# Platform.Component.OCCTWrapper

C++/CLI 薄胶层，将 **Open CASCADE Technology (OCCT)** 以**静态链接**方式链入，为 .NET (C#) 主程序提供 STEP / IGES / BREP 的读取与三角化能力（用于 3D 图纸预览）。

## 许可证（LGPL 2.1 + Open CASCADE 例外 v1.0）

本项目以 GNU **LGPL v2.1** 分发，并适用 **Open Cascade 例外 v1.0**（允许静态链接而不要求整个主程序开源）。

- `LICENSE` —— GNU LGPL v2.1 全文
- `OCCT_LGPL_EXCEPTION.txt` —— Open Cascade 例外 v1.0 全文

> 因本项目与 OCCT 采用**静态链接**（非动态库分发），依据 LGPL 2.1 第 6 条，我们提供本组件的**完整源码与构建工程**，使最终用户可获取兼容版本的 OCCT 后自行**重新编译、替换**本组件，满足"可重链接"要求。OCCT 本身仍为第三方 LGPL 代码，其源码请从 https://dev.opencascade.org/ 获取。

## OCCT 版本依赖

| 目标 | 工具集 | OCCT 版本 | 头文件 / 静态库路径（示例，按需改 vcxproj） |
|------|--------|-----------|----------------------------------------------|
| x64（主，v143） | v143 | OCCT **8.0.1** | `D:\VC\OCCT\prebuilt\opencascade-8.0.1-vc14-64\inc` 与 `D:\VC\OCCT\build\win64\vc14\lib`（自编 `TK*.lib` 静态库） |
| Win32 / XP（v140_xp） | v140_xp | OCCT **7.3** | `D:\VC\OCCT\install_xp73\inc` 与 `D:\VC\OCCT\install_xp73\win32\vc14\lib` |

> 两个 vcxproj **共用同一批 `.cpp/.h`**，仅按目标平台分别构建；程序集名称均为 `Platform.Component.OCCTWrapper`（C# 按简单名称绑定），因此通过**输出目录**区分位数（x64 在 `Components\OCCT_X64\`，Win32 在 `Components\OCCT_X86\`），不改名。

## 构建

前提：自备上表对应版本的 OCCT **静态库**（`TK*.lib`）与头文件。为得到纯 LGPL 闭包，重编 OCCT 时关闭第三方特性：

```
-DUSE_FREEIMAGE=OFF -DUSE_FREETYPE=OFF -DUSE_TBB=OFF
-DUSE_RAPIDJSON=OFF -DUSE_OPENVR=OFF -DUSE_GL2PS=OFF
```

（视 OCCT 版本增减；目的：使静态闭包仅含 OCCT 内部模块与系统 CRT，不引入 FreeImage(GPL)/FFmpeg(LGPL)/freetype/TBB 等第三方代码。）

1. 用 Visual Studio 打开对应工程：
   - x64：`Platform.Component.OCCTWrapper.vcxproj`
   - Win32 / XP：`XP\OcctWrapperXP.vcxproj`
2. 选择 **Release | x64**（或 **Release | Win32**）构建。
3. 产物：
   - x64 → `bin\Debug\Components\Platform.Component.OCCTWrapper.dll`
   - Win32 → `bin\Debug\Components\OCCT_X86\Platform.Component.OCCTWrapper.dll`

## 重新链接（替换组件）

更新 OCCT 版本或修改本组件后，用新编译的 `Platform.Component.OCCTWrapper.dll` 替换部署目录中对应位数的文件即可：
- x64 替换 `Components\` 下
- Win32 替换 `Components\OCCT_X86\` 下

本组件**不随包分发任何 OCCT 动态库（TK\*.dll）**——OCCT 已静态链入本 DLL（导入表中无任何 `TK*.dll`）。

## 目录结构

```
Platform.Component.OCCTWrapper/
├── Platform.Component.OCCTWrapper.vcxproj   # x64 / v143 / OCCT 8.0.1
├── XP/
│   └── OcctWrapperXP.vcxproj               # Win32 / v140_xp / OCCT 7.3
├── OcctViewer.cpp / .h                     # 3D 视图封装
├── OcctBrepReader.cpp / .h                 # BREP / STEP / IGES 读取与三角化
└── Dwg2DViewer.cpp / .h                    # 2D 图纸视图（与 CadPreview 共用）
```
