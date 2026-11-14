#pragma once

namespace Platform { namespace Component { namespace OCCTWrapper {

    /// <summary>
    /// B-rep 文件读取器（STEP / IGES / BREP）：OCCT 三角化后输出统一三角网格。
    /// positions = xyz 平铺（float，长度 = 顶点数*3）；indices = 三角形顶点索引（int，长度 = 面数*3）。
    /// USE_OCCT 未定义时为占位实现（IsAvailable=false，Read 返回 1）；
    /// OCCT 静态库就绪后以 USE_OCCT 宏重新编译本工程即切换为真实现，API 不变。
    /// </summary>
    public ref class OcctBrepReader abstract sealed
    {
    public:
        /// <summary>真 OCCT 实现是否已编入（占位版为 false）。</summary>
        static bool IsAvailable();

        /// <summary>
        /// 读取 B-rep 文件并三角化。
        /// 返回：0=成功；1=封装未就绪（占位版）；负=失败（error 带出原因）。
        /// </summary>
        static int Read(System::String^ filePath,
                        [System::Runtime::InteropServices::Out] array<float>^% positions,
                        [System::Runtime::InteropServices::Out] array<int>^% indices,
                        [System::Runtime::InteropServices::Out] System::String^% error);

        /// <summary>
        /// 分组版读取：按 TopoDS_Solid 分组三角化（IGES 等无实体形状退化为按面分组），
        /// faceGroups[i] = 第 i 个三角形所属组号（0 起），供前端按部件自动着色。
        /// 返回码语义同 Read。
        /// </summary>
        static int ReadParts(System::String^ filePath,
                        [System::Runtime::InteropServices::Out] array<float>^% positions,
                        [System::Runtime::InteropServices::Out] array<int>^% indices,
                        [System::Runtime::InteropServices::Out] array<int>^% faceGroups,
                        [System::Runtime::InteropServices::Out] System::String^% error);

        /// <summary>
        /// 额外导出 snapFeatures：每条 TopoDS_Edge 的精确解析几何（直线首末点 / 圆圆心、
        /// 半径、平面与参数范围）+ 离散折线，供 C# 侧做 CAD 式对象捕捉（端点/中点/圆心/
        /// 象限点/交点/垂足/节点/最近点）。布局见 OcctBrepReader.cpp::AppendSnapFeature。
        /// 无 B-rep 拓扑的格式（STL/OBJ）返回空数组。
        /// </summary>
        /// <summary>
        /// 分组读取 + 真实棱边（半透明模式的零件轮廓）。
        /// edgeLines = 线段平铺（每 12 个 float 一条线段：ax,ay,az,bx,by,bz,nAx,nAy,nAz,nBx,nBy,nBz，
        /// 其中 nA/nB 为该边两侧面的法向），取自 B-rep 的 TopoDS_Edge（已剔除 seam 与退化边），
        /// 无拓扑的格式返回空数组。
        /// edgeGroups[i] = 第 i 条线段所属组号（0 起，与 faceGroups 同一套分组）：
        /// 按部件收集，跨部件的边会在每个部件各出现一次，查看器据此在隐藏某部件时
        /// 同时隐藏其轮廓（任一侧部件可见即显示）。可为 nullptr（不需要时）。
        /// 返回码语义同 Read。
        /// </summary>
        /// <summary>Diagnostic: force the flat (OneShape) reader, bypassing XCAF.</summary>
        static int ReadFlat(System::String^ filePath,
                        [System::Runtime::InteropServices::Out] array<float>^% positions,
                        [System::Runtime::InteropServices::Out] array<int>^% indices,
                        [System::Runtime::InteropServices::Out] array<int>^% faceGroups,
                        [System::Runtime::InteropServices::Out] System::String^% error);
        static int ReadPartsEx(System::String^ filePath,
                        [System::Runtime::InteropServices::Out] array<float>^% positions,
                        [System::Runtime::InteropServices::Out] array<int>^% indices,
                        [System::Runtime::InteropServices::Out] array<int>^% faceGroups,
                        [System::Runtime::InteropServices::Out] array<float>^% edgeLines,
                        [System::Runtime::InteropServices::Out] array<int>^% edgeGroups,
                        [System::Runtime::InteropServices::Out] array<float>^% snapFeatures,
                        [System::Runtime::InteropServices::Out] array<System::String^>^% partNames,
                        [System::Runtime::InteropServices::Out] array<int>^% partParents,
                        [System::Runtime::InteropServices::Out] array<bool>^% partHasGeo,
                        [System::Runtime::InteropServices::Out] System::String^% error);
    };

}}}
