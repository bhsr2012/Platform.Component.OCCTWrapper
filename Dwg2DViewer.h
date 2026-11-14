#pragma once

// NOMINMAX before windows.h (min/max macros collide with std::min/std::max -> C2589).
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <gl/gl.h>

namespace Platform { namespace Component { namespace OCCTWrapper {

    /// <summary>
    /// Native 2D CAD (DXF/DWG) renderer hosted in the C++/CLI layer.
    /// Consumes the flat RenderPacket produced by Platform.Component.Dwg:
    /// polylines / triangle fills / text runs, with per-path RGBA, world-unit
    /// line width and a layer index for visibility control.
    ///
    /// Rendering model (legacy fixed pipeline, GL 1.1 only - no extensions,
    /// no shaders, zero extra deployment DLLs):
    ///   * paths    -> client-side vertex arrays, one GL_LINES draw call per
    ///                 (layer, colour) bucket; hairline paths collapse into a
    ///                 single call, wide paths fall back to per-path glLineWidth.
    ///   * fills    -> scissor-limited stencil INVERT odd-even fill, so holes
    ///                 and nested loops work without explicit bridging.
    ///   * text     -> GDI glyph atlas (GetGlyphOutline, GGO_GRAY8_BITMAP)
    ///                 so system fonts cover CJK; quads are rotated in world
    ///                 space before the view transform.
    /// MSAA is requested via the dummy-window wglChoosePixelFormatARB probe
    /// (same trick as OcctViewer): thin CAD lines alias badly without it.
    /// Typical usage: Create(hwnd) -> SetScene(...) -> Fit() -> Render();
    /// on interaction Zoom/Pan then Render().
    /// </summary>
    public ref class Dwg2DViewer abstract sealed
    {
    public:
        // Create the GL context on the given HWND. 0=ok, negative=fail.
        static int Create(System::IntPtr hwnd);
        // Destroy context and release DC
        static void Destroy();
        // Whether context is ready
        static bool IsReady();

        // ---- scene data (flat arrays, mirroring Platform.Component.Dwg.RenderPacket) ----
        // pathXY: 2 floats/vertex; pathStart/pathCount per path; pathRGBA 4/path;
        // pathWidth world units (0 = hairline); pathLayer layer index.
        static void SetPaths(array<float>^ xy, array<int>^ start, array<int>^ count,
                             array<unsigned char>^ rgba, array<float>^ width, array<unsigned char>^ layer);
        // fillXY: 2 floats/vertex; fillPath: fill-path index per triangle;
        // fillRGBA 4/fill-path; fillLayer layer index per fill path.
        static void SetFills(array<float>^ xy, array<int>^ triPath,
                             array<unsigned char>^ rgba, array<unsigned char>^ layer);
        // textData: 6 floats per text (x, y, height, rotDeg, halign, valign);
        // textRGBA 4/text; textLayer 1/text; textContent the strings.
        static void SetTexts(array<float>^ data, array<unsigned char>^ rgba, array<unsigned char>^ layer,
                             array<System::String^>^ content);
        // Layer names (index = slot). Must be set before SetPaths/SetFills/SetTexts
        // so layer indices resolve.
        static void SetLayerNames(array<System::String^>^ names);
        // World bounds (minX, minY, maxX, maxY) for Fit().
        static void SetBounds(float minX, float minY, float maxX, float maxY);

        // ---- view ----
        static void Fit();
        // factor > 1 zooms in; anchored at the given client pixel.
        static void ZoomAt(float factor, float clientX, float clientY);
        static void Zoom(float factor);
        // Pan by screen pixel deltas.
        static void Pan(float dxPixels, float dyPixels);
        static void ScreenToWorld(float clientX, float clientY,
                                  [System::Runtime::InteropServices::Out] float% wx,
                                  [System::Runtime::InteropServices::Out] float% wy);
        static void WorldToScreen(float wx, float wy,
                                  [System::Runtime::InteropServices::Out] float% sx,
                                  [System::Runtime::InteropServices::Out] float% sy);
        static double ViewScale();          // pixels per world unit
        static void ViewCenter([System::Runtime::InteropServices::Out] float% cx,
                               [System::Runtime::InteropServices::Out] float% cy);

        // ---- appearance ----
        // Background: 4 presets (0=white 1=black 2=dark grey 3=light grey) or a
        // custom RGB via SetBackgroundRgb (preset slot becomes -1).
        static void SetBackgroundIndex(int idx);
        static int BackgroundIndex();
        static void SetBackgroundRgb(float r, float g, float b);
        static void GetBackgroundRgb([System::Runtime::InteropServices::Out] float% r,
                                     [System::Runtime::InteropServices::Out] float% g,
                                     [System::Runtime::InteropServices::Out] float% b);
        static void SetLayerVisible(int layerIdx, bool visible);
        static bool LayerVisible(int layerIdx);
        static void SetAllLayersVisible(bool visible);

        // ---- overlay (markup / measurement annotations) ----
        // Overlay geometry is drawn ON TOP of the scene with SCREEN-SPACE widths
        // (pixels), so measurement rubber bands and markup keep a constant size
        // at any zoom. World coordinates, same as the scene.
        static void SetOverlayPaths(array<float>^ xy, array<int>^ start, array<int>^ count,
                                    array<unsigned char>^ rgba, array<float>^ widthPx,
                                    array<float>^ dashPx);   // dashPx<=0 = solid
        // Overlay fills: same triangle layout as SetFills, drawn after the scene.
        static void SetOverlayFills(array<float>^ xy, array<int>^ triPath,
                                    array<unsigned char>^ rgba);
        // Overlay texts: same 6-float layout as SetTexts.
        static void SetOverlayTexts(array<float>^ data, array<unsigned char>^ rgba,
                                    array<System::String^>^ content);
        static void ClearOverlay();

        // ---- printing ----
        // Render the CURRENT view at an arbitrary resolution into a 24-bit BMP
        // (e.g. 300dpi page size). fitToView=true re-frames the drawing to the
        // offscreen viewport first (print output is well framed regardless of
        // the on-screen zoom); false = WYSIWYG. Uses a hidden window with its
        // own context, sharing GL objects with the main one.
        static bool RenderOffscreen(int width, int height, System::String^ bmpPath,
                                    bool fitToView);

        // ---- frame ----
        static void Resize(int w, int h);
        static void Render();
        // Save the current front buffer to a 24-bit BMP (diagnostics).
        static bool SaveFrame(System::String^ path);
        static System::String^ Diagnostics();
    };

}}}
