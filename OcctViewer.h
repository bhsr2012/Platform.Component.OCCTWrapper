#pragma once

// NOMINMAX must be defined before <windows.h>; otherwise windows.h's min/max macros
// collide with std::min/std::max (C2589). Also forced via vcxproj PreprocessorDefinitions.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <gl/gl.h>

namespace Platform { namespace Component { namespace OCCTWrapper {

    /// <summary>
    /// Native OpenGL viewer hosted in the C++/CLI layer of OCCTWrapper.
    /// Creates a WGL context on a HWND passed from C#, and does GPU rasterization
    /// via the system opengl32.dll (no OCCT visualization module / TKOpenGl needed,
    /// zero extra deployment DLLs - preserves the zero-external-dependency contract).
    /// Typical usage: Create(hwnd) -> SetMesh(...) -> Fit() -> Render();
    /// on interaction Orbit/Zoom/Pan/ResetOrientation then Render().
    /// SetMesh() resets the orientation to the default view (fresh model); Fit()
    /// frames distance/target only and keeps whatever angle Orbit() produced.
    /// </summary>
    public ref class OcctViewer abstract sealed
    {
    public:
        // Create OpenGL context on the given HWND and init the viewer. 0=ok, negative=fail.
        static int Create(System::IntPtr hwnd);
        // Destroy context and release DC
        static void Destroy();
        // Load triangle mesh: positions=xyz floats; indices=triangle vertex indices; groups=per-triangle group id (for coloring).
        static void SetMesh(array<float>^ positions, array<int>^ indices, array<int>^ groups);
        // Overload that also carries the assembly tree. parents[p] is the parent node
        // index of node p, or -1 for a root. STEP/IGES read via XCAF include assembly
        // nodes that own no geometry but still occupy a node (and a checkbox); supplying
        // them lets hiding a parent cascade to all its children. Pass null for a flat
        // mesh (STL/OBJ): behaviour is identical to the 3-arg overload above.
        static void SetMesh(array<float>^ positions, array<int>^ indices, array<int>^ groups, array<int>^ parents);
        // Part outline segments for the translucent mode, taken from the B-rep topology by
        // OcctBrepReader::ReadPartsEx, 12 floats per segment:
        //   ax,ay,az, bx,by,bz, nAx,nAy,nAz, nBx,nBy,nBz
        // where nA/nB are the normals of the two faces the edge separates. At draw time the
        // sign of nA/nB against the view vector decides silhouette (outline) vs interior
        // crease, which is what keeps a densely creased part legible. Pass null to clear.
        // Without edges the translucent view relies on colour alone, and overlapping parts
        // mix into one another (red seen through green reads as brown).
        static void SetEdges(array<float>^ lines);
        // Same, plus one group id per segment (length = lines->Length/12, same numbering as
        // the SetMesh groups). Lets the viewer hide a part's outlines together with its
        // faces. Pass null for groups to keep every outline regardless of visibility.
        static void SetEdges(array<float>^ lines, array<int>^ groups);
        // Wireframe (edge overlay) display policy, orthogonal to SetTransparency: the
        // latter controls FACES, this controls EDGES. show: 0=off, 1=only while the model
        // is shown translucent (default; restores the pre-edge-system behaviour), 2=always.
        // silhouetteOnly keeps only part-silhouette outlines and drops interior creases.
        // drawBrep / drawMesh independently enable B-rep outlines and mesh-file lines.
        static void SetEdgePolicy(int show, bool silhouetteOnly, bool drawBrep, bool drawMesh);
        // Point cloud (PLY point-only files, or Mesh3D.Points): 3 floats per point.
        // Drawn as GL_POINTS in BOTH opaque and translucent modes.
        static void SetPoints(array<float>^ points);
        // Viewport resize
        static void Resize(int w, int h);
        // Redraw current frame
        static void Render();
        // Free orbit rotation (screen pixel deltas; left-drag). Rotates about the
        // current screen axes, so there is no pole and no pitch clamp.
        static void Orbit(float dx, float dy);
        // While the user drags, skip the expensive SSAO chain (MSAA is kept) so a
        // dense assembly still redraws at mouse rate. Restores full quality on release.
        static void SetInteracting(bool on);
        // Zoom (factor>1 zooms in)
        static void Zoom(float factor);
        // Pan (screen pixel deltas; right/middle-drag)
        static void Pan(float dx, float dy);
        // Fit to window (auto framing from model bounding sphere). Resets distance
        // and target only - the current orientation is preserved, as in UG/NX,
        // ProE/Creo and SolidWorks. Use ResetOrientation() to go back to home view.
        static void Fit();
        // Restore the default orientation WITHOUT changing zoom / pan. User-triggered
        // ("Reset view" button or R key); new geometry is reset inside SetMesh().
        static void ResetOrientation();
        // Set overall opacity (1.0 = opaque; <1 = semi-transparent X-ray mode)
        static void SetTransparency(float alpha);
        // Whether context is ready
        static bool IsReady();
        // Actual MSAA sample count granted by the driver (0 = the installed pixel
        // format has no multisample buffer). Diagnostic only: lets the host prove
        // anti-aliasing really engaged instead of trusting the request.
        static int MsaaSamples();
        // One-line OpenGL diagnostics: vendor / renderer / version / MSAA samples.
        // "renderer=GDI Generic" means Windows fell back to the software OpenGL 1.1
        // implementation (no hardware GL, no multisample formats) - typically
        // because the window sits on a virtual/remote display adapter.
        static System::String^ Diagnostics();
        // Save the CURRENT front buffer to a 24-bit BMP (diagnostic only). Lets a test
        // harness eyeball anti-aliasing / lighting / AO without a human at the screen.
        static bool SaveFrame(System::String^ path);
        // Turn screen-space ambient occlusion on/off. The flag is process-wide and is NOT
        // cleared by Destroy(), so a host can set it once before Create(). AO resources are
        // kept while off, so toggling back on costs nothing.
        static void SetAoEnabled(bool on);
        // Whether AO is currently switched on (the user intent, before the "driver could not
        // build the AO chain" check). Use Diagnostics() to confirm it actually engaged.
        static bool AoEnabled();

        // ---- part (layer) show / hide --------------------------------------
        // The mesh carries one group id per triangle (see SetMesh). A group is one B-rep
        // SOLID/SHELL as read by OcctBrepReader::ReadPartsEx, i.e. one PART of an assembly -
        // hiding a few of them is what makes a dense assembly legible. A mesh without group
        // information (STL/OBJ) reports GroupCount()==0 and ignores these calls.
        static int GroupCount();
        // Show/hide one part (ids outside [0, GroupCount) are ignored). Next Render picks it up.
        static void SetGroupVisible(int group, bool visible);
        static bool GroupVisible(int group);
        // Show/hide every part at once.
        static void SetAllGroupsVisible(bool visible);
        // RGB triplets, 3 floats per part, matching the colours the CURRENT view actually
        // uses: kPartPalette (24 colours) when opaque, the 8-hue glass palette when
        // translucent. Hosts must re-read this after every transparency switch so the
        // tree swatches keep following the model. Empty array when there are no parts.
        static array<float>^ GroupColors();

        // Measurement support: return the current camera so the C# host can do
        // screen<->world picking / projection without reaching into GL internals.
        // Packed floats: [ex,ey,ez, tx,ty,tz, ux,uy,uz, fovYRad, aspect].
        //   eye = target + dist * g_camF   (matches the gluLookAt call in Render)
        //   up  = g_camU                    (the up vector passed to gluLookAt)
        //   fovYRad is always 45 degrees   (see the gluPerspective call in Render)
        // Returns null only if no context/mesh has ever been set up.
        static array<float>^ GetCamera();
    };

}}}
