#include "OcctViewer.h"
#include <gl/glu.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>

// System OpenGL / window libs (ship with Windows; zero extra deployment DLLs,
// preserving OCCTWrapper's zero-external-dependency contract).
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")

// NOTE ON ENCODING: this file MUST stay pure ASCII. The project compiles as
// code page 936 (GBK), and UTF-8 multi-byte sequences can decode to a byte
// equal to '\' (0x5C). If such a byte lands at the end of a "//" comment line,
// the C preprocessor treats it as a LINE CONTINUATION and swallows the next
// line (a variable declaration), producing bogus "undeclared identifier" errors.
// Keep all comments here in English/ASCII.

using namespace Platform::Component::OCCTWrapper;

namespace {
    HWND  g_hwnd  = NULL;
    HDC   g_hdc   = NULL;
    HGLRC g_hglrc = NULL;

    std::vector<float> g_pos;      // vertex xyz
    std::vector<int>   g_idx;      // triangle indices
    std::vector<float> g_nrm;      // vertex normals (same length as g_pos)
    std::vector<float> g_col;      // vertex colors rgba (4 floats per vertex)

    // Two extra colour sets, only filled while the translucent mode is active (see
    // UpdateGlassColors). Kept separate from g_col so toggling back to opaque costs
    // nothing: g_col survives untouched as the palette source.
    std::vector<float> g_colFront; // outer membrane rgba
    std::vector<float> g_colBack;  // inner walls rgba

    // ---- Per-part depth layering -----------------------------------------
    // "A stack of objects": the user's model of what translucency should look like.
    // The part nearest the eye is the MOST transparent and opacity builds up towards the
    // back, so you look THROUGH the outer skin and SEE the parts inside instead of
    // staring at one flat veil. Each vertex remembers which part it belongs to (parts do
    // not share vertices, verified on the 679k-triangle assembly: 0 shared), so this costs
    // one array read per vertex and one small loop over the ~1.1k parts per frame.
    std::vector<int>   g_vgroup;       // vertex -> part id
    int                g_groupCount = 0;
    std::vector<float> g_groupCenter;  // centroid per part (3 floats each, model space)
    std::vector<float> g_groupLayer;   // 0 = nearest the eye, 1 = farthest (per frame)

    // Palette used only while translucent. The opaque palette is deliberately pale
    // (HSL lightness 0.74) because the three-point rig supplies the modelling there; but
    // an alpha of 0.15..0.4 over a WHITE background turns a pale colour into nothing.
    // Same golden-angle hues, more saturation and less lightness, so a thin layer still
    // carries identifiable colour.
    std::vector<float> g_groupColor;   // rgb per part, translucent palette

    // ---- per-part show / hide ----------------------------------------------
    // Complex assemblies are unreadable as a whole; hiding a handful of parts is the
    // standard way in (SolidWorks / FreeCAD). The data is already there: one group id per
    // triangle (see SetMesh), so "hide part g" = drop its triangles. The full index buffer
    // is kept and only a compacted copy of the VISIBLE triangles is built, lazily, the
    // first time something is actually hidden - with everything visible the draw call
    // keeps using g_idx directly and costs nothing.
    std::vector<int>  g_tgroup;        // triangle -> part id (same source as g_vgroup)
    std::vector<char> g_groupVisible;  // per part, 1 = shown; EMPTY = mesh has no parts
    std::vector<int>  g_partParent;    // per part, parent node index or -1 (assembly tree)
    std::vector<int>  g_idxVis;        // compacted indices, visible parts only
    bool              g_idxVisValid = false;

    // Edge segments -> part id (from OcctBrepReader::ReadPartsEx edgeGroups). The outline
    // pass runs only in translucent mode, and an outline without its part would float in
    // mid-air, so hidden parts take their outlines with them. Colours are recomputed every
    // frame by UpdateEdgeColors, so the compacted copy is rebuilt per frame while any
    // filtering is active (an O(segments) gather, next to nothing beside the per-vertex
    // glass pass it shares the frame with).
    std::vector<int>   g_egroup;       // edge segment -> part id
    std::vector<float> g_edgePosVis;   // compacted visible edge positions
    std::vector<float> g_edgeColVis;   // compacted visible edge colours

    // ---- part edges (translucent mode only) -------------------------------
    // Colour alone cannot separate 1114 overlapping parts: red seen through green is
    // brown, i.e. it reads as a third part. SolidWorks solves exactly this by drawing the
    // model translucent AND keeping its edges, and we can do the same from data we already
    // have: each "group" is one B-rep FACE, and faces do not share vertices, so the
    // triangles of a face form an isolated block. Inside such a block, an edge used by only
    // ONE triangle is precisely the boundary of that face - a real CAD edge. Collecting
    // those costs one sort per face at load time and gives every part a crisp outline.
    // NOTE: positions and normals are kept in SEPARATE arrays on purpose. The edge pass
    // feeds g_edgePos straight to glVertexPointer/glDrawArrays(GL_LINES, n/3), which draws
    // every 3 floats as one vertex; keeping the normals interleaved would draw them as two
    // extra bogus vertices per segment (a screenful of stray lines).
    std::vector<float> g_edgePos;   // 6 floats per segment (two endpoints)
    std::vector<float> g_pointPos;  // 3 floats per point (point cloud)
    std::vector<float> g_edgeNrm;   // 6 floats per segment (normals of the two adjacent faces)
    std::vector<float> g_edgeCol;   // 8 floats per segment (two rgba vertices)

    int   g_vcount = 0;
    float g_cx = 0, g_cy = 0, g_cz = 0;   // model center
    float g_radius = 1.0f;                 // bounding sphere radius
    // Axis-aligned bounds, kept only for the tight box-fit in Fit().  The bounding
    // sphere is still used for clipping planes, fog, AO radius and zoom clamps,
    // so it must NOT be replaced by the box.
    float g_bmin[3] = { 0.0f, 0.0f, 0.0f };
    float g_bmax[3] = { 0.0f, 0.0f, 0.0f };
    bool  g_hasBounds = false;
    // Camera orientation kept as an orthonormal basis instead of spherical az/el.
    // F = target -> eye, U = screen up passed to gluLookAt. R is the historical
    // cross(f, worldUp) vector used by Pan and the headlights; with gluLookAt(up = U)
    // the driver's screen-right is actually -R. Keep that convention: Orbit/Pan/light
    // formulas were all tuned against it and all three stay consistent.
    // Why: az/el has a hard pole at +/-89 deg where pitch HAD to be clamped, so a drag
    // stopped there and horizontal drag near the pole spun the model in place. A full
    // basis has no pole - any direction can rotate through 360 deg, which is how
    // UG/NX, ProE/Creo and SolidWorks behave.
    // Initial values are the exact expansion of the old az=0.6 / el=0.4 default,
    // so the very first frame is pixel-identical even if Fit() has not run yet.
    // Mirrored verbatim by SoftRender3D.cs (CamF/CamU; its CamR is -g_camR) - the
    // GPU and CPU backends must start from the SAME view.
    float g_camF[3] = { 0.520070f, 0.389418f, 0.760184f };
    float g_camR[3] = { -0.825336f, 0.000000f, 0.564642f };
    float g_camU[3] = { -0.219882f, 0.921061f, -0.321401f };
    float g_dist = 100.0f;
    float g_tx = 0, g_ty = 0, g_tz = 0;   // target point (pan)
    bool  g_interacting = false;           // drag in progress -> skip the costly SSAO pass
    int   g_w = 1, g_h = 1;
    bool  g_hasMesh = false;
    float g_alpha = 1.0f;                  // opacity; <1 = semi-transparent X-ray mode
    // Edge (wireframe) overlay policy, orthogonal to g_alpha (faces). Default = draw
    // edges only while translucent, which is the behaviour the product had before the
    // edge system was hoisted into the opaque pass.
    int  g_edgeShow = 1;                   // 0=off, 1=only translucent, 2=always
    bool g_edgeSilhouetteOnly = false;     // keep only part-silhouette outlines
    bool g_edgeDrawBrep = true;            // B-rep part outlines (STEP/IGES/BREP)
    bool g_edgeDrawMesh = true;            // mesh-file lines (STL/OBJ/glTF/3DS ...)

    // HSL -> RGB. Palette scheme ported from the C# software renderer
    // (SoftRenderer::BuildGroupColors) which gave the part-to-part
    // discrimination the user liked (see SetMesh).
    void HslToRgb(float h, float s, float l, float& r, float& g, float& b)
    {
        h = fmodf(h, 360.0f); if (h < 0) h += 360.0f;
        float c = (1.0f - fabsf(2.0f * l - 1.0f)) * s;
        float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
        float mm = l - c / 2.0f;
        float rr = 0, gg = 0, bb = 0;
        if (h < 60)      { rr = c; gg = x; bb = 0; }
        else if (h < 120){ rr = x; gg = c; bb = 0; }
        else if (h < 180){ rr = 0; gg = c; bb = x; }
        else if (h < 240){ rr = 0; gg = x; bb = c; }
        else if (h < 300){ rr = x; gg = 0; bb = c; }
        else             { rr = c; gg = 0; bb = x; }
        r = rr + mm; g = gg + mm; b = bb + mm;
    }

    void RecomputeBoundsAndNormals()
    {
        if (g_pos.empty())
        {
            g_hasBounds = false;
            g_bmin[0] = g_bmin[1] = g_bmin[2] = 0.0f;
            g_bmax[0] = g_bmax[1] = g_bmax[2] = 0.0f;
            return;
        }
        g_vcount = (int)(g_pos.size() / 3);
        float minx = 1e30f, miny = 1e30f, minz = 1e30f;
        float maxx = -1e30f, maxy = -1e30f, maxz = -1e30f;
        for (int i = 0; i < g_vcount; i++)
        {
            float x = g_pos[3 * i], y = g_pos[3 * i + 1], z = g_pos[3 * i + 2];
            minx = std::min(minx, x); maxx = std::max(maxx, x);
            miny = std::min(miny, y); maxy = std::max(maxy, y);
            minz = std::min(minz, z); maxz = std::max(maxz, z);
        }
        g_cx = (minx + maxx) / 2; g_cy = (miny + maxy) / 2; g_cz = (minz + maxz) / 2;
        g_bmin[0] = minx; g_bmin[1] = miny; g_bmin[2] = minz;
        g_bmax[0] = maxx; g_bmax[1] = maxy; g_bmax[2] = maxz;
        g_hasBounds = true;
        float rx = (maxx - minx) / 2, ry = (maxy - miny) / 2, rz = (maxz - minz) / 2;
        g_radius = sqrtf(rx * rx + ry * ry + rz * rz);
        if (g_radius < 1e-3f) g_radius = 1.0f;

        // Smooth vertex normals (Gouraud): accumulate each face normal into its
        // three vertices with area weighting, then normalize per vertex.
        // We accumulate the UN-normalized cross product, whose length is 2*area,
        // so the averaging is naturally area weighted. This removes the faceted
        // banding of per-triangle (flat) normals on dense meshes.
        g_nrm.assign(g_pos.size(), 0.0f);
        size_t ntri = g_idx.size() / 3;
        for (size_t t = 0; t < ntri; t++)
        {
            int i0 = g_idx[3 * t], i1 = g_idx[3 * t + 1], i2 = g_idx[3 * t + 2];
            if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= g_vcount || i1 >= g_vcount || i2 >= g_vcount) continue;
            float ax = g_pos[3 * i0], ay = g_pos[3 * i0 + 1], az = g_pos[3 * i0 + 2];
            float bx = g_pos[3 * i1], by = g_pos[3 * i1 + 1], bz = g_pos[3 * i1 + 2];
            float cx = g_pos[3 * i2], cy = g_pos[3 * i2 + 1], cz = g_pos[3 * i2 + 2];
            float ux = bx - ax, uy = by - ay, uz = bz - az;
            float vx = cx - ax, vy = cy - ay, vz = cz - az;
            float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
            g_nrm[3 * i0] += nx; g_nrm[3 * i0 + 1] += ny; g_nrm[3 * i0 + 2] += nz;
            g_nrm[3 * i1] += nx; g_nrm[3 * i1 + 1] += ny; g_nrm[3 * i1 + 2] += nz;
            g_nrm[3 * i2] += nx; g_nrm[3 * i2 + 1] += ny; g_nrm[3 * i2 + 2] += nz;
        }
        for (int i = 0; i < g_vcount; i++)
        {
            float nx = g_nrm[3 * i], ny = g_nrm[3 * i + 1], nz = g_nrm[3 * i + 2];
            float len = sqrtf(nx * nx + ny * ny + nz * nz);
            if (len > 1e-9f) { g_nrm[3 * i] = nx / len; g_nrm[3 * i + 1] = ny / len; g_nrm[3 * i + 2] = nz / len; }
        }
    }

    // =====================================================================
    // Animating-edge "burr" fix: multisample anti-aliasing (MSAA).
    //
    // Before this change Create() used plain ChoosePixelFormat(), which NEVER
    // returns a multisample format, and nothing ever called glEnable(GL_MULTISAMPLE)
    // / GL_POLYGON_SMOOTH / GL_LINE_SMOOTH. Every triangle edge was therefore
    // rasterised as a hard staircase. While the model is still nobody notices,
    // but as soon as the mouse orbits/pans/zooms the staircase crawls along the
    // silhouette and part boundaries, which reads as "burrs" or "sparkle" on the
    // corners. This was NOT an over-aggressive performance optimum: Render()
    // has no LOD / decimation / half-resolution branch at all (the CPU software
    // renderer in MeshPreviewProvider does halve the resolution while dragging,
    // but STEP/IGES/BREP are routed to this GPU viewer instead).
    //
    // Windows' <gl/gl.h> only declares OpenGL 1.1, so the multisample tokens and
    // the ARB pixel-format selector are declared locally, exactly like the
    // GL_SRC_ALPHA workaround used for the transparency mode below.
    //
    // wglChoosePixelFormatARB can only be resolved from a CURRENT context, and a
    // window's pixel format must be chosen before its single SetPixelFormat call.
    // So a throwaway 1x1 window + legacy context is created first just to fetch
    // the entry point, then destroyed.
    // =====================================================================
#define OGL_MULTISAMPLE        0x809D
#define OGL_SAMPLE_BUFFERS     0x80A8
#define OGL_SAMPLES            0x80A9

#define WGL_DRAW_TO_WINDOW_ARB 0x2001
#define WGL_SUPPORT_OPENGL_ARB 0x2010
#define WGL_DOUBLE_BUFFER_ARB  0x2011
#define WGL_PIXEL_TYPE_ARB     0x2013
#define WGL_COLOR_BITS_ARB     0x2014
#define WGL_DEPTH_BITS_ARB     0x2022
#define WGL_STENCIL_BITS_ARB   0x2023
#define WGL_TYPE_RGBA_ARB      0x202B
#define WGL_SAMPLE_BUFFERS_ARB 0x2041
#define WGL_SAMPLES_ARB        0x2042

    typedef BOOL (WINAPI * PFN_WGLCHOOSEPIXELFORMATARB)(HDC, const int*, const FLOAT*, UINT, int*, UINT*);

    PFN_WGLCHOOSEPIXELFORMATARB g_wglChoosePixelFormatARB = NULL;
    int g_msaaSamples = 0;   // actual sample count reported by the driver; 0 = no MSAA

    // Resolve wglChoosePixelFormatARB via a temporary window/context.
    // Returns false when the driver lacks WGL_ARB_pixel_format (old or virtual
    // GPUs); the caller then falls back to the classic ChoosePixelFormat path.
    bool LoadWglArbEntryPoints()
    {
        if (g_wglChoosePixelFormatARB) return true;

        WNDCLASSA wc = { 0 };
        wc.style = CS_OWNDC;
        wc.lpfnWndProc = ::DefWindowProcA;
        wc.hInstance = ::GetModuleHandleA(NULL);
        wc.lpszClassName = "OcctViewerProbeGL";
        ::RegisterClassA(&wc);   // returns 0 if the class already exists - fine

        HWND probe = ::CreateWindowExA(0, wc.lpszClassName, "",
            WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
            0, 0, 1, 1, NULL, NULL, wc.hInstance, NULL);
        if (!probe) return false;

        HDC probeDc = ::GetDC(probe);
        if (!probeDc) { ::DestroyWindow(probe); return false; }

        PIXELFORMATDESCRIPTOR pfd = { 0 };
        pfd.nSize = sizeof(pfd);
        pfd.nVersion = 1;
        pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        pfd.iPixelType = PFD_TYPE_RGBA;
        pfd.cColorBits = 32;
        pfd.cDepthBits = 24;
        pfd.iLayerType = PFD_MAIN_PLANE;

        int pf = ::ChoosePixelFormat(probeDc, &pfd);
        if (pf == 0 || !::SetPixelFormat(probeDc, pf, &pfd))
        {
            ::ReleaseDC(probe, probeDc);
            ::DestroyWindow(probe);
            return false;
        }

        HGLRC probeRc = ::wglCreateContext(probeDc);
        if (!probeRc || !::wglMakeCurrent(probeDc, probeRc))
        {
            if (probeRc) ::wglDeleteContext(probeRc);
            ::ReleaseDC(probe, probeDc);
            ::DestroyWindow(probe);
            return false;
        }

        g_wglChoosePixelFormatARB = (PFN_WGLCHOOSEPIXELFORMATARB)
            ::wglGetProcAddress("wglChoosePixelFormatARB");

        ::wglMakeCurrent(NULL, NULL);
        ::wglDeleteContext(probeRc);
        ::ReleaseDC(probe, probeDc);
        ::DestroyWindow(probe);
        return g_wglChoosePixelFormatARB != NULL;
    }

    // Ask the driver for a multisample format: 4x first (best quality/perf
    // balance on big assemblies), 2x as the fallback. Returns 0 when the driver
    // offers no multisample format at all. WGL_SAMPLES_ARB is a MINIMUM, so a
    // driver that only supports 2x simply returns no match for the 4x request.
    int ChooseMultiSampleFormat(HDC hdc)
    {
        if (!g_wglChoosePixelFormatARB) return 0;
        const int want[2] = { 4, 2 };
        for (int k = 0; k < 2; k++)
        {
            int attribs[] = {
                WGL_DRAW_TO_WINDOW_ARB, GL_TRUE,
                WGL_SUPPORT_OPENGL_ARB, GL_TRUE,
                WGL_DOUBLE_BUFFER_ARB,  GL_TRUE,
                WGL_PIXEL_TYPE_ARB,     WGL_TYPE_RGBA_ARB,
                WGL_COLOR_BITS_ARB,     32,
                WGL_DEPTH_BITS_ARB,     24,
                WGL_STENCIL_BITS_ARB,   8,
                WGL_SAMPLE_BUFFERS_ARB, 1,
                WGL_SAMPLES_ARB,        want[k],
                0
            };
            int fmt = 0;
            UINT n = 0;
            if (g_wglChoosePixelFormatARB(hdc, attribs, NULL, 1, &fmt, &n) && n > 0 && fmt != 0)
                return fmt;
        }
        return 0;
    }

    // =====================================================================
    // Screen-space ambient occlusion (SSAO), 2026-10-18.
    //
    // Fixed-function OpenGL cannot express AO or shadows, so this adds a small
    // modern-GL side channel instead of rewriting the renderer:
    //   1. the scene is still drawn by the EXISTING fixed pipeline (3-point rig,
    //      per-vertex colours) but into a 4x multisampled FBO - so the MSAA that
    //      was just fixed is preserved;
    //   2. the MSAA buffer is blitted (resolved) into a colour texture + depth texture;
    //   3. an SSAO pass turns the depth texture into a per-pixel occlusion factor;
    //   4. a full-screen composite multiplies colour by that factor -> window.
    // If ANY step is unavailable (old driver, shader compile failure, FBO not
    // complete) g_aoReady stays false and Render() falls back to the original
    // direct-to-window path, so nothing regresses on weaker machines.
    // =====================================================================
#define OGL_FRAMEBUFFER            0x8D40
#define OGL_READ_FRAMEBUFFER       0x8CA8
#define OGL_DRAW_FRAMEBUFFER       0x8CA9
#define OGL_RENDERBUFFER           0x8D41
#define OGL_COLOR_ATTACHMENT0      0x8CE0
#define OGL_DEPTH_ATTACHMENT       0x8D00
#define OGL_FRAMEBUFFER_COMPLETE   0x8CD5
#define OGL_FRAGMENT_SHADER        0x8B30
#define OGL_VERTEX_SHADER          0x8B31
#define OGL_COMPILE_STATUS         0x8B81
#define OGL_LINK_STATUS            0x8B82
#define OGL_TEXTURE0               0x84C0
#define OGL_TEXTURE1               0x84C1
#define OGL_TEXTURE2               0x84C2
#define OGL_DEPTH_COMPONENT24      0x81A6
#define OGL_RGBA8                  0x8058
#define OGL_CLAMP_TO_EDGE          0x812F

#define GLDECL(ret, name, params) typedef ret (APIENTRY *PFN_##name) params; PFN_##name pgl##name = NULL;

    GLDECL(GLuint, CreateShader, (GLenum))
    GLDECL(void,   ShaderSource, (GLuint, GLsizei, const char* const*, const GLint*))
    GLDECL(void,   CompileShader, (GLuint))
    GLDECL(void,   GetShaderiv, (GLuint, GLenum, GLint*))
    GLDECL(void,   GetShaderInfoLog, (GLuint, GLsizei, GLsizei*, char*))
    GLDECL(void,   DeleteShader, (GLuint))
    GLDECL(void,   DeleteProgram, (GLuint))
    GLDECL(GLuint, CreateProgram, (void))
    GLDECL(void,   AttachShader, (GLuint, GLuint))
    GLDECL(void,   LinkProgram, (GLuint))
    GLDECL(void,   GetProgramiv, (GLuint, GLenum, GLint*))
    GLDECL(void,   GetProgramInfoLog, (GLuint, GLsizei, GLsizei*, char*))
    GLDECL(void,   UseProgram, (GLuint))
    GLDECL(GLint,  GetUniformLocation, (GLuint, const char*))
    GLDECL(void,   Uniform1i, (GLint, GLint))
    GLDECL(void,   Uniform1f, (GLint, GLfloat))
    GLDECL(void,   Uniform2f, (GLint, GLfloat, GLfloat))
    GLDECL(void,   GenFramebuffers, (GLsizei, GLuint*))
    GLDECL(void,   BindFramebuffer, (GLenum, GLuint))
    GLDECL(void,   FramebufferTexture2D, (GLenum, GLenum, GLenum, GLuint, GLint))
    GLDECL(void,   FramebufferRenderbuffer, (GLenum, GLenum, GLenum, GLuint))
    GLDECL(GLenum, CheckFramebufferStatus, (GLenum))
    GLDECL(void,   DeleteFramebuffers, (GLsizei, const GLuint*))
    GLDECL(void,   GenRenderbuffers, (GLsizei, GLuint*))
    GLDECL(void,   BindRenderbuffer, (GLenum, GLuint))
    GLDECL(void,   RenderbufferStorageMultisample, (GLenum, GLsizei, GLenum, GLsizei, GLsizei))
    GLDECL(void,   DeleteRenderbuffers, (GLsizei, const GLuint*))
    GLDECL(void,   BlitFramebuffer, (GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum))
    GLDECL(void,   ActiveTexture, (GLenum))

    bool  g_aoReady = false;      // AO chain fully initialised
    bool  g_aoTried = false;      // don't retry every frame after a failure
    bool  g_aoRequested = true;   // user intent (survives Destroy so a host can preset it)
    int   g_aoW = 0, g_aoH = 0;
    GLuint g_msaaFbo = 0, g_msaaColorRb = 0, g_msaaDepthRb = 0;
    GLuint g_resolveFbo = 0, g_colorTex = 0, g_depthTex = 0;
    GLuint g_aoFbo = 0, g_aoTex = 0;
    GLuint g_aoProg = 0, g_compProg = 0;
    GLint  g_uDepth = -1, g_uResAo = -1, g_uNear = -1, g_uFar = -1;
    GLint  g_uTanY = -1, g_uAspect = -1;
    GLint  g_uRadiusCap = -1, g_uRadiusPx = -1, g_uBiasK = -1, g_uStrength = -1;
    GLint  g_uColor = -1, g_uAo = -1, g_uResComp = -1, g_uDepthComp = -1;
    GLint  g_uNearComp = -1, g_uFarComp = -1, g_uTanYComp = -1;
    GLint  g_uRadiusCapComp = -1, g_uRadiusPxComp = -1, g_uAoMix = -1;

    struct AoParams { float radiusCap, radiusPx, biasK, strength, mixAmt; };
    AoParams g_aoParams;
    bool     g_aoParamsLoaded = false;

    // ---- retained presentation ------------------------------------------------
    // The scene is rendered into g_presentFbo and then blitted to the window as one
    // textured quad. The texture keeps the last COMPLETE frame, so a repaint can simply
    // re-blit it.
    //
    // Why this exists: WinForms repaints the child window for reasons we do not control
    // (activation changes, DevExpress skin repaints, another window passing over it,
    // parent layout churn). Until now EVERY such repaint became a full re-render inside
    // WM_PAINT - 9 ms for a light model, 45-90 ms for the 679k-triangle assembly - and the
    // repaint had to start by clearing a buffer. Any hiccup in that window is visible as
    // the "occasional flash". A re-blit of a finished frame has no such window and costs
    // ~0.3 ms regardless of triangle count.
    GLuint g_presentFbo = 0, g_presentTex = 0;
    int    g_presentW = 0, g_presentH = 0;
    bool   g_frameValid = false;      // g_presentTex holds a complete frame
    bool   g_sceneDirty = true;       // something changed: next Render must redraw
    bool   g_retainEnabled = true;    // PLATFORM_CAD_NO_RETAIN=1 disables (A/B measurement)
    int    g_fullRenders = 0;         // diagnostics: how often the scene was actually redrawn
    int    g_retainedPresents = 0;    // diagnostics: how often a repaint reused the frame

    // ---- AO reach model ------------------------------------------------------------
    // The reach used to be a flat 10% of the bounding-sphere radius. Because Fit() also
    // sets the camera distance from that same radius, the reach always worked out to
    // roughly 42 SCREEN PIXELS regardless of the model - and on a dense assembly (hundreds
    // of plates a few mm apart) a 42 px reach means every plate occludes its neighbour, so
    // the whole machine is darkened and reads as a grey haze.
    //
    // The reach is now a fixed PIXEL budget evaluated at each fragment's own depth, and
    // clamped by a fraction of the bounding radius so that a close-up cannot blow it up to
    // metres. Both are overridable for tuning without a rebuild:
    //   PLATFORM_CAD_AO_RADIUS   reach cap, fraction of bounding half-diagonal (default .025)
    //   PLATFORM_CAD_AO_PX       pixel budget of the reach        (default 14)
    //   PLATFORM_CAD_AO_BIAS     depth bias, fraction of reach     (default .06)
    //   PLATFORM_CAD_AO_STRENGTH occlusion strength                (default 1.5)
    //   PLATFORM_CAD_AO_MIX      max darkening at full occlusion   (default .85)
    const char* kAoVertSrc =
        "#version 120\n"
        "void main() { gl_Position = gl_Vertex; }\n";

    // SSAO: rebuild view-space position from the depth buffer, take the normal from the
    // depth gradient (no normal pass / MRT needed), then test 16 hemisphere samples.
    const char* kAoFragSrc =
        "#version 120\n"
        "uniform sampler2D uDepth;\n"
        "uniform vec2  uRes;\n"
        "uniform float uNear, uFar, uTanY, uAspect;\n"
        "uniform float uRadiusCap, uRadiusPx, uBiasK, uStrength;\n"
        "float viewDist(vec2 uv) {\n"
        "    float d = texture2D(uDepth, uv).r;\n"
        "    return 2.0*uNear*uFar / (uFar+uNear - (d*2.0-1.0)*(uFar-uNear));\n"
        "}\n"
        "vec3 viewPos(vec2 uv) {\n"
        "    vec2 ndc = uv*2.0-1.0;\n"
        "    float z = viewDist(uv);\n"
        "    return vec3(ndc.x*z*uTanY*uAspect, ndc.y*z*uTanY, -z);\n"
        "}\n"
        "float aoRadius(float zp) {\n"
        "    float px = zp * uTanY * 2.0 / uRes.y * uRadiusPx;\n"
        "    return min(px, uRadiusCap);\n"
        "}\n"
        "vec3 hemisphereSample(int i) {\n"
        "    float fi = float(i) + 0.5;\n"
        "    float phi = fi * 2.39996323;\n"
        "    float cosT = sqrt(1.0 - fi/16.0);\n"
        "    float sinT = sqrt(max(0.0, 1.0 - cosT*cosT));\n"
        "    return vec3(cos(phi)*sinT, sin(phi)*sinT, cosT);\n"
        "}\n"
        "void main() {\n"
        "    vec2 uv = gl_FragCoord.xy / uRes;\n"
        "    float d = texture2D(uDepth, uv).r;\n"
        "    if (d >= 0.99999) { gl_FragColor = vec4(1.0); return; }\n"
        "    vec3 p  = viewPos(uv);\n"
        "    float zp = -p.z;\n"
        "    vec3 px = viewPos(uv + vec2(1.0/uRes.x, 0.0));\n"
        "    vec3 py = viewPos(uv + vec2(0.0, 1.0/uRes.y));\n"
        "    vec3 n  = normalize(cross(px - p, py - p));\n"
        "    if (dot(n, -p) < 0.0) n = -n;\n"
        "    vec3 up = abs(n.z) < 0.99 ? vec3(0.0,0.0,1.0) : vec3(1.0,0.0,0.0);\n"
        "    vec3 t  = normalize(cross(up, n));\n"
        "    vec3 b  = cross(n, t);\n"
        "    float radius = aoRadius(zp);\n"
        "    float bias = radius * uBiasK;\n"
        "    float rnd = fract(sin(dot(uv, vec2(12.9898,78.233))) * 43758.5453);\n"
        "    float ca = cos(rnd*6.2831853);\n"
        "    float sa = sin(rnd*6.2831853);\n"
        "    float occ = 0.0;\n"
        "    for (int i = 0; i < 16; i++) {\n"
        "        vec3 s = hemisphereSample(i);\n"
        "        s.xy = mat2(ca, -sa, sa, ca) * s.xy;\n"
        "        vec3 dir = t*s.x + b*s.y + n*s.z;\n"
        "        float t01 = (float(i) + 0.5) / 16.0;\n"
        "        float sc = radius * (0.25 + 0.75 * sqrt(t01));\n"
        "        vec3 sp = p + dir * sc;\n"
        "        float sz = -sp.z;\n"
        "        if (sz <= uNear) continue;\n"
        "        vec2 su = vec2(sp.x/(sz*uTanY*uAspect), sp.y/(sz*uTanY)) * 0.5 + 0.5;\n"
        "        if (su.x < 0.0 || su.x > 1.0 || su.y < 0.0 || su.y > 1.0) continue;\n"
        "        float nd = viewDist(su);\n"
        "        if (nd < sz - bias) {\n"
        "            occ += smoothstep(0.0, 1.0, radius / abs(zp - nd));\n"
        "        }\n"
        "    }\n"
        "    float ao = clamp(1.0 - (occ/16.0) * uStrength, 0.0, 1.0);\n"
        "    gl_FragColor = vec4(ao, ao, ao, 1.0);\n"
        "}\n";

    // Composite: multiply colour by AO, but average the AO over a 5x5 neighbourhood first.
    // The SSAO kernel is random-rotated per pixel, which is what keeps it from banding but
    // also leaves per-pixel noise ("grain"). The weights must reject anything further away
    // than the AO reach itself - the old tolerance was zc*0.05, i.e. 23 cm at a 4.7 m eye
    // distance, which happily blended occluders from plate faces 5 cm apart and produced
    // the hazy veil. Depth tolerance now follows the reach.
    const char* kCompFragSrc =
        "#version 120\n"
        "uniform sampler2D uColor;\n"
        "uniform sampler2D uAo;\n"
        "uniform sampler2D uDepth;\n"
        "uniform vec2 uRes;\n"
        "uniform float uNear, uFar, uTanY;\n"
        "uniform float uRadiusCap, uRadiusPx, uAoMix;\n"
        "float lin(float d) {\n"
        "    return 2.0*uNear*uFar / (uFar+uNear - (d*2.0-1.0)*(uFar-uNear));\n"
        "}\n"
        "float aoRadius(float zp) {\n"
        "    float px = zp * uTanY * 2.0 / uRes.y * uRadiusPx;\n"
        "    return min(px, uRadiusCap);\n"
        "}\n"
        "void main() {\n"
        "    vec2 uv = gl_FragCoord.xy / uRes;\n"
        "    vec3 c = texture2D(uColor, uv).rgb;\n"
        "    float dc = texture2D(uDepth, uv).r;\n"
        "    if (dc >= 0.99999) { gl_FragColor = vec4(c, 1.0); return; }\n"
        "    float zc = lin(dc);\n"
        "    float reach = max(aoRadius(zc) * 0.35, 1e-6);\n"
        "    float sum = 0.0;\n"
        "    float wsum = 0.0;\n"
        "    for (int y = -2; y <= 2; y++) {\n"
        "        for (int x = -2; x <= 2; x++) {\n"
        "            vec2 su = clamp(uv + vec2(float(x), float(y)) / uRes, vec2(0.0), vec2(1.0));\n"
        "            float q = (lin(texture2D(uDepth, su).r) - zc) / reach;\n"
        "            float w = 1.0 / (1.0 + q*q);\n"
        "            sum += texture2D(uAo, su).r * w;\n"
        "            wsum += w;\n"
        "        }\n"
        "    }\n"
        "    float ao = sum / max(wsum, 0.00001);\n"
        "    gl_FragColor = vec4(c * mix(1.0, ao, uAoMix), 1.0);\n"
        "}\n";

    // Resolve the modern-GL entry points. All of them live past OpenGL 1.1, so they can
    // only be reached through wglGetProcAddress from a current context.
    bool LoadModernGl()
    {
        if (pglCreateShader) return true;
#define GLLOAD(name) pgl##name = (PFN_##name)::wglGetProcAddress("gl" #name); if (!pgl##name) return false;
        GLLOAD(CreateShader)
        GLLOAD(ShaderSource)
        GLLOAD(CompileShader)
        GLLOAD(GetShaderiv)
        GLLOAD(GetShaderInfoLog)
        GLLOAD(DeleteShader)
        GLLOAD(DeleteProgram)
        GLLOAD(CreateProgram)
        GLLOAD(AttachShader)
        GLLOAD(LinkProgram)
        GLLOAD(GetProgramiv)
        GLLOAD(GetProgramInfoLog)
        GLLOAD(UseProgram)
        GLLOAD(GetUniformLocation)
        GLLOAD(Uniform1i)
        GLLOAD(Uniform1f)
        GLLOAD(Uniform2f)
        GLLOAD(GenFramebuffers)
        GLLOAD(BindFramebuffer)
        GLLOAD(FramebufferTexture2D)
        GLLOAD(FramebufferRenderbuffer)
        GLLOAD(CheckFramebufferStatus)
        GLLOAD(DeleteFramebuffers)
        GLLOAD(GenRenderbuffers)
        GLLOAD(BindRenderbuffer)
        GLLOAD(RenderbufferStorageMultisample)
        GLLOAD(DeleteRenderbuffers)
        GLLOAD(BlitFramebuffer)
        GLLOAD(ActiveTexture)
#undef GLLOAD
        return true;
    }

    GLuint CompileShaderStage(GLenum type, const char* src)
    {
        GLuint s = pglCreateShader(type);
        if (!s) return 0;
        pglShaderSource(s, 1, &src, NULL);
        pglCompileShader(s);
        GLint ok = 0;
        pglGetShaderiv(s, OGL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            char log[2048];
            GLsizei n = 0;
            log[0] = 0;
            pglGetShaderInfoLog(s, sizeof(log) - 1, &n, log);
            ::OutputDebugStringA("[OCCTWrapper] shader compile failed: ");
            ::OutputDebugStringA(log);
            ::OutputDebugStringA("\n");
            pglDeleteShader(s);
            return 0;
        }
        return s;
    }

    GLuint BuildProgram(const char* fragSrc)
    {
        GLuint vs = CompileShaderStage(OGL_VERTEX_SHADER, kAoVertSrc);
        GLuint fs = CompileShaderStage(OGL_FRAGMENT_SHADER, fragSrc);
        if (!vs || !fs) { if (vs) pglDeleteShader(vs); if (fs) pglDeleteShader(fs); return 0; }
        GLuint p = pglCreateProgram();
        pglAttachShader(p, vs);
        pglAttachShader(p, fs);
        pglLinkProgram(p);
        GLint ok = 0;
        pglGetProgramiv(p, OGL_LINK_STATUS, &ok);
        if (!ok)
        {
            char log[2048];
            GLsizei n = 0;
            log[0] = 0;
            pglGetProgramInfoLog(p, sizeof(log) - 1, &n, log);
            ::OutputDebugStringA("[OCCTWrapper] program link failed: ");
            ::OutputDebugStringA(log);
            ::OutputDebugStringA("\n");
            p = 0;
        }
        pglDeleteShader(vs);
        pglDeleteShader(fs);
        return p;
    }

    void DestroyAoResources()
    {
        if (g_msaaFbo) { pglDeleteFramebuffers(1, &g_msaaFbo); g_msaaFbo = 0; }
        if (g_resolveFbo) { pglDeleteFramebuffers(1, &g_resolveFbo); g_resolveFbo = 0; }
        if (g_aoFbo) { pglDeleteFramebuffers(1, &g_aoFbo); g_aoFbo = 0; }
        if (g_presentFbo) { pglDeleteFramebuffers(1, &g_presentFbo); g_presentFbo = 0; }
        if (g_msaaColorRb) { pglDeleteRenderbuffers(1, &g_msaaColorRb); g_msaaColorRb = 0; }
        if (g_msaaDepthRb) { pglDeleteRenderbuffers(1, &g_msaaDepthRb); g_msaaDepthRb = 0; }
        if (g_colorTex) { glDeleteTextures(1, &g_colorTex); g_colorTex = 0; }
        if (g_depthTex) { glDeleteTextures(1, &g_depthTex); g_depthTex = 0; }
        if (g_aoTex) { glDeleteTextures(1, &g_aoTex); g_aoTex = 0; }
        if (g_presentTex) { glDeleteTextures(1, &g_presentTex); g_presentTex = 0; }
        g_presentW = g_presentH = 0;
        g_frameValid = false;      // the retained frame died with its texture
        g_sceneDirty = true;
        g_aoW = g_aoH = 0;
        // MUST clear this too. EnsureAoResources() short-circuits on
        // "g_aoReady && size matches"; if a rebuild fails halfway (FBO incomplete,
        // driver OOM) the ids above are already zero but this flag would still say
        // "ready", so Render() would bind framebuffer 0 as if it were the MSAA target
        // and present a garbage frame - a real source of the occasional flash.
        g_aoReady = false;
    }

    bool MakeTexture(GLuint& tex, int w, int h, GLenum internalFmt, GLenum fmt, GLenum type)
    {
        if (!tex) glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, OGL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, OGL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, (GLint)internalFmt, w, h, 0, fmt, type, NULL);
        glBindTexture(GL_TEXTURE_2D, 0);
        return true;
    }

    // Read the tuning knobs once. A zero / negative / unparsable value keeps the default,
    // so a typo in the environment can never disable AO by accident.
    float EnvFloat(const char* name, float def)
    {
        char buf[64];
        DWORD n = ::GetEnvironmentVariableA(name, buf, (DWORD)sizeof(buf));
        if (n == 0 || n >= (DWORD)sizeof(buf)) return def;
        double v = atof(buf);
        if (!(v > 0.0)) return def;
        return (float)v;
    }

    void LoadAoParams()
    {
        if (g_aoParamsLoaded) return;
        g_aoParamsLoaded = true;
        g_aoParams.radiusCap = EnvFloat("PLATFORM_CAD_AO_RADIUS", 0.025f);
        g_aoParams.radiusPx = EnvFloat("PLATFORM_CAD_AO_PX", 14.0f);
        g_aoParams.biasK = EnvFloat("PLATFORM_CAD_AO_BIAS", 0.06f);
        g_aoParams.strength = EnvFloat("PLATFORM_CAD_AO_STRENGTH", 1.5f);
        g_aoParams.mixAmt = EnvFloat("PLATFORM_CAD_AO_MIX", 0.85f);
    }

    // Build (or rebuild after a resize) the whole AO chain. Returns false if unsupported.
    bool EnsureAoResources(int w, int h)
    {
        if (g_aoReady && g_aoW == w && g_aoH == h) return true;
        if (g_aoTried && !g_aoReady) return false;
        // Escape hatch for support: set PLATFORM_CAD_NO_AO=1 to fall back to the plain
        // fixed-function path without rebuilding (e.g. on a very slow/virtual GPU).
        if (::GetEnvironmentVariableA("PLATFORM_CAD_NO_AO", NULL, 0) > 0) { g_aoTried = true; return false; }
        // Same idea for the retained-frame presentation (A/B measurement of repaint cost).
        if (::GetEnvironmentVariableA("PLATFORM_CAD_NO_RETAIN", NULL, 0) > 0) g_retainEnabled = false;
        // A 0-sized / tiny first frame must not permanently disable AO, so this check
        // comes BEFORE the "already tried" latch.
        if (w < 16 || h < 16) return false;
        g_aoTried = true;
        if (!LoadModernGl()) return false;

        if (!g_aoProg)
        {
            g_aoProg = BuildProgram(kAoFragSrc);
            g_compProg = BuildProgram(kCompFragSrc);
            if (!g_aoProg || !g_compProg) return false;

            g_uDepth = pglGetUniformLocation(g_aoProg, "uDepth");
            g_uResAo = pglGetUniformLocation(g_aoProg, "uRes");
            g_uNear = pglGetUniformLocation(g_aoProg, "uNear");
            g_uFar = pglGetUniformLocation(g_aoProg, "uFar");
            g_uTanY = pglGetUniformLocation(g_aoProg, "uTanY");
            g_uAspect = pglGetUniformLocation(g_aoProg, "uAspect");
            g_uRadiusCap = pglGetUniformLocation(g_aoProg, "uRadiusCap");
            g_uRadiusPx = pglGetUniformLocation(g_aoProg, "uRadiusPx");
            g_uBiasK = pglGetUniformLocation(g_aoProg, "uBiasK");
            g_uStrength = pglGetUniformLocation(g_aoProg, "uStrength");

            g_uColor = pglGetUniformLocation(g_compProg, "uColor");
            g_uAo = pglGetUniformLocation(g_compProg, "uAo");
            g_uResComp = pglGetUniformLocation(g_compProg, "uRes");
            g_uDepthComp = pglGetUniformLocation(g_compProg, "uDepth");
            g_uNearComp = pglGetUniformLocation(g_compProg, "uNear");
            g_uFarComp = pglGetUniformLocation(g_compProg, "uFar");
            g_uTanYComp = pglGetUniformLocation(g_compProg, "uTanY");
            g_uRadiusCapComp = pglGetUniformLocation(g_compProg, "uRadiusCap");
            g_uRadiusPxComp = pglGetUniformLocation(g_compProg, "uRadiusPx");
            g_uAoMix = pglGetUniformLocation(g_compProg, "uAoMix");
        }

        LoadAoParams();
        if (!g_aoProg || !g_compProg) return false;

        DestroyAoResources();   // sizes changed: rebuild attachments
        g_aoW = w; g_aoH = h;

        // (1) 4x MSAA target so the anti-aliasing is preserved
        pglGenFramebuffers(1, &g_msaaFbo);
        pglBindFramebuffer(OGL_FRAMEBUFFER, g_msaaFbo);
        pglGenRenderbuffers(1, &g_msaaColorRb);
        pglBindRenderbuffer(OGL_RENDERBUFFER, g_msaaColorRb);
        pglRenderbufferStorageMultisample(OGL_RENDERBUFFER, 4, OGL_RGBA8, w, h);
        pglFramebufferRenderbuffer(OGL_FRAMEBUFFER, OGL_COLOR_ATTACHMENT0, OGL_RENDERBUFFER, g_msaaColorRb);
        pglGenRenderbuffers(1, &g_msaaDepthRb);
        pglBindRenderbuffer(OGL_RENDERBUFFER, g_msaaDepthRb);
        pglRenderbufferStorageMultisample(OGL_RENDERBUFFER, 4, OGL_DEPTH_COMPONENT24, w, h);
        pglFramebufferRenderbuffer(OGL_FRAMEBUFFER, OGL_DEPTH_ATTACHMENT, OGL_RENDERBUFFER, g_msaaDepthRb);
        pglBindRenderbuffer(OGL_RENDERBUFFER, 0);
        if (pglCheckFramebufferStatus(OGL_FRAMEBUFFER) != OGL_FRAMEBUFFER_COMPLETE)
        {
            pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
            DestroyAoResources();
            return false;
        }

        // (2) resolved colour + depth textures that the AO pass can sample
        MakeTexture(g_colorTex, w, h, OGL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
        MakeTexture(g_depthTex, w, h, OGL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT);
        pglGenFramebuffers(1, &g_resolveFbo);
        pglBindFramebuffer(OGL_FRAMEBUFFER, g_resolveFbo);
        pglFramebufferTexture2D(OGL_FRAMEBUFFER, OGL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_colorTex, 0);
        pglFramebufferTexture2D(OGL_FRAMEBUFFER, OGL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_depthTex, 0);
        if (pglCheckFramebufferStatus(OGL_FRAMEBUFFER) != OGL_FRAMEBUFFER_COMPLETE)
        {
            pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
            DestroyAoResources();
            return false;
        }

        // (3) AO target. Full resolution on purpose: half res would need an extra
        // bilateral upsample to avoid halos around thin parts.
        MakeTexture(g_aoTex, w, h, OGL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
        pglGenFramebuffers(1, &g_aoFbo);
        pglBindFramebuffer(OGL_FRAMEBUFFER, g_aoFbo);
        pglFramebufferTexture2D(OGL_FRAMEBUFFER, OGL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_aoTex, 0);
        if (pglCheckFramebufferStatus(OGL_FRAMEBUFFER) != OGL_FRAMEBUFFER_COMPLETE)
        {
            pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
            DestroyAoResources();
            return false;
        }

        // (4) presentation target: the final image lands here and is then blitted to the
        // window. Same size as the window, RGBA8, no depth (the scene never draws into it
        // directly - only the composite / MSAA blit writes here).
        MakeTexture(g_presentTex, w, h, OGL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
        pglGenFramebuffers(1, &g_presentFbo);
        pglBindFramebuffer(OGL_FRAMEBUFFER, g_presentFbo);
        pglFramebufferTexture2D(OGL_FRAMEBUFFER, OGL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_presentTex, 0);
        if (pglCheckFramebufferStatus(OGL_FRAMEBUFFER) != OGL_FRAMEBUFFER_COMPLETE)
        {
            pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
            DestroyAoResources();
            return false;
        }
        g_presentW = w; g_presentH = h;
        g_frameValid = false;

        pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
        g_aoReady = true;
        return true;
    }

    // Full-screen quad in clip space. Uses immediate mode, which is legal inside a
    // compatibility-profile context even with a shader program bound; the fragment
    // shader derives UVs from gl_FragCoord so no texture coordinates are needed.
    void DrawFullscreenQuad()
    {
        glBegin(GL_QUADS);
        glVertex2f(-1.0f, -1.0f);
        glVertex2f(1.0f, -1.0f);
        glVertex2f(1.0f, 1.0f);
        glVertex2f(-1.0f, 1.0f);
        glEnd();
    }

    // ---------------------------------------------------------------------
    // Glass shading, take 2 (2026-10-18).
    //
    // Take 1 painted ONE alpha over the whole mesh: outer skin, inner walls and every
    // part seen through the model all contributed the same weight. A view ray through
    // this assembly crosses 6..20 layers, and at the 0.62 step six layers of 0.62 already
    // compose to 99.8% opacity - the machine collapsed into a flat milky slab. That is
    // exactly the reported "cannot see clearly, no sense of depth".
    //
    // Real glass reads differently, and the difference is not the alpha but the split
    // between what is in FRONT and what is BEHIND:
    //   - the skin is a thin, nearly clear membrane that turns opaque only at the
    //     silhouette (grazing angle);
    //   - the body behind it is darker and more saturated, so anything seen through the
    //     skin reads as "behind";
    //   - so the per-layer opacity can stay LOW and let the depth ordering of the two
    //     passes build the overall opacity up, instead of one flat veil.
    //
    // Hence two colour sets sharing the same base palette:
    //   g_colFront - outer membrane : base colour, low alpha, brightened + opaque at
    //                                 grazing angles. Drawn LAST.
    //   g_colBack  - inner walls    : base x 0.84 (darker), higher alpha. Drawn FIRST.
    // The shell/wall contrast also outlines every part: at a part's silhouette the shell
    // goes solid while its interior stays clear over a dark wall, which is what makes the
    // individual parts separate from each other without any edge geometry.
    // Edge drawing, third attempt. The two previous ones tried to make EVERY crease readable
    // and then to cap their number; neither works, because a machined profile genuinely owns
    // dozens of long creases and any density limit eats the ones that matter.
    //
    // What the eye needs is not more edges but the RIGHT ones. Each segment arrives with the
    // normals of the two faces it separates (see OcctBrepReader::CollectEdges), so at draw
    // time we can ask the question a silhouette pass would ask:
    //
    //   do the two sides face the eye OPPOSITE ways?  -> this edge is an OUTLINE of the part
    //   do both sides face the eye the same way?       -> interior detail (groove mouth, rib)
    //
    // Outlines are drawn strong, interior detail is kept as a barely-there hint. The parts
    // therefore read as solid objects sliding past each other instead of as a haystack of
    // equal-weight lines, and the density problem disappears without deleting any data.
    void UpdateEdgeColors(float ex, float ey, float ez)
    {
        const size_t nseg = g_edgePos.size() / 6;
        if (nseg == 0) return;
        if (g_edgeCol.size() != nseg * 8) g_edgeCol.assign(nseg * 8, 1.0f);
        if (g_edgeNrm.size() != nseg * 6) g_edgeNrm.assign(nseg * 6, 0.0f);

        // View direction (eye -> target). A face normal pointing back at the eye has a
        // negative dot with it.
        float dx = g_tx - ex, dy = g_ty - ey, dz = g_tz - ez;
        float dl = sqrtf(dx * dx + dy * dy + dz * dz);
        if (dl > 1e-6f) { dx /= dl; dy /= dl; dz /= dl; }
        else { dx = 0.0f; dy = 0.0f; dz = -1.0f; }

        const float tanY = 0.41421356f;                       // tan(45/2 deg), see Render()
        const float viewH = (float)(g_h > 0 ? g_h : 1);
        const float pxPerWorldOverZ = viewH / (2.0f * tanY);  // len/z -> pixels
        const float R = (g_radius > 1e-6f) ? g_radius : 1.0f;

        for (size_t s = 0; s < nseg; s++)
        {
            const float* p = &g_edgePos[s * 6];
            const float* n = &g_edgeNrm[s * 6];
            const float mxw = (p[0] + p[3]) * 0.5f;
            const float myw = (p[1] + p[4]) * 0.5f;
            const float mzw = (p[2] + p[5]) * 0.5f;

            float a = 0.0f;

            // Sub-pixel segments are noise at any zoom level; skip them outright rather
            // than drawing a smeared pixel of grey.
            const float rx = mxw - ex, ry = myw - ey, rz = mzw - ez;
            const float z = rx * dx + ry * dy + rz * dz;
            const float ex2 = p[3] - p[0], ey2 = p[4] - p[1], ez2 = p[5] - p[2];
            const float wl = sqrtf(ex2 * ex2 + ey2 * ey2 + ez2 * ez2);
            if (z > 1e-3f && wl * pxPerWorldOverZ / z >= 1.0f)
            {
                // Layer fade: outer parts' outlines stay readable, buried ones recede.
                const float vx = mxw - g_cx, vy = myw - g_cy, vz = mzw - g_cz;
                const float sl = (vx * dx + vy * dy + vz * dz) / R;
                float t = 0.5f * (1.0f - sl);             // 0 = nearest, 1 = deepest
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                const float layerFade = 1.0f - 0.60f * t;

                // Silhouette test: -viewDir is the direction from the surface to the eye.
                const float fA = -(n[0] * dx + n[1] * dy + n[2] * dz);
                const float fB = -(n[3] * dx + n[4] * dy + n[5] * dz);
                const bool frontA = (fA > 0.0f), frontB = (fB > 0.0f);

                // Three-way classify. The missing case until now was the BACK-side crease:
                // the translucent pass runs with glDepthMask(GL_FALSE), so nothing occludes
                // anything and the far half of every part draws its creases exactly on top
                // of the near half. On the ribbed uprights of MONT_PLAINA_MESA that is a
                // dozen vertical creases x 2 sides piling onto the same pixels, which is the
                // solid navy column in the middle of the model - and it did NOT respond to
                // alpha, palette or flatness changes because it is pure edge ink.
                //
                //   normals disagree  -> silhouette of the part   -> strong
                //   both towards eye  -> crease on the visible side -> readable hint
                //   both away from eye-> crease behind the surface -> almost nothing
                float base;
                if (frontA != frontB) base = 0.62f;
                else if (frontA)      base = 0.12f;
                else                  base = 0.03f;

                // Foreshortening fade: a segment that runs nearly parallel to the view ray is
                // 1-2 px long on screen and only contributes noise, however long it is in 3D.
                float along = 0.0f;
                if (wl > 1e-6f)
                {
                    float ax = fabsf((ex2 * dx + ey2 * dy + ez2 * dz) / wl);
                    if (ax > 1.0f) ax = 1.0f;
                    along = ax * ax * ax;
                }
                a = base * layerFade * (1.0f - 0.85f * along);
                // Silhouette-only mode: keep strong outlines (base 0.62), drop the
                // interior-crease hints (0.12 / 0.03) so a crease-dense part reads as
                // a clean outlined solid instead of a haystack of lines.
                if (g_edgeSilhouetteOnly && base < 0.5f) a = 0.0f;
            }

            for (int k = 0; k < 2; k++)
            {
                float* o = &g_edgeCol[s * 8 + (size_t)k * 4];
                o[0] = 0.20f;
                o[1] = 0.22f;
                o[2] = 0.30f;
                o[3] = a;
            }
        }
    }

    // Which "layer" each part sits on, seen from the current eye: 0 = the part nearest
    // the eye, 1 = the part deepest behind it. The measure is the part centroid projected
    // onto the view axis and normalised by the bounding radius, so it varies CONTINUOUSLY
    // as the camera orbits - no popping when a part crosses another.
    void UpdateGroupLayers(float ex, float ey, float ez)
    {
        if (g_groupCount <= 0 || g_groupCenter.size() < (size_t)g_groupCount * 3) return;
        if (g_groupLayer.size() != (size_t)g_groupCount) g_groupLayer.assign((size_t)g_groupCount, 0.0f);

        float dx = ex - g_cx, dy = ey - g_cy, dz = ez - g_cz;
        float len = sqrtf(dx * dx + dy * dy + dz * dz);
        if (len < 1e-6f) return;
        dx /= len; dy /= len; dz /= len;
        float R = (g_radius > 1e-6f) ? g_radius : 1.0f;

        for (int g = 0; g < g_groupCount; g++)
        {
            float vx = g_groupCenter[3 * g + 0] - g_cx;
            float vy = g_groupCenter[3 * g + 1] - g_cy;
            float vz = g_groupCenter[3 * g + 2] - g_cz;
            float s = (vx * dx + vy * dy + vz * dz) / R;   // +1 = towards the eye area
            float t = 0.5f * (1.0f - s);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            g_groupLayer[g] = t;
        }
    }

    // Part palette (0..1 RGB). Replaces the old "golden angle hue + s=0.30" scheme whose
    // 0th entry is a desaturated pink that the 3-point rig darkens into an ugly copper/brown.
    // Hand-picked vivid colours, bright enough to survive shading (~0.63x), deliberately
    // avoiding the muddy olive / brown hue band. Order == assignment order.
    // Part palette (0..1 RGB), tuned to the Ansys SpaceClaim / UG auto-colour look:
    // LOW-MEDIUM saturation (~30-50%) with HIGH lightness (~65-80%) - pastel orange / green /
    // purple / blue. Deliberately avoids both the old copper-brown and over-saturated colours
    // (the previous vivid palette read as "too strong" on shaded solids).
    // Part colour wheel (0..1 RGB), 24 entries ordered by hue. Tuned to the Ansys SpaceClaim
    // / UG auto-colour look: LOW-MEDIUM saturation with HIGH lightness - pastel orange /
    // yellow / green / teal / blue / violet / pink / grey all the way around the wheel.
    //
    // Colours are NOT taken in order: index = (g * step) MOD n with step coprime to n, which
    // walks every entry exactly once while giving CONSECUTIVE PARTS a large hue jump (~105 deg
    // for n=24, step=7) - maximum part-to-part separation. Parity then nudges lightness one
    // more notch so neighbours never read as the same colour.
    static const float kPartPalette[][3] = {
        { 0.91f, 0.75f, 0.51f },   // 淡橙
        { 0.94f, 0.86f, 0.59f },   // 暖黄
        { 0.80f, 0.84f, 0.55f },   // 橄榄黄
        { 0.71f, 0.84f, 0.51f },   // 黄绿
        { 0.59f, 0.80f, 0.47f },   // 淡绿
        { 0.51f, 0.80f, 0.63f },   // 青绿
        { 0.47f, 0.80f, 0.75f },   // 薄荷
        { 0.43f, 0.76f, 0.73f },   // 浅青
        { 0.55f, 0.78f, 0.86f },   // 天蓝
        { 0.55f, 0.73f, 0.88f },   // 淡蓝
        { 0.51f, 0.67f, 0.88f },   // 蓝
        { 0.61f, 0.61f, 0.82f },   // 蓝紫
        { 0.71f, 0.61f, 0.80f },   // 淡紫
        { 0.76f, 0.59f, 0.80f },   // 紫
        { 0.82f, 0.61f, 0.76f },   // 紫粉
        { 0.88f, 0.65f, 0.71f },   // 粉
        { 0.90f, 0.63f, 0.65f },   // 玫红
        { 0.92f, 0.67f, 0.59f },   // 珊瑚
        { 0.80f, 0.73f, 0.57f },   // 浅卡其
        { 0.86f, 0.80f, 0.69f },   // 米色
        { 0.63f, 0.78f, 0.69f },   // 灰绿
        { 0.65f, 0.71f, 0.76f },   // 蓝灰
        { 0.73f, 0.71f, 0.76f },   // 灰紫
        { 0.78f, 0.75f, 0.71f },   // 暖灰
    };
    static const int kPartPaletteStep = 7;

    // ---- one-solid tint: seasonal -------------------------------------------------
    // Most drawings of a single part have no palette to spread, so the whole model gets
    // ONE colour - and that colour follows the season (product decision 2026-10-31):
    // winter = warm light orange, summer = cool light green. The two anchors are blended
    // with a cosine over the year, so the hue drifts a little every day and never jumps.
    //   day 0   (Jan 1) -> k = 0.0 -> light orange
    //   day 182 (Jul 2) -> k = 1.0 -> light green
    //   day 365 (Dec 31)-> k = 0.0 -> back to light orange
    // Anchors are palette entries 0 and 4, so the tint always stays inside the 24-colour
    // chart instead of introducing a colour the assembly view can never produce.
    void SeasonalTint(float& r, float& gg, float& b)
    {
        static const float kWarm[3] = { 0.91f, 0.75f, 0.51f };   // light orange  (palette 0)
        static const float kCool[3] = { 0.59f, 0.80f, 0.47f };   // light green   (palette 4)
        SYSTEMTIME st;
        ::GetLocalTime(&st);
        static const int kCum[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
        int doy = kCum[(st.wMonth - 1) % 12] + (int)st.wDay - 1;
        if (doy < 0) doy = 0;
        // Support / acceptance override: PLATFORM_CAD_SEASON_DOY=0..365 pins the day of year
        // so both ends of the ramp can be checked without waiting six months
        // (0 = January / warm orange, 182 = July / cool green).
        char sbuf[16];
        if (::GetEnvironmentVariableA("PLATFORM_CAD_SEASON_DOY", sbuf, (DWORD)sizeof(sbuf)) > 0)
        {
            int v = atoi(sbuf);
            if (v >= 0 && v <= 365) doy = v;
        }
        double k = 0.5 * (1.0 - cos((doy / 365.0) * 6.28318530717958648));
        r  = kWarm[0] + (kCool[0] - kWarm[0]) * (float)k;
        gg = kWarm[1] + (kCool[1] - kWarm[1]) * (float)k;
        b  = kWarm[2] + (kCool[2] - kWarm[2]) * (float)k;
    }

    // Single part: one colour for the whole model (it has to look good on its own).
    static void PartColor(int g, int groupCount, float& r, float& gg, float& b)
    {
        if (groupCount <= 1) { SeasonalTint(r, gg, b); return; }   // single part: seasonal tint
        const int n = (int)(sizeof(kPartPalette) / sizeof(kPartPalette[0]));
        int idx = (g * kPartPaletteStep) % n;                 // jump: neighbours differ the most
        float shade = ((g % 2) == 0) ? 1.0f : 0.9f;           // one more lightness notch
        r = kPartPalette[idx][0] * shade;
        gg = kPartPalette[idx][1] * shade;
        b = kPartPalette[idx][2] * shade;
    }

    void UpdateGlassColors(float ex, float ey, float ez)
    {
        size_t n = g_nrm.size() / 3;
        if (n == 0 || g_col.size() < n * 4 || g_pos.size() < n * 3) return;

        // Re-evaluating half a million vertices is only worth it when the eye moved; a
        // static frame (T key, exposure, repaint from occlusion) reuses the last result.
        static float lastX = 1e30f, lastY = 0.0f, lastZ = 0.0f, lastA = -1.0f;
        if (g_alpha == lastA
            && fabsf(ex - lastX) < 1e-4f && fabsf(ey - lastY) < 1e-4f && fabsf(ez - lastZ) < 1e-4f)
            return;
        lastX = ex; lastY = ey; lastZ = ez; lastA = g_alpha;

        UpdateGroupLayers(ex, ey, ez);

        if (g_colFront.size() != n * 4) g_colFront.assign(n * 4, 1.0f);
        if (g_colBack.size() != n * 4) g_colBack.assign(n * 4, 1.0f);
        bool layered = (g_vgroup.size() == n) && !g_groupLayer.empty();
        bool havePalette = (g_groupCount > 0 && g_groupColor.size() >= (size_t)g_groupCount * 3);

        // A mesh that arrives without part ids (STL / OBJ: the host passes an empty group
        // array) is ONE body, and every constant below is tuned for the opposite regime - a
        // stack of many translucent parts. A single closed body has exactly two surfaces
        // along any view ray, so the per-part alphas (0.03-0.14 x g_alpha) leave it with a
        // few percent of ink and it reads as "no colour at all", which is what happened to
        // A.stl / B.stl. Give it a flat, real glass alpha instead: two surfaces at 0.34 and
        // 0.46 compose to ~66% and still let a third of the background through.
        const bool singleBody = (g_groupCount <= 0);

        const float shellCap = 0.74f;   // a silhouette firms up, but not to solid: a hard
                                        // cap turns every thin rod into a flat coloured line
                                        // and the model starts to look like a wireframe
        const float wallCap = 0.82f;    // the wall behind it firms up too, but it must stay
                                        // short of solid: a view ray in MONT_PLAINA_MESA
                                        // crosses ~20 nested parts (sleeve inside sleeve)
                                        // and at a 0.95 cap those walls ADD UP to a dead
                                        // navy column right in the middle of the model

        for (size_t i = 0; i < n; i++)
        {
            int gi = layered ? g_vgroup[i] : 0;
            float br, bg, bb;
            if (havePalette && gi >= 0 && gi < g_groupCount)
            {
                br = g_groupColor[3 * gi + 0];
                bg = g_groupColor[3 * gi + 1];
                bb = g_groupColor[3 * gi + 2];
            }
            else
            {
                br = g_col[4 * i + 0];
                bg = g_col[4 * i + 1];
                bb = g_col[4 * i + 2];
            }

            // Translucent lift for dark parts. The palette is chosen for SOLID shading, where
            // a navy part reads fine under the shoulder light; the same navy behind 0.62
            // alpha at a grazing silhouette (alpha climbs to shellCap) composites into a
            // black hole in the middle of the assembly - that was the "dark column" in
            // MONT_PLAINA_MESA, and it is not a rendering bug: the same pixels measure
            // darker in the opaque view. Lifting dark entries toward white turns the hole
            // back into a coloured veil while leaving mid and light entries almost untouched
            // (0.24 lift on black, 0.05 on 0.8).
            //
            // A group-less body must NOT take this lift: its solid colour is already pale
            // (hue 0 at sat 0.30 / light 0.74 for STL), and lifting that toward white ends at
            // a grey-pink that is invisible behind glass. It is pushed away from its own mean
            // instead, so the single tint it does have survives the glass.
            if (singleBody)
            {
                const float mean = (br + bg + bb) * (1.0f / 3.0f);
                br = mean + (br - mean) * 3.2f;
                bg = mean + (bg - mean) * 3.2f;
                bb = mean + (bb - mean) * 3.2f;
                if (br < 0.0f) br = 0.0f; else if (br > 1.0f) br = 1.0f;
                if (bg < 0.0f) bg = 0.0f; else if (bg > 1.0f) bg = 1.0f;
                if (bb < 0.0f) bb = 0.0f; else if (bb > 1.0f) bb = 1.0f;
                br *= 0.94f; bg *= 0.94f; bb *= 0.94f;
            }
            else
            {
                const float kLift = 0.24f;
                br += (1.0f - br) * kLift;
                bg += (1.0f - bg) * kLift;
                bb += (1.0f - bb) * kLift;
            }

            float f = 0.0f;
            float vx = ex - g_pos[3 * i];
            float vy = ey - g_pos[3 * i + 1];
            float vz = ez - g_pos[3 * i + 2];
            float len = sqrtf(vx * vx + vy * vy + vz * vz);
            if (len > 1e-6f)
            {
                float d = (g_nrm[3 * i] * vx + g_nrm[3 * i + 1] * vy + g_nrm[3 * i + 2] * vz) / len;
                if (d < 0.0f) d = -d;             // two-sided: back faces count the same
                float t = 1.0f - d;               // 0 = facing the eye, 1 = edge on
                // t^3 instead of powf(t, 2.5f): visually indistinguishable at this scale and
                // it removes a transcendental call from the hot loop.
                f = t * t * t;
            }

            // Layer factor: 0 for the part nearest the eye, 1 for the deepest one.
            //
            // The step is deliberately SMALL. "Outside transparent, inside solid" sounds
            // right, but a view ray crosses a dozen parts: if the deep ones are solid they
            // black out everything behind them and the interior turns into a dark slab.
            // Keeping every layer light lets the eye keep travelling inwards, while the
            // ordering still biases towards the near parts - which is what gives the stack
            // its depth without the mud.
            float layer = layered ? g_groupLayer[gi] : 0.35f;

            // Shell: at the front of the stack this is nearly invisible face-on and only
            // its grazing rim shows; towards the back it firms up.
            //
            // Surfaces are now deliberately FAINT while the outlines carry the structure
            // (see the edge pass). That is how SolidWorks' transparent view reads: the
            // faces only say "there is material here", the edges say "this is a part, and
            // this is where it ends". With both at medium strength the two fight each
            // other and the result is the washed-out look we kept getting.
            float aShell = singleBody ? (g_alpha * 0.34f)
                                     : (g_alpha * (0.03f + 0.11f * layer));
            float aS = aShell + (shellCap - aShell) * f;
            float lift = 0.16f * f;
            g_colFront[4 * i + 0] = br + (1.0f - br) * lift;
            g_colFront[4 * i + 1] = bg + (1.0f - bg) * lift;
            g_colFront[4 * i + 2] = bb + (1.0f - bb) * lift;
            g_colFront[4 * i + 3] = aS;

            // Wall: firmer than the shell and it carries the layer gradient, but both the
            // darkness (0.88, was 0.84) and the base alpha are kept modest - the wall is
            // what a ray hits 2x per part, so any excess here multiplies into black.
            float aWall = singleBody ? (g_alpha * 0.46f)
                                     : (g_alpha * (0.05f + 0.13f * layer));
            float aB = aWall + (wallCap - aWall) * f * 0.7f;
            g_colBack[4 * i + 0] = br * 0.88f;
            g_colBack[4 * i + 1] = bg * 0.88f;
            g_colBack[4 * i + 2] = bb * 0.88f;
            g_colBack[4 * i + 3] = aB;
        }

        UpdateEdgeColors(ex, ey, ez);
    }

    // ---- per-part visibility: compacted draw arrays -------------------------
    // True when some part is hidden AND the mesh actually carries part information.
    // A group-less mesh (STL/OBJ: empty g_groupVisible) never filters, so those files
    // keep the plain single draw call.
    bool PartFilterActive()
    {
        if (g_groupVisible.empty()) return false;
        for (size_t i = 0; i < g_groupVisible.size(); i++)
            if (!g_groupVisible[i]) return true;
        return false;
    }

    // A node is effectively visible only if it and every ancestor in the assembly tree
    // is visible. This lets hiding a parent (in the tree UI) hide all its descendants
    // even when a descendant's own checkbox is still ticked. The guard caps the walk at
    // the node count so a malformed (cyclic) parent chain can never infinite-loop.
    bool GroupEffectivelyVisible(int g)
    {
        int cur = g;
        size_t guard = 0, n = g_groupVisible.size();
        while (cur >= 0 && (size_t)cur < n && guard++ < n)
        {
            if (!g_groupVisible[(size_t)cur]) return false;
            cur = g_partParent.empty() ? -1 : g_partParent[(size_t)cur];
        }
        return true;
    }

    // Triangles of the visible parts, in mesh order. Built once per visibility change
    // (lazily, on the first filtered draw); everything-visible keeps using g_idx.
    void RebuildVisibleIndices()
    {
        g_idxVis.clear();
        const size_t ntri = g_idx.size() / 3;
        if (g_tgroup.size() != ntri)
        {
            g_idxVisValid = true;   // no per-triangle map: nothing to filter
            return;
        }
        g_idxVis.reserve(g_idx.size());
        for (size_t t = 0; t < ntri; t++)
        {
            int g = g_tgroup[t];
            if (g >= 0 && (size_t)g < g_groupVisible.size() && !GroupEffectivelyVisible(g)) continue;
            g_idxVis.push_back(g_idx[3 * t + 0]);
            g_idxVis.push_back(g_idx[3 * t + 1]);
            g_idxVis.push_back(g_idx[3 * t + 2]);
        }
        g_idxVisValid = true;
    }

    const int* VisibleIndices(GLsizei& count)
    {
        if (!PartFilterActive()) { count = (GLsizei)g_idx.size(); return g_idx.data(); }
        if (!g_idxVisValid) RebuildVisibleIndices();
        count = (GLsizei)g_idxVis.size();
        return g_idxVis.data();
    }

    // Edge segments of the visible parts. Rebuilt per frame while filtering is active,
    // because UpdateEdgeColors rewrites g_edgeCol every frame and the compacted copy must
    // not lag behind it.
    void RebuildVisibleEdges()
    {
        g_edgePosVis.clear();
        g_edgeColVis.clear();
        const size_t nseg = g_edgePos.size() / 6;
        if (g_egroup.size() != nseg) return;   // no per-segment map: caller draws all
        for (size_t s = 0; s < nseg; s++)
        {
            int g = g_egroup[s];
            if (g >= 0 && (size_t)g < g_groupVisible.size() && !GroupEffectivelyVisible(g)) continue;
            for (int k = 0; k < 6; k++) g_edgePosVis.push_back(g_edgePos[s * 6 + k]);
            for (int k = 0; k < 8; k++) g_edgeColVis.push_back(g_edgeCol[s * 8 + k]);
        }
    }

    // ---- free-orbit camera basis -----------------------------------------------
    // Rotate one vector around a unit axis (Rodrigues).
    static void RotAround(float v[3], const float a[3], float ang)
    {
        float c = cosf(ang), s = sinf(ang);
        float cr[3];
        cr[0] = a[1] * v[2] - a[2] * v[1];
        cr[1] = a[2] * v[0] - a[0] * v[2];
        cr[2] = a[0] * v[1] - a[1] * v[0];
        float dot = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
        for (int i = 0; i < 3; i++)
            v[i] = v[i] * c + cr[i] * s + a[i] * dot * (1.0f - c);
    }

    static void Norm3(float v[3])
    {
        float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (l > 1e-8f) { v[0] /= l; v[1] /= l; v[2] /= l; }
    }

    static void Cross3(const float a[3], const float b[3], float o[3])
    {
        o[0] = a[1] * b[2] - a[2] * b[1];
        o[1] = a[2] * b[0] - a[0] * b[2];
        o[2] = a[0] * b[1] - a[1] * b[0];
    }

    // Rotate ALL three basis vectors around one screen axis. This is the whole point:
    // the axis is the CURRENT screen axis, so the model always follows the hand and
    // there is no privileged "up" in the world to run into. Re-orthonormalise after
    // every call to kill float drift accumulated over thousands of mouse moves.
    static void RotateBasis(const float axis[3], float ang)
    {
        float a[3] = { axis[0], axis[1], axis[2] };
        Norm3(a);
        RotAround(g_camF, a, ang);
        RotAround(g_camR, a, ang);
        RotAround(g_camU, a, ang);
        // Gram-Schmidt: F defines the eye direction; rebuild U and R from it.
        Norm3(g_camF);
        // Keep U as close as possible to the previous U (prevents sudden roll flips).
        float d = g_camF[0] * g_camU[0] + g_camF[1] * g_camU[1] + g_camF[2] * g_camU[2];
        g_camU[0] -= d * g_camF[0];
        g_camU[1] -= d * g_camF[1];
        g_camU[2] -= d * g_camF[2];
        Norm3(g_camU);
        // R = cross(F, U) matches the historical right = cross(f, worldUp) frame
        // (verified: at the default view it is exactly (-cos(az), 0, sin(az))).
        float r[3];
        Cross3(g_camF, g_camU, r);
        g_camR[0] = r[0]; g_camR[1] = r[1]; g_camR[2] = r[2];
        Norm3(g_camR);
    }

    // Default view (az=0.6 / el=0.4, the old spherical camera's starting angle).
    // Called on NEW GEOMETRY (SetMesh) and on an explicit ResetOrientation(); NEVER
    // from Fit(). Mature CAD viewers keep the user's orientation when framing, so
    // those two operations must stay separate.
    static void ResetBasis()
    {
        // SHARED CONTRACT with the CPU backend: SoftRender3D.cs uses Yaw=0.6 /
        // Pitch=0.4. Change BOTH places, or the default view and the "Reset view"
        // result diverge between GPU and software fallback.
        const float az = 0.6f, el = 0.4f;
        g_camF[0] = cosf(el) * sinf(az);
        g_camF[1] = sinf(el);
        g_camF[2] = cosf(el) * cosf(az);
        // R = normalize(cross(F, worldUp)) ; U = cross(R, F)
        float worldUp[3] = { 0.0f, 1.0f, 0.0f };
        Cross3(g_camF, worldUp, g_camR);
        Norm3(g_camR);
        Cross3(g_camR, g_camF, g_camU);
        Norm3(g_camU);
        Norm3(g_camF);
    }

    // Anything that changes what the picture looks like must call this. Forgetting a
    // mutator would freeze the image until the next camera move, so every public entry
    // point that touches geometry, camera or render settings marks the scene dirty.
    void MarkSceneDirty() { g_sceneDirty = true; }

    // Blit the retained frame to the window and swap. Returns false when there is nothing
    // valid to present (first frame, just resized, retention disabled).
    bool PresentRetainedFrame()
    {
        if (!g_retainEnabled || !g_frameValid || g_presentFbo == 0) return false;
        if (g_presentW != g_w || g_presentH != g_h) return false;

        pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
        glViewport(0, 0, g_w, g_h);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glDisable(GL_FOG);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);

        glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
        glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, g_presentTex);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex2f(1.0f, 0.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex2f(1.0f, 1.0f);
        glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, 1.0f);
        glEnd();
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);

        glMatrixMode(GL_PROJECTION); glPopMatrix();
        glMatrixMode(GL_MODELVIEW); glPopMatrix();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        // Client arrays stay disabled: Render() enables them right where it binds the
        // buffer it is about to draw, so restoring them here would only risk binding a
        // stale pointer.

        ::SwapBuffers(g_hdc);
        return true;
    }

    // Resolve the 4x MSAA target straight to the window: anti-aliasing without AO.
    // Used by the translucent pass, where depth is never written (glDepthMask false), so
    // SSAO would read an empty depth buffer and return 1.0 everywhere - i.e. it cost a
    // full-resolution 16-tap pass plus a 5x5 blur per frame and changed nothing.
    void ResolveMsaaOnly()
    {
        pglBindFramebuffer(OGL_READ_FRAMEBUFFER, g_msaaFbo);
        pglBindFramebuffer(OGL_DRAW_FRAMEBUFFER, (g_retainEnabled && g_presentFbo) ? g_presentFbo : 0);
        pglBlitFramebuffer(0, 0, g_aoW, g_aoH, 0, 0, g_w, g_h,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);
        pglBindFramebuffer(OGL_FRAMEBUFFER, 0);
    }

    void RunAoAndComposite(float nearZ, float farZ, float tanHalfFovY, float aspect)
    {
        // resolve MSAA -> textures
        pglBindFramebuffer(OGL_READ_FRAMEBUFFER, g_msaaFbo);
        pglBindFramebuffer(OGL_DRAW_FRAMEBUFFER, g_resolveFbo);
        pglBlitFramebuffer(0, 0, g_aoW, g_aoH, 0, 0, g_aoW, g_aoH,
                           GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

        // ---- SSAO pass ----
        pglBindFramebuffer(OGL_FRAMEBUFFER, g_aoFbo);
        glViewport(0, 0, g_aoW, g_aoH);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_LIGHTING);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        pglUseProgram(g_aoProg);
        pglActiveTexture(OGL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, g_depthTex);
        pglUniform1i(g_uDepth, 0);
        pglUniform2f(g_uResAo, (GLfloat)g_aoW, (GLfloat)g_aoH);
        pglUniform1f(g_uNear, nearZ);
        pglUniform1f(g_uFar, farZ);
        pglUniform1f(g_uTanY, tanHalfFovY);
        pglUniform1f(g_uAspect, aspect);
        // Reach: a fixed pixel budget at each fragment's own depth, capped by a small
        // fraction of the model size. See the comment above kAoVertSrc for why the old
        // "10% of the bounding radius" reach turned dense assemblies into grey mush.
        pglUniform1f(g_uRadiusCap, g_radius * g_aoParams.radiusCap);
        pglUniform1f(g_uRadiusPx, g_aoParams.radiusPx);
        pglUniform1f(g_uBiasK, g_aoParams.biasK);
        pglUniform1f(g_uStrength, g_aoParams.strength);
        DrawFullscreenQuad();

        // ---- composite pass ----
        pglBindFramebuffer(OGL_FRAMEBUFFER, (g_retainEnabled && g_presentFbo) ? g_presentFbo : 0);
        glViewport(0, 0, g_w, g_h);
        pglUseProgram(g_compProg);
        pglActiveTexture(OGL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, g_colorTex);
        pglUniform1i(g_uColor, 0);
        pglActiveTexture(OGL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, g_aoTex);
        pglUniform1i(g_uAo, 1);
        pglActiveTexture(OGL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, g_depthTex);
        pglUniform1i(g_uDepthComp, 2);
        pglUniform2f(g_uResComp, (GLfloat)g_w, (GLfloat)g_h);
        pglUniform1f(g_uNearComp, nearZ);
        pglUniform1f(g_uFarComp, farZ);
        pglUniform1f(g_uTanYComp, tanHalfFovY);
        pglUniform1f(g_uRadiusCapComp, g_radius * g_aoParams.radiusCap);
        pglUniform1f(g_uRadiusPxComp, g_aoParams.radiusPx);
        pglUniform1f(g_uAoMix, g_aoParams.mixAmt);
        DrawFullscreenQuad();

        pglUseProgram(0);
        pglActiveTexture(OGL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, 0);
        pglActiveTexture(OGL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        pglActiveTexture(OGL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glEnable(GL_LIGHTING);   // the AO pass turned it off; the scene pass relies on it
    }
}

int OcctViewer::Create(System::IntPtr hwnd)
{
    // A WGL context belongs to ONE window, and every GL object cached below (MSAA / resolve
    // / AO / present framebuffers, their textures, the two shader programs) is owned by the
    // context that created it. This used to simply overwrite g_hwnd/g_hdc/g_hglrc: the old
    // context leaked and its object ids stayed behind, so the NEW context hit
    // EnsureAoResources()'s "g_aoReady && same size" short-circuit and Render() bound ids
    // that do not exist in it - every frame then presented an empty texture.
    // MEASURED (mesh2 diagnostic: portalgun__2.3ds then fels.3ds in a second preview):
    // 594520/594520 pure white pixels; the same sequence with PLATFORM_CAD_NO_AO=1
    // (no FBO chain at all) renders 35.3% ink. So the live context must go first.
    if (g_hwnd || g_hdc || g_hglrc) Destroy();

    // A previous preview may have been closed mid-drag. Never inherit a stale
    // "skip SSAO" flag into the new context.
    g_interacting = false;
    g_msaaSamples = 0;
    g_hwnd = (HWND)hwnd.ToPointer();
    if (!g_hwnd) return -1;
    g_hdc = ::GetDC(g_hwnd);
    if (!g_hdc) return -2;

    PIXELFORMATDESCRIPTOR pfd = { 0 };
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;
    pfd.iLayerType = PFD_MAIN_PLANE;

    int pf = 0;
    if (LoadWglArbEntryPoints()) pf = ChooseMultiSampleFormat(g_hdc);
    if (pf == 0) pf = ::ChoosePixelFormat(g_hdc, &pfd);   // no MSAA available
    if (pf == 0) return -3;

    PIXELFORMATDESCRIPTOR chosen = pfd;
    if (::DescribePixelFormat(g_hdc, pf, sizeof(chosen), &chosen) <= 0) chosen = pfd;
    if (!::SetPixelFormat(g_hdc, pf, &chosen))
    {
        // A window can only be given a pixel format ONCE (NT and later). Reopening
        // the preview on the same HWND therefore fails here; reuse the format that
        // is already installed instead of aborting the whole preview.
        if (::GetPixelFormat(g_hdc) == 0) return -4;
    }

    g_hglrc = ::wglCreateContext(g_hdc);
    if (!g_hglrc) return -5;
    if (!::wglMakeCurrent(g_hdc, g_hglrc)) return -6;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // THREE-POINT RIG (key / fill / rim), lights are re-positioned every frame in Render().
    // Why: with one key light and a nearly flat ambient the part reads as a silhouette full of
    // colour, not as a solid - curvature on fillets, bores and chamfers has nothing to catch the
    // eye. Three lights + a real specular sheen fix that without any shader work:
    //   - key  : up and to the left of the camera, does the modelling;
    //   - fill : opposite side, weak, keeps shadowed faces from going dead flat;
    //   - rim  : from BEHIND and above, lights the silhouette so parts separate from each other
    //            and from the white background (this is what makes a shape "pop").
    // GL_LIGHT_MODEL_LOCAL_VIEWER gives a per-vertex eye vector, so the highlight lands where
    // it physically should instead of assuming an infinitely distant viewer.
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

    // Brightness budget: the final channel is colour * (ambient + sum of diffuse) + specular.
    // ambient 0.16 + key 0.60 + fill 0.16 + rim 0.14 = 1.06, but key and rim sit on OPPOSITE
    // sides so no surface can receive both at full strength; the practical maximum is ~0.80,
    // times the pastel base colour (~0.79) gives ~0.63 - nowhere near clipping to white.
    GLfloat amb[4] = { 0.16f, 0.17f, 0.20f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

    // Specular is now a REAL sheen instead of a token 0.05 x 0.05: matSpec 0.30 * lightSpec 0.35
    // = 0.105 peak, added to a ~0.63 diffuse term -> 0.74, still unsaturated, but strong enough
    // to read curvature. Shininess 28 keeps the highlight broad and soft; a tighter 40+ lobe
    // would only appear on a few large flat facets and leave cylinders looking matte.
    GLfloat specRef[4] = { 0.30f, 0.30f, 0.30f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specRef);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 28);

    glEnable(GL_LIGHT0);
    GLfloat l0Diff[4] = { 0.60f, 0.60f, 0.60f, 1.0f };
    GLfloat l0Spec[4] = { 0.35f, 0.35f, 0.35f, 1.0f };
    glLightfv(GL_LIGHT0, GL_DIFFUSE, l0Diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, l0Spec);

    // Fill light: weak side light for volume, kept low to stay under saturation.
    glEnable(GL_LIGHT1);
    GLfloat l1Diff[4] = { 0.16f, 0.17f, 0.19f, 1.0f };
    glLightfv(GL_LIGHT1, GL_DIFFUSE, l1Diff);

    // Rim light: no specular, slightly cool, only there to outline the silhouette.
    glEnable(GL_LIGHT2);
    GLfloat l2Diff[4] = { 0.14f, 0.15f, 0.18f, 1.0f };
    glLightfv(GL_LIGHT2, GL_DIFFUSE, l2Diff);

    // Read back what the driver ACTUALLY gave us instead of trusting the request:
    // if the installed pixel format has no multisample buffer, GL_MULTISAMPLE must
    // stay off. An OpenGL 1.1-only driver raises GL_INVALID_ENUM on these two
    // queries, so the error state is cleared before rendering starts.
    {
        GLint sampleBuffers = 0, samples = 0;
        glGetIntegerv(OGL_SAMPLE_BUFFERS, &sampleBuffers);
        glGetIntegerv(OGL_SAMPLES, &samples);
        while (glGetError() != GL_NO_ERROR) { }
        if (sampleBuffers > 0 && samples > 1)
        {
            g_msaaSamples = (int)samples;
            glEnable(OGL_MULTISAMPLE);   // smooth silhouette on every frame, moving or not
        }
    }
    return 0;
}

void OcctViewer::Destroy()
{
    // Release the AO chain while the context is still current (textures/FBOs/shaders are
    // context-owned). Also drop the cached "already tried" flag so a later Create() rebuilds.
    if (g_hglrc && g_hdc)
    {
        ::wglMakeCurrent(g_hdc, g_hglrc);
        DestroyAoResources();
        if (g_aoProg) { pglDeleteProgram(g_aoProg); g_aoProg = 0; }
        if (g_compProg) { pglDeleteProgram(g_compProg); g_compProg = 0; }
        g_aoReady = false;
        g_aoTried = false;
    }
    if (g_hglrc) { ::wglMakeCurrent(NULL, NULL); ::wglDeleteContext(g_hglrc); g_hglrc = NULL; }
    if (g_hdc && g_hwnd) ::ReleaseDC(g_hwnd, g_hdc);
    g_hdc = NULL; g_hwnd = NULL;
    g_interacting = false;   // no drag can survive a dead context
}

void OcctViewer::SetMesh(array<float>^ positions, array<int>^ indices, array<int>^ groups, array<int>^ parents)
{
    // A fresh model starts at the default view. The camera basis is process-wide
    // static and survives Destroy(), so without this the next drawing would inherit
    // whatever angle the previous one was left at. Deliberately NOT in Create():
    // WinForms can rebuild the child HWND mid-session (dock / DPI changes), and
    // Create() then runs again while the user is interacting - resetting there
    // would yank the view away. Deliberately NOT in Fit() either: framing keeps
    // orientation (see Fit()).
    ResetBasis();

    g_pos.clear(); g_idx.clear(); g_col.clear();
    g_colFront.clear(); g_colBack.clear();   // rebuilt lazily by UpdateGlassColors
    MarkSceneDirty();
    g_edgePos.clear(); g_edgeNrm.clear(); g_edgeCol.clear();    // host re-supplies them via SetEdges
    if (positions != nullptr)
    {
        g_pos.resize(positions->Length);
        for (int i = 0; i < positions->Length; i++) g_pos[i] = positions[i];
    }
    if (indices != nullptr)
    {
        g_idx.resize(indices->Length);
        for (int i = 0; i < indices->Length; i++) g_idx[i] = indices[i];
    }

    // RecomputeBoundsAndNormals() MUST run before the color loop below: it assigns
    // g_vcount, which the loop uses to bounds-check vertex indices. Previously it
    // was called AFTER the loop, so g_vcount was still 0 on the first load, the
    // loop was skipped, and every vertex kept the 0.7 gray default -> the whole
    // model rendered gray (all part colors lost).
    RecomputeBoundsAndNormals();

    // Color: palette by group. 4 floats per vertex (r,g,b,a) so blending can use
    // the per-vertex alpha (Windows gl.h is OpenGL 1.1, so GL_SRC_ALPHA blending
    // is used instead of the unavailable GL 1.4 glBlendColor/GL_CONSTANT_ALPHA).
    int ntri = (int)g_idx.size() / 3;
    g_col.assign((size_t)g_vcount * 4, 0.7f);
    for (int i = 0; i < g_vcount; i++) g_col[4 * i + 3] = g_alpha;

    // ---- per-part bookkeeping (see g_vgroup / g_groupCenter) ----
    // The part count comes from the MAX id, not from the array length: groups[] carries
    // one id per triangle, but the ids themselves are dense in [0, maxGroup] (1114 parts
    // on the 679k-triangle test assembly), so an id-indexed array is exact and cheap.
    g_groupCount = 0;
    if (groups != nullptr)
    {
        for (int t = 0; t < groups->Length; t++)
            if (groups[t] + 1 > g_groupCount) g_groupCount = groups[t] + 1;
    }
    // Assembly tree (XCAF): parents[p] = parent node index of node p, or -1 for a root.
    // Assembly nodes carry no geometry but still occupy a node index, so the node count
    // (and thus g_groupVisible size) must cover them too.
    g_partParent.clear();
    if (parents != nullptr && parents->Length > 0)
    {
        int n = parents->Length;
        if (n > g_groupCount) g_groupCount = n;   // node count >= geometric part count
        g_partParent.resize(n);
        for (int i = 0; i < n; i++) g_partParent[i] = parents[i];
    }
    // Per-triangle part ids for the show/hide filter (see g_tgroup). Clamped exactly like
    // the colour loop below clamps them, so both passes agree on which part a triangle is.
    g_tgroup.clear();
    if (groups != nullptr && ntri > 0)
    {
        g_tgroup.resize((size_t)ntri);
        for (int t = 0; t < ntri; t++)
        {
            int g = (t < groups->Length) ? groups[t] : 0;
            if (g < 0 || g >= g_groupCount) g = 0;
            g_tgroup[t] = g;
        }
    }
    // Every part starts shown; the compacted draw arrays are rebuilt lazily on first use.
    g_groupVisible.assign((size_t)g_groupCount, 1);
    g_idxVis.clear();
    g_idxVisValid = false;
    g_vgroup.assign((size_t)g_vcount, 0);
    g_groupCenter.assign((size_t)g_groupCount * 3, 0.0f);
    g_groupLayer.clear();
    std::vector<float> wsum((size_t)g_groupCount, 0.0f);

    // Translucent palette, one entry per part (see g_groupColor).
    //
    // NOT the golden-angle scheme the opaque view uses. That scheme spreads 1114 parts over
    // 1114 hues, i.e. 0.32 deg apart - adjacent parts are the SAME colour to the eye, and
    // once two of them overlap under alpha they merge into one indistinguishable blob.
    // A human can reliably separate ~8-12 hues in this situation, so the palette below
    // uses a small set of strongly contrasting hues and repeats them, with the repetition
    // walked by a stride coprime with the palette size so that parts with neighbouring
    // ids never land on the same hue. A second lightness band widens it to 16 combos.
    //
    // The hues are deliberately spread around the wheel (red/orange/yellow/green/cyan/
    // blue/violet/magenta) because neighbouring ids must differ a lot, and both bands stay
    // mid-light so that a red part laid over a green one still reads as two colours rather
    // than as mud.
    static const float kGlassHues[8] = { 6.0f, 40.0f, 62.0f, 130.0f, 178.0f, 212.0f, 272.0f, 322.0f };
    g_groupColor.assign((size_t)g_groupCount * 3, 0.7f);
    for (int g = 0; g < g_groupCount; g++)
    {
        // stride 3 is coprime with 8: consecutive ids are 3 hue slots apart (90-120 deg),
        // and the lightness band flips every 8 parts.
        int k = (g * 3) % 8;
        int band = (g / 8) % 2;
        float r, gg, b;
        // Lightness is deliberately HIGH (0.52 / 0.64) rather than the 0.39 / 0.50 used for
        // solid shading: a view ray in a nested assembly composites 20+ translucent surfaces
        // and the blend converges to the AVERAGE of their colours. Dark palette entries
        // therefore average into a muddy navy slab in the middle of the model, while light
        // ones average into a light haze and the edges stay the only dark thing on screen.
        HslToRgb(kGlassHues[k], 0.62f, band ? 0.64f : 0.52f, r, gg, b);
        g_groupColor[3 * g + 0] = r;
        g_groupColor[3 * g + 1] = gg;
        g_groupColor[3 * g + 2] = b;
    }

    if (ntri > 0 && g_vcount > 0)
    {
        for (int t = 0; t < ntri; t++)
        {
            int g = (groups != nullptr && t < groups->Length) ? groups[t] : 0;
            if (g < 0 || g >= g_groupCount) g = 0;
            float r, gg, b;
            // Same scheme as the C# software renderer the user liked: golden angle
            // (137.508 deg) hue step maximizes separation between consecutive groups;
            // alternating lightness adds a brightness step at every group boundary.
            PartColor(g, g_groupCount, r, gg, b);
            int i0 = g_idx[3 * t], i1 = g_idx[3 * t + 1], i2 = g_idx[3 * t + 2];
            if (i0 >= 0 && i0 < g_vcount) { g_col[4 * i0] = r; g_col[4 * i0 + 1] = gg; g_col[4 * i0 + 2] = b; }
            if (i1 >= 0 && i1 < g_vcount) { g_col[4 * i1] = r; g_col[4 * i1 + 1] = gg; g_col[4 * i1 + 2] = b; }
            if (i2 >= 0 && i2 < g_vcount) { g_col[4 * i2] = r; g_col[4 * i2 + 1] = gg; g_col[4 * i2 + 2] = b; }

            // vertex -> part, plus the running centroid that the layering pass needs.
            // Vertices belong to exactly one part on the test model, so a plain overwrite
            // is correct here (no averaging across parts).
            //
            // Guarded on g_groupCount: a mesh with no group information at all (STL / OBJ
            // arrive with an empty array) has ZERO parts, so g_groupCenter is empty and
            // writing g_groupCenter[0] here ran off the end of a zero-length vector - it
            // corrupted the heap and made STL files render nothing at all.
            if (g_groupCount > 0)
            {
                const int v3[3] = { i0, i1, i2 };
                for (int k = 0; k < 3; k++)
                {
                    int vi = v3[k];
                    if (vi < 0 || vi >= g_vcount) continue;
                    g_vgroup[vi] = g;
                    g_groupCenter[3 * g + 0] += g_pos[3 * vi + 0];
                    g_groupCenter[3 * g + 1] += g_pos[3 * vi + 1];
                    g_groupCenter[3 * g + 2] += g_pos[3 * vi + 2];
                    wsum[g] += 1.0f;
                }
            }
        }
    }
    for (int g = 0; g < g_groupCount; g++)
    {
        if (wsum[g] > 0.0f)
        {
            float inv = 1.0f / wsum[g];
            g_groupCenter[3 * g + 0] *= inv;
            g_groupCenter[3 * g + 1] *= inv;
            g_groupCenter[3 * g + 2] *= inv;
        }
    }

    // Part outlines are supplied by the host (OcctViewer::SetEdges) from the B-rep
    // topology, so nothing is derived from the triangle mesh here any more.

    g_hasMesh = !g_pos.empty();
    MarkSceneDirty();
}

void OcctViewer::SetMesh(array<float>^ positions, array<int>^ indices, array<int>^ groups)
{
    // Flat mesh (STL/OBJ): no assembly tree, so no parent links.
    SetMesh(positions, indices, groups, nullptr);
}

void OcctViewer::SetEdges(array<float>^ lines)
{
    SetEdges(lines, nullptr);
}

void OcctViewer::SetEdges(array<float>^ lines, array<int>^ groups)
{
    g_edgePos.clear();
    g_edgeCol.clear();
    g_edgeNrm.clear();
    g_egroup.clear();
    if (lines == nullptr) return;

    // 12 floats per segment: ax,ay,az, bx,by,bz, nAx,nAy,nAz, nBx,nBy,nBz
    const int nseg = lines->Length / 12;
    if (nseg <= 0) return;
    g_edgePos.resize((size_t)nseg * 6);
    g_edgeNrm.resize((size_t)nseg * 6);
    for (int s = 0; s < nseg; s++)
    {
        const int src = s * 12;
        for (int k = 0; k < 6; k++)
        {
            g_edgePos[(size_t)s * 6 + k] = lines[src + k];
            g_edgeNrm[(size_t)s * 6 + k] = lines[src + 6 + k];
        }
    }

    // Sensible default now: UpdateEdgeColors only runs on the translucent path, and a
    // frame drawn before the first orbit must not show unlit black lines.
    g_edgeCol.resize((size_t)nseg * 8);          // 2 vertices x rgba
    for (size_t i = 0; i + 3 < g_edgeCol.size(); i += 4)
    {
        g_edgeCol[i + 0] = 0.20f;
        g_edgeCol[i + 1] = 0.22f;
        g_edgeCol[i + 2] = 0.30f;
        g_edgeCol[i + 3] = 0.58f;
    }

    // Per-segment part ids (same numbering as SetMesh). Without them the outline pass
    // keeps drawing every segment even when its part is hidden - outlines floating in
    // mid-air. Ids outside the part range are clamped like the triangle groups are.
    if (groups != nullptr)
    {
        g_egroup.resize((size_t)nseg);
        for (int s = 0; s < nseg; s++)
        {
            int g = (s < groups->Length) ? groups[s] : 0;
            if (g < 0 || g >= g_groupCount) g = 0;
            g_egroup[s] = g;
        }
    }
    MarkSceneDirty();
}

void OcctViewer::SetEdgePolicy(int show, bool silhouetteOnly, bool drawBrep, bool drawMesh)
{
    if (show < 0) show = 0;
    if (show > 2) show = 2;
    g_edgeShow = show;
    g_edgeSilhouetteOnly = silhouetteOnly;
    g_edgeDrawBrep = drawBrep;
    g_edgeDrawMesh = drawMesh;
    MarkSceneDirty();
}

void OcctViewer::SetPoints(array<float>^ points)
{
    g_pointPos.clear();
    if (points == nullptr) return;
    int n = points->Length / 3;
    if (n <= 0) return;
    g_pointPos.resize((size_t)n * 3);
    for (int i = 0; i < n * 3; i++) g_pointPos[i] = points[i];
    MarkSceneDirty();
}

int OcctViewer::GroupCount()
{
    return g_groupCount;
}

void OcctViewer::SetGroupVisible(int group, bool visible)
{
    if (group < 0 || group >= (int)g_groupVisible.size()) return;
    char v = visible ? 1 : 0;
    if (g_groupVisible[(size_t)group] == v) return;
    g_groupVisible[(size_t)group] = v;

    // Showing a part must also show every ANCESTOR, otherwise the request is silently
    // swallowed: GroupEffectivelyVisible() declares a node hidden when ANY ancestor is off,
    // and "hide all" clears the assembly nodes too - so ticking a single leaf left it
    // invisible and the canvas stayed empty (only "show all" recovered it). Hiding a parent
    // still cascades down, because the children's own flags are left untouched.
    if (v)
    {
        // Walk up the parent chain. Bounds are clamped to the SHORTER of the two vectors:
        // a mesh without part info keeps g_partParent empty, and indexing it would be UB.
        size_t n = g_groupVisible.size();
        if (g_partParent.size() < n) n = g_partParent.size();
        int cur = ((size_t)group < g_partParent.size()) ? g_partParent[(size_t)group] : -1;
        size_t guard = 0;
        while (cur >= 0 && (size_t)cur < n && guard++ < n)
        {
            g_groupVisible[(size_t)cur] = 1;
            cur = g_partParent[(size_t)cur];
        }
    }

    g_idxVisValid = false;   // edges rebuild per frame while filtering is active
    MarkSceneDirty();
}

bool OcctViewer::GroupVisible(int group)
{
    if (group < 0 || group >= (int)g_groupVisible.size()) return false;
    return g_groupVisible[(size_t)group] != 0;
}

void OcctViewer::SetAllGroupsVisible(bool visible)
{
    char v = visible ? 1 : 0;
    for (size_t i = 0; i < g_groupVisible.size(); i++) g_groupVisible[i] = v;
    g_idxVisValid = false;
    MarkSceneDirty();
}

array<float>^ OcctViewer::GroupColors()
{
    // 右侧树节点色块要与 3D 实体「当前显示色」一致，因此按模式返回：
    //  - 实体模式 (g_alpha == 1.0)：复用 PartColor（kPartPalette 24 色板 / 单实体 SeasonalTint）；
    //  - 半透明 X 光模式 (g_alpha < 1.0)：用 g_groupColor —— SetMesh 里按 kGlassHues[8] 的 8 个
    //    基色循环铺满、并在 UpdateGlassColors 里做 0.24 提亮的那套，为与屏幕实际显示一致这里
    //    同样施加 0.24 提亮（单实体 g_groupCount<=0 时没有树，不进此分支）。
    // C# 侧每次切换透明度都会重新调用本方法，从而拿到对应模式的色。
    array<float>^ rgb = gcnew array<float>((size_t)g_groupCount * 3);
    const bool glass = (g_alpha < 1.0f - 1e-4f);
    const bool haveGlass = (g_groupColor.size() >= (size_t)g_groupCount * 3);
    for (int g = 0; g < g_groupCount; g++)
    {
        float r, gg, b;
        if (glass && haveGlass)
        {
            // 与 UpdateGlassColors 多实体分支一致的 0.24 提亮
            const float kLift = 0.24f;
            r  = g_groupColor[3 * g + 0]; r  += (1.0f - r)  * kLift;
            gg = g_groupColor[3 * g + 1]; gg += (1.0f - gg) * kLift;
            b  = g_groupColor[3 * g + 2]; b  += (1.0f - b)  * kLift;
        }
        else
        {
            PartColor(g, g_groupCount, r, gg, b);
        }
        rgb[(size_t)g * 3 + 0] = r;
        rgb[(size_t)g * 3 + 1] = gg;
        rgb[(size_t)g * 3 + 2] = b;
    }
    return rgb;
}

array<float>^ OcctViewer::GetCamera()
{
    // Expose the current free-orbit camera so the C# host can do screen<->world
    // picking / projection for measurement without touching GL internals.
    // eye = target + dist * F  (matches the gluLookAt call in Render).
    float fx = g_camF[0], fy = g_camF[1], fz = g_camF[2];
    float ex = g_tx + g_dist * fx, ey = g_ty + g_dist * fy, ez = g_tz + g_dist * fz;
    float aspect = (g_h > 0) ? (float)g_w / (float)g_h : 1.0f;
    array<float>^ c = gcnew array<float>(11);
    c[0] = ex; c[1] = ey; c[2] = ez;
    c[3] = g_tx; c[4] = g_ty; c[5] = g_tz;
    c[6] = g_camU[0]; c[7] = g_camU[1]; c[8] = g_camU[2];
    c[9]  = 45.0f * 3.14159265358979323846f / 180.0f;   // vertical FOV (radians)
    c[10] = aspect;
    return c;
}

void OcctViewer::SetTransparency(float alpha)
{
    if (alpha < 0.05f) alpha = 0.05f;
    if (alpha > 1.0f) alpha = 1.0f;
    g_alpha = alpha;
    // Push the new alpha into the existing color buffer's 4th component.
    for (int i = 0; i + 3 < (int)g_col.size(); i += 4) g_col[i + 3] = alpha;
    MarkSceneDirty();
}

void OcctViewer::Resize(int w, int h)
{
    int nw = (w > 0) ? w : 1;
    int nh = (h > 0) ? h : 1;
    if (nw == g_w && nh == g_h) return;
    g_w = nw;
    g_h = nh;
    // The retained frame belongs to the old size; EnsureAoResources rebuilds the targets
    // on the next Render, which invalidates it too.
    g_frameValid = false;
    MarkSceneDirty();
}

// Tight AABB fit, mirroring SoftRender3D.ComputeFitDistBox in the CPU backend.
// Screen mapping with gluPerspective(45deg): u - 0.5 = 0.5 * cx / (cz * tan * aspect),
// so requiring |u - 0.5| <= margin/2 gives cz >= |cx| / (tan * aspect * margin),
// and vertically cz >= |cy| / (tan * margin).  With eye = target + dist * F the
// camera depth of a corner is cz = dist - (p - target).F, hence
//     dist >= (p - target).F + |cx| / (tan * aspect * margin)
// for every one of the 8 corners.  g_camR has the opposite sign of the CPU
// backend's CamR, but only |cx| is used so the result is identical.
// The old bounding-sphere rule (dist = radius * 2.2) wastes a lot of screen on
// non-cubic models and can even clip a wide assembly on a narrow viewport.
float ComputeBoxFitDist(float tx, float ty, float tz, float& maxAlongF)
{
    const float tanHalf = 0.4142135624f;      // tan(45deg / 2)
    float aspect = (float)(g_w > 0 ? g_w : 1) / (float)(g_h > 0 ? g_h : 1);
    if (aspect < 1e-3f) aspect = 1.0f;
    const float margin = 0.9f;                // same as MeshPreviewProvider
    float invH = 1.0f / (tanHalf * aspect * margin);
    float invV = 1.0f / (tanHalf * margin);
    float d = g_radius * 0.05f;
    maxAlongF = -1e30f;
    for (int i = 0; i < 8; i++)
    {
        float x = (i & 1) ? g_bmax[0] : g_bmin[0];
        float y = (i & 2) ? g_bmax[1] : g_bmin[1];
        float z = (i & 4) ? g_bmax[2] : g_bmin[2];
        float rx = x - tx, ry = y - ty, rz = z - tz;
        float sf = rx * g_camF[0] + ry * g_camF[1] + rz * g_camF[2];
        float cx = rx * g_camR[0] + ry * g_camR[1] + rz * g_camR[2];
        float cy = rx * g_camU[0] + ry * g_camU[1] + rz * g_camU[2];
        if (sf > maxAlongF) maxAlongF = sf;
        float need = sf + invH * fabsf(cx);
        if (need > d) d = need;
        need = sf + invV * fabsf(cy);
        if (need > d) d = need;
    }
    return d;
}

void OcctViewer::Fit()
{
    // Framing ONLY: distance + target. Orientation is deliberately preserved.
    // UG/NX, ProE/Creo and SolidWorks never snap the model back to the iso view
    // when the user asks for "fit"; the old code called ResetBasis() here, so
    // every click on the toolbar's Fit button threw away the angle the user had
    // just spent dragging to find. ResetOrientation() is the separate "home" command.
    g_tx = g_cx; g_ty = g_cy; g_tz = g_cz;
    if (g_hasBounds)
    {
        float maxSf = 0.0f;
        float d = ComputeBoxFitDist(g_tx, g_ty, g_tz, maxSf);
        // Keep every corner comfortably in front of the near plane
        // (nearZ = radius * 0.01); a very flat model could otherwise end up
        // with its nearest corner almost on the eye.
        float minCz = d - maxSf;
        float safeCz = g_radius * 0.05f;
        if (minCz < safeCz) d += safeCz - minCz;
        g_dist = d;
    }
    else
    {
        g_dist = g_radius * 2.2f;
    }
    MarkSceneDirty();
}

// Explicit "home view": restore the default orientation WITHOUT changing distance
// or target, so the current zoom / pan survives. New geometry calls ResetBasis()
// directly from SetMesh(); this is the user-triggered counterpart.
void OcctViewer::ResetOrientation()
{
    ResetBasis();
    MarkSceneDirty();
}

// Free orbit (UG/NX, ProE/Creo, SolidWorks style).
// The two rotation axes are the CURRENT SCREEN axes, not a fixed world axis:
//   horizontal drag -> rotate around the current screen-up vector (g_camU)
//   vertical drag   -> rotate around the current screen-horizontal vector (g_camR)
// Because the axes travel with the camera, there is no pole: every direction can be
// rotated through repeatedly with no clamp, and the model surface always follows the
// hand. The old spherical az/el camera clamped pitch at +/-89 deg, which is exactly
// why dragging eventually "would not turn any further", and near that pole a
// horizontal drag degenerated into spinning the model in place.
//
// Signs are inherited from the old formula and verified equivalent at the default
// view: daz = -dx*k gives dF = +dx*k*cos(el)*R, i.e. a -dx*k rotation about U
// (since U x F = -R); del = +dy*k gives dF = +dy*k*U, i.e. a +dy*k rotation
// about R (since R x F = U). The cos(el) factor is intentionally dropped: keeping
// a CONSTANT angular speed is the point of free orbit - it removes the pole-region
// speed-up that made the old camera feel erratic.
// (dx>0 = mouse right, dy>0 = mouse down; screen Y grows downward.)
void OcctViewer::Orbit(float dx, float dy)
{
    const float k = 0.01f;   // mirrored in SoftRender3D.cs Orbit() - same drag speed on both backends
    if (dx != 0.0f) RotateBasis(g_camU, -dx * k);
    if (dy != 0.0f) RotateBasis(g_camR,  dy * k);
    MarkSceneDirty();
}

// Drag-time quality tradeoff. A 2.5M-triangle assembly plus the SSAO chain cannot
// redraw at mouse rate; mature viewers drop expensive effects while the user drags
// and restore them on mouse-up. Keep MSAA (ResolveMsaaOnly) so edges stay clean.
void OcctViewer::SetInteracting(bool on)
{
    if (g_interacting == on) return;
    g_interacting = on;
    MarkSceneDirty();
}

void OcctViewer::Zoom(float factor)
{
    g_dist /= factor;
    if (g_dist < g_radius * 0.05f) g_dist = g_radius * 0.05f;
    if (g_dist > g_radius * 50.0f) g_dist = g_radius * 50.0f;
    MarkSceneDirty();
}

void OcctViewer::Pan(float dx, float dy)
{
    // Use the camera's CURRENT screen axes. Deriving them from a fixed worldUp (the
    // old code) breaks as soon as the camera rolls past a pole: right/up flip and
    // panning suddenly moves the model the wrong way.
    float scale = g_dist / (float)(g_h > 0 ? g_h : 1);
    g_tx += (g_camR[0] * dx + g_camU[0] * dy) * scale;
    g_ty += (g_camR[1] * dx + g_camU[1] * dy) * scale;
    g_tz += (g_camR[2] * dx + g_camU[2] * dy) * scale;
    // The retained-frame fast path in Render() blits the previous frame while
    // g_sceneDirty is false. Pan moves the camera target, so the frame MUST be
    // marked dirty - without this the view never updates (pan appears dead).
    MarkSceneDirty();
}

void OcctViewer::Render()
{
    if (!g_hglrc || !g_hdc) return;
    ::wglMakeCurrent(g_hdc, g_hglrc);

    // Nothing changed since the last frame: this WM_PAINT is a pure repaint (window
    // uncovered, focus change, skin repaint). Blit the retained frame instead of
    // re-rendering - no clearing, no dependence on triangle count.
    if (!g_sceneDirty && PresentRetainedFrame()) { g_retainedPresents++; return; }

    // Draw the scene into the multisampled FBO when the AO chain is available; otherwise
    // straight to the window exactly as before. g_aoRequested is the host's on/off switch
    // (preview toolbar); when off we skip the whole AO chain but keep its GL resources, so
    // switching back on is free and does not need a context rebuild.
    bool aoChain = EnsureAoResources(g_w, g_h);
    // Translucent frames do not write depth, so AO has nothing to sample (see ResolveMsaaOnly).
    bool ao = aoChain && g_aoRequested && g_alpha >= 0.999f && !g_interacting;
    if (aoChain) pglBindFramebuffer(OGL_FRAMEBUFFER, g_msaaFbo);
    glViewport(0, 0, g_w, g_h);
    // Translucent frames get a whisper of cool grey instead of pure white. Glass over
    // white has almost no contrast: every thin layer washes out and the model reads as
    // milk. 3-6% grey is invisible as "a grey background" but restores the contrast the
    // thin layers need.
    if (g_alpha < 0.999f) glClearColor(0.938f, 0.949f, 0.965f, 1.0f);
    else                 glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)g_w / (float)g_h;
    float nearZ = g_radius * 0.01f, farZ = g_radius * 100.0f;
    gluPerspective(45.0f, aspect, nearZ, farZ);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // Free-orbit camera: eye = target + dist * F, and the VIEW UP is the camera's
    // own U - NOT a hardcoded world (0,1,0). Passing a fixed world up is what made
    // the old camera gimbal-lock at the poles; U rolls with the model so the frame
    // stays well-defined at every orientation.
    float fx = g_camF[0], fy = g_camF[1], fz = g_camF[2];
    float ex = g_tx + g_dist * fx, ey = g_ty + g_dist * fy, ez = g_tz + g_dist * fz;
    gluLookAt(ex, ey, ez, g_tx, g_ty, g_tz,
              g_camU[0], g_camU[1], g_camU[2]);

    // Camera basis for the three headlights - same R/U as panning, so lighting keeps
    // riding with the eye even after the view has been flipped upside down.
    float rx = g_camR[0], ry = g_camR[1], rz = g_camR[2];
    float ux = g_camU[0], uy = g_camU[1], uz = g_camU[2];

    // "Shoulder light": rides with the camera but offset to a fixed angle (up and to
    // the left) instead of sitting on the eye axis. The light->surface->eye angle is
    // therefore never near zero, so the highlight can never reflect straight back at
    // the viewer -> no blown-out white patch, and the form reads consistently at
    // every orbit angle. LIGHT1 is a weaker fill from the opposite side.
    GLfloat lightPos[4] = {
        ex + g_radius * (-0.60f * rx + 0.80f * ux + 0.45f * fx),
        ey + g_radius * (-0.60f * ry + 0.80f * uy + 0.45f * fy),
        ez + g_radius * (-0.60f * rz + 0.80f * uz + 0.45f * fz), 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    GLfloat lightPos1[4] = {
        ex + g_radius * (0.75f * rx - 0.30f * ux + 0.40f * fx),
        ey + g_radius * (0.75f * ry - 0.30f * uy + 0.40f * fy),
        ez + g_radius * (0.75f * rz - 0.30f * uz + 0.40f * fz), 1.0f };
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);

    // Rim light rides behind the camera axis (-f) and above (+u): it grazes the silhouette and
    // the top faces, so adjacent parts stop merging into one flat colour silhouette.
    GLfloat lightPos2[4] = {
        ex + g_radius * (0.15f * rx + 0.70f * ux - 0.85f * fx),
        ey + g_radius * (0.15f * ry + 0.70f * uy - 0.85f * fy),
        ez + g_radius * (0.15f * rz + 0.70f * uz - 0.85f * fz), 1.0f };
    glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);

    if (g_hasMesh && !g_idx.empty())
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, g_pos.data());
        glNormalPointer(GL_FLOAT, 0, g_nrm.data());
        glColorPointer(4, GL_FLOAT, 0, g_col.data());

        // Visible parts only. While nothing is hidden this IS g_idx (no copy); the first
        // time a part is switched off it becomes a compacted buffer without those triangles.
        GLsizei    idxCount = 0;
        const int* idxData  = VisibleIndices(idxCount);

        if (g_alpha < 0.999f)
        {
            // Two-layer glass (see UpdateGlassColors). Order matters: the inner walls go
            // down first, then the outer membrane composites over them. That is the pass
            // order the eye expects, and it is what turns the model from a flat veil into
            // "bright shell around a darker body".
            UpdateGlassColors(ex, ey, ez);

            // Aerial perspective. A thin-walled assembly sends the view ray through the
            // same part dozens of times; no per-layer alpha can stop that from stacking up
            // into a dark slab, because 30 layers of even 0.12 compose to ~0.98 opacity.
            // What the eye actually uses to read depth is CONTRAST, not opacity, so parts
            // deep inside fade towards the background colour. The result keeps the
            // silhouette crisp, lets the near parts read as near, and stops the interior
            // from turning into mud.
            glEnable(GL_FOG);
            glFogi(GL_FOG_MODE, GL_LINEAR);
            GLfloat fogCol[4] = { 0.938f, 0.949f, 0.965f, 1.0f };
            glFogfv(GL_FOG_COLOR, fogCol);
            glFogf(GL_FOG_START, g_dist - g_radius * 0.25f);
            glFogf(GL_FOG_END, g_dist + g_radius * 1.55f);

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);            // translucent: sort by pass, not by depth write
            glEnable(GL_CULL_FACE);

            // ---- pass 1: inner walls (back faces = the surfaces facing away from the eye)
            GLfloat wallSpec[4] = { 0.10f, 0.10f, 0.11f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, wallSpec);
            glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 16);
            glColorPointer(4, GL_FLOAT, 0, g_colBack.data());
            glCullFace(GL_FRONT);
            glDrawElements(GL_TRIANGLES, idxCount, GL_UNSIGNED_INT, idxData);

            // ---- pass 2: outer membrane (front faces). Hard, tight highlight: a glass
            // shell without a crisp specular reads like coloured plastic film.
            GLfloat shellSpec[4] = { 0.95f, 0.96f, 1.00f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, shellSpec);
            glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 88);
            glColorPointer(4, GL_FLOAT, 0, g_colFront.data());
            glCullFace(GL_BACK);
            glDrawElements(GL_TRIANGLES, idxCount, GL_UNSIGNED_INT, idxData);

            // (edge / outline pass moved to the common tail below, drawn in BOTH
            //  opaque and translucent modes - see the block right after this g_hasMesh block)

            glDisable(GL_CULL_FACE);
            glDisable(GL_BLEND);
            glDisable(GL_FOG);
            glDepthMask(GL_TRUE);
            glColorPointer(4, GL_FLOAT, 0, g_col.data());   // back to the palette colours

            GLfloat specRef[4] = { 0.30f, 0.30f, 0.30f, 1.0f };   // restore the opaque look
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specRef);
            glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 28);
        }
        else
        {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
            glDrawElements(GL_TRIANGLES, idxCount, GL_UNSIGNED_INT, idxData);
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
    }

    // ---- common tail: mesh lines (wireframe / polyline) + point cloud ----
    // Edge overlay follows a user policy (SetEdgePolicy), independent of face
    // transparency: show = off / only-translucent (default; previous behaviour) /
    // always; optionally silhouettes only; and independently for B-rep outlines vs
    // mesh-file lines.
    bool edgeWant  = (g_edgeShow == 2) || (g_edgeShow == 1 && g_alpha < 0.999f);
    bool edgeIsBrep = (g_egroup.size() == g_edgePos.size() / 6);
    bool edgeSrcOk  = edgeIsBrep ? g_edgeDrawBrep : g_edgeDrawMesh;
    if (!g_edgePos.empty() && g_edgeCol.size() == g_edgePos.size() / 3 * 4
        && edgeWant && edgeSrcOk)
    {
        const float* ePos   = g_edgePos.data();
        const float* eCol   = g_edgeCol.data();
        GLsizei      eCount = (GLsizei)(g_edgePos.size() / 3);
        if (PartFilterActive() && g_egroup.size() == g_edgePos.size() / 6)
        {
            RebuildVisibleEdges();
            ePos   = g_edgePosVis.data();
            eCol   = g_edgeColVis.data();
            eCount = (GLsizei)(g_edgePosVis.size() / 3);
        }
        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glDisableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, ePos);
        glColorPointer(4, GL_FLOAT, 0, eCol);
        glDrawArrays(GL_LINES, 0, eCount);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnable(GL_LIGHTING);
    }
    if (!g_pointPos.empty())
    {
        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, g_pointPos.data());
        // fixed dark-slate colour, opaque
        std::vector<float> pcol(g_pointPos.size() / 3 * 4);
        for (size_t i = 0; i + 3 < pcol.size(); i += 4)
        { pcol[i] = 0.20f; pcol[i + 1] = 0.22f; pcol[i + 2] = 0.30f; pcol[i + 3] = 1.0f; }
        glColorPointer(4, GL_FLOAT, 0, pcol.data());
        glPointSize(3.0f);
        glDrawArrays(GL_POINTS, 0, (GLsizei)(g_pointPos.size() / 3));
        glPointSize(1.0f);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
        glEnable(GL_LIGHTING);
    }

    if (ao)
    {
        // Resolve the MSAA buffer, compute ambient occlusion from the depth buffer and
        // composite the two back to the window (see RunAoAndComposite).
        RunAoAndComposite(nearZ, farZ, 0.41421356f /* tan(45/2 deg) */, aspect);
    }
    else if (aoChain)
    {
        ResolveMsaaOnly();   // keep the 4x anti-aliasing, skip the AO cost
    }

    // The picture is complete in g_presentTex now (composite / blit wrote there), so this
    // frame is safe to keep and to re-present for any later repaint.
    g_fullRenders++;
    if (g_retainEnabled && g_presentFbo) g_frameValid = true;
    g_sceneDirty = false;
    if (PresentRetainedFrame()) return;

    glFlush();
    ::SwapBuffers(g_hdc);
}

bool OcctViewer::IsReady() { return g_hglrc != NULL; }

int OcctViewer::MsaaSamples() { return g_msaaSamples; }

void OcctViewer::SetAoEnabled(bool on) { g_aoRequested = on; MarkSceneDirty(); }

bool OcctViewer::AoEnabled() { return g_aoRequested; }

bool OcctViewer::SaveFrame(System::String^ path)
{
    if (!g_hglrc || !g_hdc || g_w <= 0 || g_h <= 0) return false;
    ::wglMakeCurrent(g_hdc, g_hglrc);

    int w = g_w, h = g_h;
    size_t rowBytes = (size_t)w * 3;
    size_t rowStride = (rowBytes + 3) & ~(size_t)3;      // BMP rows are 4-byte aligned
    size_t dataSize = rowStride * (size_t)h;

    std::vector<unsigned char> px(rowBytes * (size_t)h);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    // Read the frame that is actually on screen. With retained presentation that frame
    // lives in g_presentFbo: the window's buffers are undefined right after SwapBuffers,
    // and the front buffer is not readable while the window is occluded or minimised.
    if (g_frameValid && g_presentFbo)
    {
        pglBindFramebuffer(OGL_READ_FRAMEBUFFER, g_presentFbo);
        glReadBuffer(OGL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, &px[0]);
        pglBindFramebuffer(OGL_READ_FRAMEBUFFER, 0);
    }
    else
    {
        glReadBuffer(GL_FRONT);                          // window front buffer: what the user sees
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, &px[0]);
    }

    // Build a 24-bit bottom-up BMP (glReadPixels already returns rows bottom-up, which is
    // exactly BMP's order, so no vertical flip is needed).
    array<System::Byte>^ out = gcnew array<System::Byte>((int)(54 + dataSize));
    pin_ptr<System::Byte> p = &out[0];
    unsigned char* o = (unsigned char*)p;

    unsigned char* fh = o;                               // BITMAPFILEHEADER (14 bytes)
    fh[0] = 'B'; fh[1] = 'M';
    unsigned int fileSize = (unsigned int)(54 + dataSize);
    fh[2] = (unsigned char)(fileSize); fh[3] = (unsigned char)(fileSize >> 8);
    fh[4] = (unsigned char)(fileSize >> 16); fh[5] = (unsigned char)(fileSize >> 24);
    fh[10] = 54;                                         // pixel data offset

    unsigned char* ih = o + 14;                          // BITMAPINFOHEADER (40 bytes)
    ih[0] = 40;
    ih[4] = (unsigned char)(w); ih[5] = (unsigned char)(w >> 8);
    ih[6] = (unsigned char)(w >> 16); ih[7] = (unsigned char)(w >> 24);
    ih[8] = (unsigned char)(h); ih[9] = (unsigned char)(h >> 8);
    ih[10] = (unsigned char)(h >> 16); ih[11] = (unsigned char)(h >> 24);
    ih[12] = 1;                                          // planes
    ih[14] = 24;                                         // bpp
    unsigned int imgSize = (unsigned int)dataSize;
    ih[20] = (unsigned char)(imgSize); ih[21] = (unsigned char)(imgSize >> 8);
    ih[22] = (unsigned char)(imgSize >> 16); ih[23] = (unsigned char)(imgSize >> 24);

    for (int y = 0; y < h; y++)
    {
        unsigned char* dst = o + 54 + rowStride * (size_t)y;
        const unsigned char* src = &px[0] + rowBytes * (size_t)y;
        for (int x = 0; x < w; x++)
        {
            // BMP stores BGR
            dst[3 * x + 0] = src[3 * x + 2];
            dst[3 * x + 1] = src[3 * x + 1];
            dst[3 * x + 2] = src[3 * x + 0];
        }
    }

    try
    {
        System::IO::File::WriteAllBytes(path, out);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

System::String^ OcctViewer::Diagnostics()
{
    if (!g_hglrc || !g_hdc) return "no-context";
    ::wglMakeCurrent(g_hdc, g_hglrc);
    const char* ven = (const char*)glGetString(GL_VENDOR);
    const char* ren = (const char*)glGetString(GL_RENDERER);
    const char* ver = (const char*)glGetString(GL_VERSION);
    System::String^ ao = "unsupported";
    if (g_aoReady)
        ao = System::String::Format("ready px={0:0.#} cap={1:0.###} str={2:0.##} enabled={3}",
             g_aoParams.radiusPx, g_aoParams.radiusCap, g_aoParams.strength,
             g_aoRequested ? 1 : 0);
    return System::String::Format(
        "vendor={0}; renderer={1}; version={2}; msaa={3}x; radius={4:0.###}; dist={5:0.###}; ao={6};"
        + " retain={7} valid={8} dirty={9} {10}x{11} full={12} keep={13};",
        gcnew System::String(ven ? ven : "?"),
        gcnew System::String(ren ? ren : "?"),
        gcnew System::String(ver ? ver : "?"),
        g_msaaSamples, g_radius, g_dist, ao,
        g_retainEnabled ? "on" : "off", g_frameValid ? 1 : 0, g_sceneDirty ? 1 : 0,
        g_presentW, g_presentH, g_fullRenders, g_retainedPresents);
}
