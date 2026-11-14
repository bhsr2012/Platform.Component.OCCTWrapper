// Dwg2DViewer.cpp - native 2D CAD renderer (GL 1.1 fixed pipeline + GDI glyph atlas).
// All comments are ASCII on purpose: this file is compiled with the GBK/936
// codepage, and UTF-8 multibyte tails decode to '\' which turns a trailing
// "//" comment into a line-continuation and eats the next line.
#include "Dwg2DViewer.h"

#include <vector>
#include <map>
#include <set>
#include <string>
#include <cmath>
#include <vcclr.h>

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")

using namespace System;
using namespace System::Runtime::InteropServices;

namespace Platform { namespace Component { namespace OCCTWrapper {

namespace {

// ---------------------------------------------------------------------------
// GL context (dummy-window WGL ARB probe, same approach as OcctViewer)
// ---------------------------------------------------------------------------
HDC  g_hdc = NULL;
HGLRC g_hglrc = NULL;
HWND g_hwnd = NULL;
int  g_winW = 0, g_winH = 0;
int  g_msaaSamples = 0;

#define WGL_DRAW_TO_WINDOW_ARB 0x2001
#define WGL_SUPPORT_OPENGL_ARB 0x2010
#define WGL_DOUBLE_BUFFER_ARB  0x2011
#define WGL_PIXEL_TYPE_ARB     0x2013
#define WGL_TYPE_RGBA_ARB      0x202B
#define WGL_COLOR_BITS_ARB     0x2014
#define WGL_DEPTH_BITS_ARB     0x2022
#define WGL_STENCIL_BITS_ARB   0x2023
#define WGL_SAMPLE_BUFFERS_ARB 0x2041
#define WGL_SAMPLES_ARB        0x2042

typedef BOOL (WINAPI * PFN_WGLCHOOSEPIXELFORMATARB)(HDC, const int*, const FLOAT*, UINT, int*, UINT*);
PFN_WGLCHOOSEPIXELFORMATARB g_choosePF = NULL;

bool LoadWglArbEntryPoints()
{
    if (g_choosePF) return true;
    WNDCLASSA wc = { 0 };
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = ::DefWindowProcA;
    wc.hInstance = ::GetModuleHandleA(NULL);
    wc.lpszClassName = "Dwg2DViewerProbeGL";
    ::RegisterClassA(&wc);
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
    g_choosePF = (PFN_WGLCHOOSEPIXELFORMATARB)::wglGetProcAddress("wglChoosePixelFormatARB");
    ::wglMakeCurrent(NULL, NULL);
    ::wglDeleteContext(probeRc);
    ::ReleaseDC(probe, probeDc);
    ::DestroyWindow(probe);
    return g_choosePF != NULL;
}

int ChooseMultiSampleFormat(HDC hdc)
{
    if (!g_choosePF) return 0;
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
        if (g_choosePF(hdc, attribs, NULL, 1, &fmt, &n) && n > 0 && fmt != 0)
            return fmt;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// View transform
// ---------------------------------------------------------------------------
double g_cx = 0.0, g_cy = 0.0;      // world coords at window centre
double g_scale = 1.0;               // pixels per world unit
float  g_minX = 0, g_minY = 0, g_maxX = 100, g_maxY = 100;
int    g_bgIndex = 1;               // 0=white 1=black 2=dark grey 3=light grey

// ---------------------------------------------------------------------------
// Scene data
// ---------------------------------------------------------------------------
struct PathMeta
{
    int start;            // first vertex index (vertex = 2 floats in g_pathXY)
    int count;            // vertex count
    float width;          // world units; 0 = hairline
    unsigned char layer;
    unsigned char rgba[4];
};
std::vector<float>    g_pathXY;
std::vector<PathMeta> g_paths;

struct FillMeta
{
    int triStart;         // first triangle index (3 verts = 6 floats in g_fillXY)
    int triCount;
    unsigned char layer;
    unsigned char rgba[4];
    float minX, minY, maxX, maxY;
};
std::vector<float>    g_fillXY;
std::vector<FillMeta> g_fills;

struct TextMeta
{
    float x, y, height, rotDeg;
    int halign, valign;
    unsigned char layer;
    unsigned char rgba[4];
    std::wstring text;
};
std::vector<TextMeta> g_texts;

std::vector<std::wstring> g_layerNames;
std::vector<bool>         g_layerVisible;

void g_atlasReset();   // defined in the glyph atlas section below
void ResetOverlay();   // defined next to ClearScene

// Hairline batch buckets: key = layer<<32 | rgba. Each bucket owns a contiguous
// GL_LINES segment array (4 floats per segment) so a whole bucket draws with a
// single glDrawArrays call - that is what keeps 3M-line layouts interactive.
struct Bucket
{
    unsigned char layer;
    unsigned char rgba[4];
    std::vector<float> segs;   // x1,y1,x2,y2 per segment
};
std::map<unsigned long long, int> g_bucketIndex;
std::vector<Bucket> g_buckets;

// Wide (non-hairline) paths, drawn per path with glLineWidth
std::vector<int> g_widePaths;

// ---- overlay (markup / measurement) ----
struct OvMeta
{
    int start;            // first vertex index in g_ovXY
    int count;            // vertex count
    float widthPx;        // screen-space line width in pixels
    float dashPx;         // dash length in pixels (<=0 = solid)
    unsigned char rgba[4];
};
std::vector<float>  g_ovXY;
std::vector<OvMeta> g_ovPaths;
std::vector<float>  g_ovFillXY;
std::vector<FillMeta> g_ovFills;
std::vector<TextMeta> g_otexts;

// background: custom RGB overrides the preset slot
bool  g_bgCustom = false;
float g_bgR = 0.0f, g_bgG = 0.0f, g_bgB = 0.0f;

unsigned long long BucketKey(unsigned char layer, const unsigned char* rgba)
{
    unsigned long long k = (unsigned long long)layer << 32;
    k |= ((unsigned long long)rgba[0] << 24) | ((unsigned long long)rgba[1] << 16)
       | ((unsigned long long)rgba[2] << 8) | (unsigned long long)rgba[3];
    return k;
}

void ClearScene()
{
    g_pathXY.clear();
    g_paths.clear();
    g_fillXY.clear();
    g_fills.clear();
    g_texts.clear();
    g_bucketIndex.clear();
    g_buckets.clear();
    g_widePaths.clear();
    g_layerNames.clear();
    g_layerVisible.clear();
    ResetOverlay();
    g_atlasReset();
}

void ResetOverlay()
{
    g_ovXY.clear();
    g_ovPaths.clear();
    g_ovFillXY.clear();
    g_ovFills.clear();
    g_otexts.clear();
    g_atlasReset();
}

void RebuildBuckets()
{
    g_bucketIndex.clear();
    g_buckets.clear();
    g_widePaths.clear();
    for (size_t i = 0; i < g_paths.size(); i++)
    {
        const PathMeta& p = g_paths[i];
        if (p.count < 2) continue;
        if (p.width > 0.0f)
        {
            g_widePaths.push_back((int)i);
            continue;
        }
        unsigned long long key = BucketKey(p.layer, p.rgba);
        std::map<unsigned long long, int>::iterator it = g_bucketIndex.find(key);
        int bi;
        if (it == g_bucketIndex.end())
        {
            Bucket b;
            b.layer = p.layer;
            memcpy(b.rgba, p.rgba, 4);
            g_buckets.push_back(b);
            bi = (int)g_buckets.size() - 1;
            g_bucketIndex[key] = bi;
        }
        else bi = it->second;
        Bucket& bk = g_buckets[bi];
        const float* v = &g_pathXY[(size_t)p.start * 2];
        for (int s = 0; s < p.count - 1; s++)
        {
            bk.segs.push_back(v[(size_t)s * 2 + 0]);
            bk.segs.push_back(v[(size_t)s * 2 + 1]);
            bk.segs.push_back(v[(size_t)s * 2 + 2]);
            bk.segs.push_back(v[(size_t)s * 2 + 3]);
        }
    }
}

// ---------------------------------------------------------------------------
// Glyph atlas (GDI text rendering into a memory DIB -> alpha texture)
// ---------------------------------------------------------------------------
const int ATLAS_SIZE = 1024;
const int GLYPH_PX = 96;          // render glyphs at 96px, scale at draw time

struct Glyph
{
    float u0, v0, u1, v1;         // atlas UVs (v0 = bottom in GL space)
    float adv;                    // advance in atlas px
    float ox, oy;                 // bitmap origin relative to baseline (px, up+)
    float w, h;                   // bitmap size (px)
    int   page;                   // index into g_atlas (-1 = no bitmap)
};
struct AtlasTex
{
    GLuint tex;
    int cursorX, cursorY, rowH;
};
std::vector<AtlasTex> g_atlas;
std::map<wchar_t, Glyph> g_glyphs;
bool   g_atlasBuilt = false;
HDC    g_glyphDC = NULL;
HFONT  g_glyphFont = NULL;
int    g_tmAscent = 0, g_tmDescent = 0;

// Glyph rasterization via GDI text rendering into a memory DIB.
// MEASURED on this machine: GetGlyphOutlineW(GGO_GRAY8_BITMAP) returns a
// correctly-sized buffer whose contents are empty (every row held a single
// byte of value 4 for a CJK glyph - no usable anti-aliased bitmap at all),
// while plain GDI text output into a 32bpp DIB produces correct coverage
// (2302 non-zero pixels for U+4E2D in Microsoft YaHei). So the atlas is
// built from TextOutW output instead of GGO.
const int GLYPH_DIB_W = 256;   // wide enough for a GLYPH_PX glyph + advance
const int GLYPH_DIB_H = 192;   // tall enough for ascent + descent + slack
const int GLYPH_PAD_X = 16;    // left padding inside the DIB
HBITMAP g_glyphDib = NULL;
HBITMAP g_glyphDibOld = NULL;
void*    g_glyphDibBits = NULL;

void g_atlasReset()
{
    if (g_glyphDibOld && g_glyphDC) { ::SelectObject(g_glyphDC, g_glyphDibOld); g_glyphDibOld = NULL; }
    if (g_glyphDib) { ::DeleteObject(g_glyphDib); g_glyphDib = NULL; g_glyphDibBits = NULL; }
    if (g_glyphFont) { ::DeleteObject(g_glyphFont); g_glyphFont = NULL; }
    if (g_glyphDC)   { ::DeleteDC(g_glyphDC);     g_glyphDC = NULL; }
    g_atlas.clear();
    g_glyphs.clear();
    g_atlasBuilt = false;
    g_tmAscent = g_tmDescent = 0;
}

// Rasterize one character with GDI text output. Returns false when the glyph
// has no ink (space / missing). cov is top-down coverage (0..255), ox/oy are
// the bitmap origin relative to the baseline (y up), adv the cell advance -
// all in atlas px. dc/bits are passed explicitly so the font probe can call
// this before the globals are committed.
bool RasterizeGlyph(HDC dc, void* bits, wchar_t ch, std::vector<unsigned char>& cov,
                    int& w, int& h, float& ox, float& oy, float& adv)
{
    if (!dc || !bits) return false;
    memset(bits, 0, (size_t)GLYPH_DIB_W * GLYPH_DIB_H * 4);
    ::SetTextAlign(dc, TA_LEFT | TA_TOP);
    ::SetBkColor(dc, RGB(0, 0, 0));
    ::SetTextColor(dc, RGB(255, 255, 255));
    if (!::TextOutW(dc, GLYPH_PAD_X, 0, &ch, 1)) return false;

    const unsigned char* px = (const unsigned char*)bits;
    int bx0 = GLYPH_DIB_W, by0 = GLYPH_DIB_H, bx1 = -1, by1 = -1;
    for (int y = 0; y < GLYPH_DIB_H; y++)
    {
        const unsigned char* row = px + (size_t)y * GLYPH_DIB_W * 4;
        for (int x = 0; x < GLYPH_DIB_W; x++)
        {
            if (row[x * 4 + 2] > 0)          // R channel of the top-down DIB
            {
                if (x < bx0) bx0 = x;
                if (x > bx1) bx1 = x;
                if (y < by0) by0 = y;
                if (y > by1) by1 = y;
            }
        }
    }
    SIZE sz;
    if (!::GetTextExtentPoint32W(dc, &ch, 1, &sz)) sz.cx = GLYPH_PX / 2;
    adv = (float)sz.cx;
    if (bx1 < bx0) return false;             // no ink: advance only

    w = bx1 - bx0 + 1;
    h = by1 - by0 + 1;
    cov.resize((size_t)w * (size_t)h);
    for (int y = 0; y < h; y++)
    {
        const unsigned char* row = px + (size_t)(by0 + y) * GLYPH_DIB_W * 4;
        for (int x = 0; x < w; x++)
            cov[(size_t)y * (size_t)w + x] = row[(size_t)(bx0 + x) * 4 + 2];
    }
    ox = (float)(bx0 - GLYPH_PAD_X);         // bitmap left edge vs pen position
    oy = (float)(g_tmAscent - by0);          // bitmap top edge vs baseline (y up)
    return true;
}

// Pick a font that can actually render CJK (probe by rasterizing U+4E2D into
// the DIB and checking for ink). Returns true on success.
bool EnsureGlyphFont()
{
    if (g_glyphDC && g_glyphFont && g_glyphDib) return true;
    const wchar_t* faces[] = { L"Microsoft YaHei", L"SimSun", L"MS Gothic", L"Arial Unicode MS" };
    for (int f = 0; f < 4 && !g_glyphFont; f++)
    {
        HDC dc = ::CreateCompatibleDC(NULL);
        if (!dc) continue;
        HFONT font = ::CreateFontW(
            -GLYPH_PX, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
            DEFAULT_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
            ANTIALIASED_QUALITY, DEFAULT_PITCH | FF_DONTCARE, faces[f]);
        if (!font) { ::DeleteDC(dc); continue; }
        HGDIOBJ old = ::SelectObject(dc, font);

        // 32bpp top-down DIB for glyph rasterization
        BITMAPINFO bi;
        memset(&bi, 0, sizeof(bi));
        bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth = GLYPH_DIB_W;
        bi.bmiHeader.biHeight = -GLYPH_DIB_H;
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 32;
        bi.bmiHeader.biCompression = BI_RGB;
        void* bits = NULL;
        HBITMAP dib = ::CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, NULL, 0);
        if (!dib || !bits)
        {
            if (dib) ::DeleteObject(dib);
            ::SelectObject(dc, old);
            ::DeleteObject(font);
            ::DeleteDC(dc);
            continue;
        }
        HBITMAP dibOld = (HBITMAP)::SelectObject(dc, dib);

        TEXTMETRICW tm;
        ::GetTextMetricsW(dc, &tm);

        // probe: render U+4E2D and require actual ink
        std::vector<unsigned char> cov;
        int w = 0, h = 0; float ox = 0, oy = 0, adv = 0;
        bool hasInk = RasterizeGlyph(dc, bits, 0x4E2D, cov, w, h, ox, oy, adv);

        if (!hasInk)
        {
            ::SelectObject(dc, dibOld);
            ::DeleteObject(dib);
            ::SelectObject(dc, old);
            ::DeleteObject(font);
            ::DeleteDC(dc);
            continue;
        }
        g_tmAscent = tm.tmAscent;
        g_tmDescent = tm.tmDescent;
        g_glyphFont = font;
        g_glyphDC = dc;
        g_glyphDib = dib;
        g_glyphDibOld = dibOld;
        g_glyphDibBits = bits;
        return true;
    }
    return false;
}

bool BuildGlyphs()
{
    if (g_atlasBuilt) return true;
    if (!EnsureGlyphFont()) return false;

    AtlasTex first;
    glGenTextures(1, &first.tex);
    glBindTexture(GL_TEXTURE_2D, first.tex);
    std::vector<unsigned char> blank(ATLAS_SIZE * ATLAS_SIZE * 4, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ATLAS_SIZE, ATLAS_SIZE, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, &blank[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    first.cursorX = 0; first.cursorY = 0; first.rowH = 0;
    g_atlas.push_back(first);

    // unique characters across all texts (scene + overlay)
    std::set<wchar_t> chars;
    for (size_t i = 0; i < g_texts.size(); i++)
        for (size_t c = 0; c < g_texts[i].text.size(); c++)
        {
            wchar_t ch = g_texts[i].text[c];
            if (ch == L'\n' || ch == L'\r' || ch == L'\t') continue;
            chars.insert(ch);
        }
    for (size_t i = 0; i < g_otexts.size(); i++)
        for (size_t c = 0; c < g_otexts[i].text.size(); c++)
        {
            wchar_t ch = g_otexts[i].text[c];
            if (ch == L'\n' || ch == L'\r' || ch == L'\t') continue;
            chars.insert(ch);
        }

    HGDIOBJ oldFont = ::SelectObject(g_glyphDC, g_glyphFont);

    for (std::set<wchar_t>::iterator it = chars.begin(); it != chars.end(); ++it)
    {
        wchar_t ch = *it;
        std::vector<unsigned char> cov;   // top-down coverage 0..255
        int bw = 0, bh = 0;
        float gox = 0, goy = 0, gadv = 0;
        bool hasInk = RasterizeGlyph(g_glyphDC, g_glyphDibBits, ch, cov, bw, bh, gox, goy, gadv);
        if (!hasInk)
        {
            // blank glyph (space / missing): advance only
            Glyph g;
            g.u0 = g.v0 = g.u1 = g.v1 = 0.0f;
            g.adv = gadv;
            g.ox = 0; g.oy = 0; g.w = 0; g.h = 0;
            g.page = -1;
            g_glyphs[ch] = g;
            continue;
        }

        // place into the current atlas texture (shelf packing)
        if (g_atlas.back().cursorX + bw > ATLAS_SIZE)
        {
            g_atlas.back().cursorX = 0;
            g_atlas.back().cursorY += g_atlas.back().rowH + 2;
            g_atlas.back().rowH = 0;
        }
        if (g_atlas.back().cursorY + bh > ATLAS_SIZE)
        {
            AtlasTex t;
            glGenTextures(1, &t.tex);
            glBindTexture(GL_TEXTURE_2D, t.tex);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ATLAS_SIZE, ATLAS_SIZE, 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, &blank[0]);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            t.cursorX = 0; t.cursorY = 0; t.rowH = 0;
            g_atlas.push_back(t);
        }
        AtlasTex& at = g_atlas.back();
        if (bh > at.rowH) at.rowH = bh;

        // RGBA with white RGB + coverage alpha; flip rows so texel row 0 is the
        // glyph top (GL textures are bottom-up, cov is top-down).
        std::vector<unsigned char> rgba((size_t)bw * (size_t)bh * 4, 0);
        for (int y = 0; y < bh; y++)
        {
            const unsigned char* src = &cov[(size_t)(bh - 1 - y) * (size_t)bw];
            unsigned char* dst = &rgba[(size_t)y * (size_t)bw * 4];
            for (int x = 0; x < bw; x++)
            {
                dst[x * 4 + 0] = 255;
                dst[x * 4 + 1] = 255;
                dst[x * 4 + 2] = 255;
                dst[x * 4 + 3] = src[x];            // 0..255 coverage
            }
        }
        glBindTexture(GL_TEXTURE_2D, at.tex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, at.cursorX, at.cursorY, bw, bh,
                        GL_RGBA, GL_UNSIGNED_BYTE, &rgba[0]);

        Glyph g;
        g.u0 = (float)at.cursorX / (float)ATLAS_SIZE;
        g.u1 = (float)(at.cursorX + bw) / (float)ATLAS_SIZE;
        // GL v axis: row 0 of the texture (glyph top) maps to v1
        g.v0 = (float)(at.cursorY + bh) / (float)ATLAS_SIZE;
        g.v1 = (float)at.cursorY / (float)ATLAS_SIZE;
        g.adv = gadv;
        g.ox = gox;
        g.oy = goy;
        g.w = (float)bw;
        g.h = (float)bh;
        g.page = (int)g_atlas.size() - 1;   // page uploaded just above
        g_glyphs[ch] = g;

        at.cursorX += bw + 2;
    }
    ::SelectObject(g_glyphDC, oldFont);
    g_atlasBuilt = true;
    return true;
}

// Build the glyph atlas on the viewer's OWN context.
//
// Build the glyph atlas on the viewer's OWN context.
//
// RenderOffscreen() renders into a temporary context that merely SHARES the GL
// objects. Building (and uploading) the atlas inside such a temporary context
// was measured to leave the text invisible on that first frame, so every
// setter that invalidates the atlas rebuilds it here, while the main RC is
// current - never lazily inside an offscreen frame.
void EnsureAtlasBuilt()
{
    if (g_atlasBuilt) return;
    if (!g_hglrc || !g_hdc) return;   // no context yet: lazily built on first draw
    ::wglMakeCurrent(g_hdc, g_hglrc);
    BuildGlyphs();
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
void ApplyBackground()
{
    if (g_bgCustom)
    {
        glClearColor(g_bgR, g_bgG, g_bgB, 1.0f);
        return;
    }
    switch (g_bgIndex)
    {
    case 0:  glClearColor(1.0f, 1.0f, 1.0f, 1.0f); break;   // white
    case 1:  glClearColor(0.0f, 0.0f, 0.0f, 1.0f); break;   // black
    case 2:  glClearColor(0.13f, 0.14f, 0.16f, 1.0f); break;// dark grey
    default: glClearColor(0.86f, 0.87f, 0.89f, 1.0f); break;// light grey
    }
}

void DrawFills()
{
    if (g_fills.empty()) return;
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);
    glEnable(GL_SCISSOR_TEST);
    glStencilMask(0xFF);
    glClearStencil(0);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnableClientState(GL_VERTEX_ARRAY);

    for (size_t i = 0; i < g_fills.size(); i++)
    {
        const FillMeta& f = g_fills[i];
        if (f.layer < g_layerVisible.size() && !g_layerVisible[f.layer]) continue;
        if (f.triCount <= 0) continue;

        // scissor to the fill-path bbox (screen coords, origin bottom-left)
        double s = g_scale;
        int sx0 = (int)floor((f.minX - g_cx) * s + g_winW * 0.5);
        int sx1 = (int)ceil ((f.maxX - g_cx) * s + g_winW * 0.5);
        int sy0 = (int)floor((f.minY - g_cy) * s + g_winH * 0.5);
        int sy1 = (int)ceil ((f.maxY - g_cy) * s + g_winH * 0.5);
        if (sx1 < sx0) { int t = sx0; sx0 = sx1; sx1 = t; }
        if (sy1 < sy0) { int t = sy0; sy0 = sy1; sy1 = t; }
        if (sx0 < 0) sx0 = 0;
        if (sy0 < 0) sy0 = 0;
        if (sx1 > g_winW) sx1 = g_winW;
        if (sy1 > g_winH) sy1 = g_winH;
        if (sx1 - sx0 < 1 || sy1 - sy0 < 1) continue;
        glScissor(sx0, sy0, sx1 - sx0, sy1 - sy0);
        glClear(GL_STENCIL_BUFFER_BIT);

        // odd-even: every triangle inverts the stencil
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glStencilOp(GL_INVERT, GL_INVERT, GL_INVERT);
        glVertexPointer(2, GL_FLOAT, 0, &g_fillXY[(size_t)f.triStart * 6]);
        glDrawArrays(GL_TRIANGLES, 0, f.triCount * 3);

        // paint the bbox where stencil != 0
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
        glColor4ub(f.rgba[0], f.rgba[1], f.rgba[2], f.rgba[3]);
        glBegin(GL_QUADS);
            glVertex2d(f.minX, f.minY);
            glVertex2d(f.maxX, f.minY);
            glVertex2d(f.maxX, f.maxY);
            glVertex2d(f.minX, f.maxY);
        glEnd();
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
}

void DrawPaths()
{
    glDisable(GL_DEPTH_TEST);
    glEnableClientState(GL_VERTEX_ARRAY);
    glLineWidth(1.0f);

    // hairline buckets: one draw call each
    for (size_t b = 0; b < g_buckets.size(); b++)
    {
        const Bucket& bk = g_buckets[b];
        if (bk.segs.empty()) continue;
        if (bk.layer < g_layerVisible.size() && !g_layerVisible[bk.layer]) continue;
        glColor4ub(bk.rgba[0], bk.rgba[1], bk.rgba[2], bk.rgba[3]);
        glVertexPointer(2, GL_FLOAT, 0, &bk.segs[0]);
        glDrawArrays(GL_LINES, 0, (GLsizei)(bk.segs.size() / 4));
    }

    // wide paths: per path, glLineWidth in pixels
    for (size_t i = 0; i < g_widePaths.size(); i++)
    {
        const PathMeta& p = g_paths[g_widePaths[i]];
        if (p.layer < g_layerVisible.size() && !g_layerVisible[p.layer]) continue;
        float px = (float)(p.width * g_scale);
        if (px < 1.0f) px = 1.0f;
        if (px > 64.0f) px = 64.0f;
        glLineWidth(px);
        glColor4ub(p.rgba[0], p.rgba[1], p.rgba[2], p.rgba[3]);
        glVertexPointer(2, GL_FLOAT, 0, &g_pathXY[(size_t)p.start * 2]);
        glDrawArrays(GL_LINE_STRIP, 0, p.count);
    }
    glLineWidth(1.0f);
    glDisableClientState(GL_VERTEX_ARRAY);
}

// Draw one text set. screenSpace=false: world coords + world-unit heights under
// the world ortho. screenSpace=true: t.x/t.y are world coords converted to
// screen pixels here, t.height is in PIXELS (overlay markup stays legible at
// any zoom). Layer visibility is not applied to overlay text.
void DrawTextSet(const std::vector<TextMeta>& texts, bool screenSpace)
{
    if (texts.empty()) return;
    if (!BuildGlyphs()) return;
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    const double kInv96 = 1.0 / (double)GLYPH_PX;
    int lastPage = -2;   // force the first bind (do not rely on the current binding)

    for (size_t i = 0; i < texts.size(); i++)
    {
        const TextMeta& t = texts[i];
        if (!screenSpace && t.layer < g_layerVisible.size() && !g_layerVisible[t.layer]) continue;
        if (t.text.empty() || t.height <= 0.0f) continue;

        double oxw, oyw;
        if (screenSpace)
        {
            float sx, sy;
            // reuse the view transform: world -> screen pixels (y up)
            oxw = (t.x - g_cx) * g_scale + g_winW * 0.5f;
            oyw = g_winH * 0.5f - (t.y - g_cy) * g_scale;
            sx = 0.0f; sy = 0.0f;
        }
        else { oxw = t.x; oyw = t.y; }

        double s = (double)t.height * kInv96;      // size units per atlas px
        double rad = (double)t.rotDeg * 3.14159265358979323846 / 180.0;
        double dx = cos(rad), dy = sin(rad);       // text direction (y up)
        double px = -dy, py = dx;                  // perpendicular (up)

        double advTotal = 0.0;
        for (size_t c = 0; c < t.text.size(); c++)
        {
            std::map<wchar_t, Glyph>::iterator gi = g_glyphs.find(t.text[c]);
            if (gi != g_glyphs.end()) advTotal += (double)gi->second.adv * s;
        }
        double xoff = 0.0;
        if (t.halign == 1) xoff = -advTotal * 0.5;
        else if (t.halign == 2) xoff = -advTotal;

        double baseOff = 0.0;
        if (t.valign == 1)      baseOff =  (double)g_tmDescent * s;
        else if (t.valign == 2) baseOff =  ((double)g_tmAscent - (double)g_tmDescent) * 0.5 * s;
        else if (t.valign == 3) baseOff =  (double)g_tmAscent * s;

        double penX = xoff;
        glColor4ub(t.rgba[0], t.rgba[1], t.rgba[2], t.rgba[3]);

        for (size_t c = 0; c < t.text.size(); c++)
        {
            wchar_t ch = t.text[c];
            if (ch == L'\n' || ch == L'\r' || ch == L'\t') continue;
            std::map<wchar_t, Glyph>::iterator gi = g_glyphs.find(ch);
            if (gi == g_glyphs.end()) continue;
            const Glyph& g = gi->second;
            if (g.w <= 0 || g.h <= 0) { penX += (double)g.adv * s; continue; }

            // bind the atlas page this glyph lives on. Never rely on whatever
            // texture happens to be current (that breaks across contexts and
            // with multi-page atlases).
            // bind the atlas page this glyph lives on. Never rely on whatever
            // texture happens to be current (that breaks across contexts and
            // with multi-page atlases).
            if (g.page != lastPage && g.page >= 0 && (size_t)g.page < g_atlas.size())
            {
                glBindTexture(GL_TEXTURE_2D, g_atlas[(size_t)g.page].tex);
                lastPage = g.page;
            }

            double ox = oxw + dx * penX + px * baseOff;
            double oy = oyw + dy * penX + py * baseOff;
            double gx = dx * ((double)g.ox * s) + px * ((double)g.oy * s);
            double gy = dy * ((double)g.ox * s) + py * ((double)g.oy * s);
            double wx = dx * ((double)g.w * s);
            double wy = dy * ((double)g.w * s);
            double hx = px * (-(double)g.h * s);
            double hy = py * (-(double)g.h * s);

            double x0 = ox + gx,           y0 = oy + gy;
            double x1 = ox + gx + wx,      y1 = oy + gy + wy;
            double x2 = ox + gx + wx + hx, y2 = oy + gy + wy + hy;
            double x3 = ox + gx + hx,      y3 = oy + gy + hy;

            glBegin(GL_QUADS);
                glTexCoord2f(g.u0, g.v1); glVertex2d(x0, y0);
                glTexCoord2f(g.u1, g.v1); glVertex2d(x1, y1);
                glTexCoord2f(g.u1, g.v0); glVertex2d(x2, y2);
                glTexCoord2f(g.u0, g.v0); glVertex2d(x3, y3);
            glEnd();

            penX += (double)g.adv * s;
        }
    }
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
}

void DrawTexts() { DrawTextSet(g_texts, false); }

// ---- overlay pass: screen-space ortho, drawn last (on top of everything) ----
void DrawOverlay()
{
    bool anyFill = !g_ovFills.empty();
    bool anyPath = !g_ovPaths.empty();
    bool anyText = !g_otexts.empty();
    if (!anyFill && !anyPath && !anyText) return;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, (double)g_winW, 0.0, (double)g_winH, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // overlay fills: stencil odd-even in screen space
    if (anyFill)
    {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_STENCIL_TEST);
        glEnable(GL_SCISSOR_TEST);
        glStencilMask(0xFF);
        glClearStencil(0);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glEnableClientState(GL_VERTEX_ARRAY);
        std::vector<float> scr;
        for (size_t i = 0; i < g_ovFills.size(); i++)
        {
            const FillMeta& f = g_ovFills[i];
            if (f.triCount <= 0) continue;
            scr.resize((size_t)f.triCount * 6);
            float sminX = 1e30f, sminY = 1e30f, smaxX = -1e30f, smaxY = -1e30f;
            for (int t = 0; t < f.triCount; t++)
            {
                const float* v = &g_ovFillXY[(size_t)(f.triStart + t) * 6];
                for (int k = 0; k < 6; k += 2)
                {
                    float sx = (float)((v[k] - g_cx) * g_scale + g_winW * 0.5f);
                    float sy = (float)(g_winH * 0.5f - (v[k + 1] - g_cy) * g_scale);
                    scr[(size_t)t * 6 + k] = sx;
                    scr[(size_t)t * 6 + k + 1] = sy;
                    if (sx < sminX) sminX = sx;
                    if (sx > smaxX) smaxX = sx;
                    if (sy < sminY) sminY = sy;
                    if (sy > smaxY) smaxY = sy;
                }
            }
            int sx0 = (int)floor(sminX), sx1 = (int)ceil(smaxX);
            int sy0 = (int)floor(sminY), sy1 = (int)ceil(smaxY);
            if (sx0 < 0) sx0 = 0;
            if (sy0 < 0) sy0 = 0;
            if (sx1 > g_winW) sx1 = g_winW;
            if (sy1 > g_winH) sy1 = g_winH;
            if (sx1 - sx0 < 1 || sy1 - sy0 < 1) continue;
            glScissor(sx0, sy0, sx1 - sx0, sy1 - sy0);
            glClear(GL_STENCIL_BUFFER_BIT);
            glStencilFunc(GL_ALWAYS, 0, 0xFF);
            glStencilOp(GL_INVERT, GL_INVERT, GL_INVERT);
            glVertexPointer(2, GL_FLOAT, 0, &scr[0]);
            glDrawArrays(GL_TRIANGLES, 0, f.triCount * 3);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glColor4ub(f.rgba[0], f.rgba[1], f.rgba[2], f.rgba[3]);
            glBegin(GL_QUADS);
                glVertex2d(sminX, sminY);
                glVertex2d(smaxX, sminY);
                glVertex2d(smaxX, smaxY);
                glVertex2d(sminX, smaxY);
            glEnd();
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        }
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_STENCIL_TEST);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    }

    // overlay paths: world -> screen, pixel widths, optional dash
    if (anyPath)
    {
        glDisable(GL_DEPTH_TEST);
        glEnableClientState(GL_VERTEX_ARRAY);
        std::vector<float> scr;
        for (size_t i = 0; i < g_ovPaths.size(); i++)
        {
            const OvMeta& p = g_ovPaths[i];
            if (p.count < 2) continue;
            scr.resize((size_t)p.count * 2);
            for (int v = 0; v < p.count; v++)
            {
                scr[(size_t)v * 2 + 0] = (float)((g_ovXY[(size_t)p.start * 2 + (size_t)v * 2 + 0] - g_cx) * g_scale + g_winW * 0.5f);
                scr[(size_t)v * 2 + 1] = (float)(g_winH * 0.5f - (g_ovXY[(size_t)p.start * 2 + (size_t)v * 2 + 1] - g_cy) * g_scale);
            }
            float wpx = p.widthPx;
            if (wpx < 1.0f) wpx = 1.0f;
            if (wpx > 32.0f) wpx = 32.0f;
            glLineWidth(wpx);
            glColor4ub(p.rgba[0], p.rgba[1], p.rgba[2], p.rgba[3]);
            if (p.dashPx > 0.0f)
            {
                int factor = (int)floor(p.dashPx / 8.0f + 0.5f);
                if (factor < 1) factor = 1;
                if (factor > 16) factor = 16;
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(factor, 0x00FF);
            }
            glVertexPointer(2, GL_FLOAT, 0, &scr[0]);
            glDrawArrays(GL_LINE_STRIP, 0, p.count);
            if (p.dashPx > 0.0f) glDisable(GL_LINE_STIPPLE);
        }
        glLineWidth(1.0f);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    if (anyText) DrawTextSet(g_otexts, true);
}

void RenderFrame(bool swap)
{
    if (!g_hglrc || !g_hdc || g_winW <= 0 || g_winH <= 0) return;
    ::wglMakeCurrent(g_hdc, g_hglrc);
    glViewport(0, 0, g_winW, g_winH);
    ApplyBackground();
    glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double hw = g_winW * 0.5 / g_scale, hh = g_winH * 0.5 / g_scale;
    glOrtho(g_cx - hw, g_cx + hw, g_cy - hh, g_cy + hh, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    DrawFills();
    DrawPaths();
    DrawTexts();
    DrawOverlay();
    glFlush();
    if (swap) ::SwapBuffers(g_hdc);
}

// ---------------------------------------------------------------------------
// marshalling helpers
// ---------------------------------------------------------------------------
template<typename T> void CopyArray(array<T>^ src, std::vector<T>& dst)
{
    dst.clear();
    if (src == nullptr || src->Length == 0) return;
    dst.resize((size_t)src->Length);
    pin_ptr<T> p = &src[0];
    memcpy(&dst[0], p, sizeof(T) * (size_t)src->Length);
}

std::wstring ToWString(String^ s)
{
    if (String::IsNullOrEmpty(s)) return std::wstring();
    IntPtr p = Marshal::StringToHGlobalUni(s);
    if (p == IntPtr::Zero) return std::wstring();
    std::wstring r((const wchar_t*)(void*)p);
    Marshal::FreeHGlobal(p);
    return r;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// managed API
// ---------------------------------------------------------------------------
int Dwg2DViewer::Create(System::IntPtr hwnd)
{
    if (g_hglrc) return 0;
    g_hwnd = (HWND)(void*)hwnd;
    if (!g_hwnd) return -1;
    g_hdc = ::GetDC(g_hwnd);
    if (!g_hdc) return -2;

    LoadWglArbEntryPoints();
    int pf = ChooseMultiSampleFormat(g_hdc);
    if (pf == 0)
    {
        PIXELFORMATDESCRIPTOR pfd = { 0 };
        pfd.nSize = sizeof(pfd);
        pfd.nVersion = 1;
        pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        pfd.iPixelType = PFD_TYPE_RGBA;
        pfd.cColorBits = 32;
        pfd.cStencilBits = 8;
        pfd.cDepthBits = 24;
        pfd.iLayerType = PFD_MAIN_PLANE;
        pf = ::ChoosePixelFormat(g_hdc, &pfd);
    }
    if (pf == 0) { ::ReleaseDC(g_hwnd, g_hdc); g_hdc = NULL; return -3; }
    PIXELFORMATDESCRIPTOR chosen = { 0 };
    chosen.nSize = sizeof(chosen);
    if (!::SetPixelFormat(g_hdc, pf, &chosen))
    { ::ReleaseDC(g_hwnd, g_hdc); g_hdc = NULL; return -4; }

    g_hglrc = ::wglCreateContext(g_hdc);
    if (!g_hglrc) { ::ReleaseDC(g_hwnd, g_hdc); g_hdc = NULL; return -5; }
    if (!::wglMakeCurrent(g_hdc, g_hglrc))
    {
        ::wglDeleteContext(g_hglrc); g_hglrc = NULL;
        ::ReleaseDC(g_hwnd, g_hdc); g_hdc = NULL;
        return -6;
    }

    // actual MSAA sample count granted by the driver.
    // NOTE: these are GL enums (0x80A8/0x80A9), NOT the WGL_ARB attribs
    // (0x2041/0x2042) - querying the WGL tokens here silently returns 0.
    g_msaaSamples = 0;
    int sb = 0, sm = 0;
    glGetIntegerv(0x80A8 /*GL_SAMPLE_BUFFERS*/, &sb);
    if (sb) glGetIntegerv(0x80A9 /*GL_SAMPLES*/, &sm);
    g_msaaSamples = sb ? sm : 0;

    RECT rc;
    ::GetClientRect(g_hwnd, &rc);
    g_winW = rc.right - rc.left;
    g_winH = rc.bottom - rc.top;

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glShadeModel(GL_FLAT);
    return 0;
}

void Dwg2DViewer::Destroy()
{
    if (g_hglrc)
    {
        ::wglMakeCurrent(NULL, NULL);
        ::wglDeleteContext(g_hglrc);
        g_hglrc = NULL;
    }
    if (g_hdc && g_hwnd) { ::ReleaseDC(g_hwnd, g_hdc); }
    g_hdc = NULL;
    g_hwnd = NULL;
    g_atlasReset();
}

bool Dwg2DViewer::IsReady() { return g_hglrc != NULL && g_hdc != NULL; }

void Dwg2DViewer::SetLayerNames(array<System::String^>^ names)
{
    g_layerNames.clear();
    g_layerVisible.clear();
    if (names != nullptr)
    {
        g_layerNames.reserve((size_t)names->Length);
        for (int i = 0; i < names->Length; i++)
            g_layerNames.push_back(ToWString(names[i]));
    }
    g_layerVisible.assign(g_layerNames.size(), true);
    // NOTE: layer names do not affect the glyph set - do NOT reset the atlas here
}

void Dwg2DViewer::SetPaths(array<float>^ xy, array<int>^ start, array<int>^ count,
                           array<unsigned char>^ rgba, array<float>^ width, array<unsigned char>^ layer)
{
    CopyArray<float>(xy, g_pathXY);
    g_paths.clear();
    int n = (start != nullptr) ? start->Length : 0;
    if (count != nullptr && count->Length < n) n = count->Length;
    if (n <= 0) { RebuildBuckets(); return; }
    pin_ptr<int> ps = &start[0];
    pin_ptr<int> pc = &count[0];
    pin_ptr<unsigned char> prgba = &rgba[0];
    pin_ptr<float> pw = (width != nullptr && width->Length >= n) ? &width[0] : nullptr;
    pin_ptr<unsigned char> pl = &layer[0];
    g_paths.reserve((size_t)n);
    for (int i = 0; i < n; i++)
    {
        PathMeta p;
        p.start = ps[i];
        p.count = pc[i];
        p.width = pw ? pw[i] : 0.0f;
        p.layer = pl[i];
        memcpy(p.rgba, &prgba[(size_t)i * 4], 4);
        g_paths.push_back(p);
    }
    RebuildBuckets();
}

void Dwg2DViewer::SetFills(array<float>^ xy, array<int>^ triPath,
                           array<unsigned char>^ rgba, array<unsigned char>^ layer)
{
    CopyArray<float>(xy, g_fillXY);
    g_fills.clear();
    int nt = (triPath != nullptr) ? triPath->Length : 0;   // triangles
    if (nt <= 0) return;
    pin_ptr<int> pt = &triPath[0];
    pin_ptr<unsigned char> prgba = &rgba[0];
    pin_ptr<unsigned char> pl = &layer[0];
    int nfp = (rgba != nullptr) ? rgba->Length / 4 : 0;
    g_fills.resize((size_t)nfp);
    for (int i = 0; i < nfp; i++)
    {
        FillMeta f;
        f.triStart = 0; f.triCount = 0;
        f.layer = pl[i];
        memcpy(f.rgba, &prgba[(size_t)i * 4], 4);
        f.minX = f.minY = 1e30f;
        f.maxX = f.maxY = -1e30f;
        g_fills[(size_t)i] = f;
    }
    for (int t = 0; t < nt; t++)
    {
        int fp = pt[t];
        if (fp < 0 || fp >= nfp) continue;
        FillMeta& f = g_fills[(size_t)fp];
        if (f.triCount == 0) f.triStart = t;
        f.triCount++;
        const float* v = &g_fillXY[(size_t)t * 6];
        for (int k = 0; k < 6; k += 2)
        {
            if (v[k] < f.minX) f.minX = v[k];
            if (v[k] > f.maxX) f.maxX = v[k];
            if (v[k + 1] < f.minY) f.minY = v[k + 1];
            if (v[k + 1] > f.maxY) f.maxY = v[k + 1];
        }
    }
}

void Dwg2DViewer::SetTexts(array<float>^ data, array<unsigned char>^ rgba, array<unsigned char>^ layer,
                           array<System::String^>^ content)
{
    g_texts.clear();
    g_atlasReset();
    if (data == nullptr || content == nullptr) return;
    int n = content->Length;
    if (data->Length / 6 < n) n = data->Length / 6;
    pin_ptr<float> pd = &data[0];
    pin_ptr<unsigned char> prgba = &rgba[0];
    pin_ptr<unsigned char> pl = &layer[0];
    g_texts.reserve((size_t)n);
    for (int i = 0; i < n; i++)
    {
        TextMeta t;
        t.x = pd[(size_t)i * 6 + 0];
        t.y = pd[(size_t)i * 6 + 1];
        t.height = pd[(size_t)i * 6 + 2];
        t.rotDeg = pd[(size_t)i * 6 + 3];
        t.halign = (int)pd[(size_t)i * 6 + 4];
        t.valign = (int)pd[(size_t)i * 6 + 5];
        t.layer = pl[i];
        memcpy(t.rgba, &prgba[(size_t)i * 4], 4);
        t.text = ToWString(content[i]);
        g_texts.push_back(t);
    }
    // rebuild the atlas on the main RC (never lazily inside an offscreen frame)
    EnsureAtlasBuilt();
}

void Dwg2DViewer::SetBounds(float minX, float minY, float maxX, float maxY)
{
    g_minX = minX; g_minY = minY; g_maxX = maxX; g_maxY = maxY;
}

void Dwg2DViewer::Fit()
{
    double w = (double)g_maxX - (double)g_minX;
    double h = (double)g_maxY - (double)g_minY;
    if (w < 1e-9 && h < 1e-9) { w = 100; h = 100; }
    if (w < 1e-9) w = h;
    if (h < 1e-9) h = w;
    double margin = 0.92;
    double sx = (g_winW > 0 ? g_winW : 800) * margin / w;
    double sy = (g_winH > 0 ? g_winH : 600) * margin / h;
    g_scale = (sx < sy ? sx : sy);
    if (!(g_scale > 1e-12)) g_scale = 1e-12;
    g_cx = (g_minX + g_maxX) * 0.5;
    g_cy = (g_minY + g_maxY) * 0.5;
}

void Dwg2DViewer::ZoomAt(float factor, float clientX, float clientY)
{
    if (factor <= 0.0f) return;
    float wx, wy;
    ScreenToWorld(clientX, clientY, wx, wy);
    g_scale *= (double)factor;
    if (g_scale < 1e-12) g_scale = 1e-12;
    if (g_scale > 1e12) g_scale = 1e12;
    // keep the world point under the cursor fixed
    g_cx = wx - (clientX - g_winW * 0.5f) / g_scale;
    g_cy = wy + (clientY - g_winH * 0.5f) / g_scale;
}

void Dwg2DViewer::Zoom(float factor)
{
    ZoomAt(factor, g_winW * 0.5f, g_winH * 0.5f);
}

void Dwg2DViewer::Pan(float dxPixels, float dyPixels)
{
    if (g_scale <= 0) return;
    g_cx -= (double)dxPixels / g_scale;
    g_cy += (double)dyPixels / g_scale;
}

void Dwg2DViewer::ScreenToWorld(float clientX, float clientY,
                                [System::Runtime::InteropServices::Out] float% wx,
                                [System::Runtime::InteropServices::Out] float% wy)
{
    double x = (clientX - g_winW * 0.5f) / g_scale + g_cx;
    double y = g_cy - (clientY - g_winH * 0.5f) / g_scale;
    wx = (float)x; wy = (float)y;
}

void Dwg2DViewer::WorldToScreen(float wx, float wy,
                                [System::Runtime::InteropServices::Out] float% sx,
                                [System::Runtime::InteropServices::Out] float% sy)
{
    double x = (wx - g_cx) * g_scale + g_winW * 0.5f;
    double y = g_winH * 0.5f - (wy - g_cy) * g_scale;
    sx = (float)x; sy = (float)y;
}

double Dwg2DViewer::ViewScale() { return g_scale; }

void Dwg2DViewer::ViewCenter([System::Runtime::InteropServices::Out] float% cx,
                             [System::Runtime::InteropServices::Out] float% cy)
{
    cx = (float)g_cx; cy = (float)g_cy;
}

void Dwg2DViewer::SetBackgroundIndex(int idx)
{
    if (idx < 0) idx = 0;
    if (idx > 3) idx = 3;
    g_bgIndex = idx;
    g_bgCustom = false;
}

int Dwg2DViewer::BackgroundIndex() { return g_bgCustom ? -1 : g_bgIndex; }

void Dwg2DViewer::SetBackgroundRgb(float r, float g, float b)
{
    g_bgR = r; g_bgG = g; g_bgB = b;
    g_bgCustom = true;
}

void Dwg2DViewer::GetBackgroundRgb([System::Runtime::InteropServices::Out] float% r,
                                   [System::Runtime::InteropServices::Out] float% g,
                                   [System::Runtime::InteropServices::Out] float% b)
{
    if (g_bgCustom) { r = g_bgR; g = g_bgG; b = g_bgB; return; }
    switch (g_bgIndex)
    {
    case 0:  r = 1.0f; g = 1.0f; b = 1.0f; break;
    case 1:  r = 0.0f; g = 0.0f; b = 0.0f; break;
    case 2:  r = 0.13f; g = 0.14f; b = 0.16f; break;
    default: r = 0.86f; g = 0.87f; b = 0.89f; break;
    }
}

void Dwg2DViewer::SetLayerVisible(int layerIdx, bool visible)
{
    if (layerIdx < 0) return;
    if ((size_t)layerIdx >= g_layerVisible.size()) return;
    g_layerVisible[(size_t)layerIdx] = visible;
}

bool Dwg2DViewer::LayerVisible(int layerIdx)
{
    if (layerIdx < 0 || (size_t)layerIdx >= g_layerVisible.size()) return false;
    return g_layerVisible[(size_t)layerIdx];
}

void Dwg2DViewer::SetAllLayersVisible(bool visible)
{
    for (size_t i = 0; i < g_layerVisible.size(); i++) g_layerVisible[i] = visible;
}

void Dwg2DViewer::Resize(int w, int h)
{
    g_winW = w; g_winH = h;
}

// ---------------- overlay (markup / measurement) ----------------
void Dwg2DViewer::ClearOverlay()
{
    ResetOverlay();
    // ResetOverlay() cleared the atlas; rebuild the scene glyphs right away on
    // the main RC so the next (possibly offscreen) frame still shows text.
    EnsureAtlasBuilt();
}

void Dwg2DViewer::SetOverlayPaths(array<float>^ xy, array<int>^ start, array<int>^ count,
                                  array<unsigned char>^ rgba, array<float>^ widthPx,
                                  array<float>^ dashPx)
{
    g_ovPaths.clear();
    // NOTE: overlay paths do not affect the glyph set - do NOT reset the atlas here
    CopyArray<float>(xy, g_ovXY);
    int n = (start != nullptr) ? start->Length : 0;
    if (count != nullptr && count->Length < n) n = count->Length;
    if (n <= 0) return;
    pin_ptr<int> ps = &start[0];
    pin_ptr<int> pc = &count[0];
    pin_ptr<unsigned char> prgba = &rgba[0];
    pin_ptr<float> pw = (widthPx != nullptr && widthPx->Length >= n) ? &widthPx[0] : nullptr;
    pin_ptr<float> pd = (dashPx != nullptr && dashPx->Length >= n) ? &dashPx[0] : nullptr;
    g_ovPaths.reserve((size_t)n);
    for (int i = 0; i < n; i++)
    {
        OvMeta p;
        p.start = ps[i];
        p.count = pc[i];
        p.widthPx = pw ? pw[i] : 1.0f;
        p.dashPx = pd ? pd[i] : 0.0f;
        memcpy(p.rgba, &prgba[(size_t)i * 4], 4);
        g_ovPaths.push_back(p);
    }
}

void Dwg2DViewer::SetOverlayFills(array<float>^ xy, array<int>^ triPath,
                                  array<unsigned char>^ rgba)
{
    g_ovFills.clear();
    CopyArray<float>(xy, g_ovFillXY);
    int nt = (triPath != nullptr) ? triPath->Length : 0;
    if (nt <= 0) return;
    pin_ptr<int> pt = &triPath[0];
    pin_ptr<unsigned char> prgba = &rgba[0];
    int nfp = (rgba != nullptr) ? rgba->Length / 4 : 0;
    g_ovFills.resize((size_t)nfp);
    for (int i = 0; i < nfp; i++)
    {
        FillMeta f;
        f.triStart = 0; f.triCount = 0;
        f.layer = 0;
        memcpy(f.rgba, &prgba[(size_t)i * 4], 4);
        f.minX = f.minY = 1e30f;
        f.maxX = f.maxY = -1e30f;
        g_ovFills[(size_t)i] = f;
    }
    for (int t = 0; t < nt; t++)
    {
        int fp = pt[t];
        if (fp < 0 || fp >= nfp) continue;
        FillMeta& f = g_ovFills[(size_t)fp];
        if (f.triCount == 0) f.triStart = t;
        f.triCount++;
        const float* v = &g_ovFillXY[(size_t)t * 6];
        for (int k = 0; k < 6; k += 2)
        {
            if (v[k] < f.minX) f.minX = v[k];
            if (v[k] > f.maxX) f.maxX = v[k];
            if (v[k + 1] < f.minY) f.minY = v[k + 1];
            if (v[k + 1] > f.maxY) f.maxY = v[k + 1];
        }
    }
}

void Dwg2DViewer::SetOverlayTexts(array<float>^ data, array<unsigned char>^ rgba,
                                  array<System::String^>^ content)
{
    g_otexts.clear();
    g_atlasReset();
    if (data == nullptr || content == nullptr) return;
    int n = content->Length;
    if (data->Length / 6 < n) n = data->Length / 6;
    pin_ptr<float> pd = &data[0];
    pin_ptr<unsigned char> prgba = &rgba[0];
    g_otexts.reserve((size_t)n);
    for (int i = 0; i < n; i++)
    {
        TextMeta t;
        t.x = pd[(size_t)i * 6 + 0];
        t.y = pd[(size_t)i * 6 + 1];
        t.height = pd[(size_t)i * 6 + 2];
        t.rotDeg = pd[(size_t)i * 6 + 3];
        t.halign = (int)pd[(size_t)i * 6 + 4];
        t.valign = (int)pd[(size_t)i * 6 + 5];
        t.layer = 0;
        memcpy(t.rgba, &prgba[(size_t)i * 4], 4);
        t.text = ToWString(content[i]);
        g_otexts.push_back(t);
    }
    // rebuild the atlas on the main RC (never lazily inside an offscreen frame)
    EnsureAtlasBuilt();
}

bool Dwg2DViewer::RenderOffscreen(int width, int height, System::String^ bmpPath,
                                  bool fitToView)
{
    if (!g_hglrc || !g_hdc || width <= 0 || height <= 0) return false;
    ::wglMakeCurrent(NULL, NULL);   // required before wglShareLists

    // optional re-framing: save the view, Fit() against the offscreen size,
    // render, then restore. Fit() reads g_winW/g_winH which the offscreen
    // render sets below.
    double saveCx = g_cx, saveCy = g_cy, saveScale = g_scale;

    WNDCLASSA wc = { 0 };
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = ::DefWindowProcA;
    wc.hInstance = ::GetModuleHandleA(NULL);
    wc.lpszClassName = "Dwg2DViewerOffscreen";
    ::RegisterClassA(&wc);
    HWND hwnd = ::CreateWindowExA(0, wc.lpszClassName, "",
        WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
        0, 0, width, height, NULL, NULL, wc.hInstance, NULL);
    if (!hwnd) { ::wglMakeCurrent(g_hdc, g_hglrc); return false; }
    HDC dc = ::GetDC(hwnd);
    if (!dc) { ::DestroyWindow(hwnd); ::wglMakeCurrent(g_hdc, g_hglrc); return false; }

    int pf = ChooseMultiSampleFormat(dc);
    if (pf == 0)
    {
        PIXELFORMATDESCRIPTOR pfd = { 0 };
        pfd.nSize = sizeof(pfd);
        pfd.nVersion = 1;
        pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        pfd.iPixelType = PFD_TYPE_RGBA;
        pfd.cColorBits = 32;
        pfd.cStencilBits = 8;
        pfd.cDepthBits = 24;
        pfd.iLayerType = PFD_MAIN_PLANE;
        pf = ::ChoosePixelFormat(dc, &pfd);
    }
    bool ok = false;
    HGLRC rc = NULL;
    if (pf != 0)
    {
        PIXELFORMATDESCRIPTOR chosen = { 0 };
        chosen.nSize = sizeof(chosen);
        if (::SetPixelFormat(dc, pf, &chosen))
        {
            rc = ::wglCreateContext(dc);
            // share GL objects (glyph atlas textures) with the main context
            if (rc && ::wglShareLists(g_hglrc, rc))
            {
                HDC saveDc = g_hdc; HGLRC saveRc = g_hglrc;
                int saveW = g_winW, saveH = g_winH;
                g_hdc = dc; g_hglrc = rc;
                g_winW = width; g_winH = height;
                if (::wglMakeCurrent(dc, rc))
                {
                    if (fitToView) Fit();
                    RenderFrame(false);
                    glPixelStorei(GL_PACK_ALIGNMENT, 1);
                    glReadBuffer(GL_BACK);
                    // read back into a bottom-up RGB buffer, then BMP (same layout
                    // as SaveFrame, so reuse the writer by faking the swap path)
                    std::vector<unsigned char> px((size_t)width * 3 * (size_t)height);
                    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, &px[0]);
                    // write 24-bit BMP
                    size_t rowBytes = (size_t)width * 3;
                    size_t rowStride = (rowBytes + 3) & ~(size_t)3;
                    size_t dataSize = rowStride * (size_t)height;
                    array<System::Byte>^ out = gcnew array<System::Byte>((int)(54 + dataSize));
                    pin_ptr<System::Byte> pp = &out[0];
                    unsigned char* o = (unsigned char*)pp;
                    unsigned char* fh = o;
                    fh[0] = 'B'; fh[1] = 'M';
                    unsigned int fileSize = (unsigned int)(54 + dataSize);
                    fh[2] = (unsigned char)(fileSize); fh[3] = (unsigned char)(fileSize >> 8);
                    fh[4] = (unsigned char)(fileSize >> 16); fh[5] = (unsigned char)(fileSize >> 24);
                    fh[10] = 54;
                    unsigned char* ih = o + 14;
                    ih[0] = 40;
                    ih[4] = (unsigned char)(width); ih[5] = (unsigned char)(width >> 8);
                    ih[6] = (unsigned char)(width >> 16); ih[7] = (unsigned char)(width >> 24);
                    ih[8] = (unsigned char)(height); ih[9] = (unsigned char)(height >> 8);
                    ih[10] = (unsigned char)(height >> 16); ih[11] = (unsigned char)(height >> 24);
                    ih[12] = 1; ih[14] = 24;
                    unsigned int imgSize = (unsigned int)dataSize;
                    ih[20] = (unsigned char)(imgSize); ih[21] = (unsigned char)(imgSize >> 8);
                    ih[22] = (unsigned char)(imgSize >> 16); ih[23] = (unsigned char)(imgSize >> 24);
                    for (int y = 0; y < height; y++)
                    {
                        unsigned char* dst = o + 54 + rowStride * (size_t)y;
                        const unsigned char* src = &px[0] + rowBytes * (size_t)y;
                        for (int x = 0; x < width; x++)
                        {
                            dst[3 * x + 0] = src[3 * x + 2];
                            dst[3 * x + 1] = src[3 * x + 1];
                            dst[3 * x + 2] = src[3 * x + 0];
                        }
                    }
                    try { System::IO::File::WriteAllBytes(bmpPath, out); ok = true; }
                    catch (...) { ok = false; }
                }
                g_hdc = saveDc; g_hglrc = saveRc;
                g_winW = saveW; g_winH = saveH;
                g_cx = saveCx; g_cy = saveCy; g_scale = saveScale;
                ::wglMakeCurrent(NULL, NULL);
                ::wglDeleteContext(rc);
            }
            else if (rc) ::wglDeleteContext(rc);
        }
    }
    ::ReleaseDC(hwnd, dc);
    ::DestroyWindow(hwnd);
    ::wglMakeCurrent(g_hdc, g_hglrc);
    return ok;
}

void Dwg2DViewer::Render()
{
    RenderFrame(true);
}

bool Dwg2DViewer::SaveFrame(System::String^ path)
{
    if (!g_hglrc || !g_hdc || g_winW <= 0 || g_winH <= 0) return false;
    // render into the back buffer without swapping, then read it back
    RenderFrame(false);
    int w = g_winW, h = g_winH;
    size_t rowBytes = (size_t)w * 3;
    size_t rowStride = (rowBytes + 3) & ~(size_t)3;
    size_t dataSize = rowStride * (size_t)h;
    std::vector<unsigned char> px(rowBytes * (size_t)h);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, &px[0]);

    array<unsigned char>^ out = gcnew array<unsigned char>((int)(54 + dataSize));
    pin_ptr<unsigned char> p = &out[0];
    unsigned char* o = (unsigned char*)p;
    unsigned char* fh = o;
    fh[0] = 'B'; fh[1] = 'M';
    unsigned int fileSize = (unsigned int)(54 + dataSize);
    fh[2] = (unsigned char)(fileSize); fh[3] = (unsigned char)(fileSize >> 8);
    fh[4] = (unsigned char)(fileSize >> 16); fh[5] = (unsigned char)(fileSize >> 24);
    fh[10] = 54;
    unsigned char* ih = o + 14;
    ih[0] = 40;
    ih[4] = (unsigned char)(w); ih[5] = (unsigned char)(w >> 8);
    ih[6] = (unsigned char)(w >> 16); ih[7] = (unsigned char)(w >> 24);
    ih[8] = (unsigned char)(h); ih[9] = (unsigned char)(h >> 8);
    ih[10] = (unsigned char)(h >> 16); ih[11] = (unsigned char)(h >> 24);
    ih[12] = 1; ih[14] = 24;
    unsigned int imgSize = (unsigned int)dataSize;
    ih[20] = (unsigned char)(imgSize); ih[21] = (unsigned char)(imgSize >> 8);
    ih[22] = (unsigned char)(imgSize >> 16); ih[23] = (unsigned char)(imgSize >> 24);
    for (int y = 0; y < h; y++)
    {
        unsigned char* dst = o + 54 + rowStride * (size_t)y;
        const unsigned char* src = &px[0] + rowBytes * (size_t)y;
        for (int x = 0; x < w; x++)
        {
            dst[3 * x + 0] = src[3 * x + 2];
            dst[3 * x + 1] = src[3 * x + 1];
            dst[3 * x + 2] = src[3 * x + 0];
        }
    }
    try { System::IO::File::WriteAllBytes(path, out); return true; }
    catch (...) { return false; }
}

System::String^ Dwg2DViewer::Diagnostics()
{
    if (!g_hglrc || !g_hdc) return "no-context";
    ::wglMakeCurrent(g_hdc, g_hglrc);
    const char* ven = (const char*)glGetString(GL_VENDOR);
    const char* ren = (const char*)glGetString(GL_RENDERER);
    const char* ver = (const char*)glGetString(GL_VERSION);
    System::Text::StringBuilder^ sb = gcnew System::Text::StringBuilder(256);
    sb->Append("vendor=")->Append(gcnew System::String(ven ? ven : "?"));
    sb->Append(" renderer=")->Append(gcnew System::String(ren ? ren : "?"));
    sb->Append(" version=")->Append(gcnew System::String(ver ? ver : "?"));
    sb->Append(" msaa=")->Append(g_msaaSamples);
    sb->Append(" paths=")->Append((int)g_paths.size());
    sb->Append(" buckets=")->Append((int)g_buckets.size());
    sb->Append(" fills=")->Append((int)g_fills.size());
    sb->Append(" texts=")->Append((int)g_texts.size());
    sb->Append(" glyphs=")->Append((int)g_glyphs.size());
    sb->Append(" scale=")->Append(g_scale.ToString("G4"));
    // atlas diagnostics
    sb->Append(" atlasPages=")->Append((int)g_atlas.size());
    sb->Append(" atlasBuilt=")->Append(g_atlasBuilt ? 1 : 0);
    sb->Append(" atlas0Tex=")->Append(g_atlas.empty() ? 0 : (int)g_atlas[0].tex);
    return sb->ToString();
}

}}}
