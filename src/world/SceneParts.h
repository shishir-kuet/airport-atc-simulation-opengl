// Pieces every part of the scenery is built from: the grey levels and the
// small helpers that place a box, a painted marking, a post or a digit.
// Internal header: only the files in src/world/ include it.
#pragma once

#include "world/Environment.h"

#include <cmath>

using namespace Layout;

// The airport's palette: colour + gloss per material, so a surface is
// changed here once and looks the same everywhere it is used.
namespace Paint
{
    constexpr Material GRASS    {{0.24f, 0.40f, 0.20f}, 0.10f};   // field: matte
    constexpr Material ASPHALT  {{0.20f, 0.21f, 0.23f}, 0.25f};   // runway
    constexpr Material TAXIWAY  {{0.27f, 0.28f, 0.30f}, 0.25f};   // taxiways, a little lighter
    constexpr Material CONCRETE {{0.62f, 0.62f, 0.59f}, 0.30f};   // apron and pads
    constexpr Material MARKING  {{0.93f, 0.93f, 0.90f}, 0.45f};   // white paint
    constexpr Material TAXILINE {{0.88f, 0.72f, 0.12f}, 0.45f};   // yellow taxi guidance paint
    constexpr Material BUILDING {{0.76f, 0.74f, 0.70f}, 0.35f};   // terminal walls
    constexpr Material ROOF     {{0.33f, 0.36f, 0.42f}, 0.50f};   // slate roofs
    constexpr Material GLASS    {{0.13f, 0.27f, 0.35f}, 2.20f};   // tinted windows: shiny
    constexpr Material METAL    {{0.58f, 0.61f, 0.66f}, 1.60f};   // steel structures
    constexpr Material LIGHT    {{0.97f, 0.95f, 0.82f}, 1.00f};   // lamps and lit edges
    constexpr Material DARK     {{0.07f, 0.07f, 0.08f}, 0.60f};   // shadow gaps, tyres
}

// Painted markings are thin boxes lying just above the pavement.
constexpr float MARK_THICKNESS = 0.02f;

// ---- Small helpers ---------------------------------------------------------

inline void box(const Renderer& r, const Primitives& p, const Mat4& base,
                Vec3 center, Vec3 size, const Material& material)
{
    r.drawPart(p.cube, base * translate(center.x, center.y, center.z) * scale(size.x, size.y, size.z), material);
}

// A flat painted rectangle centred at (x, z) on a surface whose top is at surfaceY.
inline void paint(const Renderer& r, const Primitives& p, const Mat4& base,
                  float x, float z, float sizeX, float sizeZ, float surfaceY,
                  const Material& material = Paint::MARKING)
{
    box(r, p, base, {x, surfaceY + MARK_THICKNESS * 0.5f, z}, {sizeX, MARK_THICKNESS, sizeZ}, material);
}

// Vertical post (cylinder) standing on y = baseY.
inline void post(const Renderer& r, const Primitives& p, const Mat4& base,
                 float x, float baseY, float z, float diameter, float height, const Material& material)
{
    r.drawPart(p.cylinder, base * translate(x, baseY + height * 0.5f, z) * scale(diameter, height, diameter), material);
}

// Seven-segment digit painted on the ground.
// In `frame`, local +X is the reading "up" direction and local +Z is "right".
inline void paintDigit(const Renderer& r, const Primitives& p, const Mat4& frame,
                       int digit, float u, float surfaceY)
{
    //                              a     b     c     d     e     f     g
    static const bool SEG[10][7] = {{1,    1,    1,    1,    1,    1,    0},   // 0
                                    {0,    1,    1,    0,    0,    0,    0},   // 1
                                    {1,    1,    0,    1,    1,    0,    1},   // 2
                                    {1,    1,    1,    1,    0,    0,    1},   // 3
                                    {0,    1,    1,    0,    0,    1,    1},   // 4
                                    {1,    0,    1,    1,    0,    1,    1},   // 5
                                    {1,    0,    1,    1,    1,    1,    1},   // 6
                                    {1,    1,    1,    0,    0,    0,    0},   // 7
                                    {1,    1,    1,    1,    1,    1,    1},   // 8
                                    {1,    1,    1,    1,    0,    1,    1}};  // 9
    const float W = 2.4f, H = 4.4f, S = 0.5f;   // digit width, height, stroke

    // Segment centre (along "right", along "up") and size (right, up).
    struct Seg { float cu, cv, su, sv; };
    const Seg segs[7] = {
        {0.0f,           H - S / 2, W, S},       // a  top
        {W / 2 - S / 2,  3 * H / 4, S, H / 2},   // b  top right
        {W / 2 - S / 2,  H / 4,     S, H / 2},   // c  bottom right
        {0.0f,           S / 2,     W, S},       // d  bottom
        {-W / 2 + S / 2, H / 4,     S, H / 2},   // e  bottom left
        {-W / 2 + S / 2, 3 * H / 4, S, H / 2},   // f  top left
        {0.0f,           H / 2,     W, S},       // g  middle
    };

    for (int i = 0; i < 7; ++i)
        if (SEG[digit][i])
            paint(r, p, frame, segs[i].cv, u + segs[i].cu, segs[i].sv, segs[i].su, surfaceY);
}
