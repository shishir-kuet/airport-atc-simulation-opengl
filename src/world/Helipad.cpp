#include "world/SceneParts.h"

// ---- Helipad -------------------------------------------------------------------

void drawHelipad(const Renderer& r, const Primitives& p, const Mat4& base)
{
    const float top = HELIPAD_TOP;

    // Round concrete pad.
    r.drawPart(p.cylinder, base * translate(0, top / 2, 0) * scale(16, top, 16), Paint::CONCRETE);

    // Touchdown circle and the "H".
    r.drawPart(p.ringThin, base * translate(0, top + MARK_THICKNESS / 2, 0) * scale(11, MARK_THICKNESS, 11), Paint::MARKING);
    paint(r, p, base, 0, -1.4f, 5.0f, 0.8f, top);    // left leg
    paint(r, p, base, 0,  1.4f, 5.0f, 0.8f, top);    // right leg
    paint(r, p, base, 0,  0.0f, 0.8f, 2.0f, top);    // cross bar

    // Perimeter lights.
    for (int i = 0; i < 16; ++i)
    {
        float a = radians(i * 22.5f);
        post(r, p, base, 7.6f * std::cos(a), top, 7.6f * std::sin(a), 0.2f, 0.25f, Paint::LIGHT);
    }

    // Windsock: pole with a tapered sock pointing downwind (+X).
    const float wx = 9.5f, wz = 9.5f, poleH = 5.0f;
    post(r, p, base, wx, 0, wz, 0.15f, poleH, Paint::METAL);
    r.drawPart(p.frustum, base * translate(wx + 1.25f, poleH - 0.3f, wz) * rotateZ(-95.0f)
                              * scale(0.7f, 2.4f, 0.7f), Paint::LIGHT);
}

// ---- Booster landing zones ------------------------------------------------------

void drawLandingZone(const Renderer& r, const Primitives& p, const Mat4& base, int number)
{
    const float top = LANDING_ZONE_TOP;
    r.drawPart(p.cylinder, base * translate(0, top / 2, 0) * scale(14, top, 14), Paint::CONCRETE);
    r.drawPart(p.ringThin, base * translate(0, top + MARK_THICKNESS / 2, 0) * scale(10, MARK_THICKNESS, 10), Paint::MARKING);

    // Big "X" in the middle (two bars at +/-45 degrees).
    for (float a : {45.0f, -45.0f})
        r.drawPart(p.cube, base * translate(0, top + MARK_THICKNESS / 2, 0) * rotateY(a)
                               * scale(6.0f, MARK_THICKNESS, 0.7f), Paint::MARKING);

    // Zone number, readable from the south side.
    Mat4 frame = base * translate(0, 0, 6.8f) * rotateY(90.0f) * scale(0.35f, 1.0f, 0.35f);
    paintDigit(r, p, frame, number, 0.0f, top);

    for (int i = 0; i < 8; ++i)
    {
        float a = radians(i * 45.0f);
        post(r, p, base, 6.6f * std::cos(a), top, 6.6f * std::sin(a), 0.2f, 0.25f, Paint::LIGHT);
    }
}
