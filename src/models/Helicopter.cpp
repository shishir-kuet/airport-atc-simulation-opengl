#include "models/ModelParts.h"

// ============================================================================
// Helicopter
// ============================================================================
void drawHelicopter(const Renderer& r, const Primitives& p, const Mat4& base, float rotorAngle, bool cockpitView)
{
    // Main rotor: mast, hub and four blades spinning around the Y axis.
    Mat4 hub = base * translate(0.0f, 1.95f, 0.0f);
    r.drawPart(p.cylinder, hub * scale(0.5f, 0.2f, 0.5f), Livery::ROTOR);
    for (int i = 0; i < 4; ++i)
        r.drawPart(p.cube, hub * rotateY(rotorAngle + i * 90.0f) * translate(2.25f, 0.0f, 0.0f) * scale(4.2f, 0.05f, 0.3f), Livery::ROTOR);
    r.drawPart(p.cylinder, base * translate(0.0f, 1.6f, 0.0f) * scale(0.18f, 0.6f, 0.18f), Livery::METAL);

    // Cabin (not drawn from the cockpit: the pilot sits inside it),
    // engine housing and tail boom.
    if (!cockpitView)
        r.drawPart(p.sphere, base * translate(0.3f, 0.0f, 0.0f) * scale(3.4f, 2.2f, 2.0f), Livery::HELI_BODY);
    r.drawPart(p.cube,    base * translate(-0.3f, 1.1f, 0.0f) * scale(1.8f, 0.5f, 1.0f), Livery::HELI_ENGINE);
    r.drawPart(p.frustum, base * translate(-3.5f, 0.35f, 0.0f) * alongMinusX() * scale(0.7f, 5.0f, 0.7f), Livery::HELI_BODY);

    // Tail surfaces.
    r.drawPart(p.wing, base * translate(-5.6f, 0.45f, 0.0f) * rotateX(-90.0f) * scale(1.0f, 0.12f, 1.4f), Livery::HELI_TRIM);
    r.drawPart(p.cube, base * translate(-4.8f, 0.35f, 0.0f) * scale(0.6f, 0.06f, 1.8f), Livery::HELI_TRIM);

    // Tail rotor: two blades spinning around the Z axis (faster than the main rotor).
    Mat4 tailHub = base * translate(-6.2f, 1.3f, 0.2f);
    r.drawPart(p.cylinder, tailHub * rotateX(90.0f) * scale(0.2f, 0.15f, 0.2f), Livery::METAL);
    for (int i = 0; i < 2; ++i)
        r.drawPart(p.cube, tailHub * translate(0.0f, 0.0f, 0.08f) * rotateZ(rotorAngle * 3.0f + i * 180.0f)
                               * translate(0.0f, 0.6f, 0.0f) * scale(0.14f, 1.2f, 0.04f), Livery::ROTOR);

    // Landing skids with their struts.
    for (float side : {1.0f, -1.0f})
    {
        float z = 0.95f * side;
        r.drawPart(p.cylinder, base * translate(0.2f, -1.45f, z) * alongPlusX() * scale(0.14f, 3.8f, 0.14f), Livery::METAL);
        r.drawPart(p.cylinder, base * translate(2.3f, -1.31f, z) * rotateZ(-55.0f) * scale(0.14f, 0.5f, 0.14f), Livery::METAL);

        for (float x : {1.0f, -0.7f})
            r.drawPart(p.cylinder, base * translate(x, -1.05f, 0.88f * side) * rotateX(-12.0f * side) * scale(0.1f, 0.85f, 0.1f), Livery::METAL);
    }
}
