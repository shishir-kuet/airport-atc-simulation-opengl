#include "models/ModelParts.h"

// ============================================================================
// Airplane (twin-engine airliner)
// ============================================================================
void drawAirplane(const Renderer& r, const Primitives& p, const Mat4& base, float gear, bool cockpitView)
{
    const float fold = (1.0f - gear) * 90.0f;   // landing gear retraction angle

    // Fuselage: long cylinder, ellipsoid nose, tapered tail cone. Skipped in
    // the cockpit view (the pilot's eye is inside it) so that looking back or
    // down shows the wings, engines and tail.
    if (!cockpitView)
    {
        r.drawPart(p.cylinder, base * alongPlusX() * scale(1.2f, 8.0f, 1.2f), Livery::PLANE_BODY);
        r.drawPart(p.sphere,   base * translate(4.0f, 0.0f, 0.0f) * scale(2.2f, 1.2f, 1.2f), Livery::PLANE_BODY);
        r.drawPart(p.frustum,  base * translate(-5.4f, 0.0f, 0.0f) * alongMinusX() * scale(1.2f, 2.8f, 1.2f), Livery::PLANE_BODY);
    }

    for (float side : {1.0f, -1.0f})   // +Z = right, -Z = left (mirrored)
    {
        // Main wing (low-mounted, swept back).
        r.drawPart(p.wing, base * translate(0.5f, -0.35f, 0.0f) * scale(2.6f, 0.22f, 6.0f * side), Livery::PLANE_WING);

        // Horizontal stabilizer.
        r.drawPart(p.wing, base * translate(-5.9f, 0.1f, 0.0f) * scale(1.5f, 0.12f, 2.3f * side), Livery::PLANE_WING);

        // Engine: pylon, nacelle and exhaust cone under the wing.
        float z = 2.3f * side;
        r.drawPart(p.cube,     base * translate(0.6f, -0.55f, z) * scale(1.2f, 0.3f, 0.12f), Livery::PLANE_WING);
        r.drawPart(p.cylinder, base * translate(0.8f, -0.85f, z) * alongPlusX() * scale(0.65f, 1.8f, 0.65f), Livery::NACELLE);
        r.drawPart(p.cone,     base * translate(-0.35f, -0.85f, z) * alongMinusX() * scale(0.45f, 0.5f, 0.45f), Livery::EXHAUST);

        // Main landing gear: strut + two wheels, hinged at the top of the
        // strut and folding inwards (rotation about the X axis).
        float gz = 1.0f * side;
        Mat4 mainGear = base * translate(-0.2f, -0.45f, gz) * rotateX(side * fold) * translate(0.2f, 0.45f, -gz);
        r.drawPart(p.cylinder, mainGear * translate(-0.2f, -0.975f, gz) * scale(0.14f, 1.05f, 0.14f), Livery::METAL);
        for (float w : {-0.17f, 0.17f})
            r.drawPart(p.cylinder, mainGear * translate(-0.2f, -1.5f, gz + w) * rotateX(90.0f) * scale(0.56f, 0.22f, 0.56f), Livery::TYRE);
    }

    // Vertical fin (the wing shape stood upright).
    r.drawPart(p.wing, base * translate(-5.6f, 0.35f, 0.0f) * rotateX(-90.0f) * scale(2.0f, 0.18f, 2.4f), Livery::PLANE_TAIL);

    // Nose landing gear, folding backwards into the fuselage (rotation about Z).
    Mat4 noseGear = base * translate(3.6f, -0.5f, 0.0f) * rotateZ(-fold) * translate(-3.6f, 0.5f, 0.0f);
    r.drawPart(p.cylinder, noseGear * translate(3.6f, -1.0f, 0.0f) * scale(0.1f, 1.0f, 0.1f), Livery::METAL);
    for (float w : {-0.12f, 0.12f})
        r.drawPart(p.cylinder, noseGear * translate(3.6f, -1.53f, w) * rotateX(90.0f) * scale(0.5f, 0.16f, 0.5f), Livery::TYRE);
}
