#include "Models.h"

// Rotations that turn a Y-axis primitive (cylinder/cone) to lie along X.
static Mat4 alongPlusX()  { return rotateZ(-90.0f); }   // local +Y -> world +X
static Mat4 alongMinusX() { return rotateZ(90.0f); }    // local +Y -> world -X

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
        r.drawPart(p.cylinder, base * alongPlusX() * scale(1.2f, 8.0f, 1.2f));
        r.drawPart(p.sphere,   base * translate(4.0f, 0.0f, 0.0f) * scale(2.2f, 1.2f, 1.2f));
        r.drawPart(p.frustum,  base * translate(-5.4f, 0.0f, 0.0f) * alongMinusX() * scale(1.2f, 2.8f, 1.2f));
    }

    for (float side : {1.0f, -1.0f})   // +Z = right, -Z = left (mirrored)
    {
        // Main wing (low-mounted, swept back).
        r.drawPart(p.wing, base * translate(0.5f, -0.35f, 0.0f) * scale(2.6f, 0.22f, 6.0f * side));

        // Horizontal stabilizer.
        r.drawPart(p.wing, base * translate(-5.9f, 0.1f, 0.0f) * scale(1.5f, 0.12f, 2.3f * side));

        // Engine: pylon, nacelle and exhaust cone under the wing.
        float z = 2.3f * side;
        r.drawPart(p.cube,     base * translate(0.6f, -0.55f, z) * scale(1.2f, 0.3f, 0.12f));
        r.drawPart(p.cylinder, base * translate(0.8f, -0.85f, z) * alongPlusX() * scale(0.65f, 1.8f, 0.65f));
        r.drawPart(p.cone,     base * translate(-0.35f, -0.85f, z) * alongMinusX() * scale(0.45f, 0.5f, 0.45f));

        // Main landing gear: strut + two wheels, hinged at the top of the
        // strut and folding inwards (rotation about the X axis).
        float gz = 1.0f * side;
        Mat4 mainGear = base * translate(-0.2f, -0.45f, gz) * rotateX(side * fold) * translate(0.2f, 0.45f, -gz);
        r.drawPart(p.cylinder, mainGear * translate(-0.2f, -0.975f, gz) * scale(0.14f, 1.05f, 0.14f));
        for (float w : {-0.17f, 0.17f})
            r.drawPart(p.cylinder, mainGear * translate(-0.2f, -1.5f, gz + w) * rotateX(90.0f) * scale(0.56f, 0.22f, 0.56f));
    }

    // Vertical fin (the wing shape stood upright).
    r.drawPart(p.wing, base * translate(-5.6f, 0.35f, 0.0f) * rotateX(-90.0f) * scale(2.0f, 0.18f, 2.4f));

    // Nose landing gear, folding backwards into the fuselage (rotation about Z).
    Mat4 noseGear = base * translate(3.6f, -0.5f, 0.0f) * rotateZ(-fold) * translate(-3.6f, 0.5f, 0.0f);
    r.drawPart(p.cylinder, noseGear * translate(3.6f, -1.0f, 0.0f) * scale(0.1f, 1.0f, 0.1f));
    for (float w : {-0.12f, 0.12f})
        r.drawPart(p.cylinder, noseGear * translate(3.6f, -1.53f, w) * rotateX(90.0f) * scale(0.5f, 0.16f, 0.5f));
}

// ============================================================================
// Helicopter
// ============================================================================
void drawHelicopter(const Renderer& r, const Primitives& p, const Mat4& base, float rotorAngle, bool cockpitView)
{
    // Main rotor: mast, hub and four blades spinning around the Y axis.
    Mat4 hub = base * translate(0.0f, 1.95f, 0.0f);
    r.drawPart(p.cylinder, hub * scale(0.5f, 0.2f, 0.5f));
    for (int i = 0; i < 4; ++i)
        r.drawPart(p.cube, hub * rotateY(rotorAngle + i * 90.0f) * translate(2.25f, 0.0f, 0.0f) * scale(4.2f, 0.05f, 0.3f));
    r.drawPart(p.cylinder, base * translate(0.0f, 1.6f, 0.0f) * scale(0.18f, 0.6f, 0.18f));

    // Cabin (not drawn from the cockpit: the pilot sits inside it),
    // engine housing and tail boom.
    if (!cockpitView)
        r.drawPart(p.sphere, base * translate(0.3f, 0.0f, 0.0f) * scale(3.4f, 2.2f, 2.0f));
    r.drawPart(p.cube,    base * translate(-0.3f, 1.1f, 0.0f) * scale(1.8f, 0.5f, 1.0f));
    r.drawPart(p.frustum, base * translate(-3.5f, 0.35f, 0.0f) * alongMinusX() * scale(0.7f, 5.0f, 0.7f));

    // Tail surfaces.
    r.drawPart(p.wing, base * translate(-5.6f, 0.45f, 0.0f) * rotateX(-90.0f) * scale(1.0f, 0.12f, 1.4f));
    r.drawPart(p.cube, base * translate(-4.8f, 0.35f, 0.0f) * scale(0.6f, 0.06f, 1.8f));

    // Tail rotor: two blades spinning around the Z axis (faster than the main rotor).
    Mat4 tailHub = base * translate(-6.2f, 1.3f, 0.2f);
    r.drawPart(p.cylinder, tailHub * rotateX(90.0f) * scale(0.2f, 0.15f, 0.2f));
    for (int i = 0; i < 2; ++i)
        r.drawPart(p.cube, tailHub * translate(0.0f, 0.0f, 0.08f) * rotateZ(rotorAngle * 3.0f + i * 180.0f)
                               * translate(0.0f, 0.6f, 0.0f) * scale(0.14f, 1.2f, 0.04f));

    // Landing skids with their struts.
    for (float side : {1.0f, -1.0f})
    {
        float z = 0.95f * side;
        r.drawPart(p.cylinder, base * translate(0.2f, -1.45f, z) * alongPlusX() * scale(0.14f, 3.8f, 0.14f));
        r.drawPart(p.cylinder, base * translate(2.3f, -1.31f, z) * rotateZ(-55.0f) * scale(0.14f, 0.5f, 0.14f));

        for (float x : {1.0f, -0.7f})
            r.drawPart(p.cylinder, base * translate(x, -1.05f, 0.88f * side) * rotateX(-12.0f * side) * scale(0.1f, 0.85f, 0.1f));
    }
}


// ============================================================================
// Rocket (reusable: core stage with two side boosters, all with landing legs)
// ============================================================================
// Engine exhaust: a bright cone pointing down from the nozzle exit at y = exitY.
// `thrust` (0..1) sets the flame length.
static void drawFlame(const Renderer& r, const Primitives& p, const Mat4& base,
                      float exitY, float width, float length, float thrust)
{
    if (thrust <= 0.0f)
        return;
    float len = length * thrust;
    r.drawSolid(p.cone, base * translate(0.0f, exitY - len / 2, 0.0f) * rotateX(180.0f)
                            * scale(width, len, width), 1.0f);
}

// Landing legs hinged on the side of a stage. Stowed (deploy = 0) they lie
// along the body pointing up; deployed (deploy = 1) they swing out and down
// by LEG_DEPLOY_ANGLE (rotation about the hinge) so the feet stand below the engine.
static void drawLegs(const Renderer& r, const Primitives& p, const Mat4& base, const LegGeometry& g,
                     float deploy)
{
    if (deploy <= 0.001f)
        return;   // folded flat into the body
    float angle = LEG_DEPLOY_ANGLE * deploy;
    for (int i = 0; i < g.count; ++i)
    {
        Mat4 hinge = base * rotateY(g.firstAngle + i * 360.0f / g.count)
                   * translate(g.hingeRadius, g.hingeY, 0.0f) * rotateZ(-angle);
        r.drawPart(p.cube, hinge * translate(0.0f, g.length / 2, 0.0f) * scale(0.14f, g.length, 0.14f));
        // Foot pad stays level with the ground.
        r.drawPart(p.cylinder, hinge * translate(0.0f, g.length, 0.0f) * rotateZ(angle)
                                   * scale(g.footSize, 0.1f, g.footSize));
    }
}

Mat4 rocketBoosterMatrix(const Mat4& rocketBase, int index)
{
    return rocketBase * rotateY(index * 180.0f) * translate(1.15f, 0.0f, 0.0f);
}

void drawRocketBooster(const Renderer& r, const Primitives& p, const Mat4& booster, float thrust, float legs)
{
    r.drawPart(p.frustum,  booster * translate(0.0f, 0.65f, 0.0f) * scale(0.55f, 0.5f, 0.55f));
    r.drawPart(p.cylinder, booster * translate(0.0f, 2.9f, 0.0f)  * scale(0.7f, 4.0f, 0.7f));
    r.drawPart(p.cone,     booster * translate(0.0f, 5.4f, 0.0f)  * scale(0.7f, 1.0f, 0.7f));
    drawLegs(r, p, booster, BOOSTER_LEGS, legs);
    drawFlame(r, p, booster, 0.4f, 0.5f, 3.0f, thrust);
}

void drawRocket(const Renderer& r, const Primitives& p, const Mat4& base, const RocketLook& look)
{
    // Core stack, bottom to top.
    r.drawPart(p.frustum,     base * translate(0.0f, 0.45f, 0.0f) * scale(1.2f, 0.9f, 1.2f));    // engine nozzle
    r.drawPart(p.cylinder,    base * translate(0.0f, 3.65f, 0.0f) * scale(1.6f, 5.5f, 1.6f));    // first stage
    r.drawPart(p.frustumWide, base * translate(0.0f, 6.65f, 0.0f) * scale(1.6f, 0.5f, 1.6f));    // interstage
    r.drawPart(p.cylinder,    base * translate(0.0f, 8.9f, 0.0f)  * scale(1.28f, 4.0f, 1.28f));  // second stage
    r.drawPart(p.cone,        base * translate(0.0f, 12.1f, 0.0f) * scale(1.28f, 2.4f, 1.28f));  // nose cone

    // Four fins around the base, 90 degrees apart.
    for (int i = 0; i < 4; ++i)
        r.drawPart(p.fin, base * rotateY(45.0f + i * 90.0f) * translate(0.0f, 1.9f, 0.75f)
                              * rotateZ(90.0f) * scale(2.0f, 0.12f, 1.1f));

    drawLegs(r, p, base, CORE_LEGS, look.legs);
    drawFlame(r, p, base, 0.0f, 1.0f, 5.0f, look.mainFlame);

    if (look.boosters)
        for (int i = 0; i < 2; ++i)
            drawRocketBooster(r, p, rocketBoosterMatrix(base, i), look.boosterFlame, 0.0f);
}
