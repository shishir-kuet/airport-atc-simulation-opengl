#include "models/ModelParts.h"

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
                            * scale(width, len, width), Livery::FLAME);
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
        r.drawPart(p.cube, hinge * translate(0.0f, g.length / 2, 0.0f) * scale(0.14f, g.length, 0.14f), Livery::METAL);
        // Foot pad stays level with the ground.
        r.drawPart(p.cylinder, hinge * translate(0.0f, g.length, 0.0f) * rotateZ(angle)
                                   * scale(g.footSize, 0.1f, g.footSize), Livery::METAL);
    }
}

Mat4 rocketBoosterMatrix(const Mat4& rocketBase, int index)
{
    return rocketBase * rotateY(index * 180.0f) * translate(1.15f, 0.0f, 0.0f);
}

void drawRocketBooster(const Renderer& r, const Primitives& p, const Mat4& booster, float thrust)
{
    r.drawPart(p.frustum,  booster * translate(0.0f, 0.65f, 0.0f) * scale(0.55f, 0.5f, 0.55f), Livery::NOZZLE);
    r.drawPart(p.cylinder, booster * translate(0.0f, 2.9f, 0.0f)  * scale(0.7f, 4.0f, 0.7f), Livery::ROCKET_BODY);
    r.drawPart(p.cone,     booster * translate(0.0f, 5.4f, 0.0f)  * scale(0.7f, 1.0f, 0.7f), Livery::ROCKET_BODY);
    drawFlame(r, p, booster, 0.4f, 0.5f, 3.0f, thrust);
}

void drawRocketFirstStage(const Renderer& r, const Primitives& p, const Mat4& base, float thrust, float legs)
{
    r.drawPart(p.frustum,     base * translate(0.0f, 0.45f, 0.0f) * scale(1.2f, 0.9f, 1.2f), Livery::NOZZLE);    // engine nozzle
    r.drawPart(p.cylinder,    base * translate(0.0f, 3.65f, 0.0f) * scale(1.6f, 5.5f, 1.6f), Livery::ROCKET_BODY);    // first stage
    r.drawPart(p.frustumWide, base * translate(0.0f, 6.65f, 0.0f) * scale(1.6f, 0.5f, 1.6f), Livery::INTERSTAGE);    // interstage

    // Four fins around the base, 90 degrees apart.
    for (int i = 0; i < 4; ++i)
        r.drawPart(p.fin, base * rotateY(45.0f + i * 90.0f) * translate(0.0f, 1.9f, 0.75f)
                              * rotateZ(90.0f) * scale(2.0f, 0.12f, 1.1f), Livery::FIN);

    drawLegs(r, p, base, CORE_LEGS, legs);
    drawFlame(r, p, base, 0.0f, 1.0f, 5.0f, thrust);
}

void drawRocketUpperStage(const Renderer& r, const Primitives& p, const Mat4& base, float thrust, float legs)
{
    // Its own engine sits inside the interstage until the first stage drops away.
    r.drawPart(p.frustum,  base * translate(0.0f, UPPER_STAGE_BOTTOM + 0.25f, 0.0f) * scale(0.8f, 0.5f, 0.8f), Livery::NOZZLE);
    r.drawPart(p.cylinder, base * translate(0.0f, 8.9f, 0.0f)  * scale(1.28f, 4.0f, 1.28f), Livery::ROCKET_BODY);   // second stage
    r.drawPart(p.cone,     base * translate(0.0f, 12.1f, 0.0f) * scale(1.28f, 2.4f, 1.28f), Livery::ROCKET_BODY);   // nose cone
    drawLegs(r, p, base, UPPER_LEGS, legs);
    drawFlame(r, p, base, UPPER_STAGE_BOTTOM, 0.9f, 5.0f, thrust);
}

void drawRocket(const Renderer& r, const Primitives& p, const Mat4& base, const RocketLook& look)
{
    drawRocketFirstStage(r, p, base, look.mainFlame, look.legs);
    drawRocketUpperStage(r, p, base, 0.0f, 0.0f);

    if (look.boosters)
        for (int i = 0; i < 2; ++i)
            drawRocketBooster(r, p, rocketBoosterMatrix(base, i), look.boosterFlame);
}
