#include "world/SceneParts.h"

// ---- Rocket base -----------------------------------------------------------------

void drawRocketBase(const Renderer& r, const Primitives& p)
{
    const Mat4 I = Mat4::identity();
    const Vec3& c = ROCKET_BASE_POS;

    // Concrete apron under the three pads.
    box(r, p, I, {c.x, 0.05f, c.z}, {60.0f, 0.1f, 22.0f}, Paint::CONCRETE * 0.85f);

    // Landing pads 3, 4 and 5.
    for (int i = 0; i < 3; ++i)
    {
        const Vec3& pad = ROCKET_BASE_PADS[i];
        drawLandingZone(r, p, translate(pad.x, pad.y, pad.z), 3 + i);
    }

    // Recovery hangar behind the pads, big door facing them (north, -Z).
    const float hx = c.x, hz = c.z + 24.0f, w = 34.0f, d = 14.0f, h = 11.0f;
    box(r, p, I, {hx, h / 2, hz}, {w, h, d}, Paint::BUILDING);
    box(r, p, I, {hx, h + 0.3f, hz}, {w + 1.0f, 0.6f, d + 1.0f}, Paint::ROOF);
    box(r, p, I, {hx, 4.5f, hz - d / 2 - 0.05f}, {22.0f, 9.0f, 0.1f}, Paint::GLASS);   // door
    for (float x = -8.8f; x <= 8.9f; x += 4.4f)                                       // door panels
        box(r, p, I, {hx + x, 4.5f, hz - d / 2 - 0.12f}, {0.15f, 9.0f, 0.1f}, Paint::METAL);

    // Concrete path from the apron to the hangar door.
    box(r, p, I, {hx, 0.04f, c.z + 14.0f}, {22.0f, 0.08f, 8.0f}, Paint::CONCRETE * 0.85f);
}

// ---- Rocket launch complex -----------------------------------------------------

// Square lattice tower: 4 corner columns, horizontal rings and X-bracing.
static void drawLatticeTower(const Renderer& r, const Primitives& p, const Mat4& base,
                             float half, float height, int bays)
{
    const float bay = height / bays;
    const float diag = std::sqrt((2 * half) * (2 * half) + bay * bay);
    const float angle = std::atan2(bay, 2 * half) * 180.0f / PI;

    for (float sx : {-half, half})
        for (float sz : {-half, half})
            box(r, p, base, {sx, height / 2, sz}, {0.3f, height, 0.3f}, Paint::METAL);

    for (int i = 0; i <= bays; ++i)
    {
        float y = i * bay;
        for (float s : {-half, half})
        {
            box(r, p, base, {0, y, s}, {2 * half, 0.18f, 0.18f}, Paint::METAL);   // faces at +/-Z
            box(r, p, base, {s, y, 0}, {0.18f, 0.18f, 2 * half}, Paint::METAL);   // faces at +/-X
        }
        if (i == bays)
            break;

        // Diagonal brace in every bay, alternating direction.
        float yc = y + bay / 2;
        float dir = (i % 2 == 0) ? 1.0f : -1.0f;
        for (float s : {-half, half})
        {
            r.drawPart(p.cube, base * translate(0, yc, s) * rotateZ(dir * angle)
                                   * scale(diag, 0.12f, 0.12f), Paint::METAL);
            r.drawPart(p.cube, base * translate(s, yc, 0) * rotateX(-dir * angle)
                                   * scale(0.12f, 0.12f, diag), Paint::METAL);
        }
    }
}

void drawLaunchPad(const Renderer& r, const Primitives& p, const Mat4& base, float armSwing)
{
    const float padH = 1.5f;

    // Octagonal concrete pad and the access ramp (for the crawler).
    r.drawPart(p.octagon, base * translate(0, padH / 2, 0) * rotateY(22.5f) * scale(22, padH, 22), Paint::CONCRETE);
    r.drawPart(p.cube, base * translate(-15.5f, 0.55f, 0) * rotateZ(8.0f) * scale(11, 0.4f, 7), Paint::CONCRETE);

    // Flame trench opening running out from under the rocket.
    paint(r, p, base, 0, 5.0f, 3.5f, 10.0f, padH, Paint::DARK);

    // Launch mount: four pillars carrying a ring the rocket stands on.
    for (float sx : {-1.9f, 1.9f})
        for (float sz : {-1.9f, 1.9f})
            box(r, p, base, {sx, padH + 1.5f, sz}, {0.8f, 3.0f, 0.8f}, Paint::METAL);
    r.drawPart(p.ringThick, base * translate(0, LAUNCH_MOUNT_TOP - 0.25f, 0) * scale(5.4f, 0.5f, 5.4f), Paint::METAL);

    // Hold-down clamps gripping the first stage.
    for (float sz : {-0.95f, 0.95f})
        box(r, p, base, {0, LAUNCH_MOUNT_TOP + 0.45f, sz}, {0.3f, 0.9f, 0.5f}, Paint::DARK);

    // Service tower beside the rocket.
    const float towerX = 6.0f, towerH = 18.0f;
    Mat4 tower = base * translate(towerX, padH, 0);
    drawLatticeTower(r, p, tower, 1.5f, towerH, 8);
    box(r, p, tower, {0, towerH + 0.15f, 0}, {3.6f, 0.3f, 3.6f}, Paint::METAL);           // top platform
    post(r, p, tower, 0, towerH + 0.3f, 0, 0.15f, 5.0f, Paint::METAL);                   // lightning mast

    // Service arms reaching to the second stage. They are hinged at the tower
    // face and swing away (rotate about Y) before launch.
    const float hingeX = towerX - 1.5f, armLen = 3.8f;
    for (float y : {LAUNCH_MOUNT_TOP + 7.0f, LAUNCH_MOUNT_TOP + 9.5f})
        r.drawPart(p.cube, base * translate(hingeX, y, 0) * rotateY(armSwing * 80.0f)
                               * translate(-armLen / 2, 0, 0) * scale(armLen, 0.5f, 0.9f), Paint::METAL);

    // Propellant tank on legs.
    const float tx = -5.5f, tz = -5.5f, tankY = padH + 3.5f;
    r.drawPart(p.sphere, base * translate(tx, tankY, tz) * scale(4.5f, 4.5f, 4.5f), Paint::BUILDING);
    for (float dx : {-1.4f, 1.4f})
        for (float dz : {-1.4f, 1.4f})
            post(r, p, base, tx + dx, padH, tz + dz, 0.3f, 3.5f, Paint::METAL);
}
