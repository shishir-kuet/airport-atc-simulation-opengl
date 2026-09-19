#include "Environment.h"

#include <cmath>

using namespace Layout;

// Grey levels (week 1 has no colour; different greys keep surfaces apart).
namespace Shade
{
    constexpr float GROUND   = 0.18f;
    constexpr float ASPHALT  = 0.30f;
    constexpr float TAXIWAY  = 0.36f;
    constexpr float CONCRETE = 0.50f;
    constexpr float MARKING  = 0.95f;
    constexpr float TAXILINE = 0.72f;
    constexpr float BUILDING = 0.68f;
    constexpr float ROOF     = 0.55f;
    constexpr float GLASS    = 0.25f;
    constexpr float METAL    = 0.62f;
    constexpr float LIGHT    = 0.90f;
    constexpr float DARK     = 0.08f;
}

// Painted markings are thin boxes lying just above the pavement.
constexpr float MARK_THICKNESS = 0.02f;

// ---- Small helpers ---------------------------------------------------------

static void box(const Renderer& r, const Primitives& p, const Mat4& base,
                Vec3 center, Vec3 size, float shade)
{
    r.drawPart(p.cube, base * translate(center.x, center.y, center.z) * scale(size.x, size.y, size.z), shade);
}

// A flat painted rectangle centred at (x, z) on a surface whose top is at surfaceY.
static void paint(const Renderer& r, const Primitives& p, const Mat4& base,
                  float x, float z, float sizeX, float sizeZ, float surfaceY,
                  float shade = Shade::MARKING)
{
    box(r, p, base, {x, surfaceY + MARK_THICKNESS * 0.5f, z}, {sizeX, MARK_THICKNESS, sizeZ}, shade);
}

// Vertical post (cylinder) standing on y = baseY.
static void post(const Renderer& r, const Primitives& p, const Mat4& base,
                 float x, float baseY, float z, float diameter, float height, float shade)
{
    r.drawPart(p.cylinder, base * translate(x, baseY + height * 0.5f, z) * scale(diameter, height, diameter), shade);
}

// Seven-segment digit painted on the ground.
// In `frame`, local +X is the reading "up" direction and local +Z is "right".
static void paintDigit(const Renderer& r, const Primitives& p, const Mat4& frame,
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

// ---- Ground ----------------------------------------------------------------

void drawGround(const Renderer& r, const Primitives& p)
{
    // Large enough for the holding patterns and the rocket stages flying downrange.
    box(r, p, Mat4::identity(), {0.0f, -0.07f, 0.0f}, {5000.0f, 0.1f, 5000.0f}, Shade::GROUND);
}

// ---- Runway ----------------------------------------------------------------

static void drawRunway(const Renderer& r, const Primitives& p)
{
    const Mat4 base = translate(RUNWAY_CENTER.x, RUNWAY_CENTER.y, RUNWAY_CENTER.z);
    const float halfL = RUNWAY_LENGTH / 2;   // 65
    const float halfW = RUNWAY_WIDTH / 2;    // 6
    const float top = 0.06f;                 // runway sits slightly above taxiways

    // Asphalt surface and blast pads beyond both ends.
    box(r, p, base, {0, top / 2, 0}, {RUNWAY_LENGTH, top, RUNWAY_WIDTH}, Shade::ASPHALT);
    for (float end : {-1.0f, 1.0f})
        box(r, p, base, {end * (halfL + 4), 0.025f, 0}, {8, 0.05f, RUNWAY_WIDTH}, Shade::TAXIWAY);

    // Side stripes along both edges.
    for (float side : {-1.0f, 1.0f})
        paint(r, p, base, 0, side * (halfW - 0.4f), RUNWAY_LENGTH - 1, 0.3f, top);

    // Dashed centreline.
    for (float x = -46; x <= 46; x += 7)
        paint(r, p, base, x, 0, 4, 0.35f, top);

    for (float end : {-1.0f, 1.0f})
    {
        // Threshold "piano keys": 4 stripes on each side of the centreline.
        for (int k = 0; k < 4; ++k)
            for (float side : {-1.0f, 1.0f})
                paint(r, p, base, end * (halfL - 3.5f), side * (0.9f + k * 1.2f), 5, 0.6f, top);

        // Aiming point (two large bars) and touchdown zone bars.
        for (float side : {-1.0f, 1.0f})
        {
            paint(r, p, base, end * 44, side * 2.8f, 6, 1.2f, top);
            paint(r, p, base, end * 36, side * 2.4f, 3.5f, 0.45f, top);
            paint(r, p, base, end * 36, side * 3.2f, 3.5f, 0.45f, top);
        }

        // Threshold lights across the runway end.
        for (float z = -halfW + 0.5f; z <= halfW - 0.4f; z += 1.1f)
            post(r, p, base, end * (halfL + 0.4f), 0, z, 0.2f, 0.3f, Shade::LIGHT);

        // Approach lighting: poles with cross bars leading to the runway.
        for (int i = 1; i <= 5; ++i)
        {
            float x = end * (halfL + 6 + i * 4);
            float h = 0.8f + i * 0.25f;
            post(r, p, base, x, 0, 0, 0.15f, h, Shade::METAL);
            box(r, p, base, {x, h, 0}, {0.2f, 0.15f, 3.0f}, Shade::LIGHT);
        }
    }

    // Runway designators, read by a pilot on approach: "09" at the west end,
    // "27" at the east end (the frame is turned 180 degrees).
    Mat4 west = base * translate(-halfL + 9, 0, 0);
    paintDigit(r, p, west, 0, -1.6f, top);
    paintDigit(r, p, west, 9, 1.6f, top);
    Mat4 east = base * translate(halfL - 9, 0, 0) * rotateY(180.0f);
    paintDigit(r, p, east, 2, -1.6f, top);
    paintDigit(r, p, east, 7, 1.6f, top);

    // Edge lights along both sides.
    for (float x = -60; x <= 60; x += 10)
        for (float side : {-1.0f, 1.0f})
            post(r, p, base, x, 0, side * (halfW + 0.4f), 0.25f, 0.4f, Shade::LIGHT);
}

// ---- Taxiways and apron ----------------------------------------------------

static void drawTaxiwaysAndApron(const Renderer& r, const Primitives& p)
{
    const Mat4 I = Mat4::identity();
    const float top = PAVEMENT_TOP;
    const float taxiMinX = TAXI_MIN_X, taxiMaxX = TAXI_MAX_X;
    const float runwayEdgeZ = RUNWAY_CENTER.z + RUNWAY_WIDTH / 2;   // -24

    // Parallel taxiway with its centreline.
    box(r, p, I, {(taxiMinX + taxiMaxX) / 2, top / 2, TAXIWAY_Z},
        {taxiMaxX - taxiMinX + TAXIWAY_WIDTH, top, TAXIWAY_WIDTH}, Shade::TAXIWAY);
    paint(r, p, I, (taxiMinX + taxiMaxX) / 2, TAXIWAY_Z, taxiMaxX - taxiMinX, 0.2f, top, Shade::TAXILINE);

    // Connectors from the taxiway to the runway, each with a holding line.
    for (float x : {taxiMinX, TAXI_MID_X, taxiMaxX})
    {
        // Pavement runs from just under the runway edge to the taxiway edge.
        float z0 = runwayEdgeZ - 0.5f, z1 = TAXIWAY_Z - TAXIWAY_WIDTH / 2;
        box(r, p, I, {x, top / 2, (z0 + z1) / 2}, {TAXIWAY_WIDTH, top, z1 - z0}, Shade::TAXIWAY);
        float len = TAXIWAY_Z - runwayEdgeZ;
        paint(r, p, I, x, runwayEdgeZ + len / 2, 0.2f, len, top, Shade::TAXILINE);

        // Runway holding position: two solid bars across the connector.
        paint(r, p, I, x, HOLD_LINE_Z,        TAXIWAY_WIDTH - 0.4f, 0.25f, top, Shade::TAXILINE);
        paint(r, p, I, x, HOLD_LINE_Z - 0.6f, TAXIWAY_WIDTH - 0.4f, 0.25f, top, Shade::TAXILINE);
    }

    // Taxiway edge lights (skipping the side that joins the apron).
    for (float x = taxiMinX; x < taxiMaxX; x += 10)
    {
        post(r, p, I, x + 5, 0, TAXIWAY_Z - TAXIWAY_WIDTH / 2 - 0.4f, 0.2f, 0.3f, Shade::LIGHT);
        if (x + 5 < APRON_MIN_X || x + 5 > APRON_MAX_X)
            post(r, p, I, x + 5, 0, TAXIWAY_Z + TAXIWAY_WIDTH / 2 + 0.4f, 0.2f, 0.3f, Shade::LIGHT);
    }

    // Apron (concrete parking area).
    box(r, p, I, {(APRON_MIN_X + APRON_MAX_X) / 2, top / 2, (APRON_MIN_Z + APRON_MAX_Z) / 2},
        {APRON_MAX_X - APRON_MIN_X, top, APRON_MAX_Z - APRON_MIN_Z}, Shade::CONCRETE);

    // Parking stands: lead-in line, stop bar and stand number.
    for (int i = 0; i < 3; ++i)
    {
        float sx = STAND_X[i];
        float leadStart = TAXIWAY_Z, leadEnd = STAND_STOP_Z + 8;
        paint(r, p, I, sx, (leadStart + leadEnd) / 2, 0.2f, leadEnd - leadStart, top, Shade::TAXILINE);
        paint(r, p, I, sx, STAND_STOP_Z - 0.4f, 2.5f, 0.3f, top, Shade::MARKING);

        // Number faces a pilot taxiing in (+Z): up = +Z, right = -X.
        Mat4 frame = translate(sx, 0, -7.0f) * rotateY(-90.0f) * scale(0.6f, 1.0f, 0.6f);
        paintDigit(r, p, frame, i + 1, -4.0f, top);
    }
}

// ---- Terminal building with jet bridges -------------------------------------

static void drawTerminal(const Renderer& r, const Primitives& p)
{
    const Mat4 I = Mat4::identity();
    const float cx = (APRON_MIN_X + APRON_MAX_X) / 2;
    const float width = APRON_MAX_X - APRON_MIN_X;
    const float frontZ = 20.0f, depth = 8.0f, height = 7.0f;

    box(r, p, I, {cx, height / 2, frontZ + depth / 2}, {width, height, depth}, Shade::BUILDING);
    box(r, p, I, {cx, height + 0.25f, frontZ + depth / 2}, {width + 1, 0.5f, depth + 1}, Shade::ROOF);

    // Two window bands on the apron side.
    for (float y : {2.2f, 5.0f})
        box(r, p, I, {cx, y, frontZ - 0.05f}, {width - 2, 1.4f, 0.1f}, Shade::GLASS);

    // Jet bridge at each stand: tunnel from the terminal to the aircraft door.
    for (float sx : STAND_X)
    {
        float tunnelX = sx - 2.6f;
        float nearZ = 2.4f;
        float len = frontZ - nearZ;
        box(r, p, I, {tunnelX, 2.9f, nearZ + len / 2}, {1.4f, 1.6f, len}, Shade::METAL);
        box(r, p, I, {sx - 1.3f, 2.9f, 3.2f}, {1.2f, 1.6f, 1.6f}, Shade::METAL);   // head at the door

        for (float z : {10.0f, 16.0f})
        {
            post(r, p, I, tunnelX, 0, z, 0.3f, 2.1f, Shade::METAL);                   // support leg
            box(r, p, I, {tunnelX, 0.25f, z}, {1.2f, 0.4f, 0.6f}, Shade::DARK);        // wheel bogie
        }
    }
}

// ---- ATC tower ---------------------------------------------------------------

static void drawTower(const Renderer& r, const Primitives& p, float radarAngle)
{
    const Mat4 base = translate(TOWER_POS.x, TOWER_POS.y, TOWER_POS.z);

    box(r, p, base, {0, 1.75f, 0}, {7, 3.5f, 7}, Shade::BUILDING);                     // base building
    r.drawPart(p.cylinder, base * translate(0, 11, 0) * scale(2.6f, 15, 2.6f), Shade::BUILDING);  // shaft
    r.drawPart(p.cylinder, base * translate(0, 18.7f, 0) * scale(5.6f, 0.4f, 5.6f), Shade::ROOF); // cab floor

    // Control cab: glass that widens towards the top (upside-down frustum).
    r.drawPart(p.frustumWide, base * translate(0, 20.4f, 0) * rotateX(180.0f) * scale(5.2f, 3, 5.2f), Shade::GLASS);

    // Window frames, leaning outwards with the glass.
    for (int i = 0; i < 8; ++i)
        r.drawPart(p.cube, base * rotateY(i * 45.0f) * translate(2.34f, 20.4f, 0)
                               * rotateZ(-9.8f) * scale(0.12f, 3.05f, 0.12f), Shade::METAL);

    r.drawPart(p.cylinder, base * translate(0, 22.1f, 0) * scale(6, 0.4f, 6), Shade::ROOF);        // roof

    // Rotating radar antenna and a radio mast.
    r.drawPart(p.cylinder, base * translate(0, 22.9f, 0) * scale(0.3f, 1.2f, 0.3f), Shade::METAL);
    box(r, p, base * translate(0, 23.6f, 0) * rotateY(radarAngle), {0, 0, 0}, {3, 0.6f, 0.2f}, Shade::LIGHT);
    post(r, p, base, 1.8f, 22.3f, 1.8f, 0.08f, 3.0f, Shade::METAL);
}

void drawAirport(const Renderer& r, const Primitives& p, float radarAngle)
{
    drawRunway(r, p);
    drawTaxiwaysAndApron(r, p);
    drawTerminal(r, p);
    drawTower(r, p, radarAngle);
}

// ---- Helipad -------------------------------------------------------------------

void drawHelipad(const Renderer& r, const Primitives& p, const Mat4& base)
{
    const float top = HELIPAD_TOP;

    // Round concrete pad.
    r.drawPart(p.cylinder, base * translate(0, top / 2, 0) * scale(16, top, 16), Shade::CONCRETE);

    // Touchdown circle and the "H".
    r.drawPart(p.ringThin, base * translate(0, top + MARK_THICKNESS / 2, 0) * scale(11, MARK_THICKNESS, 11), Shade::MARKING);
    paint(r, p, base, 0, -1.4f, 5.0f, 0.8f, top);    // left leg
    paint(r, p, base, 0,  1.4f, 5.0f, 0.8f, top);    // right leg
    paint(r, p, base, 0,  0.0f, 0.8f, 2.0f, top);    // cross bar

    // Perimeter lights.
    for (int i = 0; i < 16; ++i)
    {
        float a = radians(i * 22.5f);
        post(r, p, base, 7.6f * std::cos(a), top, 7.6f * std::sin(a), 0.2f, 0.25f, Shade::LIGHT);
    }

    // Windsock: pole with a tapered sock pointing downwind (+X).
    const float wx = 9.5f, wz = 9.5f, poleH = 5.0f;
    post(r, p, base, wx, 0, wz, 0.15f, poleH, Shade::METAL);
    r.drawPart(p.frustum, base * translate(wx + 1.25f, poleH - 0.3f, wz) * rotateZ(-95.0f)
                              * scale(0.7f, 2.4f, 0.7f), Shade::LIGHT);
}

// ---- Booster landing zones ------------------------------------------------------

void drawLandingZone(const Renderer& r, const Primitives& p, const Mat4& base, int number)
{
    const float top = LANDING_ZONE_TOP;
    r.drawPart(p.cylinder, base * translate(0, top / 2, 0) * scale(14, top, 14), Shade::CONCRETE);
    r.drawPart(p.ringThin, base * translate(0, top + MARK_THICKNESS / 2, 0) * scale(10, MARK_THICKNESS, 10), Shade::MARKING);

    // Big "X" in the middle (two bars at +/-45 degrees).
    for (float a : {45.0f, -45.0f})
        r.drawPart(p.cube, base * translate(0, top + MARK_THICKNESS / 2, 0) * rotateY(a)
                               * scale(6.0f, MARK_THICKNESS, 0.7f), Shade::MARKING);

    // Zone number, readable from the south side.
    Mat4 frame = base * translate(0, 0, 6.8f) * rotateY(90.0f) * scale(0.35f, 1.0f, 0.35f);
    paintDigit(r, p, frame, number, 0.0f, top);

    for (int i = 0; i < 8; ++i)
    {
        float a = radians(i * 45.0f);
        post(r, p, base, 6.6f * std::cos(a), top, 6.6f * std::sin(a), 0.2f, 0.25f, Shade::LIGHT);
    }
}

// ---- Rocket base -----------------------------------------------------------------

void drawRocketBase(const Renderer& r, const Primitives& p)
{
    const Mat4 I = Mat4::identity();
    const Vec3& c = ROCKET_BASE_POS;

    // Concrete apron under the three pads.
    box(r, p, I, {c.x, 0.05f, c.z}, {60.0f, 0.1f, 22.0f}, Shade::CONCRETE * 0.85f);

    // Landing pads 3, 4 and 5.
    for (int i = 0; i < 3; ++i)
    {
        const Vec3& pad = ROCKET_BASE_PADS[i];
        drawLandingZone(r, p, translate(pad.x, pad.y, pad.z), 3 + i);
    }

    // Recovery hangar behind the pads, big door facing them (north, -Z).
    const float hx = c.x, hz = c.z + 24.0f, w = 34.0f, d = 14.0f, h = 11.0f;
    box(r, p, I, {hx, h / 2, hz}, {w, h, d}, Shade::BUILDING);
    box(r, p, I, {hx, h + 0.3f, hz}, {w + 1.0f, 0.6f, d + 1.0f}, Shade::ROOF);
    box(r, p, I, {hx, 4.5f, hz - d / 2 - 0.05f}, {22.0f, 9.0f, 0.1f}, Shade::GLASS);   // door
    for (float x = -8.8f; x <= 8.9f; x += 4.4f)                                       // door panels
        box(r, p, I, {hx + x, 4.5f, hz - d / 2 - 0.12f}, {0.15f, 9.0f, 0.1f}, Shade::METAL);

    // Concrete path from the apron to the hangar door.
    box(r, p, I, {hx, 0.04f, c.z + 14.0f}, {22.0f, 0.08f, 8.0f}, Shade::CONCRETE * 0.85f);
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
            box(r, p, base, {sx, height / 2, sz}, {0.3f, height, 0.3f}, Shade::METAL);

    for (int i = 0; i <= bays; ++i)
    {
        float y = i * bay;
        for (float s : {-half, half})
        {
            box(r, p, base, {0, y, s}, {2 * half, 0.18f, 0.18f}, Shade::METAL);   // faces at +/-Z
            box(r, p, base, {s, y, 0}, {0.18f, 0.18f, 2 * half}, Shade::METAL);   // faces at +/-X
        }
        if (i == bays)
            break;

        // Diagonal brace in every bay, alternating direction.
        float yc = y + bay / 2;
        float dir = (i % 2 == 0) ? 1.0f : -1.0f;
        for (float s : {-half, half})
        {
            r.drawPart(p.cube, base * translate(0, yc, s) * rotateZ(dir * angle)
                                   * scale(diag, 0.12f, 0.12f), Shade::METAL);
            r.drawPart(p.cube, base * translate(s, yc, 0) * rotateX(-dir * angle)
                                   * scale(0.12f, 0.12f, diag), Shade::METAL);
        }
    }
}

void drawLaunchPad(const Renderer& r, const Primitives& p, const Mat4& base, float armSwing)
{
    const float padH = 1.5f;

    // Octagonal concrete pad and the access ramp (for the crawler).
    r.drawPart(p.octagon, base * translate(0, padH / 2, 0) * rotateY(22.5f) * scale(22, padH, 22), Shade::CONCRETE);
    r.drawPart(p.cube, base * translate(-15.5f, 0.55f, 0) * rotateZ(8.0f) * scale(11, 0.4f, 7), Shade::CONCRETE);

    // Flame trench opening running out from under the rocket.
    paint(r, p, base, 0, 5.0f, 3.5f, 10.0f, padH, Shade::DARK);

    // Launch mount: four pillars carrying a ring the rocket stands on.
    for (float sx : {-1.9f, 1.9f})
        for (float sz : {-1.9f, 1.9f})
            box(r, p, base, {sx, padH + 1.5f, sz}, {0.8f, 3.0f, 0.8f}, Shade::METAL);
    r.drawPart(p.ringThick, base * translate(0, LAUNCH_MOUNT_TOP - 0.25f, 0) * scale(5.4f, 0.5f, 5.4f), Shade::METAL);

    // Hold-down clamps gripping the first stage.
    for (float sz : {-0.95f, 0.95f})
        box(r, p, base, {0, LAUNCH_MOUNT_TOP + 0.45f, sz}, {0.3f, 0.9f, 0.5f}, Shade::DARK);

    // Service tower beside the rocket.
    const float towerX = 6.0f, towerH = 18.0f;
    Mat4 tower = base * translate(towerX, padH, 0);
    drawLatticeTower(r, p, tower, 1.5f, towerH, 8);
    box(r, p, tower, {0, towerH + 0.15f, 0}, {3.6f, 0.3f, 3.6f}, Shade::METAL);           // top platform
    post(r, p, tower, 0, towerH + 0.3f, 0, 0.15f, 5.0f, Shade::METAL);                   // lightning mast

    // Service arms reaching to the second stage. They are hinged at the tower
    // face and swing away (rotate about Y) before launch.
    const float hingeX = towerX - 1.5f, armLen = 3.8f;
    for (float y : {LAUNCH_MOUNT_TOP + 7.0f, LAUNCH_MOUNT_TOP + 9.5f})
        r.drawPart(p.cube, base * translate(hingeX, y, 0) * rotateY(armSwing * 80.0f)
                               * translate(-armLen / 2, 0, 0) * scale(armLen, 0.5f, 0.9f), Shade::METAL);

    // Propellant tank on legs.
    const float tx = -5.5f, tz = -5.5f, tankY = padH + 3.5f;
    r.drawPart(p.sphere, base * translate(tx, tankY, tz) * scale(4.5f, 4.5f, 4.5f), Shade::BUILDING);
    for (float dx : {-1.4f, 1.4f})
        for (float dz : {-1.4f, 1.4f})
            post(r, p, base, tx + dx, padH, tz + dz, 0.3f, 3.5f, Shade::METAL);
}
