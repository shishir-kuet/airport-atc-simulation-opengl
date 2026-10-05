#include "world/SceneParts.h"

// ---- Ground ----------------------------------------------------------------

void drawGround(const Renderer& r, const Primitives& p)
{
    // Large enough for the holding patterns and the rocket stages flying
    // downrange. It is a deep slab, not a thin sheet: seen from a rocket more
    // than a kilometre up, the depth buffer can no longer tell two surfaces a
    // fraction of a unit apart from each other, and a thin ground would fight
    // with its own underside in stripes across the whole screen.
    box(r, p, Mat4::identity(), {0.0f, -30.02f, 0.0f}, {5000.0f, 60.0f, 5000.0f}, Paint::GRASS);
}

// ---- Runway ----------------------------------------------------------------

static void drawRunway(const Renderer& r, const Primitives& p)
{
    const Mat4 base = translate(RUNWAY_CENTER.x, RUNWAY_CENTER.y, RUNWAY_CENTER.z);
    const float halfL = RUNWAY_LENGTH / 2;   // 65
    const float halfW = RUNWAY_WIDTH / 2;    // 6
    const float top = 0.06f;                 // runway sits slightly above taxiways

    // Asphalt surface and blast pads beyond both ends.
    box(r, p, base, {0, top / 2, 0}, {RUNWAY_LENGTH, top, RUNWAY_WIDTH}, Paint::ASPHALT);
    for (float end : {-1.0f, 1.0f})
        box(r, p, base, {end * (halfL + 4), 0.025f, 0}, {8, 0.05f, RUNWAY_WIDTH}, Paint::TAXIWAY);

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
            post(r, p, base, end * (halfL + 0.4f), 0, z, 0.2f, 0.3f, Paint::LIGHT);

        // Approach lighting: poles with cross bars leading to the runway.
        for (int i = 1; i <= 5; ++i)
        {
            float x = end * (halfL + 6 + i * 4);
            float h = 0.8f + i * 0.25f;
            post(r, p, base, x, 0, 0, 0.15f, h, Paint::METAL);
            box(r, p, base, {x, h, 0}, {0.2f, 0.15f, 3.0f}, Paint::LIGHT);
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
            post(r, p, base, x, 0, side * (halfW + 0.4f), 0.25f, 0.4f, Paint::LIGHT);
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
        {taxiMaxX - taxiMinX + TAXIWAY_WIDTH, top, TAXIWAY_WIDTH}, Paint::TAXIWAY);
    paint(r, p, I, (taxiMinX + taxiMaxX) / 2, TAXIWAY_Z, taxiMaxX - taxiMinX, 0.2f, top, Paint::TAXILINE);

    // Connectors from the taxiway to the runway, each with a holding line.
    for (float x : {taxiMinX, TAXI_MID_X, taxiMaxX})
    {
        // Pavement runs from just under the runway edge to the taxiway edge.
        float z0 = runwayEdgeZ - 0.5f, z1 = TAXIWAY_Z - TAXIWAY_WIDTH / 2;
        box(r, p, I, {x, top / 2, (z0 + z1) / 2}, {TAXIWAY_WIDTH, top, z1 - z0}, Paint::TAXIWAY);
        float len = TAXIWAY_Z - runwayEdgeZ;
        paint(r, p, I, x, runwayEdgeZ + len / 2, 0.2f, len, top, Paint::TAXILINE);

        // Runway holding position: two solid bars across the connector.
        paint(r, p, I, x, HOLD_LINE_Z,        TAXIWAY_WIDTH - 0.4f, 0.25f, top, Paint::TAXILINE);
        paint(r, p, I, x, HOLD_LINE_Z - 0.6f, TAXIWAY_WIDTH - 0.4f, 0.25f, top, Paint::TAXILINE);
    }

    // Taxiway edge lights (skipping the side that joins the apron).
    for (float x = taxiMinX; x < taxiMaxX; x += 10)
    {
        post(r, p, I, x + 5, 0, TAXIWAY_Z - TAXIWAY_WIDTH / 2 - 0.4f, 0.2f, 0.3f, Paint::LIGHT);
        if (x + 5 < APRON_MIN_X || x + 5 > APRON_MAX_X)
            post(r, p, I, x + 5, 0, TAXIWAY_Z + TAXIWAY_WIDTH / 2 + 0.4f, 0.2f, 0.3f, Paint::LIGHT);
    }

    // Apron (concrete parking area).
    box(r, p, I, {(APRON_MIN_X + APRON_MAX_X) / 2, top / 2, (APRON_MIN_Z + APRON_MAX_Z) / 2},
        {APRON_MAX_X - APRON_MIN_X, top, APRON_MAX_Z - APRON_MIN_Z}, Paint::CONCRETE);

    // Parking stands: lead-in line, stop bar and stand number.
    for (int i = 0; i < 3; ++i)
    {
        float sx = STAND_X[i];
        float leadStart = TAXIWAY_Z, leadEnd = STAND_STOP_Z + 8;
        paint(r, p, I, sx, (leadStart + leadEnd) / 2, 0.2f, leadEnd - leadStart, top, Paint::TAXILINE);
        paint(r, p, I, sx, STAND_STOP_Z - 0.4f, 2.5f, 0.3f, top, Paint::MARKING);

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

    box(r, p, I, {cx, height / 2, frontZ + depth / 2}, {width, height, depth}, Paint::BUILDING);
    box(r, p, I, {cx, height + 0.25f, frontZ + depth / 2}, {width + 1, 0.5f, depth + 1}, Paint::ROOF);

    // Two window bands on the apron side.
    for (float y : {2.2f, 5.0f})
        box(r, p, I, {cx, y, frontZ - 0.05f}, {width - 2, 1.4f, 0.1f}, Paint::GLASS);

    // Jet bridge at each stand: tunnel from the terminal to the aircraft door.
    for (float sx : STAND_X)
    {
        float tunnelX = sx - 2.6f;
        float nearZ = 2.4f;
        float len = frontZ - nearZ;
        box(r, p, I, {tunnelX, 2.9f, nearZ + len / 2}, {1.4f, 1.6f, len}, Paint::METAL);
        box(r, p, I, {sx - 1.3f, 2.9f, 3.2f}, {1.2f, 1.6f, 1.6f}, Paint::METAL);   // head at the door

        for (float z : {10.0f, 16.0f})
        {
            post(r, p, I, tunnelX, 0, z, 0.3f, 2.1f, Paint::METAL);                   // support leg
            box(r, p, I, {tunnelX, 0.25f, z}, {1.2f, 0.4f, 0.6f}, Paint::DARK);        // wheel bogie
        }
    }
}

// ---- ATC tower ---------------------------------------------------------------

static void drawTower(const Renderer& r, const Primitives& p, float radarAngle)
{
    const Mat4 base = translate(TOWER_POS.x, TOWER_POS.y, TOWER_POS.z);

    box(r, p, base, {0, 1.75f, 0}, {7, 3.5f, 7}, Paint::BUILDING);                     // base building
    r.drawPart(p.cylinder, base * translate(0, 11, 0) * scale(2.6f, 15, 2.6f), Paint::BUILDING);  // shaft
    r.drawPart(p.cylinder, base * translate(0, 18.7f, 0) * scale(5.6f, 0.4f, 5.6f), Paint::ROOF); // cab floor

    // Control cab: glass that widens towards the top (upside-down frustum).
    r.drawPart(p.frustumWide, base * translate(0, 20.4f, 0) * rotateX(180.0f) * scale(5.2f, 3, 5.2f), Paint::GLASS);

    // Window frames, leaning outwards with the glass.
    for (int i = 0; i < 8; ++i)
        r.drawPart(p.cube, base * rotateY(i * 45.0f) * translate(2.34f, 20.4f, 0)
                               * rotateZ(-9.8f) * scale(0.12f, 3.05f, 0.12f), Paint::METAL);

    r.drawPart(p.cylinder, base * translate(0, 22.1f, 0) * scale(6, 0.4f, 6), Paint::ROOF);        // roof

    // Rotating radar antenna and a radio mast.
    r.drawPart(p.cylinder, base * translate(0, 22.9f, 0) * scale(0.3f, 1.2f, 0.3f), Paint::METAL);
    box(r, p, base * translate(0, 23.6f, 0) * rotateY(radarAngle), {0, 0, 0}, {3, 0.6f, 0.2f}, Paint::LIGHT);
    post(r, p, base, 1.8f, 22.3f, 1.8f, 0.08f, 3.0f, Paint::METAL);
}

void drawAirport(const Renderer& r, const Primitives& p, float radarAngle)
{
    drawRunway(r, p);
    drawTaxiwaysAndApron(r, p);
    drawTerminal(r, p);
    drawTower(r, p, radarAngle);
}
