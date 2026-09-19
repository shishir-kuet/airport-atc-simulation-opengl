// Airport environment: ground, runway, taxiways, apron + terminal, ATC tower,
// helipad and rocket launch complex.
//
// World axes: X runs along the runway, Y is up, Z points from the runway
// towards the terminal. The ground is at y = 0.
#pragma once

#include "Math3D.h"
#include "Primitives.h"
#include "Renderer.h"

namespace Layout
{
    // Runway (along X)
    constexpr float RUNWAY_Z      = -45.0f;
    const Vec3  RUNWAY_CENTER(0.0f, 0.0f, RUNWAY_Z);
    constexpr float RUNWAY_LENGTH = 130.0f;
    constexpr float RUNWAY_WIDTH  = 12.0f;

    // Parallel taxiway and the apron (parking area) in front of the terminal
    constexpr float TAXIWAY_Z     = -14.0f;
    constexpr float TAXIWAY_WIDTH = 6.0f;
    constexpr float TAXI_MIN_X = -55.0f, TAXI_MID_X = -5.0f, TAXI_MAX_X = 45.0f;   // runway connectors
    // Runway holding position line on the connectors (aircraft wait before it).
    constexpr float HOLD_LINE_Z = RUNWAY_Z + RUNWAY_WIDTH / 2 + 6.0f;
    constexpr float APRON_MIN_X = -45.0f, APRON_MAX_X = 5.0f;
    constexpr float APRON_MIN_Z = -11.0f, APRON_MAX_Z = 18.0f;
    constexpr float STAND_X[3]  = {-35.0f, -20.0f, -5.0f};  // parking stands
    constexpr float STAND_STOP_Z = 2.0f;                     // nose wheel stops here

    // Other facilities
    const Vec3 TOWER_POS     (15.0f, 0.0f, 22.0f);
    const Vec3 HELIPAD_POS   (32.0f, 0.0f,  5.0f);
    const Vec3 LAUNCH_PAD_POS(75.0f, 0.0f, 12.0f);

    // Landing zones where the rocket's side boosters come back down
    // (LZ-1 east for booster 0, LZ-2 west for booster 1).
    const Vec3 LANDING_ZONE[2] = {
        Vec3(LAUNCH_PAD_POS.x + 22.0f, 0.0f, LAUNCH_PAD_POS.z + 32.0f),
        Vec3(LAUNCH_PAD_POS.x - 22.0f, 0.0f, LAUNCH_PAD_POS.z + 32.0f),
    };

    // Surface heights that vehicles stand on
    constexpr float PAVEMENT_TOP       = 0.05f;
    constexpr float HELIPAD_TOP        = 0.30f;
    constexpr float LAUNCH_MOUNT_TOP   = 5.00f;
    constexpr float LANDING_ZONE_TOP   = 0.20f;
}

void drawGround(const Renderer& r, const Primitives& p);

// Runway, taxiways, apron, terminal with jet bridges and the ATC tower.
// radarAngle (degrees) turns the radar antenna on top of the tower.
void drawAirport(const Renderer& r, const Primitives& p, float radarAngle);

// Circular helipad with "H" marking, lights and a windsock.
void drawHelipad(const Renderer& r, const Primitives& p, const Mat4& base);

// Round concrete landing zone with a ring, an "X" and its number (1 or 2).
void drawLandingZone(const Renderer& r, const Primitives& p, const Mat4& base, int number);

// Octagonal launch pad with launch mount, service tower, arms and a tank.
// armSwing: 0 = service arms attached to the rocket, 1 = swung away.
void drawLaunchPad(const Renderer& r, const Primitives& p, const Mat4& base, float armSwing);
