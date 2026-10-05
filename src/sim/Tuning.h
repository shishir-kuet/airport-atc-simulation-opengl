// Every tuned number of the simulation in one place: speeds, accelerations,
// turn rates, altitudes and timings for the three vehicles.
// Internal header: only the files in src/sim/ include it.
#pragma once

#include "sim/Simulation.h"

#include "models/Models.h"       // ground offsets and the landing-leg geometry
#include "world/Environment.h"   // the airport layout the numbers refer to

using namespace Layout;

// ---- Tuning ----------------------------------------------------------------------

// Airplane
constexpr float PLANE_GROUND_Y   = PAVEMENT_TOP + AIRPLANE_GROUND_OFFSET;
constexpr float TAXI_SPEED       = 4.0f;    // units / s
constexpr float TAXI_TURN_RATE   = 50.0f;   // degrees / s
constexpr float PUSHBACK_SPEED   = 1.5f;
constexpr float PUSHBACK_RADIUS  = 10.0f;   // tail swings onto the taxiway
constexpr float TAKEOFF_ACCEL    = 5.0f;    // units / s^2 on the runway
constexpr float ROTATE_SPEED     = 20.0f;   // Vr: start raising the nose
constexpr float LIFTOFF_PITCH    = 9.0f;
constexpr float CLIMB_SPEED      = 32.0f;
constexpr float CRUISE_SPEED     = 30.0f;
constexpr float APPROACH_SPEED   = 14.0f;   // Vref on final approach (slow and steady)
constexpr float TOUCHDOWN_SPEED  = 11.0f;   // after the flare
constexpr float PLANE_BANK       = 30.0f;
constexpr float PLANE_TURN_RATE  = 25.0f;   // degrees / s at full bank

// Airplane holding pattern: left-hand circle right over the airport, so the
// whole airport can be seen below from the cockpit.
constexpr float PLANE_HOLD_X      = 40.0f;                 // middle of the airport
constexpr float PLANE_HOLD_Z      = RUNWAY_Z + 35.0f;
constexpr float PLANE_HOLD_RADIUS = 70.0f;   // wider than the tightest turn (~69)
constexpr float PLANE_HOLD_ALT    = 30.0f;   // the two holding circles do not overlap
constexpr float PLANE_HOLD_TIME   = 8.0f;    // seconds on the holding circle

// Landing on runway 09 (approaching from the west, flying east).
const float THRESHOLD_X  = RUNWAY_CENTER.x - RUNWAY_LENGTH / 2;   // -65
const float AIM_POINT_X  = THRESHOLD_X + 20.0f;                   // aiming point markings
constexpr float GLIDE_SLOPE  = 7.5f;    // degrees
constexpr float FLARE_HEIGHT = 1.5f;

// Manual flight (key M): a simple point-mass model. Thrust minus drag minus
// the slope of gravity sets the speed; the stick sets bank and climb angle.
constexpr float THROTTLE_RATE     = 0.5f;    // lever travel per second (full range in 2 s)
constexpr float MANUAL_THRUST     = 7.0f;    // acceleration at full power (units / s^2)
constexpr float MANUAL_DRAG       = 0.005f;  // drag = k * speed^2, so full power levels off near 37
constexpr float ROLLING_FRICTION  = 0.4f;    // tyres on the runway
constexpr float BRAKE_DECEL       = 6.0f;    // wheel brakes (key X)
constexpr float STALL_SPEED       = 12.0f;   // below this the wings cannot hold the nose up
constexpr float MANUAL_MAX_SPEED  = 60.0f;
constexpr float MANUAL_PITCH_RATE = 12.0f;   // change of climb angle per second at full stick
constexpr float MANUAL_MAX_CLIMB  = 20.0f;   // degrees
constexpr float MANUAL_MAX_DIVE   = 25.0f;   // degrees
constexpr float MANUAL_ROLL_RATE  = 45.0f;   // degrees / s
constexpr float MANUAL_MAX_BANK   = 35.0f;   // degrees
constexpr float STEER_RATE        = 30.0f;   // nose-wheel steering on the ground, degrees / s
constexpr float HARD_SINK         = 4.0f;    // sink rate (units / s) that makes a hard landing
constexpr float MANUAL_CEILING    = 400.0f;  // highest altitude allowed
constexpr float WORLD_EDGE        = 2400.0f; // stay over the ground slab

// Helicopter
constexpr float HELI_GROUND_Y    = HELIPAD_TOP + HELICOPTER_GROUND_OFFSET;
constexpr float ROTOR_FULL_SPEED = 1000.0f; // degrees / s
constexpr float HOVER_HEIGHT     = 4.0f;
constexpr float HELI_CRUISE      = 18.0f;
constexpr float HELI_ALTITUDE    = 35.0f;
constexpr float HELI_BANK        = 20.0f;
constexpr float HELI_TURN_RATE   = 15.0f;
const float HELI_HOLD_X      = HELIPAD_POS.x + 50.0f;   // circle south of the helipad
const float HELI_HOLD_Z      = HELIPAD_POS.z + 110.0f;
constexpr float HELI_HOLD_RADIUS = 60.0f;
constexpr float HELI_HOLD_TIME   = 20.0f;

// Rocket (separates twice; only the final second stage flies back and lands)
constexpr float BOOSTER_SEP_TIME = 10.0f;   // seconds after liftoff
constexpr float STAGE_SEP_TIME   = 20.0f;   // main engine cut-off + stage separation
constexpr float STAGE2_DELAY     = 1.0f;    // short coast before the second stage lights
constexpr float STAGE2_BURN_TIME = 8.0f;
constexpr float STAGE2_ACCEL     = 12.0f;   // lighter stage, so it speeds up faster
constexpr float STAGE2_MAX_TILT  = 25.0f;   // steep, lofted arc so it can come back
constexpr float GRAVITY          = 9.8f;
constexpr float BOOSTBACK_ACCEL  = 25.0f;   // units / s^2 from the engine
constexpr float ENTRY_BURN_ACCEL = 25.0f;
constexpr float ENTRY_BURN_SPEED = 70.0f;   // falling speed after the entry burn
constexpr float LANDING_ACCEL    = 25.0f;   // engine deceleration in the landing burn
// Height of the second stage's origin once standing on its deployed legs.
const float UPPER_LAND_Y   = LANDING_ZONE_TOP - legFootY(UPPER_LEGS);   // on pad 4 at the rocket base
