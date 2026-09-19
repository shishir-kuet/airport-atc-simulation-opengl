#include "Simulation.h"

#include "Environment.h"
#include "Models.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

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
constexpr float PLANE_BANK       = 25.0f;
constexpr float PLANE_TURN_RATE  = 12.0f;   // degrees / s at full bank

// Airplane holding pattern: left-hand circle right over the airport, so the
// whole airport can be seen below from the cockpit.
constexpr float PLANE_HOLD_X      = 40.0f;                 // middle of the airport
constexpr float PLANE_HOLD_Z      = RUNWAY_Z + 35.0f;
constexpr float PLANE_HOLD_RADIUS = 150.0f;   // wider than the tightest turn (~143)
constexpr float PLANE_HOLD_ALT    = 60.0f;   // above the helicopter hold (35)
constexpr float PLANE_HOLD_TIME   = 25.0f;  // seconds in the hold before approach

// Landing on runway 09 (approaching from the west, flying east).
const float THRESHOLD_X  = RUNWAY_CENTER.x - RUNWAY_LENGTH / 2;   // -65
const float AIM_POINT_X  = THRESHOLD_X + 20.0f;                     // aiming point markings
constexpr float GLIDE_SLOPE  = 5.0f;    // degrees
constexpr float FLARE_HEIGHT = 1.5f;

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

// Rocket (reusable: boosters and core fly back and land)
constexpr float BOOSTER_SEP_TIME = 10.0f;   // seconds after liftoff
constexpr float MECO_TIME        = 20.0f;   // main engine cut-off: the return begins
constexpr float GRAVITY          = 9.8f;
constexpr float BOOSTBACK_ACCEL  = 25.0f;   // units / s^2 from the engine
constexpr float ENTRY_BURN_ACCEL = 25.0f;
constexpr float ENTRY_BURN_SPEED = 70.0f;   // falling speed after the entry burn
constexpr float LANDING_ACCEL    = 25.0f;   // engine deceleration in the landing burn
// Height of each stage's origin once standing on its deployed legs.
const float CORE_LAND_Y    = LANDING_ZONE_TOP - legFootY(CORE_LEGS);   // on a rocket-base pad
const float BOOSTER_LAND_Y = LANDING_ZONE_TOP - legFootY(BOOSTER_LEGS);

// Ground routes (x, z). `stop` = come to a smooth stop exactly on that point.
struct Waypoint { float x, z; bool stop; };

// Stand 2 -> runway 09 holding point.
static const Waypoint TAXI_ROUTE[] = {
    {STAND_X[1], TAXIWAY_Z, false},              // along the lead-in line
    {TAXI_MIN_X, TAXIWAY_Z, false},              // west along the taxiway
    {TAXI_MIN_X, HOLD_LINE_Z + 5.7f, true},      // stop with the nose at the holding line
};
// Holding point -> onto the runway (then the line-up turn).
static const Waypoint ENTRY_ROUTE[] = {
    {TAXI_MIN_X, RUNWAY_Z + 3.5f, false},
};
// After landing: leave the runway at the next connector ahead (middle or
// east) and taxi to stand 2, parking nose-in.
static const Waypoint TAXI_IN_MIDDLE[] = {
    {TAXI_MID_X, RUNWAY_Z + 5.0f, false},
    {TAXI_MID_X, TAXIWAY_Z, false},
    {STAND_X[1] + 5.0f, TAXIWAY_Z, false},
    {STAND_X[1], TAXIWAY_Z + 5.0f, false},       // turn in onto the lead-in line
    {STAND_X[1], STAND_STOP_Z - 6.0f, true},     // nose just short of the jet bridge
};
static const Waypoint TAXI_IN_EAST[] = {
    {TAXI_MAX_X, RUNWAY_Z + 5.0f, false},
    {TAXI_MAX_X, TAXIWAY_Z, false},
    {STAND_X[1] + 5.0f, TAXIWAY_Z, false},
    {STAND_X[1], TAXIWAY_Z + 5.0f, false},
    {STAND_X[1], STAND_STOP_Z - 6.0f, true},
};
constexpr int TAXI_IN_POINTS = sizeof(TAXI_IN_EAST) / sizeof(TAXI_IN_EAST[0]);

// Landing circuit flown after the hold (x, z, altitude, speed): fly west on
// the downwind leg north of the runway, then one continuous left turn (base
// and final) brings the aircraft round onto the extended centreline. The
// downwind is about two turning radii from the runway so the turn ends on it.
struct PatternPoint { float x, z, alt, speed; };
static const PatternPoint PATTERN[] = {
    {THRESHOLD_X - 500.0f, RUNWAY_Z - 185.0f, 30.0f, 16.0f},   // end of downwind
};
constexpr int PATTERN_POINTS = sizeof(PATTERN) / sizeof(PATTERN[0]);

// ---- Helpers ---------------------------------------------------------------------

// Moves `value` towards `target` by at most `step`.
static float approach(float value, float target, float step)
{
    return value < target ? std::min(value + step, target) : std::max(value - step, target);
}

static float wrapAngle(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Horizontal unit vector for a heading (0 = +X, 90 = -Z).
static Vec3 forward(float heading)
{
    return {std::cos(radians(heading)), 0.0f, -std::sin(radians(heading))};
}

static float headingTo(const Vec3& from, float x, float z)
{
    return std::atan2(-(z - from.z), x - from.x) * 180.0f / PI;
}

// Turns `heading` towards `desired` at no more than `rate` degrees per second.
static float steer(float heading, float desired, float rate, float dt)
{
    float diff = wrapAngle(desired - heading);
    float step = rate * dt;
    return heading + std::clamp(diff, -step, step);
}

// Heading that flies a circle of `radius` around (cx, cz), turning left
// (centre kept on the left). Too far out -> turn in; too close -> turn out.
static float orbitHeading(const Vec3& pos, float cx, float cz, float radius, float gain)
{
    float dx = cx - pos.x, dz = cz - pos.z;
    float dist = std::sqrt(dx * dx + dz * dz);
    float toCentre = headingTo(pos, cx, cz);
    return toCentre - 90.0f + std::clamp((dist - radius) * gain, -60.0f, 60.0f);
}

// Heading that brings an aircraft flying east onto the runway centreline
// (like an ILS localizer): the further off, the steeper the intercept.
static float localizerHeading(const Vec3& pos, float gain, float limit)
{
    return std::clamp(gain * (pos.z - RUNWAY_Z), -limit, limit);
}

static float distanceXZ(const Vec3& a, float x, float z)
{
    return std::sqrt((x - a.x) * (x - a.x) + (z - a.z) * (z - a.z));
}

static float randomFloat()   // 0 .. 1
{
    static unsigned seed = 12345u;
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) / 16777216.0f;
}

static void radio(const char* text)
{
    std::cout << text << std::endl;
}

// ---- Commands ----------------------------------------------------------------------

void Simulation::reset()
{
    plane = AirplaneMotion{};
    plane.pos = Vec3(STAND_X[1], PLANE_GROUND_Y, STAND_STOP_Z + 3.6f);
    plane.heading = 90.0f;   // nose towards the taxiway (-Z)

    heli = HelicopterMotion{};
    heli.pos = Vec3(HELIPAD_POS.x, HELI_GROUND_Y, HELIPAD_POS.z);

    rocket = RocketMotion{};
    rocket.core.pos = Vec3(LAUNCH_PAD_POS.x, LAUNCH_MOUNT_TOP + ROCKET_GROUND_OFFSET, LAUNCH_PAD_POS.z);

    puffs.clear();
    for (bool& started : trailStarted)
        started = false;
}

void Simulation::startAirplane()
{
    if (plane.state != PlaneState::Parking)
        return;
    plane.timer = 0;
    plane.waypoint = 0;
    radio("[PILOT]  Tower, airplane at stand 2, request taxi for departure.");
    if (plane.noseIn)
    {
        // Parked facing the terminal: a tug pushes it back onto the taxiway first.
        radio("[TOWER]  Airplane, pushback approved.");
        plane.state = PlaneState::Pushback;
        plane.heading = wrapAngle(plane.heading);
    }
    else
    {
        radio("[TOWER]  Airplane, taxi to holding point runway 09 via taxiway.");
        plane.state = PlaneState::Taxi;
    }
}

void Simulation::startHelicopter()
{
    if (heli.state != HeliState::Parked)
        return;
    radio("[PILOT]  Helicopter on the helipad, starting engine.");
    heli.state = HeliState::Startup;
    heli.timer = 0;
}

void Simulation::launchRocket()
{
    RocketMotion& k = rocket;
    if (k.core.state == RocketState::Landed)
    {
        // Reusable: once every stage is back down, restack and fly again.
        bool boostersDown = k.boostersAttached || (k.boosters[0].state == RocketState::Landed &&
                                                   k.boosters[1].state == RocketState::Landed);
        if (!boostersDown)
            return;
        radio("[LAUNCH] Rocket moved from the rocket base back to the launch pad, boosters restacked.");
        k = RocketMotion{};
        k.core.pos = Vec3(LAUNCH_PAD_POS.x, LAUNCH_MOUNT_TOP + ROCKET_GROUND_OFFSET, LAUNCH_PAD_POS.z);
    }
    if (k.core.state != RocketState::OnPad)
        return;
    radio("[LAUNCH] Countdown started. Service arms retracting.");
    k.core.state = RocketState::Countdown;
    k.core.timer = 0;
}

// ---- Update ------------------------------------------------------------------------

void Simulation::update(float dt)
{
    dt = std::min(dt, 0.05f);   // avoid big jumps if a frame is slow
    time += dt;
    updateAirplane(dt);
    updateHelicopter(dt);
    updateRocket(dt);
    updateEffects(dt);
}

void Simulation::updateAirplane(float dt)
{
    AirplaneMotion& a = plane;
    a.timer += dt;
    const float altitude = a.pos.y - PLANE_GROUND_Y;

    auto setState = [&](PlaneState s, const char* message)
    {
        a.state = s;
        a.timer = 0;
        a.waypoint = 0;
        if (message)
            radio(message);
    };

    // Drives along a list of ground waypoints; returns true at the end.
    auto followRoute = [&](const Waypoint* route, int count, float cruiseSpeed)
    {
        const Waypoint& w = route[a.waypoint];
        float dx = w.x - a.pos.x, dz = w.z - a.pos.z;
        float dist = std::sqrt(dx * dx + dz * dz);

        // Slow down smoothly when this waypoint is a stopping point.
        float target = w.stop ? std::clamp(0.5f * dist, 0.3f, cruiseSpeed) : cruiseSpeed;
        a.speed = approach(a.speed, target, 2.0f * dt);

        if (w.stop && a.waypoint > 0)
        {
            // Final segment: track the painted line itself (aim 3 units ahead
            // of our position along it) so the aircraft stops straight on it.
            const Waypoint& p = route[a.waypoint - 1];
            float sx = w.x - p.x, sz = w.z - p.z;
            float len = std::sqrt(sx * sx + sz * sz);
            sx /= len;
            sz /= len;
            float along = (a.pos.x - p.x) * sx + (a.pos.z - p.z) * sz + 3.0f;
            a.heading = steer(a.heading, headingTo(a.pos, p.x + sx * along, p.z + sz * along), TAXI_TURN_RATE, dt);
        }
        else if (dist > 1.0f)
        {
            a.heading = steer(a.heading, headingTo(a.pos, w.x, w.z), TAXI_TURN_RATE, dt);
        }
        a.pos = a.pos + forward(a.heading) * (a.speed * dt);

        Vec3 f = forward(a.heading);
        bool passed = dist < 3.0f && (f.x * dx + f.z * dz) <= 0.0f;
        bool reached = dist < (w.stop ? 0.3f : 3.0f) || passed;
        if (reached && ++a.waypoint >= count)
        {
            a.waypoint = 0;
            return true;
        }
        return false;
    };

    // Rolling on the runway: stay locked on the centreline, heading east.
    auto runwayRoll = [&]()
    {
        a.heading = steer(a.heading, localizerHeading(a.pos, 3.0f, 8.0f), 15.0f, dt);
        a.pos = a.pos + forward(a.heading) * (a.speed * dt);
    };

    // Airborne flight: turn towards `desiredHeading` (banking into the turn),
    // climb or descend at `desiredClimb` degrees and change speed.
    auto fly = [&](float desiredHeading, float desiredClimb, float targetSpeed, float maxTurnRate)
    {
        float before = a.heading;
        a.heading = steer(a.heading, desiredHeading, maxTurnRate, dt);
        float turnRate = wrapAngle(a.heading - before) / dt;   // + = turning left
        float bank = -std::clamp(turnRate / PLANE_TURN_RATE, -1.0f, 1.0f) * PLANE_BANK;
        a.roll = approach(a.roll, bank, 10.0f * dt);

        a.climbAngle = approach(a.climbAngle, desiredClimb, 5.0f * dt);
        a.speed = approach(a.speed, targetSpeed, 2.5f * dt);

        // Slower flight needs a higher angle of attack (more nose-up).
        float angleOfAttack = 3.0f + (CLIMB_SPEED - a.speed) * 0.25f;
        a.pitch = approach(a.pitch, a.climbAngle + angleOfAttack, 3.0f * dt);

        float gamma = radians(a.climbAngle);
        a.pos = a.pos + forward(a.heading) * (a.speed * std::cos(gamma) * dt);
        a.pos.y = std::max(PLANE_GROUND_Y, a.pos.y + a.speed * std::sin(gamma) * dt);
    };

    // Landing gear retracts once safely airborne.
    auto retractGear = [&]()
    {
        if (altitude > 3.0f && a.gear > 0.0f)
        {
            if (a.gear == 1.0f)
                radio("[PILOT]  Gear up.");
            a.gear = std::max(0.0f, a.gear - dt / 3.0f);
        }
    };

    switch (a.state)
    {
    case PlaneState::Parking:
        break;

    case PlaneState::Pushback:
        // Pushed backwards by the tug while the tail swings round onto the taxiway.
        a.speed = PUSHBACK_SPEED;
        a.heading -= (PUSHBACK_SPEED / PUSHBACK_RADIUS) * 180.0f / PI * dt;
        a.pos = a.pos - forward(a.heading) * (PUSHBACK_SPEED * dt);
        if (a.heading <= -180.0f)
        {
            a.heading = 180.0f;
            a.speed = 0;
            a.noseIn = false;
            setState(PlaneState::Taxi, "[TOWER]  Airplane, taxi to holding point runway 09 via taxiway.");
            a.waypoint = 1;   // already on the taxiway
        }
        break;

    case PlaneState::Taxi:
        if (followRoute(TAXI_ROUTE, 3, TAXI_SPEED))
        {
            a.speed = 0;
            setState(PlaneState::HoldShort, "[TOWER]  Airplane, hold short of runway 09.");
        }
        break;

    case PlaneState::HoldShort:
        // ATC waits until the runway is clear.
        if (a.timer > 3.0f)
            setState(PlaneState::RunwayEntry, "[TOWER]  Airplane, runway 09, line up and wait.");
        break;

    case PlaneState::RunwayEntry:
        if (a.waypoint == 0)
        {
            // Roll forward onto the runway.
            if (followRoute(ENTRY_ROUTE, 1, 3.0f))
                a.waypoint = 1;
        }
        else if (a.waypoint == 1)
        {
            // Line-up turn: steer onto the centreline until pointing straight down the runway.
            a.speed = approach(a.speed, 1.5f, 2.0f * dt);
            a.heading = steer(a.heading, headingTo(a.pos, a.pos.x + 4.0f, RUNWAY_Z), 40.0f, dt);
            a.pos = a.pos + forward(a.heading) * (a.speed * dt);
            if (std::fabs(wrapAngle(a.heading)) < 2.0f && std::fabs(a.pos.z - RUNWAY_Z) < 0.5f)
            {
                a.waypoint = 2;
                a.timer = 0;
            }
        }
        else
        {
            // Lined up: stop on the centreline and wait for the clearance.
            a.speed = approach(a.speed, 0.0f, 3.0f * dt);
            runwayRoll();
            if (a.speed == 0.0f && a.timer > 2.0f)
            {
                setState(PlaneState::Acceleration, "[TOWER]  Airplane, wind calm, runway 09, cleared for takeoff.");
                radio("[PILOT]  Takeoff power set, rolling.");
            }
        }
        break;

    case PlaneState::Acceleration:
        a.speed += TAKEOFF_ACCEL * dt;
        runwayRoll();
        if (a.speed >= ROTATE_SPEED)
            setState(PlaneState::Rotation, "[PILOT]  V1 ... Rotate.");
        break;

    case PlaneState::Rotation:
        // Nose comes up while the main wheels are still on the runway.
        a.speed += TAKEOFF_ACCEL * dt;
        a.pitch += 6.0f * dt;
        runwayRoll();
        if (a.pitch >= LIFTOFF_PITCH)
            setState(PlaneState::Climb, "[PILOT]  Liftoff, positive climb.");
        break;

    case PlaneState::Climb:
        // Straight ahead along the runway heading.
        fly(localizerHeading(a.pos, 0.5f, 10.0f), 7.0f, CLIMB_SPEED, 5.0f);
        retractGear();
        if (altitude > 10.0f)
            setState(PlaneState::CruiseHold, "[TOWER]  Airplane, turn left, climb to 60, hold over the airport.");
        break;

    case PlaneState::CruiseHold:
        fly(orbitHeading(a.pos, PLANE_HOLD_X, PLANE_HOLD_Z, PLANE_HOLD_RADIUS, 0.5f),
            std::clamp((PLANE_HOLD_ALT - altitude) * 0.3f, -4.0f, 6.0f), CRUISE_SPEED, PLANE_TURN_RATE);
        retractGear();
        // Only time spent on the circle counts, so the aircraft really flies
        // a lap over the airport before it is cleared to leave.
        if (std::fabs(distanceXZ(a.pos, PLANE_HOLD_X, PLANE_HOLD_Z) - PLANE_HOLD_RADIUS) > 30.0f)
            a.timer = 0.0f;
        if (a.timer > PLANE_HOLD_TIME)
        {
            radio("[PILOT]  Tower, airplane request landing.");
            setState(PlaneState::Approach, "[TOWER]  Airplane, leave the hold, join left downwind runway 09.");
        }
        break;

    case PlaneState::Approach:
    {
        // Fly the circuit: downwind leg, then the base turn.
        const PatternPoint& w = PATTERN[a.waypoint];
        float dist = distanceXZ(a.pos, w.x, w.z);
        fly(headingTo(a.pos, w.x, w.z), std::clamp((w.alt - altitude) * 0.3f, -5.0f, 5.0f), w.speed, 10.0f);

        Vec3 f = forward(a.heading);
        // An airliner turns in a wide circle, so a point counts as reached
        // once it is close by and falls behind the aircraft.
        bool passed = dist < 150.0f && (f.x * (w.x - a.pos.x) + f.z * (w.z - a.pos.z)) <= 0.0f;
        if (dist < 20.0f || passed)
        {
            if (++a.waypoint >= PATTERN_POINTS)
            {
                radio("[PILOT]  Turning base and final.");
                setState(PlaneState::Descent, "[TOWER]  Airplane, runway 09, wind calm, cleared to land.");
                radio("[PILOT]  Final approach, gear down.");
            }
        }
        break;
    }

    case PlaneState::Descent:
    {
        // Final approach: localizer keeps us on the centreline, glide slope
        // gives the altitude we should have at this distance from the runway.
        a.gear = std::min(1.0f, a.gear + dt / 3.0f);
        float glideAlt = std::max(0.0f, AIM_POINT_X - a.pos.x) * std::tan(radians(GLIDE_SLOPE));
        float climb = std::clamp(-GLIDE_SLOPE + (glideAlt - altitude) * 0.8f, -9.0f, 0.0f);
        fly(localizerHeading(a.pos, 1.2f, 45.0f), climb, APPROACH_SPEED, 10.0f);
        if (altitude < FLARE_HEIGHT)
            setState(PlaneState::Flare, "[PILOT]  Flare.");
        break;
    }

    case PlaneState::Flare:
    {
        // Just above the runway: raise the nose to soften the sink rate.
        a.gear = 1.0f;
        a.heading = steer(a.heading, localizerHeading(a.pos, 2.0f, 5.0f), 5.0f, dt);
        a.roll = approach(a.roll, 0.0f, 10.0f * dt);
        a.climbAngle = approach(a.climbAngle, -2.5f, 5.0f * dt);
        a.pitch = approach(a.pitch, 6.0f, 4.0f * dt);
        a.speed = approach(a.speed, TOUCHDOWN_SPEED, 1.0f * dt);

        float gamma = radians(a.climbAngle);
        a.pos = a.pos + forward(a.heading) * (a.speed * std::cos(gamma) * dt);
        a.pos.y += a.speed * std::sin(gamma) * dt;
        if (a.pos.y <= PLANE_GROUND_Y)
        {
            a.pos.y = PLANE_GROUND_Y;
            a.climbAngle = 0;
            setState(PlaneState::Touchdown, "[PILOT]  Touchdown.");
        }
        break;
    }

    case PlaneState::Touchdown:
        // Main wheels first, then the nose comes down gently; then brakes.
        a.roll = approach(a.roll, 0.0f, 10.0f * dt);
        a.pitch = approach(a.pitch, 0.0f, 2.0f * dt);
        a.speed = std::max(0.0f, a.speed - (a.pitch > 0.5f ? 1.0f : 2.5f) * dt);
        runwayRoll();
        if (a.speed <= TAXI_SPEED)
        {
            // Leave by the next connector ahead that we can still turn into.
            a.exitEast = a.pos.x > TAXI_MID_X - 8.0f;
            setState(PlaneState::TaxiIn, a.exitEast
                ? "[TOWER]  Airplane, welcome. Vacate via the east taxiway, taxi to stand 2."
                : "[TOWER]  Airplane, welcome. Vacate via the middle taxiway, taxi to stand 2.");
        }
        break;

    case PlaneState::TaxiIn:
        if (followRoute(a.exitEast ? TAXI_IN_EAST : TAXI_IN_MIDDLE, TAXI_IN_POINTS, TAXI_SPEED))
        {
            a.speed = 0;
            a.noseIn = true;
            setState(PlaneState::Parking, "[PILOT]  Parked at stand 2, engines shut down.");
        }
        break;
    }

    a.heading = wrapAngle(a.heading);
}

void Simulation::updateHelicopter(float dt)
{
    HelicopterMotion& h = heli;
    h.timer += dt;
    const float altitude = h.pos.y - HELI_GROUND_Y;

    auto setState = [&](HeliState s, const char* message)
    {
        h.state = s;
        h.timer = 0;
        if (message)
            radio(message);
    };

    // Forward flight: turn (banking into the turn), change speed and vertical
    // speed. Speeding up tilts the nose down, slowing down tilts it up.
    auto fly = [&](float desiredHeading, float targetSpeed, float targetVerticalSpeed, float maxTurnRate)
    {
        float before = h.heading;
        h.heading = steer(h.heading, desiredHeading, maxTurnRate, dt);
        float turnRate = wrapAngle(h.heading - before) / dt;
        float bank = -std::clamp(turnRate / HELI_TURN_RATE, -1.0f, 1.0f) * HELI_BANK;
        h.roll = approach(h.roll, bank, 12.0f * dt);

        float oldSpeed = h.speed;
        h.speed = approach(h.speed, targetSpeed, 2.5f * dt);
        float accel = (h.speed - oldSpeed) / dt;
        h.pitch = approach(h.pitch, std::clamp(-accel * 2.5f - h.speed * 0.25f, -12.0f, 12.0f), 6.0f * dt);

        h.verticalSpeed = approach(h.verticalSpeed, targetVerticalSpeed, 1.5f * dt);
    };

    switch (h.state)
    {
    case HeliState::Parked:
        break;

    case HeliState::Startup:
        // Rotor spools up before the helicopter can lift.
        h.rotorSpeed = approach(h.rotorSpeed, ROTOR_FULL_SPEED, 200.0f * dt);
        if (h.rotorSpeed >= ROTOR_FULL_SPEED)
            setState(HeliState::LiftOff, "[TOWER]  Helicopter, cleared for takeoff from the helipad.");
        break;

    case HeliState::LiftOff:
        // Straight up to a low hover.
        h.verticalSpeed = approach(h.verticalSpeed, 1.5f, 1.0f * dt);
        if (altitude >= HOVER_HEIGHT)
            setState(HeliState::Hover, "[PILOT]  In the hover, turning south.");
        break;

    case HeliState::Hover:
        // Hold height and turn on the spot (pedal turn) to face south (+Z).
        h.verticalSpeed = approach(h.verticalSpeed, 0.0f, 2.0f * dt);
        h.heading = steer(h.heading, -90.0f, 25.0f, dt);
        if (h.timer > 4.0f && std::fabs(wrapAngle(h.heading + 90.0f)) < 1.0f)
            setState(HeliState::Transition, "[PILOT]  Nose down, transition to forward flight.");
        break;

    case HeliState::Transition:
        // Tilting the rotor forward trades lift for forward speed.
        fly(h.heading, HELI_CRUISE, 2.0f, 0.0f);
        if (h.speed >= HELI_CRUISE)
            setState(HeliState::Climb, nullptr);
        break;

    case HeliState::Climb:
        fly(h.heading, HELI_CRUISE, 2.0f, 0.0f);
        if (altitude >= HELI_ALTITUDE)
            setState(HeliState::CruiseHold, "[TOWER]  Helicopter, hold south of the helipad.");
        break;

    case HeliState::CruiseHold:
        fly(orbitHeading(h.pos, HELI_HOLD_X, HELI_HOLD_Z, HELI_HOLD_RADIUS, 0.8f), HELI_CRUISE,
            std::clamp((HELI_ALTITUDE - altitude) * 0.3f, -2.0f, 2.0f), HELI_TURN_RATE);
        if (h.timer > HELI_HOLD_TIME)
        {
            radio("[PILOT]  Tower, helicopter request landing on the helipad.");
            setState(HeliState::Approach, "[TOWER]  Helicopter, cleared to land on the helipad.");
        }
        break;

    case HeliState::Approach:
    {
        // Fly to the pad, slowing down and descending as it gets closer.
        float dist = distanceXZ(h.pos, HELIPAD_POS.x, HELIPAD_POS.z);
        float heading = dist > 3.0f ? headingTo(h.pos, HELIPAD_POS.x, HELIPAD_POS.z) : h.heading;
        float targetSpeed = std::clamp(dist * 0.18f, 0.0f, HELI_CRUISE);
        float targetAlt = std::clamp(dist * 0.12f, HOVER_HEIGHT, HELI_ALTITUDE);
        fly(heading, targetSpeed, std::clamp((targetAlt - altitude) * 0.5f, -2.5f, 2.0f), 20.0f);
        if (dist < 1.5f && h.speed < 0.8f)
            setState(HeliState::Landing, "[PILOT]  Hovering over the pad, landing.");
        break;
    }

    case HeliState::Landing:
        // Settle over the centre of the pad, turn back to the parked heading and descend.
        h.speed = approach(h.speed, 0.0f, 2.0f * dt);
        h.pos.x += (HELIPAD_POS.x - h.pos.x) * 1.2f * dt;
        h.pos.z += (HELIPAD_POS.z - h.pos.z) * 1.2f * dt;
        h.heading = steer(h.heading, 0.0f, 20.0f, dt);
        h.pitch = approach(h.pitch, 0.0f, 6.0f * dt);
        h.roll = approach(h.roll, 0.0f, 6.0f * dt);
        {
            // Only come down once facing the parked heading.
            bool aligned = std::fabs(wrapAngle(h.heading)) < 5.0f;
            float sink = !aligned ? 0.0f : (altitude > 1.5f ? -1.2f : -0.4f);
            h.verticalSpeed = approach(h.verticalSpeed, sink, 1.5f * dt);
        }
        if (altitude <= 0.0f)
        {
            h.pos.y = HELI_GROUND_Y;
            h.verticalSpeed = 0;
            setState(HeliState::Shutdown, "[PILOT]  Landed on the helipad, shutting down.");
        }
        break;

    case HeliState::Shutdown:
        h.rotorSpeed = approach(h.rotorSpeed, 0.0f, 150.0f * dt);
        if (h.rotorSpeed <= 0.0f)
        {
            h.heading = wrapAngle(h.heading);
            setState(HeliState::Parked, "[PILOT]  Rotor stopped, helicopter parked.");
        }
        break;
    }

    h.heading = wrapAngle(h.heading);
    h.rotorAngle += h.rotorSpeed * dt;
    h.pos = h.pos + forward(h.heading) * (h.speed * dt);
    h.pos.y += h.verticalSpeed * dt;
}

void Simulation::updateRocket(float dt)
{
    RocketMotion& k = rocket;
    StageMotion& c = k.core;
    float altitude = c.pos.y - LAUNCH_MOUNT_TOP;

    auto setState = [&](RocketState s, const char* message)
    {
        c.state = s;
        c.timer = 0;
        if (message)
            radio(message);
    };

    switch (c.state)
    {
    case RocketState::OnPad:
        break;

    case RocketState::Countdown:
    {
        c.timer += dt;
        k.armSwing = std::min(1.0f, c.timer / 3.0f);
        int count = 5 - static_cast<int>(c.timer);
        if (count < k.lastCountdown && count > 0)
        {
            char text[32];
            std::snprintf(text, sizeof(text), "[LAUNCH] T-%d", count);
            radio(text);
            k.lastCountdown = count;
        }
        if (c.timer >= 5.0f)
            setState(RocketState::Ignition, "[LAUNCH] Ignition!");
        break;
    }

    case RocketState::Ignition:
        // Engines build up thrust while the clamps still hold the rocket.
        c.timer += dt;
        c.flame = k.boosterFlame = std::min(1.0f, c.timer / 1.2f);
        if (c.timer >= 2.0f)
            setState(RocketState::Ascent, "[LAUNCH] Clamps released. Liftoff!");
        break;

    case RocketState::Ascent:
    {
        c.timer += dt;
        float t = c.timer;

        // Thrust grows as propellant burns off (the rocket gets lighter).
        k.speed += (2.0f + 0.25f * t) * dt;

        // Gravity turn: once clear of the tower, tilt slowly downrange (+X).
        if (altitude > 50.0f)
            c.tiltX = approach(c.tiltX, 45.0f, 2.0f * dt);

        Vec3 dir(std::sin(radians(c.tiltX)), std::cos(radians(c.tiltX)), 0.0f);
        c.velocity = dir * k.speed;
        c.pos = c.pos + c.velocity * dt;

        if (t >= BOOSTER_SEP_TIME && k.boostersAttached)
        {
            radio("[LAUNCH] Booster separation. Boosters returning to LZ-1 and LZ-2.");
            Mat4 base = rocketMatrix();
            for (int i = 0; i < 2; ++i)
            {
                StageMotion& b = k.boosters[i];
                b = StageMotion{};
                Mat4 m = rocketBoosterMatrix(base, i);
                b.pos = transformPoint(m, {0, 0, 0});
                Vec3 out = transformPoint(m, {1, 0, 0}) - b.pos;
                b.velocity = c.velocity + out * 4.0f;   // pushed away sideways
                b.tiltX = c.tiltX;
                b.target = Vec3(LANDING_ZONE[i].x, BOOSTER_LAND_Y, LANDING_ZONE[i].z);
                b.state = RocketState::Flip;
            }
            k.boostersAttached = false;
            k.boosterFlame = 0;
        }

        if (t >= MECO_TIME)
        {
            c.flame = 0;
            const Vec3& pad = ROCKET_BASE_PADS[ROCKET_BASE_FREE_PAD];
            c.target = Vec3(pad.x, CORE_LAND_Y, pad.z);   // free pad at the rocket base
            setState(RocketState::Flip, "[LAUNCH] Main engine cut-off. Coasting up, flipping to head home.");
        }
        break;
    }

    default:   // Flip ... Landed: the return flight
        updateReturningStage(c, dt, true, "[ROCKET]  ");
        break;
    }

    if (!k.boostersAttached)
    {
        updateReturningStage(k.boosters[0], dt, false, "[BOOSTER 1] ");
        updateReturningStage(k.boosters[1], dt, false, "[BOOSTER 2] ");
    }
}

// Return flight of a stage, like a reusable rocket:
//   Flip         - engine off, turn the rocket so the engine can push it back
//   Boostback    - burn to reverse the sideways speed, aiming at the landing spot
//   Coast        - engine off, fall engine-first; grid fins steer a little
//   Entry burn   - (core only) slow down before the thick lower atmosphere
//   Landing burn - legs out, decelerate to touch down gently on the target
void Simulation::updateReturningStage(StageMotion& s, float dt, bool isCore, const char* name)
{
    if (s.state == RocketState::Landed)
        return;
    s.timer += dt;

    auto setState = [&](RocketState st, const char* message)
    {
        s.state = st;
        s.timer = 0;
        std::cout << name << message << std::endl;
    };

    const float height = s.pos.y - s.target.y;
    const Vec3 sideways(s.velocity.x, 0.0f, s.velocity.z);
    const Vec3 toTarget(s.target.x - s.pos.x, 0.0f, s.target.z - s.pos.z);

    // Sideways speed that would carry the stage onto its target during the
    // fall (the burns stretch the fall a little, hence the extra factor).
    float vy = s.velocity.y;
    float fallTime = (vy + std::sqrt(vy * vy + 2.0f * GRAVITY * std::max(height, 0.0f))) / GRAVITY;
    fallTime = std::max(fallTime, 2.0f) * (isCore ? 1.25f : 1.1f);
    Vec3 wanted = toTarget * (1.0f / fallTime);
    Vec3 correction = wanted - sideways;
    float correctionLen = std::sqrt(dot(correction, correction));

    // Changes the sideways velocity by at most accel * dt towards `goal`.
    auto pushSideways = [&](const Vec3& goal, float accel)
    {
        Vec3 d = goal - Vec3(s.velocity.x, 0.0f, s.velocity.z);
        float len = std::sqrt(dot(d, d));
        if (len < 1e-4f)
            return;
        float step = std::min(accel * dt, len);
        s.velocity.x += d.x / len * step;
        s.velocity.z += d.z / len * step;
    };

    // Leans the rocket axis `lean` degrees towards direction v; true when there.
    auto pointAlong = [&](const Vec3& v, float lean, float rate)
    {
        float len = std::sqrt(dot(v, v));
        float tx = len > 0.01f ? lean * v.x / len : 0.0f;
        float tz = len > 0.01f ? lean * v.z / len : 0.0f;
        s.tiltX = approach(s.tiltX, tx, rate * dt);
        s.tiltZ = approach(s.tiltZ, tz, rate * dt);
        return std::fabs(s.tiltX - tx) < 3.0f && std::fabs(s.tiltZ - tz) < 3.0f;
    };

    const float flipRate = isCore ? 30.0f : 35.0f;
    const float netDecel = LANDING_ACCEL - GRAVITY;
    bool engineHoldsVertical = false;   // during the entry and landing burns

    // Time to start the landing burn: falling, and just high enough to stop in time.
    const bool mustBrake = vy < 0.0f && height <= vy * vy / (2.0f * netDecel) + 5.0f;

    switch (s.state)
    {
    case RocketState::Flip:
        s.flame = 0;
        if (pointAlong(correction, 70.0f, flipRate))
            setState(RocketState::Boostback, "Boostback burn.");
        else if (mustBrake)
            setState(RocketState::LandingBurn, "Landing burn, legs deploying.");
        break;

    case RocketState::Boostback:
        s.flame = 1;
        pointAlong(correction, 70.0f, flipRate);
        pushSideways(wanted, BOOSTBACK_ACCEL);
        if (correctionLen < 0.5f || s.timer > 12.0f)
            setState(RocketState::Coast, "Boostback complete, engine off.");
        else if (mustBrake)
            setState(RocketState::LandingBurn, "Landing burn, legs deploying.");
        break;

    case RocketState::Coast:
        s.flame = 0;
        // Turn back upright (engine down) and steer gently with the grid fins.
        s.tiltX = approach(s.tiltX, 0.0f, 15.0f * dt);
        s.tiltZ = approach(s.tiltZ, 0.0f, 15.0f * dt);
        pushSideways(wanted, 1.5f);
        if (mustBrake)
            setState(RocketState::LandingBurn, "Landing burn, legs deploying.");
        else if (isCore && !s.entryBurnDone && vy < -(ENTRY_BURN_SPEED + 30.0f) && height > 300.0f)
            setState(RocketState::EntryBurn, "Entry burn.");
        break;

    case RocketState::EntryBurn:
        s.flame = 1;
        engineHoldsVertical = true;
        s.velocity.y = approach(s.velocity.y, -ENTRY_BURN_SPEED, ENTRY_BURN_ACCEL * dt);
        pushSideways(wanted, 3.0f);
        s.tiltX = approach(s.tiltX, 0.0f, 15.0f * dt);
        s.tiltZ = approach(s.tiltZ, 0.0f, 15.0f * dt);
        if (s.velocity.y >= -ENTRY_BURN_SPEED - 0.1f)
        {
            s.entryBurnDone = true;
            setState(RocketState::Coast, "Entry burn complete.");
        }
        else if (mustBrake)
        {
            setState(RocketState::LandingBurn, "Landing burn, legs deploying.");
        }
        break;

    case RocketState::LandingBurn:
    {
        engineHoldsVertical = true;
        s.legs = std::min(1.0f, s.legs + dt / 2.0f);

        // "Suicide burn": the falling speed allowed at this height, so the
        // stage reaches zero speed just as it reaches the pad.
        float targetVy = -std::max(1.0f, std::sqrt(2.0f * netDecel * std::max(height, 0.0f)) * 0.9f);
        if (s.velocity.y < targetVy)
            s.velocity.y = std::min(s.velocity.y + LANDING_ACCEL * dt, targetVy);
        else
            s.velocity.y = std::max(s.velocity.y - GRAVITY * dt, targetVy);

        // Slide over the exact landing spot, leaning slightly into the correction.
        Vec3 over = toTarget * 1.2f;
        float overLen = std::sqrt(dot(over, over));
        if (overLen > 20.0f)
            over = over * (20.0f / overLen);
        Vec3 lean = over - sideways;
        pushSideways(over, 10.0f);
        s.tiltX = approach(s.tiltX, std::clamp(lean.x * 3.0f, -10.0f, 10.0f), 20.0f * dt);
        s.tiltZ = approach(s.tiltZ, std::clamp(lean.z * 3.0f, -10.0f, 10.0f), 20.0f * dt);
        s.flame = 0.4f + 0.6f * std::clamp(height / 30.0f, 0.0f, 1.0f);

        if (height <= 0.0f)
        {
            s.pos = s.target;
            s.velocity = Vec3(0, 0, 0);
            s.tiltX = s.tiltZ = 0.0f;
            s.flame = 0.0f;
            s.legs = 1.0f;
            setState(RocketState::Landed, isCore ? "Touchdown on pad 4 at the rocket base. Welcome home!"
                                                  : "Touchdown on the landing zone.");
            return;
        }
        break;
    }

    default:
        break;
    }

    if (!engineHoldsVertical)
        s.velocity.y -= GRAVITY * dt;
    s.pos = s.pos + s.velocity * dt;
}

void Simulation::emitPuff(Vec3 pos, Vec3 velocity, float life, float size0, float size1)
{
    const size_t MAX_PUFFS = 1500;
    if (puffs.size() >= MAX_PUFFS)
        puffs.erase(puffs.begin());   // drop the oldest

    Puff p;
    p.pos = pos;
    p.velocity = velocity;
    p.life = life;
    p.size0 = size0;
    p.size1 = size1;
    puffs.push_back(p);
}

// Exhaust trail: one puff every `spacing` units travelled, so the trail stays
// continuous however fast the stage moves.
void Simulation::emitTrail(int emitter, const Vec3& exit, float size)
{
    if (!trailStarted[emitter])
    {
        lastTrail[emitter] = exit;
        trailStarted[emitter] = true;
    }

    const float spacing = 1.2f;
    Vec3 d = exit - lastTrail[emitter];
    float len = std::sqrt(dot(d, d));
    int count = std::min(static_cast<int>(len / spacing), 60);
    for (int i = 1; i <= count; ++i)
    {
        Vec3 pos = lastTrail[emitter] + d * (i * spacing / len);
        Vec3 drift(randomFloat() - 0.5f, randomFloat() - 0.5f, randomFloat() - 0.5f);
        emitPuff(pos, drift, 5.0f, 1.8f * size, 5.0f * size);
    }
    if (count > 0)
        lastTrail[emitter] = (count == 60) ? exit : lastTrail[emitter] + d * (count * spacing / len);
}

void Simulation::updateEffects(float dt)
{
    const RocketMotion& k = rocket;
    const StageMotion& c = k.core;
    bool coreBurning = c.flame > 0.0f;

    // Ground cloud rolling out of the flame trench (+Z side of the pad) while
    // the engine fires close to the pad - at launch and again at landing.
    smokeTimer += dt;
    const float interval = 0.04f;
    while (smokeTimer >= interval)
    {
        smokeTimer -= interval;
        bool nearPad = c.pos.y - LAUNCH_MOUNT_TOP < 25.0f &&
                       distanceXZ(c.pos, LAUNCH_PAD_POS.x, LAUNCH_PAD_POS.z) < 15.0f;
        if (coreBurning && nearPad)
        {
            Vec3 pos = LAUNCH_PAD_POS + Vec3((randomFloat() - 0.5f) * 3.0f, 1.8f, 9.0f + randomFloat() * 2.0f);
            Vec3 vel((randomFloat() - 0.5f) * 3.0f, 1.0f + randomFloat() * 1.5f, 5.0f + randomFloat() * 5.0f);
            emitPuff(pos, vel, 5.0f, 2.0f, 7.0f);
        }

        // Dust blown outwards across a landing pad by a stage's landing burn.
        const StageMotion* landing[3] = {&c, &k.boosters[0], &k.boosters[1]};
        for (int i = 0; i < 3; ++i)
        {
            const StageMotion& s = *landing[i];
            bool flying = i == 0 || !k.boostersAttached;
            if (!flying || s.state != RocketState::LandingBurn || s.pos.y - s.target.y > 12.0f)
                continue;
            float a = randomFloat() * 2.0f * PI;
            Vec3 dir(std::cos(a), 0.0f, std::sin(a));
            float size = i == 0 ? 1.0f : 0.6f;
            emitPuff(Vec3(s.target.x, 0.6f, s.target.z) + dir * 1.5f,
                     dir * (6.0f + randomFloat() * 4.0f) + Vec3(0.0f, 0.8f, 0.0f), 3.0f, 1.2f * size, 3.5f * size);
        }
    }

    // Exhaust trails of the core and of each separated booster.
    if (coreBurning && c.state != RocketState::Ignition)
        emitTrail(0, transformPoint(rocketMatrix(), {0.0f, -1.5f, 0.0f}), 1.0f);
    else
        trailStarted[0] = false;

    for (int i = 0; i < 2; ++i)
    {
        if (!k.boostersAttached && k.boosters[i].flame > 0.0f)
            emitTrail(1 + i, transformPoint(boosterMatrix(i), {0.0f, -0.8f, 0.0f}), 0.45f);
        else
            trailStarted[1 + i] = false;
    }

    for (Puff& p : puffs)
    {
        p.age += dt;
        p.pos = p.pos + p.velocity * dt;
        p.velocity = p.velocity * (1.0f - 0.8f * dt);   // air drag slows the smoke
    }
    puffs.erase(std::remove_if(puffs.begin(), puffs.end(),
                               [](const Puff& p) { return p.age >= p.life; }),
                puffs.end());
}

// ---- Matrices ----------------------------------------------------------------------

Mat4 Simulation::airplaneMatrix() const
{
    const AirplaneMotion& a = plane;
    // Pitch around the main wheels so the tail sinks during rotation.
    const Vec3 pivot(-0.2f, -AIRPLANE_GROUND_OFFSET, 0.0f);
    return translate(a.pos.x, a.pos.y, a.pos.z) * rotateY(a.heading)
         * translate(pivot.x, pivot.y, pivot.z) * rotateZ(a.pitch) * translate(-pivot.x, -pivot.y, -pivot.z)
         * rotateX(a.roll);
}

Mat4 Simulation::helicopterMatrix() const
{
    const HelicopterMotion& h = heli;
    float bob = (h.state == HeliState::Hover) ? 0.06f * std::sin(time * 2.5f) : 0.0f;
    return translate(h.pos.x, h.pos.y + bob, h.pos.z) * rotateY(h.heading)
         * rotateZ(h.pitch) * rotateX(h.roll);
}

Mat4 Simulation::rocketMatrix() const
{
    const StageMotion& c = rocket.core;
    // Vibration while the engines are at full power near the pad.
    bool shaking = c.state == RocketState::Ignition || (c.state == RocketState::Ascent && c.timer < 3.0f);
    float sx = shaking ? 0.04f * std::sin(time * 61.0f) : 0.0f;
    float sz = shaking ? 0.04f * std::cos(time * 47.0f) : 0.0f;
    return translate(c.pos.x + sx, c.pos.y, c.pos.z + sz) * rotateZ(-c.tiltX) * rotateX(c.tiltZ);
}

Mat4 Simulation::boosterMatrix(int index) const
{
    const StageMotion& b = rocket.boosters[index];
    // rotateY keeps each booster turned the way it was mounted on the core.
    return translate(b.pos.x, b.pos.y, b.pos.z) * rotateZ(-b.tiltX) * rotateX(b.tiltZ)
         * rotateY(index * 180.0f);
}

Vec3 Simulation::focusPoint(int vehicle) const
{
    switch (vehicle)
    {
    case 0:  return plane.pos;
    case 1:  return heli.pos;
    default: return transformPoint(rocketMatrix(), {0.0f, 7.0f, 0.0f});
    }
}

// ---- Text --------------------------------------------------------------------------

const char* toString(PlaneState s)
{
    switch (s)
    {
    case PlaneState::Parking:      return "PARKING";
    case PlaneState::Pushback:     return "PUSHBACK";
    case PlaneState::Taxi:         return "TAXI";
    case PlaneState::HoldShort:    return "HOLD SHORT";
    case PlaneState::RunwayEntry:  return "RUNWAY ENTRY";
    case PlaneState::Acceleration: return "ACCELERATION";
    case PlaneState::Rotation:     return "ROTATION";
    case PlaneState::Climb:        return "CLIMB";
    case PlaneState::CruiseHold:   return "CRUISE/HOLD";
    case PlaneState::Approach:     return "APPROACH";
    case PlaneState::Descent:      return "DESCENT";
    case PlaneState::Flare:        return "FLARE";
    case PlaneState::Touchdown:    return "TOUCHDOWN";
    case PlaneState::TaxiIn:       return "TAXI IN";
    }
    return "";
}

const char* toString(HeliState s)
{
    switch (s)
    {
    case HeliState::Parked:     return "PARKED";
    case HeliState::Startup:    return "ENGINE START";
    case HeliState::LiftOff:    return "LIFT OFF";
    case HeliState::Hover:      return "HOVER";
    case HeliState::Transition: return "TRANSITION";
    case HeliState::Climb:      return "CLIMB";
    case HeliState::CruiseHold: return "CRUISE/HOLD";
    case HeliState::Approach:   return "APPROACH";
    case HeliState::Landing:    return "LANDING";
    case HeliState::Shutdown:   return "SHUTDOWN";
    }
    return "";
}

const char* toString(RocketState s)
{
    switch (s)
    {
    case RocketState::OnPad:       return "ON PAD";
    case RocketState::Countdown:   return "COUNTDOWN";
    case RocketState::Ignition:    return "IGNITION";
    case RocketState::Ascent:      return "ASCENT";
    case RocketState::Flip:        return "FLIP";
    case RocketState::Boostback:   return "BOOSTBACK";
    case RocketState::Coast:       return "COAST";
    case RocketState::EntryBurn:   return "ENTRY BURN";
    case RocketState::LandingBurn: return "LANDING BURN";
    case RocketState::Landed:      return "LANDED";
    }
    return "";
}

std::string Simulation::statusText() const
{
    // Rocket altitude: above the launch mount on the way up, above its
    // landing pad on the way back.
    const StageMotion& c = rocket.core;
    float rocketAlt = c.state >= RocketState::Flip ? c.pos.y - c.target.y : c.pos.y - LAUNCH_MOUNT_TOP;

    char text[256];
    std::snprintf(text, sizeof(text),
                  "Plane: %s (spd %.0f, alt %.0f) | Heli: %s (alt %.0f) | Rocket: %s (alt %.0f)",
                  toString(plane.state), plane.speed, plane.pos.y - PLANE_GROUND_Y,
                  toString(heli.state), heli.pos.y - HELI_GROUND_Y,
                  toString(c.state), rocketAlt);
    return text;
}
