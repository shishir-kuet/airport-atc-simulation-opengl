// Airplane: pushback, taxi, take-off roll, climb, holding pattern, approach,
// flare, touchdown and taxi back to the stand - one state at a time.

#include "sim/Flight.h"
#include "sim/Tuning.h"

#include <algorithm>
#include <cmath>

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
    {THRESHOLD_X - 120.0f, RUNWAY_Z - 95.0f, 15.0f, 28.0f},   // end of downwind
};
constexpr int PATTERN_POINTS = sizeof(PATTERN) / sizeof(PATTERN[0]);

void Simulation::updateAirplane(float dt)
{
    if (plane.state == PlaneState::Manual)
    {
        updateManualAirplane(dt);   // the user is flying (sim/ManualFlight.cpp)
        return;
    }

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
        if (altitude > 6.0f)
            setState(PlaneState::CruiseHold, "[TOWER]  Airplane, turn left, climb to 30, hold over the airport.");
        break;

    case PlaneState::CruiseHold:
        fly(orbitHeading(a.pos, PLANE_HOLD_X, PLANE_HOLD_Z, PLANE_HOLD_RADIUS, 0.5f),
            std::clamp((PLANE_HOLD_ALT - altitude) * 0.5f, -4.0f, 10.0f), CRUISE_SPEED, PLANE_TURN_RATE);
        retractGear();
        // A fixed time in the hold: long enough to swing round a good part of
        // the circle over the airport, short enough to keep the whole flight
        // to about half a minute.
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
        fly(headingTo(a.pos, w.x, w.z), std::clamp((w.alt - altitude) * 0.3f, -8.0f, 5.0f), w.speed, 20.0f);

        Vec3 f = forward(a.heading);
        // An airliner turns in a wide circle, so a point counts as reached
        // once it is close by and falls behind the aircraft.
        bool passed = dist < 60.0f && (f.x * (w.x - a.pos.x) + f.z * (w.z - a.pos.z)) <= 0.0f;
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
        // The glide slope is a cone around the aiming point, so the height to
        // hold comes from the real distance to it, not just from the distance
        // along the runway. While the aircraft is still turning onto final it
        // is further away and therefore stays higher, instead of sinking into
        // the ground beside the runway during the turn.
        float toAim = distanceXZ(a.pos, AIM_POINT_X, RUNWAY_Z);
        float glideAlt = (a.pos.x < AIM_POINT_X ? toAim : 0.0f) * std::tan(radians(GLIDE_SLOPE));
        float climb = std::clamp(-GLIDE_SLOPE + (glideAlt - altitude) * 0.8f, -9.0f, 0.0f);
        fly(localizerHeading(a.pos, 2.5f, 70.0f), climb, APPROACH_SPEED, 28.0f);
        if (altitude < FLARE_HEIGHT)
            setState(PlaneState::Flare, "[PILOT]  Flare.");
        break;
    }

    case PlaneState::Flare:
    {
        // Just above the runway: raise the nose to soften the sink rate.
        a.gear = 1.0f;
        a.heading = steer(a.heading, localizerHeading(a.pos, 2.5f, 12.0f), 10.0f, dt);
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
            tyreSmoke();   // burnt rubber as the wheels spin up
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
