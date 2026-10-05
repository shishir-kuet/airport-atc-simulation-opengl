// Commands (what the keys trigger), the update step that advances all three
// vehicles, the model matrices they are drawn with, and the status line.
// The motion of each vehicle lives in AirplaneSim.cpp, HelicopterSim.cpp and
// RocketSim.cpp; the smoke in Effects.cpp.

#include "sim/Flight.h"
#include "sim/Tuning.h"

#include <cmath>
#include <cstdio>

// ---- Commands ----------------------------------------------------------------------

void Simulation::reset()
{
    parkAirplane();

    heli = HelicopterMotion{};
    heli.pos = Vec3(HELIPAD_POS.x, HELI_GROUND_Y, HELIPAD_POS.z);

    rocket = RocketMotion{};
    rocket.core.pos = Vec3(LAUNCH_PAD_POS.x, LAUNCH_MOUNT_TOP + ROCKET_GROUND_OFFSET, LAUNCH_PAD_POS.z);

    debris.clear();
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
    if (!k.upperAttached && k.upper.state == RocketState::Landed)
    {
        // The second stage is back: a new rocket is stacked on the launch pad.
        radio("[LAUNCH] New rocket stacked on the launch pad.");
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

Mat4 Simulation::upperStageMatrix() const
{
    const StageMotion& u = rocket.upper;
    return translate(u.pos.x, u.pos.y, u.pos.z) * rotateZ(-u.tiltX) * rotateX(u.tiltZ);
}

Mat4 Simulation::debrisMatrix(const Debris& d) const
{
    // Where it was at separation, moved by its fall, tumbling about its middle.
    return translate(d.offset.x, d.offset.y, d.offset.z) * d.start
         * translate(0, d.pivotY, 0) * rotateZ(-d.angle) * translate(0, -d.pivotY, 0);
}

Vec3 Simulation::focusPoint(int vehicle) const
{
    switch (vehicle)
    {
    case 0:  return plane.pos;
    case 1:  return heli.pos;
    default: // the rocket; after stage separation, the second stage
        return rocket.upperAttached ? transformPoint(rocketMatrix(), {0.0f, 7.0f, 0.0f})
                                    : transformPoint(upperStageMatrix(), {0.0f, 10.0f, 0.0f});
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
    case PlaneState::Manual:       return "MANUAL";
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
    case RocketState::Stage2Burn:  return "STAGE 2 FLIGHT";
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
    // The flying rocket: the whole stack, then the second stage after separation.
    // Altitude above the launch mount on the way up, above its landing pad on the way back.
    const StageMotion& s = rocket.upperAttached ? rocket.core : rocket.upper;
    float rocketAlt = s.state >= RocketState::Flip ? s.pos.y - s.target.y : s.pos.y - LAUNCH_MOUNT_TOP;

    // In manual flight the pilot also needs the throttle, the gear and a stall warning.
    char planeText[96];
    if (plane.state == PlaneState::Manual)
        std::snprintf(planeText, sizeof(planeText), "MANUAL thr %.0f%% gear %s%s",
                      plane.throttle * 100.0f, plane.gear > 0.5f ? "DOWN" : "UP",
                      !plane.onGround && plane.speed < STALL_SPEED ? " STALL" : "");
    else
        std::snprintf(planeText, sizeof(planeText), "%s", toString(plane.state));

    char text[320];
    std::snprintf(text, sizeof(text),
                  "Plane: %s (spd %.0f, alt %.0f) | Heli: %s (alt %.0f) | Rocket: %s (alt %.0f)",
                  planeText, plane.speed, plane.pos.y - PLANE_GROUND_Y,
                  toString(heli.state), heli.pos.y - HELI_GROUND_Y,
                  toString(s.state), rocketAlt);
    return text;
}
