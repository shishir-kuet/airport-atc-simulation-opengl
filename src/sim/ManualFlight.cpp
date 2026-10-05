// Manual flight: the user flies the airliner. Take-off, flight and landing
// are in the pilot's hands; the autopilot can be handed back at any moment
// and then finishes the flight (or the aircraft is towed back to its stand).
//
// The flight model is a point mass moving along its flight path:
//   speed       changes with  thrust - drag - gravity * sin(climb angle)
//   heading     changes with  the bank angle (coordinated turn)
//   climb angle changes with  the stick, unless the wings have stalled.

#include "sim/Flight.h"
#include "sim/Tuning.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

// Puts the airliner back at stand 2, nose towards the taxiway, ready to go.
void Simulation::parkAirplane()
{
    plane = AirplaneMotion{};
    plane.pos = Vec3(STAND_X[1], PLANE_GROUND_Y, STAND_STOP_Z + 3.6f);
    plane.heading = 90.0f;   // nose towards the taxiway (-Z)
}

void Simulation::toggleManualAirplane()
{
    AirplaneMotion& a = plane;
    const float altitude = a.pos.y - PLANE_GROUND_Y;

    // The engine trails restart from wherever the aircraft is now.
    trailStarted[TRAIL_ENGINE_L] = trailStarted[TRAIL_ENGINE_R] = false;

    if (a.state != PlaneState::Manual)
    {
        if (altitude > 0.5f)
        {
            // Taking over in flight: keep the current speed, heading and
            // attitude; set the throttle that holds this speed in level flight.
            a.onGround = false;
            a.throttle = std::clamp(MANUAL_DRAG * a.speed * a.speed / MANUAL_THRUST, 0.2f, 1.0f);
            a.gearTarget = a.gear > 0.5f ? 1.0f : 0.0f;
            radio("[TOWER]  Airplane, you have control.");
        }
        else
        {
            // On the ground: lined up at the start of runway 09, engines idle.
            a = AirplaneMotion{};
            a.pos = Vec3(THRESHOLD_X + 8.0f, PLANE_GROUND_Y, RUNWAY_Z);
            a.heading = 0.0f;
            radio("[TOWER]  Airplane, lined up on runway 09, cleared for takeoff. You have control.");
        }
        a.state = PlaneState::Manual;
        a.timer = 0;
        a.stallWarned = false;
        radio("[INFO]   I/K throttle, 8/5 climb/descend, 4/6 bank, U gear, X brakes, M = autopilot.");
        return;
    }

    if (!a.onGround)
    {
        // The autopilot takes the aircraft from here: it flies to the hold
        // over the airport, then the circuit, and lands on runway 09.
        a.state = PlaneState::CruiseHold;
        a.timer = 0;
        a.waypoint = 0;
        radio("[TOWER]  Airplane, autopilot engaged. Hold over the airport, then land runway 09.");
    }
    else
    {
        parkAirplane();
        radio("[TOWER]  Airplane, the tug has towed you back to stand 2.");
    }
}

void Simulation::togglePlaneGear()
{
    AirplaneMotion& a = plane;
    if (a.state != PlaneState::Manual)
        return;
    if (a.onGround)
    {
        radio("[COCKPIT] The gear cannot be raised on the ground.");
        return;
    }
    a.gearTarget = a.gearTarget > 0.5f ? 0.0f : 1.0f;
    radio(a.gearTarget > 0.5f ? "[YOU]    Gear down." : "[YOU]    Gear up.");
}

void Simulation::updateManualAirplane(float dt)
{
    AirplaneMotion& a = plane;
    const PlaneControls& in = planeInput;
    a.timer += dt;

    // Engines and gear: the levers move, the parts follow at their own speed.
    a.throttle = std::clamp(a.throttle + in.throttle * THROTTLE_RATE * dt, 0.0f, 1.0f);
    a.gear = approach(a.gear, a.gearTarget, dt / 3.0f);

    // Speed along the flight path: thrust pushes, drag (growing with the
    // square of the speed) holds back, and gravity takes speed away in a
    // climb and gives it back in a descent.
    float gamma = radians(a.climbAngle);
    float accel = a.throttle * MANUAL_THRUST
                - MANUAL_DRAG * a.speed * a.speed
                - GRAVITY * std::sin(gamma);
    if (a.onGround)
        accel -= ROLLING_FRICTION + (in.brake ? BRAKE_DECEL : 0.0f);
    a.speed = std::clamp(a.speed + accel * dt, 0.0f, MANUAL_MAX_SPEED);

    if (a.onGround)
    {
        // Nose-wheel steering: works once rolling, a little less at speed.
        float steer = STEER_RATE * std::clamp(a.speed / 3.0f, 0.0f, 1.0f) * (a.speed > 25.0f ? 0.4f : 1.0f);
        a.heading -= in.roll * steer * dt;   // heading grows to the left
        a.roll = approach(a.roll, 0.0f, 20.0f * dt);
        a.climbAngle = 0.0f;

        // Rotation: pulling back near take-off speed raises the nose; once
        // fast enough with the nose up, the wings lift the aircraft off.
        float wantedPitch = (in.pitch > 0.0f && a.speed > 0.8f * ROTATE_SPEED) ? LIFTOFF_PITCH : 0.0f;
        a.pitch = approach(a.pitch, wantedPitch, 6.0f * dt);
        if (in.pitch > 0.0f && a.speed >= ROTATE_SPEED && a.pitch >= LIFTOFF_PITCH - 1.0f)
        {
            a.onGround = false;
            a.climbAngle = 3.0f;
            radio("[YOU]    Liftoff!");
        }

        a.pos = a.pos + forward(a.heading) * (a.speed * dt);
        a.pos.y = PLANE_GROUND_Y;
    }
    else
    {
        // Ailerons: bank towards the stick; released, the wings level out.
        a.roll = approach(a.roll, in.roll * MANUAL_MAX_BANK, MANUAL_ROLL_RATE * dt);

        // Coordinated turn: the bank angle sets the turn rate, with the same
        // ratio as the autopilot (PLANE_BANK degrees -> PLANE_TURN_RATE deg/s).
        float turnRate = -PLANE_TURN_RATE * std::tan(radians(a.roll)) / std::tan(radians(PLANE_BANK));
        a.heading += turnRate * dt;

        // Elevator: the stick changes the climb angle; released, the angle is
        // held (as if trimmed). Too slow, and the nose drops by itself.
        if (a.speed < STALL_SPEED)
        {
            a.climbAngle = approach(a.climbAngle, -15.0f, 12.0f * dt);
            if (!a.stallWarned)
                radio("[COCKPIT] STALL! Lower the nose and add power.");
            a.stallWarned = true;
        }
        else
        {
            if (a.speed > STALL_SPEED + 4.0f)
                a.stallWarned = false;   // re-arm only once well clear of the stall
            a.climbAngle = std::clamp(a.climbAngle + in.pitch * MANUAL_PITCH_RATE * dt,
                                      -MANUAL_MAX_DIVE, MANUAL_MAX_CLIMB);
        }
        if (a.pos.y - PLANE_GROUND_Y > MANUAL_CEILING)
            a.climbAngle = std::min(a.climbAngle, 0.0f);

        // Slower flight needs more angle of attack, so the nose sits higher.
        float angleOfAttack = std::clamp(3.0f + (CLIMB_SPEED - a.speed) * 0.25f, 0.0f, 10.0f);
        a.pitch = approach(a.pitch, a.climbAngle + angleOfAttack, 10.0f * dt);

        gamma = radians(a.climbAngle);
        a.pos = a.pos + forward(a.heading) * (a.speed * std::cos(gamma) * dt);
        a.pos.y += a.speed * std::sin(gamma) * dt;

        if (a.pos.y <= PLANE_GROUND_Y)
        {
            // Touchdown: judge it by the sink rate, the gear and where it is.
            float sink = -a.speed * std::sin(gamma);
            bool onRunway = std::fabs(a.pos.z - RUNWAY_Z) < RUNWAY_WIDTH / 2 &&
                            std::fabs(a.pos.x - RUNWAY_CENTER.x) < RUNWAY_LENGTH / 2;
            a.pos.y = PLANE_GROUND_Y;
            a.onGround = true;
            a.climbAngle = 0.0f;

            char text[160];
            if (a.gear < 0.9f)
            {
                a.speed *= 0.5f;   // scraping along on the belly
                std::snprintf(text, sizeof(text), "[COCKPIT] Belly landing - the gear was up!%s",
                              onRunway ? "" : " And off the runway.");
            }
            else
            {
                tyreSmoke();
                std::snprintf(text, sizeof(text), "[YOU]    %s touchdown (sink %.1f)%s",
                              sink > HARD_SINK ? "Hard" : (sink < 1.5f ? "Smooth" : "Firm"), sink,
                              onRunway ? " on runway 09." : " - off the runway!");
            }
            radio(text);
        }
    }

    // Stay over the ground slab.
    a.pos.x = std::clamp(a.pos.x, -WORLD_EDGE, WORLD_EDGE);
    a.pos.z = std::clamp(a.pos.z, -WORLD_EDGE, WORLD_EDGE);
    a.heading = wrapAngle(a.heading);
}
