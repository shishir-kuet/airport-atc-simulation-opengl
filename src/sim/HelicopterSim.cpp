// Helicopter: rotor start, vertical lift-off, hover, transition to forward
// flight, holding circle, approach and vertical landing on the helipad.

#include "sim/Flight.h"
#include "sim/Tuning.h"

#include <algorithm>
#include <cmath>

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
