// Rocket: countdown, ascent with a gravity turn, booster separation, stage
// separation, the second stage's own burn, and its flip-boostback-entry-
// landing burn sequence back onto pad 4 of the rocket base.

#include "sim/Flight.h"
#include "sim/Tuning.h"

#include <algorithm>
#include <cmath>

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

        // First separation: the side boosters are empty and fall away.
        if (t >= BOOSTER_SEP_TIME && k.boostersAttached)
        {
            radio("[LAUNCH] Booster separation. The empty boosters fall away.");
            Mat4 base = rocketMatrix();
            for (int i = 0; i < 2; ++i)
            {
                Debris d{Debris::Booster};
                d.start = rocketBoosterMatrix(base, i);
                Vec3 out = transformPoint(d.start, {1, 0, 0}) - transformPoint(d.start, {0, 0, 0});
                d.velocity = c.velocity + out * 4.0f;   // pushed away sideways
                d.spinRate = 25.0f;
                d.pivotY = 3.0f;
                debris.push_back(d);
            }
            k.boostersAttached = false;
            k.boosterFlame = 0;
        }

        // Second separation: the first stage is empty and falls away; the
        // second stage carries on alone and later flies back.
        if (t >= STAGE_SEP_TIME)
        {
            radio("[LAUNCH] Main engine cut-off. Stage separation! The empty first stage falls away.");

            Debris d{Debris::FirstStage};
            d.start = rocketMatrix();
            d.velocity = c.velocity - dir * 1.0f;
            d.spinRate = 12.0f;
            d.pivotY = 3.5f;
            debris.push_back(d);

            // Second stage: starts where it sat on top of the first stage,
            // pushed gently forward by the separation springs.
            const Vec3& pad = ROCKET_BASE_PADS[ROCKET_BASE_FREE_PAD];
            StageMotion& u = k.upper;
            u = StageMotion{};
            u.pos = c.pos;
            u.velocity = c.velocity + dir * 2.0f;
            u.tiltX = c.tiltX;
            u.target = Vec3(pad.x, UPPER_LAND_Y, pad.z);   // free pad at the rocket base
            u.surfaceY = LANDING_ZONE_TOP;
            u.site = "pad 4 at the rocket base";
            u.state = RocketState::Stage2Burn;
            k.upperAttached = false;

            c.flame = 0;
            c.state = RocketState::Stage2Burn;   // from now on the mission is the second stage
        }
        break;
    }

    default:
        break;
    }

    if (!k.upperAttached)
    {
        StageMotion& u = k.upper;
        if (u.state == RocketState::Stage2Burn)
        {
            // Second stage burn: lights after a short coast and speeds up much
            // faster than the heavy first stage did, pitching further over.
            // After cut-off it flies a free (ballistic) arc up to its highest point.
            u.timer += dt;
            float burnTime = u.timer - STAGE2_DELAY;
            bool burning = burnTime >= 0.0f && burnTime < STAGE2_BURN_TIME;
            if (burning && u.flame == 0.0f)
                radio("[STAGE 2] Second stage ignition.");
            if (!burning && u.flame > 0.0f)
                radio("[STAGE 2] Engine cut-off. Flying its trajectory up to the highest point.");
            u.flame = burning ? std::min(1.0f, burnTime * 2.0f + 0.05f) : 0.0f;

            if (burning)
            {
                u.tiltX = approach(u.tiltX, STAGE2_MAX_TILT, 3.0f * dt);
                Vec3 axis(std::sin(radians(u.tiltX)), std::cos(radians(u.tiltX)), 0.0f);
                u.velocity = u.velocity + axis * (STAGE2_ACCEL * u.flame * dt);
            }
            u.velocity.y -= GRAVITY * dt;
            u.pos = u.pos + u.velocity * dt;

            // Near the top of the arc it turns round and heads home.
            if (burnTime > STAGE2_BURN_TIME && u.velocity.y < 30.0f)
            {
                u.state = RocketState::Flip;
                u.timer = 0;
                radio("[STAGE 2] Highest point reached. Flipping to fly back to the rocket base.");
            }
        }
        else
        {
            updateReturningStage(u, dt, "[STAGE 2] ");
        }
    }
}

// Return flight of the second stage, like a reusable rocket:
//   Flip         - engine off, turn the rocket so the engine can push it back
//   Boostback    - burn to reverse the sideways speed, aiming at the landing spot
//   Coast        - engine off, fall engine-first; grid fins steer a little
//   Entry burn   - slow down before the thick lower atmosphere
//   Landing burn - legs out, decelerate to touch down gently on the target
void Simulation::updateReturningStage(StageMotion& s, float dt, const char* name)
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
    fallTime = std::max(fallTime, 2.0f) * 1.25f;
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

    const float flipRate = 30.0f;
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
        else if (!s.entryBurnDone && vy < -(ENTRY_BURN_SPEED + 30.0f) && height > 300.0f)
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

        // Slide over the exact landing spot: the sideways motion also brakes
        // evenly to zero, so it must start at twice the average speed
        // (2 x distance / time left). Time left while braking evenly is about
        // 2 x height / falling speed. Lean slightly into the correction.
        float timeLeft = std::max(0.6f, 2.0f * std::max(height, 0.0f) / (std::fabs(s.velocity.y) + 1.0f));
        Vec3 over = toTarget * (2.0f / timeLeft);
        float overLen = std::sqrt(dot(over, over));
        if (overLen > 60.0f)
            over = over * (60.0f / overLen);
        Vec3 lean = over - sideways;
        pushSideways(over, 15.0f);
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
            std::string message = std::string("Touchdown on ") + s.site + ". Welcome home!";
            setState(RocketState::Landed, message.c_str());
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
