// Smoke: single puffs, the exhaust trail behind a flying stage, and the
// update that ages every puff and makes the spent parts tumble down.

#include "sim/Flight.h"
#include "sim/Tuning.h"

#include <algorithm>
#include <cmath>

void Simulation::emitPuff(Vec3 pos, Vec3 velocity, float life, float size0, float size1,
                          const Color& tint)
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
    p.tint = tint;
    puffs.push_back(p);
}

// Exhaust trail: one puff every `spacing` units travelled, so the trail stays
// continuous however fast the stage moves.
void Simulation::emitTrail(int emitter, const Vec3& exit, float size, float life, const Color& tint)
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
        emitPuff(pos, drift, life, 1.8f * size, 5.0f * size, tint);
    }
    if (count > 0)
        lastTrail[emitter] = (count == 60) ? exit : lastTrail[emitter] + d * (count * spacing / len);
}

// The wheels are standing still when they hit the runway at landing speed,
// so they are dragged until they spin up: that is the puff of burnt rubber
// seen at every landing. It is thrown backwards from both main gear legs.
void Simulation::tyreSmoke()
{
    Vec3 back = forward(plane.heading) * -1.0f;
    for (float side : {1.0f, -1.0f})
    {
        Vec3 wheel = transformPoint(airplaneMatrix(), {-0.2f, -AIRPLANE_GROUND_OFFSET + 0.1f, 1.0f * side});
        for (int i = 0; i < 12; ++i)
        {
            Vec3 vel = back * (3.0f + randomFloat() * 7.0f)
                     + Vec3((randomFloat() - 0.5f) * 2.0f, 1.0f + randomFloat() * 2.0f, (randomFloat() - 0.5f) * 2.0f);
            emitPuff(wheel, vel, 1.4f + randomFloat() * 0.6f, 0.35f, 2.4f, Smoke::TYRE);
        }
    }
}

void Simulation::updateEffects(float dt)
{
    const RocketMotion& k = rocket;
    const StageMotion& c = k.core;
    const StageMotion& u = k.upper;
    bool coreBurning = c.flame > 0.0f;

    smokeTimer += dt;
    const float interval = 0.04f;
    while (smokeTimer >= interval)
    {
        smokeTimer -= interval;

        // Ground cloud rolling out of the flame trench (+Z side of the pad)
        // while the engines fire at launch.
        bool nearPad = c.pos.y - LAUNCH_MOUNT_TOP < 25.0f &&
                       distanceXZ(c.pos, LAUNCH_PAD_POS.x, LAUNCH_PAD_POS.z) < 15.0f;
        if (coreBurning && nearPad)
        {
            Vec3 pos = LAUNCH_PAD_POS + Vec3((randomFloat() - 0.5f) * 3.0f, 1.8f, 9.0f + randomFloat() * 2.0f);
            Vec3 vel((randomFloat() - 0.5f) * 3.0f, 1.0f + randomFloat() * 1.5f, 5.0f + randomFloat() * 5.0f);
            emitPuff(pos, vel, 5.0f, 2.0f, 7.0f);
        }

        // Dust blown outwards across the landing pad by the second stage's landing burn.
        if (!k.upperAttached && u.state == RocketState::LandingBurn && u.pos.y - u.target.y < 12.0f)
        {
            float a = randomFloat() * 2.0f * PI;
            Vec3 dir(std::cos(a), 0.0f, std::sin(a));
            emitPuff(Vec3(u.target.x, u.surfaceY + 0.4f, u.target.z) + dir * 1.5f,
                     dir * (6.0f + randomFloat() * 4.0f) + Vec3(0.0f, 0.8f, 0.0f), 3.0f, 1.2f, 3.5f);
        }
    }

    // Exhaust trails of the whole rocket and, after separation, of the second stage.
    if (coreBurning && c.state != RocketState::Ignition)
        emitTrail(TRAIL_ROCKET, transformPoint(rocketMatrix(), {0.0f, -1.5f, 0.0f}), 1.0f);
    else
        trailStarted[TRAIL_ROCKET] = false;

    if (!k.upperAttached && u.flame > 0.0f)
        emitTrail(TRAIL_UPPER, transformPoint(upperStageMatrix(), {0.0f, UPPER_STAGE_BOTTOM - 1.2f, 0.0f}), 0.7f);
    else
        trailStarted[TRAIL_UPPER] = false;

    // Jet exhaust, from the moment takeoff power is set until the wheels are
    // back on the runway. Near the ground it is the thin sooty plume behind
    // the engines; once the aircraft is up it turns into a white trail that
    // hangs in the air much longer, the way a contrail does.
    // In manual flight the trail follows the throttle: idle engines are clean.
    bool autopilotPower = plane.state >= PlaneState::Acceleration && plane.state <= PlaneState::Touchdown;
    bool manualPower = plane.state == PlaneState::Manual && plane.throttle > 0.25f;
    bool enginesRunning = (autopilotPower || manualPower) && plane.speed > 8.0f;
    float planeAlt = plane.pos.y - PLANE_GROUND_Y;
    bool upHigh = planeAlt > 12.0f;
    for (int i = 0; i < 2; ++i)
    {
        int slot = TRAIL_ENGINE_L + i;
        float side = i == 0 ? 1.0f : -1.0f;
        if (enginesRunning)
            emitTrail(slot, transformPoint(airplaneMatrix(), {-1.0f, -0.85f, 2.3f * side}),
                      upHigh ? 0.26f : 0.18f,
                      upHigh ? 4.0f : 1.1f,
                      upHigh ? Smoke::CONTRAIL : Smoke::EXHAUST);
        else
            trailStarted[slot] = false;
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

    // Spent boosters and first stage fall away under gravity while slowly
    // tumbling; their job is done, so they are removed once they reach the ground.
    for (Debris& d : debris)
    {
        d.velocity.y -= GRAVITY * dt;
        d.offset = d.offset + d.velocity * dt;
        d.angle += d.spinRate * dt;
    }
    debris.erase(std::remove_if(debris.begin(), debris.end(),
                                [this](const Debris& d)
                                { return transformPoint(debrisMatrix(d), {0, d.pivotY, 0}).y < 0.0f; }),
                 debris.end());
}
