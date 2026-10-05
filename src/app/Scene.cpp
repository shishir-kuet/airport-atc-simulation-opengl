// Drawing the frame. Nothing here changes the simulation: it only turns the
// current state into draw calls.

#include "app/Scene.h"

#include "app/Camera.h"
#include "models/Models.h"
#include "world/Environment.h"

#include <glad/gl.h>

#include <algorithm>
#include <cmath>

void drawWorld(const Renderer& r, const Primitives& p, const Simulation& sim,
               const Mesh& grid, float radarAngle)
{
    drawGround(r, p);
    r.drawLines(grid, Mat4::identity());
    drawAirport(r, p, radarAngle);
    drawHelipad(r, p, translate(Layout::HELIPAD_POS.x, Layout::HELIPAD_POS.y, Layout::HELIPAD_POS.z));
    drawLaunchPad(r, p, translate(Layout::LAUNCH_PAD_POS.x, Layout::LAUNCH_PAD_POS.y, Layout::LAUNCH_PAD_POS.z),
                  sim.rocket.armSwing);
    drawRocketBase(r, p);
}

// cockpitVehicle: the vehicle we are sitting in (-1 = none). Its body (the
// fuselage or cabin around the camera) is left out so it doesn't block the
// view, but the wings, engines, tail, skids and rotor stay visible when the
// pilot looks around.
void drawVehicles(const Renderer& r, const Primitives& p, const Simulation& sim, int cockpitVehicle)
{
    drawAirplane(r, p, sim.airplaneMatrix(), sim.plane.gear, cockpitVehicle == 0);
    drawHelicopter(r, p, sim.helicopterMatrix(), sim.heli.rotorAngle, cockpitVehicle == 1);

    const RocketMotion& k = sim.rocket;
    float flicker = 0.85f + 0.15f * std::sin(sim.time * 45.0f);   // flames flicker slightly

    if (k.upperAttached)
    {
        // The whole rocket, until the stages separate.
        RocketLook look;
        look.boosters = k.boostersAttached;
        look.mainFlame = k.core.flame * flicker;
        look.boosterFlame = k.boosterFlame * flicker;
        drawRocket(r, p, sim.rocketMatrix(), look);
    }
    else
    {
        // The second stage flying its own trajectory, or landed at the base.
        drawRocketUpperStage(r, p, sim.upperStageMatrix(), k.upper.flame * flicker, k.upper.legs);
    }

    // Spent boosters and first stage falling away.
    for (const Debris& d : sim.debris)
    {
        if (d.kind == Debris::Booster)
            drawRocketBooster(r, p, sim.debrisMatrix(d), 0.0f);
        else
            drawRocketFirstStage(r, p, sim.debrisMatrix(d), 0.0f, 0.0f);
    }

    // Two recovered rockets already standing on their legs at the rocket base;
    // the free pad in between is where the returning second stage lands.
    RocketLook parked;
    parked.boosters = false;
    parked.legs = 1.0f;
    const float standY = Layout::LANDING_ZONE_TOP - legFootY(CORE_LEGS);
    for (int i = 0; i < 3; ++i)
    {
        if (i == Layout::ROCKET_BASE_FREE_PAD)
            continue;
        const Vec3& pad = Layout::ROCKET_BASE_PADS[i];
        drawRocket(r, p, translate(pad.x, standY, pad.z) * rotateY(i * 30.0f), parked);
    }
}

// Instrument readings shown on the cockpit panel.
struct Instruments
{
    float speed, maxSpeed;
    float altitude, maxAltitude;
    float heading, pitch, roll;   // degrees
};

// Cockpit frame: window posts, roof and an instrument panel with working
// gauges. `eyeFrame` maps eye coordinates (x right, y up, looking down -z)
// into the world, so the frame moves and banks with the aircraft.
static void drawCockpitFrame(const Renderer& r, const Primitives& p, const Mat4& eyeFrame,
                             bool helicopter, const Instruments& ins)
{
    auto box = [&](Vec3 c, Vec3 s, const Color& color, float angleZ = 0.0f)
    {
        r.drawPart(p.cube, eyeFrame * translate(c.x, c.y, c.z) * rotateZ(angleZ) * scale(s.x, s.y, s.z), color);
    };

    // Round gauge with a needle; `needle` is the needle angle (0 = up, + = clockwise).
    auto gauge = [&](float x, float y, float z, float needle)
    {
        Mat4 g = eyeFrame * translate(x, y, z) * rotateX(90.0f);   // face the pilot
        r.drawPart(p.cylinder, g * scale(0.19f, 0.01f, 0.19f), Color(0.08f, 0.09f, 0.10f));   // dial face
        r.drawPart(p.ringThin, g * scale(0.20f, 0.02f, 0.20f), Color(0.72f, 0.74f, 0.78f));   // bezel
        r.drawSolid(p.cube, eyeFrame * translate(x, y, z + 0.012f) * rotateZ(-needle)
                                * translate(0.0f, 0.04f, 0.0f) * scale(0.012f, 0.085f, 0.004f), Color(0.98f, 0.82f, 0.30f));
    };

    // Artificial horizon: the horizon bar tilts with the bank and moves with the pitch.
    auto attitude = [&](float x, float y, float z)
    {
        Mat4 g = eyeFrame * translate(x, y, z) * rotateX(90.0f);
        r.drawPart(p.cylinder, g * scale(0.19f, 0.01f, 0.19f), Color(0.08f, 0.09f, 0.10f));
        r.drawPart(p.ringThin, g * scale(0.20f, 0.02f, 0.20f), Color(0.72f, 0.74f, 0.78f));
        float offset = std::clamp(-ins.pitch * 0.004f, -0.07f, 0.07f);
        r.drawSolid(p.cube, eyeFrame * translate(x, y, z + 0.012f) * rotateZ(ins.roll)
                                * translate(0.0f, offset, 0.0f) * scale(0.17f, 0.008f, 0.004f), Color(0.96f, 0.96f, 0.94f));
        r.drawSolid(p.cube, eyeFrame * translate(x, y, z + 0.014f) * scale(0.06f, 0.006f, 0.004f), Color(0.98f, 0.82f, 0.30f));
    };

    float speedNeedle = -135.0f + 270.0f * std::clamp(ins.speed / ins.maxSpeed, 0.0f, 1.0f);
    float altNeedle = -135.0f + 270.0f * std::clamp(ins.altitude / ins.maxAltitude, 0.0f, 1.0f);
    float compass = -ins.heading;   // heading 0 (east) points the needle up

    const Color post{0.21f, 0.22f, 0.24f}, panel{0.15f, 0.16f, 0.18f};
    if (!helicopter)
    {
        // Airliner: roof, centre and side window posts, glareshield, wide panel.
        box({0.0f, 0.56f, -1.0f}, {2.6f, 0.12f, 0.2f}, post);
        box({0.0f, 0.10f, -1.0f}, {0.04f, 0.9f, 0.04f}, post);
        box({-0.98f, 0.0f, -1.0f}, {0.08f, 1.3f, 0.05f}, post, -15.0f);
        box({0.98f, 0.0f, -1.0f}, {0.08f, 1.3f, 0.05f}, post, 15.0f);
        box({0.0f, -0.30f, -0.95f}, {2.4f, 0.04f, 0.2f}, Color(0.11f, 0.12f, 0.13f));          // glareshield
        box({0.0f, -0.46f, -1.0f}, {2.4f, 0.30f, 0.05f}, panel);          // instrument panel
        gauge(-0.45f, -0.44f, -0.97f, speedNeedle);
        attitude(-0.15f, -0.44f, -0.97f);
        gauge(0.15f, -0.44f, -0.97f, altNeedle);
        gauge(0.45f, -0.44f, -0.97f, compass);
    }
    else
    {
        // Helicopter: big bubble canopy, thin frame, small centre console.
        box({0.0f, 0.58f, -1.0f}, {2.6f, 0.06f, 0.2f}, post);
        box({-1.02f, 0.0f, -1.0f}, {0.05f, 1.3f, 0.05f}, post, -8.0f);
        box({1.02f, 0.0f, -1.0f}, {0.05f, 1.3f, 0.05f}, post, 8.0f);
        box({0.0f, -0.28f, -0.92f}, {0.95f, 0.03f, 0.15f}, Color(0.11f, 0.12f, 0.13f));
        box({0.0f, -0.43f, -0.95f}, {0.95f, 0.28f, 0.05f}, panel);
        gauge(-0.3f, -0.42f, -0.92f, speedNeedle);
        attitude(0.0f, -0.42f, -0.92f);
        gauge(0.3f, -0.42f, -0.92f, altNeedle);
    }
}

void drawSmoke(const Renderer& r, const Primitives& p, const Simulation& sim)
{
    for (const Puff& puff : sim.puffs)
    {
        float t = puff.age / puff.life;
        float size = puff.size0 + (puff.size1 - puff.size0) * std::sqrt(t);
        if (t > 0.75f)
            size *= (1.0f - t) / 0.25f;   // shrink away at the end of its life
        // Each kind of smoke starts in its own colour (white propellant, sooty
        // jet exhaust, burnt rubber) and all of them cool towards the same grey.
        r.drawSolid(p.smoke, translate(puff.pos.x, puff.pos.y, puff.pos.z) * scale(size, size, size),
                    mix(puff.tint, Color(0.45f, 0.45f, 0.48f), t));
    }
}

// Picks the gauge readings of the vehicle we sit in, then draws its frame on
// top of the scene (depth cleared, so the outside world never pokes through).
void drawCockpitOverlay(Renderer& r, const Primitives& p, const Simulation& sim, int vehicle)
{
    const CockpitSpec& c = COCKPITS[vehicle];
    if (!c.frame)
        return;

    const bool heli = vehicle == 1;
    // Eye coordinates -> vehicle: forward (+X) is eye -z, up stays up.
    Mat4 eyeFrame = cockpitVehicleMatrix() * translate(c.eye.x, c.eye.y, c.eye.z) * rotateY(-90.0f);

    Instruments ins;
    if (heli)
        ins = {sim.heli.speed, 20.0f, sim.heli.pos.y - Layout::HELIPAD_TOP - HELICOPTER_GROUND_OFFSET,
               40.0f, sim.heli.heading, sim.heli.pitch, sim.heli.roll};
    else
        ins = {sim.plane.speed, 35.0f, sim.plane.pos.y - Layout::PAVEMENT_TOP - AIRPLANE_GROUND_OFFSET,
               60.0f, sim.plane.heading, sim.plane.pitch, sim.plane.roll};

    // The frame and the panel belong to the picture, not to the world: they
    // are drawn unlit so the gauges stay readable in every shading mode (a
    // lit cockpit interior faces away from the sun and goes almost black).
    Shading outside = r.shading;
    r.shading = Shading::Flat;

    glClear(GL_DEPTH_BUFFER_BIT);
    drawCockpitFrame(r, p, eyeFrame, heli, ins);

    r.shading = outside;
}
