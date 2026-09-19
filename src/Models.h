// The three vehicles of the simulation: Airplane, Helicopter and Rocket.
//
// Each model is built hierarchically: a `base` matrix places the whole
// vehicle in the world, and every part is `base * (local translate/rotate/scale)`
// applied to one of the shared unit primitives.
#pragma once

#include "Math3D.h"
#include "Primitives.h"
#include "Renderer.h"

// Nose points to +X, up is +Y. Origin at fuselage centre; wheels touch y = -1.78.
// gear: 1 = landing gear down, 0 = fully retracted.
void drawAirplane(const Renderer& r, const Primitives& p, const Mat4& base, float gear = 1.0f);

// Nose points to +X, up is +Y. Origin at cabin centre; skids touch y = -1.52.
// rotorAngle (degrees) spins the main and tail rotors.
// rotorOnly: draw just the main rotor (seen from the cockpit).
void drawHelicopter(const Renderer& r, const Primitives& p, const Mat4& base, float rotorAngle,
                    bool rotorOnly = false);

// ---- Rocket ----------------------------------------------------------------------

// Landing legs of a rocket stage, hinged on the side of the body.
struct LegGeometry
{
    int count;
    float firstAngle;    // around the body (degrees)
    float hingeRadius, hingeY, length, footSize;
};
constexpr float LEG_DEPLOY_ANGLE = 150.0f;       // swing out and down when deployed
constexpr float COS_LEG_DEPLOY   = -0.8660254f;  // cos(150 deg)
constexpr LegGeometry CORE_LEGS    = {4, 0.0f,  0.80f, 2.3f, 2.9f, 0.45f};
constexpr LegGeometry BOOSTER_LEGS = {4, 45.0f, 0.35f, 1.6f, 1.8f, 0.30f};

// Height of the feet below the model origin with the legs deployed
// (hinge + leg reaching down, minus half the foot pad). Negative = below.
constexpr float legFootY(const LegGeometry& g) { return g.hingeY + g.length * COS_LEG_DEPLOY - 0.05f; }

// Which parts are attached, how strongly each engine burns (0..1) and how
// far the core's landing legs are deployed (0..1).
struct RocketLook
{
    bool boosters = true;
    float mainFlame = 0, boosterFlame = 0, legs = 0;
};

// Stands upright along +Y. Origin at the bottom of the engine nozzle.
void drawRocket(const Renderer& r, const Primitives& p, const Mat4& base, const RocketLook& look = {});

// Side boosters (drawn on their own after separation). Origin at the bottom of the booster.
Mat4 rocketBoosterMatrix(const Mat4& rocketBase, int index);   // index 0 (+X side) or 1 (-X side)
void drawRocketBooster(const Renderer& r, const Primitives& p, const Mat4& booster, float thrust,
                       float legs = 0.0f);

// Heights used to rest each model on the ground (y = 0).
constexpr float AIRPLANE_GROUND_OFFSET   = 1.78f;
constexpr float HELICOPTER_GROUND_OFFSET = 1.52f;
constexpr float ROCKET_GROUND_OFFSET     = 0.0f;
