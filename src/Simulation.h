// Motion of the three vehicles as state machines (takeoff, holding and
// landing sequences), plus the rocket's falling debris and smoke puffs.
//
// Each vehicle stores its position, orientation (heading / pitch / roll) and
// speed; update() advances the current state and decides the next one.
#pragma once

#include "Math3D.h"

#include <string>
#include <vector>

// ---- Airplane ----------------------------------------------------------------

// Full cycle: Parking -> (Pushback) -> Taxi -> Hold Short -> Runway Entry ->
// Acceleration -> Rotation -> Climb -> Cruise/Hold -> Approach -> Descent ->
// Flare -> Touchdown -> Taxi In -> Parking
enum class PlaneState
{
    Parking, Pushback, Taxi, HoldShort, RunwayEntry, Acceleration, Rotation, Climb,
    CruiseHold, Approach, Descent, Flare, Touchdown, TaxiIn
};

struct AirplaneMotion
{
    PlaneState state = PlaneState::Parking;
    Vec3 pos;                 // centre of the fuselage
    float heading = 0;        // degrees; 0 = +X (runway 09), 90 = -Z
    float pitch = 0;          // nose up positive
    float roll = 0;           // right wing down positive
    float speed = 0;          // along the flight path (units / s)
    float climbAngle = 0;     // flight path angle (degrees)
    float gear = 1;           // 1 = down, 0 = retracted
    float timer = 0;          // time spent in the current state
    int waypoint = 0;         // progress inside the current state
    bool noseIn = false;      // parked facing the terminal (needs a pushback)
    bool exitEast = false;    // after landing: leave by the east connector (else the middle one)
};

// ---- Helicopter --------------------------------------------------------------

enum class HeliState
{
    Parked, Startup, LiftOff, Hover, Transition, Climb, CruiseHold, Approach, Landing, Shutdown
};

struct HelicopterMotion
{
    HeliState state = HeliState::Parked;
    Vec3 pos;
    float heading = 0, pitch = 0, roll = 0;
    float speed = 0;          // forward speed
    float verticalSpeed = 0;
    float rotorSpeed = 0;     // degrees per second
    float rotorAngle = 0;
    float timer = 0;
};

// ---- Rocket ------------------------------------------------------------------

// The rocket separates twice; only the final (second) stage comes back:
//   whole rocket : OnPad -> Countdown -> Ignition -> Ascent
//   boosters     : separate at 10 s, their job is done - they fall away
//   first stage  : separates at 20 s, its job is done - it falls away
//   second stage : Stage 2 Burn (faster, higher) -> Flip -> Boostback -> Coast
//                  -> Entry Burn -> Coast -> Landing Burn -> Landed (rocket base, pad 4)
enum class RocketState
{
    OnPad, Countdown, Ignition, Ascent, Stage2Burn,
    Flip, Boostback, Coast, EntryBurn, LandingBurn, Landed
};

// The flying rocket: the whole stack until stage separation, then the second
// stage. It uses the whole rocket's frame (origin at the first stage's engine
// nozzle), so the second stage keeps its place when it separates.
struct StageMotion
{
    RocketState state = RocketState::OnPad;
    Vec3 pos;                 // origin of the rocket's frame
    Vec3 velocity;
    float tiltX = 0;          // lean of the rocket axis towards +X (degrees)
    float tiltZ = 0;          // lean of the rocket axis towards +Z (degrees)
    float flame = 0;          // engine thrust 0..1
    float legs = 0;           // landing legs: 0 = stowed, 1 = deployed
    float timer = 0;          // time in the current state
    Vec3 target;              // where `pos` must end up after landing
    float surfaceY = 0;       // height of the surface it lands on (for the dust)
    const char* site = "";    // name of the landing site, for the radio call
    bool entryBurnDone = false;
};

// A spent part (side booster or first stage) falling away after separation.
struct Debris
{
    enum Kind { Booster, FirstStage } kind;
    Mat4 start;               // world transform at the moment of separation
    Vec3 offset, velocity;    // movement since separation
    float angle = 0, spinRate = 0, pivotY = 0;   // slow tumble about its middle
};

struct RocketMotion
{
    StageMotion core;             // whole rocket until stage separation; core.state is the mission state
    StageMotion upper;            // second stage, flying on its own after stage separation
    bool boostersAttached = true;
    bool upperAttached = true;    // false once the stages have separated
    float boosterFlame = 0;       // boosters' engines while still attached
    float speed = 0;              // ascent speed along the rocket axis
    float armSwing = 0;           // service arms: 0 = attached, 1 = swung away
    int lastCountdown = 6;
};

// Smoke puff: a sphere that grows, drifts and then shrinks away.
struct Puff
{
    Vec3 pos, velocity;
    float age = 0, life = 1, size0 = 1, size1 = 1;
};

// ---- Simulation ----------------------------------------------------------------

class Simulation
{
public:
    void reset();
    void update(float dt);

    // User commands.
    void startAirplane();
    void startHelicopter();
    void launchRocket();

    // World matrices used to draw each vehicle.
    Mat4 airplaneMatrix() const;
    Mat4 helicopterMatrix() const;
    Mat4 rocketMatrix() const;               // whole rocket (before stage separation)
    Mat4 upperStageMatrix() const;           // second stage (after stage separation)
    Mat4 debrisMatrix(const Debris& d) const;

    // Point the follow camera should look at:
    // 0 = airplane, 1 = heli, 2 = rocket (the second stage once separated).
    Vec3 focusPoint(int vehicle) const;
    std::string statusText() const;

    AirplaneMotion plane;
    HelicopterMotion heli;
    RocketMotion rocket;
    std::vector<Debris> debris;
    std::vector<Puff> puffs;
    float time = 0;

private:
    void updateAirplane(float dt);
    void updateHelicopter(float dt);
    void updateRocket(float dt);
    void updateReturningStage(StageMotion& s, float dt, const char* name);
    void updateEffects(float dt);
    void emitPuff(Vec3 pos, Vec3 velocity, float life, float size0, float size1);
    void emitTrail(int emitter, const Vec3& exit, float size);

    float smokeTimer = 0;
    // Where the last trail puff was placed: whole rocket, second stage.
    Vec3 lastTrail[2];
    bool trailStarted[2] = {false, false};
};

const char* toString(PlaneState s);
const char* toString(HeliState s);
const char* toString(RocketState s);
