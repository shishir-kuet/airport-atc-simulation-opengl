// The two ways of looking at the simulation:
//   * an orbit camera outside, circling a target point (free or following a vehicle);
//   * a cockpit camera sitting inside a vehicle, carried by its model matrix.
#pragma once

#include "core/Math3D.h"

#include <algorithm>
#include <cmath>

struct CameraView
{
    const char* name;
    Vec3 target;
    float distance, yaw, pitch;
    int follow;   // vehicle to follow (0 plane, 1 heli, 2 rocket / its second stage) or -1
};

// Preset views selected with the number keys 0-6.
constexpr int VIEW_COUNT = 7;
extern const CameraView VIEWS[VIEW_COUNT];

struct OrbitCamera
{
    Vec3 target;
    float distance = 100.0f;
    float yaw = 0.0f;      // degrees around the Y axis
    float pitch = 30.0f;   // degrees above the horizon
    int follow = -1;

    Vec3 position() const
    {
        float cp = std::cos(radians(pitch));
        return target + Vec3(cp * std::sin(radians(yaw)),
                             std::sin(radians(pitch)),
                             cp * std::cos(radians(yaw))) * distance;
    }

    Mat4 view() const { return lookAt(position(), target, {0, 1, 0}); }

    void orbit(float dYaw, float dPitch)
    {
        yaw += dYaw;
        pitch = std::clamp(pitch + dPitch, -5.0f, 89.0f);
    }

    void zoom(float factor) { distance = std::clamp(distance * factor, 4.0f, 250.0f); }

    void apply(const CameraView& v)
    {
        target = v.target;
        distance = v.distance;
        yaw = v.yaw;
        pitch = v.pitch;
        follow = v.follow;
    }
};

// ---- Cockpit camera ----------------------------------------------------------

// Where the pilot's eye sits inside each vehicle (in the vehicle's own
// coordinates) and which way it looks. The camera moves, pitches and banks
// with the vehicle because it is transformed by the vehicle's model matrix.
struct CockpitSpec
{
    const char* name;
    Vec3 eye, forward, up;
    bool frame;   // draw a cockpit frame (windows + instrument panel)
};

extern const CockpitSpec COCKPITS[3];

// Model matrix of the vehicle the cockpit camera belongs to, and the view
// matrix looking out of its windows.
Mat4 cockpitVehicleMatrix();
Mat4 cockpitView();
