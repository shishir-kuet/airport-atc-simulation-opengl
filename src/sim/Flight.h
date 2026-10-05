// Flight-mechanics helpers shared by the airplane, the helicopter and the
// rocket: smoothing a value, angles, headings and simple guidance laws.
// Internal header: only the files in src/sim/ include it.
#pragma once

#include "core/Math3D.h"
#include "world/Environment.h"   // the runway centreline the localizer flies to

#include <algorithm>
#include <cmath>
#include <iostream>

// Moves `value` towards `target` by at most `step`.
inline float approach(float value, float target, float step)
{
    return value < target ? std::min(value + step, target) : std::max(value - step, target);
}

inline float wrapAngle(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Horizontal unit vector for a heading (0 = +X, 90 = -Z).
inline Vec3 forward(float heading)
{
    return {std::cos(radians(heading)), 0.0f, -std::sin(radians(heading))};
}

inline float headingTo(const Vec3& from, float x, float z)
{
    return std::atan2(-(z - from.z), x - from.x) * 180.0f / PI;
}

// Turns `heading` towards `desired` at no more than `rate` degrees per second.
inline float steer(float heading, float desired, float rate, float dt)
{
    float diff = wrapAngle(desired - heading);
    float step = rate * dt;
    return heading + std::clamp(diff, -step, step);
}

// Heading that flies a circle of `radius` around (cx, cz), turning left
// (centre kept on the left). Too far out -> turn in; too close -> turn out.
inline float orbitHeading(const Vec3& pos, float cx, float cz, float radius, float gain)
{
    float dx = cx - pos.x, dz = cz - pos.z;
    float dist = std::sqrt(dx * dx + dz * dz);
    float toCentre = headingTo(pos, cx, cz);
    return toCentre - 90.0f + std::clamp((dist - radius) * gain, -60.0f, 60.0f);
}

// Heading that brings an aircraft flying east onto the runway centreline
// (like an ILS localizer): the further off, the steeper the intercept.
inline float localizerHeading(const Vec3& pos, float gain, float limit)
{
    return std::clamp(gain * (pos.z - Layout::RUNWAY_Z), -limit, limit);
}

inline float distanceXZ(const Vec3& a, float x, float z)
{
    return std::sqrt((x - a.x) * (x - a.x) + (z - a.z) * (z - a.z));
}

inline float randomFloat()   // 0 .. 1
{
    static unsigned seed = 12345u;
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) / 16777216.0f;
}

inline void radio(const char* text)
{
    std::cout << text << std::endl;
}
