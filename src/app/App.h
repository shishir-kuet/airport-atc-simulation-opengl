// Everything the application remembers between frames: the camera, the
// renderer, the simulation and what the user has switched on.
#pragma once

#include "app/Camera.h"
#include "core/Renderer.h"
#include "sim/Simulation.h"

#include <string>

struct AppState
{
    OrbitCamera camera;
    Renderer renderer;
    Simulation sim;
    bool paused = false;
    int timeScale = 1;         // simulation speed: 1x, 2x, 4x or 8x
    std::string viewName = "Overview";

    bool cockpit = false;      // first-person view from the followed vehicle
    float headYaw = 0.0f;      // looking around inside the cockpit (degrees)
    float headPitch = 0.0f;
    float cockpitFov = 60.0f;  // cockpit field of view (scroll / W, S to change)

    // The sun, moved by the user with [ ] (around) and ; ' (up / down).
    Light light;
    float sunAzimuth = 40.0f;     // degrees around the Y axis
    float sunElevation = 52.0f;   // degrees above the horizon

    bool dragging = false;
    double lastX = 0, lastY = 0;
};

// The one instance, created in main.cpp.
extern AppState app;
