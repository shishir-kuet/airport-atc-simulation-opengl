// Everything the user can press or drag.

#include "app/Input.h"

#include "app/App.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <iostream>
#include <string>

void updateTitle(GLFWwindow* window)
{
    std::string view = app.cockpit ? COCKPITS[app.camera.follow].name : app.viewName;
    std::string title = "ATC Simulation | View: " + view +
                        (app.renderer.wireframe ? " (wireframe)" : "") +
                        " | Shading: " + shadingName(app.renderer.shading) +
                        (app.paused ? " | PAUSED" : "") +
                        (app.timeScale > 1 ? " | x" + std::to_string(app.timeScale) : "") +
                        " | " + app.sim.statusText();
    glfwSetWindowTitle(window, title.c_str());
}

void selectView(int index)
{
    app.camera.apply(VIEWS[index]);
    app.viewName = VIEWS[index].name;
    if (VIEWS[index].follow < 0)
        app.cockpit = false;   // fixed views have no cockpit
    app.headYaw = app.headPitch = 0.0f;   // new seat: look straight ahead
}

// C: jump into (or out of) the cockpit of the vehicle being followed.
static void toggleCockpit()
{
    if (app.camera.follow < 0)
        selectView(1);          // from a fixed view, start in the airplane
    app.cockpit = !app.cockpit;
    app.headYaw = app.headPitch = 0.0f;
}

// G / B / N: switch the shading model while everything keeps running, so the
// same scene can be compared side by side.
static void selectShading(Shading mode)
{
    app.renderer.shading = mode;
    std::cout << "Shading: " << shadingName(mode) << "\n";
}

// Moves the sun. The lighting is recomputed from its new direction, so the
// bright sides, the dark sides and the highlights all move with it.
static void moveSun(float dAzimuth, float dElevation)
{
    app.sunAzimuth += dAzimuth;
    while (app.sunAzimuth > 360.0f) app.sunAzimuth -= 360.0f;
    while (app.sunAzimuth < 0.0f)   app.sunAzimuth += 360.0f;
    // Keep it above the horizon: below it the whole airport goes dark.
    app.sunElevation = std::clamp(app.sunElevation + dElevation, 8.0f, 89.0f);
    app.light.direction = sunDirection(app.sunAzimuth, app.sunElevation);
}

// Moves the pilot's head (cockpit) or the orbit camera (all other views).
static void lookAround(float dYaw, float dPitch)
{
    if (app.cockpit)
    {
        // Turn the head all the way round (e.g. out of the side window to
        // see the airport below); + yaw = look left, + pitch = look up.
        app.headYaw += dYaw;
        while (app.headYaw > 180.0f)  app.headYaw -= 360.0f;
        while (app.headYaw < -180.0f) app.headYaw += 360.0f;
        app.headPitch = std::clamp(app.headPitch + dPitch, -85.0f, 85.0f);
    }
    else
    {
        app.camera.orbit(dYaw, dPitch);
    }
}

// ---- Input callbacks -------------------------------------------------------

static void keyCallback(GLFWwindow* window, int key, int, int action, int)
{
    // The sun keys also work as single taps (one step per press), not only
    // held down: handy when showing one lighting angle after another.
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        const float step = 10.0f;
        switch (key)
        {
        case GLFW_KEY_LEFT_BRACKET:  moveSun(-step, 0); return;
        case GLFW_KEY_RIGHT_BRACKET: moveSun(step, 0);  return;
        case GLFW_KEY_APOSTROPHE:    moveSun(0, step);  return;
        case GLFW_KEY_SEMICOLON:     moveSun(0, -step); return;
        default: break;
        }
    }

    if (action != GLFW_PRESS)
        return;

    if (key >= GLFW_KEY_0 && key <= GLFW_KEY_6)
    {
        // While flying by hand, 4 / 5 / 6 are flight controls, not views.
        bool flightKey = key == GLFW_KEY_4 || key == GLFW_KEY_5 || key == GLFW_KEY_6;
        if (!(flightKey && app.sim.plane.state == PlaneState::Manual))
            selectView(key - GLFW_KEY_0);
        return;
    }

    switch (key)
    {
    case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, true); break;
    case GLFW_KEY_F:      app.renderer.wireframe = !app.renderer.wireframe; break;
    case GLFW_KEY_C:      toggleCockpit(); break;

    // Lighting: the shading model is chosen with its own key.
    case GLFW_KEY_N:
    case GLFW_KEY_F1:     selectShading(Shading::Flat); break;
    case GLFW_KEY_G:
    case GLFW_KEY_F2:     selectShading(Shading::Gouraud); break;
    case GLFW_KEY_B:
    case GLFW_KEY_F3:     selectShading(Shading::Phong); break;

    case GLFW_KEY_V:      // cockpit: look straight ahead again, normal zoom
        app.headYaw = app.headPitch = 0.0f;
        app.cockpitFov = 60.0f;
        break;
    case GLFW_KEY_SPACE:  app.paused = !app.paused; break;
    case GLFW_KEY_EQUAL:
    case GLFW_KEY_KP_ADD:      app.timeScale = std::min(8, app.timeScale * 2); break;
    case GLFW_KEY_MINUS:
    case GLFW_KEY_KP_SUBTRACT: app.timeScale = std::max(1, app.timeScale / 2); break;
    case GLFW_KEY_P:      app.sim.startAirplane(); break;
    case GLFW_KEY_M:      // fly the airliner yourself (or hand it back to the autopilot)
        app.sim.toggleManualAirplane();
        if (app.sim.plane.state == PlaneState::Manual && app.camera.follow != 0)
            selectView(1);   // follow the aircraft you are flying
        break;
    case GLFW_KEY_U:      app.sim.togglePlaneGear(); break;
    case GLFW_KEY_H:      app.sim.startHelicopter(); break;
    case GLFW_KEY_L:      app.sim.launchRocket(); break;
    case GLFW_KEY_ENTER:
        app.sim.startAirplane();
        app.sim.startHelicopter();
        app.sim.launchRocket();
        break;
    case GLFW_KEY_R:
        std::cout << "\n--- Reset ---\n";
        app.sim.reset();
        break;
    default: break;
    }
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        app.dragging = (action == GLFW_PRESS);
        glfwGetCursorPos(window, &app.lastX, &app.lastY);
    }
}

static void cursorPosCallback(GLFWwindow* window, double x, double y)
{
    // Check the real button state: if the release happened while the window
    // was not focused, the release event is lost and the drag would get stuck.
    app.dragging = app.dragging && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS
                   && glfwGetWindowAttrib(window, GLFW_FOCUSED);
    if (app.dragging)
    {
        // In the cockpit, dragging down looks down (like a game camera).
        float dy = float(y - app.lastY) * 0.3f;
        lookAround(float(app.lastX - x) * 0.3f, app.cockpit ? -dy : dy);
    }
    app.lastX = x;
    app.lastY = y;
}

// Cockpit zoom: a narrower field of view zooms in, a wider one shows more of
// the ground (from the sky the whole airport fits in the view).
static void cockpitZoom(float factor)
{
    app.cockpitFov = std::clamp(app.cockpitFov * factor, 30.0f, 100.0f);
}

static void scrollCallback(GLFWwindow*, double, double yOffset)
{
    if (app.cockpit)
        cockpitZoom(yOffset > 0 ? 0.92f : 1.08f);
    else
        app.camera.zoom(yOffset > 0 ? 0.9f : 1.1f);
}

static void framebufferSizeCallback(GLFWwindow*, int width, int height)
{
    glViewport(0, 0, width, height);
}

// Continuous (held-key) camera controls.
void processHeldKeys(GLFWwindow* window, float dt)
{
    auto held = [&](int key) { return glfwGetKey(window, key) == GLFW_PRESS; };

    // Moving the sun: [ ] swing it around the airport, ; ' raise and lower it.
    const float sunSpeed = 45.0f * dt;
    if (held(GLFW_KEY_LEFT_BRACKET))  moveSun(-sunSpeed, 0);
    if (held(GLFW_KEY_RIGHT_BRACKET)) moveSun(sunSpeed, 0);
    if (held(GLFW_KEY_APOSTROPHE))    moveSun(0, sunSpeed);
    if (held(GLFW_KEY_SEMICOLON))     moveSun(0, -sunSpeed);

    // Manual flight: I / K throttle, 8 / 5 climb and descend, 4 / 6 bank (the
    // digits on the top row or the numpad), X brakes. The arrows and W / S stay
    // with the camera, so the view can be turned and zoomed while flying.
    const bool flying = app.sim.plane.state == PlaneState::Manual;
    auto either = [&](int top, int pad) { return held(top) || held(pad); };
    PlaneControls controls;
    if (flying)
    {
        controls.pitch    = (either(GLFW_KEY_8, GLFW_KEY_KP_8) ? 1.0f : 0.0f) - (either(GLFW_KEY_5, GLFW_KEY_KP_5) ? 1.0f : 0.0f);
        controls.roll     = (either(GLFW_KEY_6, GLFW_KEY_KP_6) ? 1.0f : 0.0f) - (either(GLFW_KEY_4, GLFW_KEY_KP_4) ? 1.0f : 0.0f);
        controls.throttle = (held(GLFW_KEY_I) ? 1.0f : 0.0f) - (held(GLFW_KEY_K) ? 1.0f : 0.0f);
        controls.brake    = held(GLFW_KEY_X);
    }
    app.sim.planeInput = controls;

    // In the cockpit the arrows turn the pilot's head: left arrow looks left.
    const float speed = (app.cockpit ? 90.0f : 70.0f) * dt;
    const float left = app.cockpit ? speed : -speed;
    if (held(GLFW_KEY_LEFT))  lookAround(left, 0);
    if (held(GLFW_KEY_RIGHT)) lookAround(-left, 0);
    if (held(GLFW_KEY_UP))    lookAround(0, speed);
    if (held(GLFW_KEY_DOWN))  lookAround(0, -speed);

    bool zoomIn = held(GLFW_KEY_W);
    bool zoomOut = held(GLFW_KEY_S);
    if (app.cockpit)
    {
        if (zoomIn)  cockpitZoom(1.0f - 1.0f * dt);
        if (zoomOut) cockpitZoom(1.0f + 1.0f * dt);
    }
    else
    {
        if (zoomIn)  app.camera.zoom(1.0f - 1.5f * dt);
        if (zoomOut) app.camera.zoom(1.0f + 1.5f * dt);
    }
}

void printControls()
{
    std::cout << "\nControls\n"
              << "  P                       : airplane - taxi, take off, hold, land, park\n"
              << "  M                       : fly the airplane yourself / give it back to the autopilot\n"
              << "     manual flight:         I/K throttle, 8/5 climb/descend, 4/6 bank,\n"
              << "                            U gear, X brakes (camera: arrows, W/S, mouse)\n"
              << "  H                       : helicopter - take off, hold, land on the helipad\n"
              << "  L                       : rocket - countdown and launch\n"
              << "  Enter                   : start all three\n"
              << "  R                       : reset everything to the start\n"
              << "  0                       : overview of the whole airport\n"
              << "  1 / 2 / 3               : follow Airplane / Helicopter / Rocket (2nd stage)\n"
              << "  4 / 5 / 6               : Runway / ATC tower / Rocket base\n"
              << "  C                       : cockpit view of the followed vehicle (on / off)\n"
              << "  N / G / B               : shading - none (flat) / Gouraud / Phong  (also F1, F2, F3)\n"
              << "  [ / ]                   : swing the sun around the airport\n"
              << "  ; / '                   : lower / raise the sun\n"
              << "  Mouse drag / Arrow keys : orbit camera (cockpit: look around 360, e.g. down at the airport)\n"
              << "  Scroll / W, S           : zoom in / out (cockpit: field of view)\n"
              << "  V                       : cockpit: look straight ahead again\n"
              << "  F                       : toggle wireframe / solid\n"
              << "  Space                   : pause / resume\n"
              << "  + / -                   : simulation speed x1 / x2 / x4 / x8\n"
              << "  Esc                     : quit\n\n";
}

void installCallbacks(GLFWwindow* window)
{
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
}
