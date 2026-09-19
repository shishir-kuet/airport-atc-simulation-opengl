// Airport / ATC Simulation
// Airplane, Helicopter and Rocket with their takeoff (and, for the aircraft,
// holding and landing) sequences, on an airport with runway, helipad and launch pad.
// No colour and no lighting yet: geometry, transformations and motion only.

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "Environment.h"
#include "Math3D.h"
#include "Mesh.h"
#include "Models.h"
#include "Primitives.h"
#include "Renderer.h"
#include "Simulation.h"

// ---- Orbit camera ----------------------------------------------------------

struct CameraView
{
    const char* name;
    Vec3 target;
    float distance, yaw, pitch;
    int follow;   // vehicle to follow (0 plane, 1 heli, 2 rocket) or -1 for a fixed view
};

// Preset views selected with the number keys 0-6.
const CameraView VIEWS[] = {
    {"Overview",   {15.0f, 0.0f, -3.0f},                    150.0f, 200.0f, 35.0f, -1},
    {"Airplane",   {},                                       30.0f, 200.0f, 20.0f,  0},
    {"Helicopter", {},                                       22.0f, 210.0f, 20.0f,  1},
    {"Rocket",     {},                                       45.0f, 240.0f, 12.0f,  2},
    {"Runway",     Layout::RUNWAY_CENTER,                    95.0f, 160.0f, 30.0f, -1},
    {"ATC Tower",  {Layout::TOWER_POS.x, 16.0f, Layout::TOWER_POS.z}, 34.0f, 200.0f, 12.0f, -1},
    {"Rocket base", {Layout::ROCKET_BASE_POS.x, 4.0f, Layout::ROCKET_BASE_POS.z}, 70.0f, 200.0f, 25.0f, -1},
};

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

const CockpitSpec COCKPITS[] = {
    {"Airplane cockpit",      {4.6f, 0.35f, 0.0f},  {1, 0, 0},  {0, 1, 0}, true},
    {"Helicopter cockpit",    {1.6f, 0.25f, 0.0f},  {1, 0, 0},  {0, 1, 0}, true},
    // Rocket: an onboard camera on the side of the upper stage looking down
    // past the engine, like real launch footage.
    {"Rocket onboard camera", {0.0f, 10.8f, 1.25f}, {0, -1, 0}, {0, 0, 1}, false},
};

// ---- Application state -----------------------------------------------------

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

    bool dragging = false;
    double lastX = 0, lastY = 0;
};

static AppState app;

static void updateTitle(GLFWwindow* window)
{
    std::string view = app.cockpit ? COCKPITS[app.camera.follow].name : app.viewName;
    std::string title = "ATC Simulation | View: " + view +
                        (app.renderer.wireframe ? " (wireframe)" : "") +
                        (app.paused ? " | PAUSED" : "") +
                        (app.timeScale > 1 ? " | x" + std::to_string(app.timeScale) : "") +
                        " | " + app.sim.statusText();
    glfwSetWindowTitle(window, title.c_str());
}

static void selectView(int index)
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

// Model matrix of the vehicle the cockpit camera belongs to.
static Mat4 cockpitVehicleMatrix()
{
    switch (app.camera.follow)
    {
    case 0:  return app.sim.airplaneMatrix();
    case 1:  return app.sim.helicopterMatrix();
    default: return app.sim.rocketMatrix();
    }
}

// View matrix from the pilot's eye: the eye point and the look direction are
// given in vehicle coordinates and transformed into the world with the
// vehicle's model matrix. Head yaw/pitch turn the look direction.
static Mat4 cockpitView()
{
    const CockpitSpec& c = COCKPITS[app.camera.follow];
    Mat4 m = cockpitVehicleMatrix();

    Vec3 right = cross(c.forward, c.up);
    float cy = std::cos(radians(app.headYaw)), sy = std::sin(radians(app.headYaw));
    float cp = std::cos(radians(app.headPitch)), sp = std::sin(radians(app.headPitch));
    Vec3 look = c.forward * (cp * cy) - right * (cp * sy) + c.up * sp;   // + yaw = look left

    Vec3 eye = transformPoint(m, c.eye);
    Vec3 target = transformPoint(m, c.eye + look);
    Vec3 up = transformPoint(m, c.eye + c.up) - eye;
    return lookAt(eye, target, up);
}

static void printControls()
{
    std::cout << "\nControls\n"
              << "  P                       : airplane - taxi, take off, hold, land, park\n"
              << "  H                       : helicopter - take off, hold, land on the helipad\n"
              << "  L                       : rocket - countdown and launch\n"
              << "  Enter                   : start all three\n"
              << "  R                       : reset everything to the start\n"
              << "  0                       : overview of the whole airport\n"
              << "  1 / 2 / 3               : follow Airplane / Helicopter / Rocket\n"
              << "  4 / 5 / 6               : Runway / ATC tower / Rocket base\n"
              << "  C                       : cockpit view of the followed vehicle (on / off)\n"
              << "  Mouse drag / Arrow keys : orbit camera (cockpit: look around 360, e.g. down at the airport)\n"
              << "  Scroll / W, S           : zoom in / out (cockpit: field of view)\n"
              << "  V                       : cockpit: look straight ahead again\n"
              << "  F                       : toggle wireframe / solid\n"
              << "  Space                   : pause / resume\n"
              << "  + / -                   : simulation speed x1 / x2 / x4 / x8\n"
              << "  Esc                     : quit\n\n";
}

// ---- Input callbacks -------------------------------------------------------

static void keyCallback(GLFWwindow* window, int key, int, int action, int)
{
    if (action != GLFW_PRESS)
        return;

    if (key >= GLFW_KEY_0 && key <= GLFW_KEY_6)
    {
        selectView(key - GLFW_KEY_0);
        return;
    }

    switch (key)
    {
    case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, true); break;
    case GLFW_KEY_F:      app.renderer.wireframe = !app.renderer.wireframe; break;
    case GLFW_KEY_C:      toggleCockpit(); break;
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
static void processHeldKeys(GLFWwindow* window, float dt)
{
    // In the cockpit the arrows turn the pilot's head: left arrow looks left.
    const float speed = (app.cockpit ? 90.0f : 70.0f) * dt;
    const float left = app.cockpit ? speed : -speed;
    if (glfwGetKey(window, GLFW_KEY_LEFT)  == GLFW_PRESS) lookAround(left, 0);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) lookAround(-left, 0);
    if (glfwGetKey(window, GLFW_KEY_UP)    == GLFW_PRESS) lookAround(0, speed);
    if (glfwGetKey(window, GLFW_KEY_DOWN)  == GLFW_PRESS) lookAround(0, -speed);

    bool zoomIn = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    bool zoomOut = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
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

// ---- Drawing -----------------------------------------------------------------

// cockpitVehicle: the vehicle we are sitting in (-1 = none). Its body (the
// fuselage or cabin around the camera) is left out so it doesn't block the
// view, but the wings, engines, tail, skids and rotor stay visible when the
// pilot looks around.
static void drawVehicles(const Renderer& r, const Primitives& p, const Simulation& sim, int cockpitVehicle)
{
    drawAirplane(r, p, sim.airplaneMatrix(), sim.plane.gear, cockpitVehicle == 0);
    drawHelicopter(r, p, sim.helicopterMatrix(), sim.heli.rotorAngle, cockpitVehicle == 1);

    const RocketMotion& k = sim.rocket;
    float flicker = 0.85f + 0.15f * std::sin(sim.time * 45.0f);   // flames flicker slightly

    RocketLook look;
    look.boosters = k.boostersAttached;
    look.mainFlame = k.core.flame * flicker;
    look.boosterFlame = k.boosterFlame * flicker;
    look.legs = k.core.legs;
    drawRocket(r, p, sim.rocketMatrix(), look);

    if (!k.boostersAttached)   // flying back / landed on their landing zones
        for (int i = 0; i < 2; ++i)
            drawRocketBooster(r, p, sim.boosterMatrix(i), k.boosters[i].flame * flicker, k.boosters[i].legs);

    // Two recovered rockets already standing on their legs at the rocket base;
    // the free pad in between is where the returning core lands.
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
    auto box = [&](Vec3 c, Vec3 s, float shade, float angleZ = 0.0f)
    {
        r.drawPart(p.cube, eyeFrame * translate(c.x, c.y, c.z) * rotateZ(angleZ) * scale(s.x, s.y, s.z), shade);
    };

    // Round gauge with a needle; `needle` is the needle angle (0 = up, + = clockwise).
    auto gauge = [&](float x, float y, float z, float needle)
    {
        Mat4 g = eyeFrame * translate(x, y, z) * rotateX(90.0f);   // face the pilot
        r.drawPart(p.cylinder, g * scale(0.19f, 0.01f, 0.19f), 0.12f);   // dial face
        r.drawPart(p.ringThin, g * scale(0.20f, 0.02f, 0.20f), 0.85f);   // bezel
        r.drawSolid(p.cube, eyeFrame * translate(x, y, z + 0.012f) * rotateZ(-needle)
                                * translate(0.0f, 0.04f, 0.0f) * scale(0.012f, 0.085f, 0.004f), 0.95f);
    };

    // Artificial horizon: the horizon bar tilts with the bank and moves with the pitch.
    auto attitude = [&](float x, float y, float z)
    {
        Mat4 g = eyeFrame * translate(x, y, z) * rotateX(90.0f);
        r.drawPart(p.cylinder, g * scale(0.19f, 0.01f, 0.19f), 0.12f);
        r.drawPart(p.ringThin, g * scale(0.20f, 0.02f, 0.20f), 0.85f);
        float offset = std::clamp(-ins.pitch * 0.004f, -0.07f, 0.07f);
        r.drawSolid(p.cube, eyeFrame * translate(x, y, z + 0.012f) * rotateZ(ins.roll)
                                * translate(0.0f, offset, 0.0f) * scale(0.17f, 0.008f, 0.004f), 0.95f);
        r.drawSolid(p.cube, eyeFrame * translate(x, y, z + 0.014f) * scale(0.06f, 0.006f, 0.004f), 0.6f);
    };

    float speedNeedle = -135.0f + 270.0f * std::clamp(ins.speed / ins.maxSpeed, 0.0f, 1.0f);
    float altNeedle = -135.0f + 270.0f * std::clamp(ins.altitude / ins.maxAltitude, 0.0f, 1.0f);
    float compass = -ins.heading;   // heading 0 (east) points the needle up

    const float post = 0.2f, panel = 0.28f;
    if (!helicopter)
    {
        // Airliner: roof, centre and side window posts, glareshield, wide panel.
        box({0.0f, 0.56f, -1.0f}, {2.6f, 0.12f, 0.2f}, post);
        box({0.0f, 0.10f, -1.0f}, {0.04f, 0.9f, 0.04f}, post);
        box({-0.98f, 0.0f, -1.0f}, {0.08f, 1.3f, 0.05f}, post, -15.0f);
        box({0.98f, 0.0f, -1.0f}, {0.08f, 1.3f, 0.05f}, post, 15.0f);
        box({0.0f, -0.30f, -0.95f}, {2.4f, 0.04f, 0.2f}, 0.16f);          // glareshield
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
        box({0.0f, -0.28f, -0.92f}, {0.95f, 0.03f, 0.15f}, 0.16f);
        box({0.0f, -0.43f, -0.95f}, {0.95f, 0.28f, 0.05f}, panel);
        gauge(-0.3f, -0.42f, -0.92f, speedNeedle);
        attitude(0.0f, -0.42f, -0.92f);
        gauge(0.3f, -0.42f, -0.92f, altNeedle);
    }
}

static void drawSmoke(const Renderer& r, const Primitives& p, const Simulation& sim)
{
    for (const Puff& puff : sim.puffs)
    {
        float t = puff.age / puff.life;
        float size = puff.size0 + (puff.size1 - puff.size0) * std::sqrt(t);
        if (t > 0.75f)
            size *= (1.0f - t) / 0.25f;   // shrink away at the end of its life
        r.drawSolid(p.smoke, translate(puff.pos.x, puff.pos.y, puff.pos.z) * scale(size, size, size),
                    0.85f - 0.2f * t);
    }
}

// ---- Main ------------------------------------------------------------------

int main()
{
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(1280, 720, "ATC Simulation", nullptr, nullptr);
    if (!window)
    {
        std::cout << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (gladLoadGL((GLADloadfunc)glfwGetProcAddress) == 0)
    {
        std::cout << "Failed to initialize GLAD\n";
        glfwTerminate();
        return -1;
    }
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << "\n";

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    if (!app.renderer.init())
    {
        glfwTerminate();
        return -1;
    }

    Primitives primitives;
    primitives.create();
    Mesh grid = uploadMesh(makeGrid(400.0f, 20.0f), GL_LINES);

    app.sim.reset();
    selectView(0);
    printControls();

    float radarAngle = 0.0f;
    float titleTimer = 0.0f;
    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window))
    {
        double now = glfwGetTime();
        float dt = float(now - lastTime);
        lastTime = now;

        processHeldKeys(window, dt);
        if (!app.paused)
        {
            for (int i = 0; i < app.timeScale; ++i)   // several steps when sped up
                app.sim.update(dt);
            radarAngle += 90.0f * dt;   // 1 revolution every 4 seconds
        }

        // Follow cameras keep the selected vehicle in the centre.
        if (app.camera.follow >= 0)
            app.camera.target = app.sim.focusPoint(app.camera.follow);
        const bool inCockpit = app.cockpit && app.camera.follow >= 0;

        titleTimer += dt;
        if (titleTimer > 0.2f)
        {
            titleTimer = 0.0f;
            updateTitle(window);
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        if (width == 0 || height == 0)    // minimised
        {
            glfwPollEvents();
            continue;
        }

        glClearColor(0.10f, 0.10f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // The cockpit uses a wider field of view, like a pilot's eyes.
        float fov = inCockpit ? app.cockpitFov : 45.0f;
        Mat4 projection = perspective(fov, float(width) / float(height), 0.5f, 5000.0f);
        app.renderer.setCamera(inCockpit ? cockpitView() : app.camera.view(), projection);

        // Environment
        drawGround(app.renderer, primitives);
        app.renderer.drawLines(grid, Mat4::identity());
        drawAirport(app.renderer, primitives, radarAngle);
        drawHelipad(app.renderer, primitives,
                    translate(Layout::HELIPAD_POS.x, Layout::HELIPAD_POS.y, Layout::HELIPAD_POS.z));
        drawLaunchPad(app.renderer, primitives,
                      translate(Layout::LAUNCH_PAD_POS.x, Layout::LAUNCH_PAD_POS.y, Layout::LAUNCH_PAD_POS.z),
                      app.sim.rocket.armSwing);
        drawRocketBase(app.renderer, primitives);
        for (int i = 0; i < 2; ++i)
            drawLandingZone(app.renderer, primitives,
                            translate(Layout::LANDING_ZONE[i].x, Layout::LANDING_ZONE[i].y, Layout::LANDING_ZONE[i].z), i + 1);

        // Vehicles and effects
        drawVehicles(app.renderer, primitives, app.sim, inCockpit ? app.camera.follow : -1);
        drawSmoke(app.renderer, primitives, app.sim);

        // Cockpit frame on top of the scene (depth cleared so the outside
        // world never pokes through it). It is attached to the aircraft.
        if (inCockpit && COCKPITS[app.camera.follow].frame)
        {
            bool heli = app.camera.follow == 1;
            const CockpitSpec& c = COCKPITS[app.camera.follow];
            // Eye coordinates -> vehicle: forward (+X) is eye -z, up stays up.
            Mat4 eyeFrame = cockpitVehicleMatrix() * translate(c.eye.x, c.eye.y, c.eye.z) * rotateY(-90.0f);

            Instruments ins;
            if (heli)
                ins = {app.sim.heli.speed, 20.0f, app.sim.heli.pos.y - Layout::HELIPAD_TOP - HELICOPTER_GROUND_OFFSET,
                       40.0f, app.sim.heli.heading, app.sim.heli.pitch, app.sim.heli.roll};
            else
                ins = {app.sim.plane.speed, 35.0f, app.sim.plane.pos.y - Layout::PAVEMENT_TOP - AIRPLANE_GROUND_OFFSET,
                       60.0f, app.sim.plane.heading, app.sim.plane.pitch, app.sim.plane.roll};

            glClear(GL_DEPTH_BUFFER_BIT);
            drawCockpitFrame(app.renderer, primitives, eyeFrame, heli, ins);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    grid.destroy();
    primitives.destroy();
    app.renderer.destroy();
    glfwTerminate();
    return 0;
}
