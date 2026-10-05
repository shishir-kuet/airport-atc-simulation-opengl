// The camera tables (preset views, cockpit seats) and the cockpit view matrix.

#include "app/Camera.h"

#include "app/App.h"
#include "world/Environment.h"

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

const CockpitSpec COCKPITS[] = {
    {"Airplane cockpit",      {4.6f, 0.35f, 0.0f},  {1, 0, 0},  {0, 1, 0}, true},
    {"Helicopter cockpit",    {1.6f, 0.25f, 0.0f},  {1, 0, 0},  {0, 1, 0}, true},
    // Rocket: an onboard camera on the side of the upper stage looking down
    // past the engine, like real launch footage.
    {"Rocket onboard camera", {0.0f, 10.8f, 1.25f}, {0, -1, 0}, {0, 0, 1}, false},
};

// Model matrix of the vehicle the cockpit camera belongs to.
Mat4 cockpitVehicleMatrix()
{
    switch (app.camera.follow)
    {
    case 0:  return app.sim.airplaneMatrix();
    case 1:  return app.sim.helicopterMatrix();
    default: return app.sim.rocket.upperAttached ? app.sim.rocketMatrix() : app.sim.upperStageMatrix();
    }
}

// View matrix from the pilot's eye: the eye point and the look direction are
// given in vehicle coordinates and transformed into the world with the
// vehicle's model matrix. Head yaw/pitch turn the look direction.
Mat4 cockpitView()
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
