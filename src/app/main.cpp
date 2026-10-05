// Airport / ATC Simulation
// Airplane, Helicopter and Rocket with their takeoff (and, for the aircraft,
// holding and landing) sequences, on an airport with runway, helipad and launch pad.
// Colour and lighting: one sun, with flat, Gouraud or Phong shading chosen
// by the user while the simulation runs.
//
// This file only sets up the window and runs the frame loop:
//   input (app/Input.cpp) -> simulation step (sim/) -> camera (app/Camera.cpp)
//   -> drawing (app/Scene.cpp).

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <iostream>

#include "app/App.h"
#include "app/Input.h"
#include "app/Scene.h"

AppState app;

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

    installCallbacks(window);

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

    app.light.direction = sunDirection(app.sunAzimuth, app.sunElevation);
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

        glClearColor(0.49f, 0.66f, 0.85f, 1.0f);   // daytime sky
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // The cockpit uses a wider field of view, like a pilot's eyes.
        float fov = inCockpit ? app.cockpitFov : 45.0f;
        // The depth buffer's precision depends almost entirely on the near
        // plane, and it is worst far away, so the near plane is kept as far
        // out as the view allows: outside, nothing comes closer than the
        // minimum orbit distance; in the cockpit the panel is about a unit
        // from the eye, so there it has to stay close.
        float nearPlane = inCockpit ? 0.3f : 1.0f;
        Mat4 projection = perspective(fov, float(width) / float(height), nearPlane, 5000.0f);
        app.renderer.setLight(app.light);   // the sun can have been moved
        app.renderer.setCamera(inCockpit ? cockpitView() : app.camera.view(), projection);

        drawWorld(app.renderer, primitives, app.sim, grid, radarAngle);
        drawVehicles(app.renderer, primitives, app.sim, inCockpit ? app.camera.follow : -1);
        drawSmoke(app.renderer, primitives, app.sim);
        if (inCockpit)
            drawCockpitOverlay(app.renderer, primitives, app.sim, app.camera.follow);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    grid.destroy();
    primitives.destroy();
    app.renderer.destroy();
    glfwTerminate();
    return 0;
}
