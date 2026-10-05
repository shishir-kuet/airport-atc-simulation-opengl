cmake_minimum_required(VERSION 3.20)

project(OpenGLProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenGL
find_package(OpenGL REQUIRED)

# GLFW
set(GLFW_INCLUDE_DIR
    "${CMAKE_SOURCE_DIR}/glfw/include"
)

set(GLFW_LIBRARY
    "${CMAKE_SOURCE_DIR}/glfw/lib-mingw-w64/libglfw3dll.a"
)

# Sources, grouped the same way the folders are:
#   core/   maths, mesh upload, shader + draw helpers, unit primitives
#   models/ the three vehicles
#   world/  the airport scenery
#   sim/    motion and state machines
#   app/    window, input, cameras, frame drawing
add_executable(OpenGLProject
    src/app/main.cpp
    src/app/Camera.cpp
    src/app/Input.cpp
    src/app/Scene.cpp

    src/core/Mesh.cpp
    src/core/Primitives.cpp
    src/core/Renderer.cpp
    src/core/gl.c

    src/models/Airplane.cpp
    src/models/Helicopter.cpp
    src/models/Rocket.cpp

    src/world/Airport.cpp
    src/world/Helipad.cpp
    src/world/RocketSite.cpp

    src/sim/Simulation.cpp
    src/sim/AirplaneSim.cpp
    src/sim/ManualFlight.cpp
    src/sim/HelicopterSim.cpp
    src/sim/RocketSim.cpp
    src/sim/Effects.cpp
)

# Header files ("core/Mesh.h", "sim/Tuning.h", ... are found from src/)
target_include_directories(OpenGLProject PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/include"
    "${GLFW_INCLUDE_DIR}"
)

# Link libraries
target_link_libraries(OpenGLProject PRIVATE
    "${GLFW_LIBRARY}"
    OpenGL::GL
)

# Copy GLFW DLL beside the executable
add_custom_command(TARGET OpenGLProject POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/glfw/lib-mingw-w64/glfw3.dll"
    "$<TARGET_FILE_DIR:OpenGLProject>"
)

