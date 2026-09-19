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

# Create executable
add_executable(OpenGLProject
    src/main.cpp
    src/Mesh.cpp
    src/Renderer.cpp
    src/Models.cpp
    src/Primitives.cpp
    src/Environment.cpp
    src/Simulation.cpp
    src/gl.c
)

# Header files
target_include_directories(OpenGLProject PRIVATE
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