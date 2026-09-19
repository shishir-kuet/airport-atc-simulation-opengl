#include "Renderer.h"

#include <iostream>

static const char* vertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;   // reserved for lighting (week 2)

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

void main()
{
    gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);
}
)";

static const char* fragmentShaderSrc = R"(
#version 330 core
out vec4 FragColor;

uniform float uShade;   // single grey level, no colour yet

void main()
{
    FragColor = vec4(vec3(uShade), 1.0);
}
)";

static GLuint compileShader(GLenum type, const char* src)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::cout << "Shader compile error:\n" << log << "\n";
    }
    return shader;
}

bool Renderer::init()
{
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertexShaderSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSrc);

    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::cout << "Shader link error:\n" << log << "\n";
        return false;
    }

    locModel      = glGetUniformLocation(program, "uModel");
    locView       = glGetUniformLocation(program, "uView");
    locProjection = glGetUniformLocation(program, "uProjection");
    locShade      = glGetUniformLocation(program, "uShade");

    glUseProgram(program);
    return true;
}

void Renderer::destroy()
{
    glDeleteProgram(program);
    program = 0;
}

void Renderer::setCamera(const Mat4& view, const Mat4& projection)
{
    glUseProgram(program);
    glUniformMatrix4fv(locView, 1, GL_FALSE, view.data());
    glUniformMatrix4fv(locProjection, 1, GL_FALSE, projection.data());
}

void Renderer::setModel(const Mat4& model) const
{
    glUniformMatrix4fv(locModel, 1, GL_FALSE, model.data());
}

void Renderer::setShade(float value) const
{
    glUniform1f(locShade, value);
}

void Renderer::drawPart(const Mesh& mesh, const Mat4& model, float shade) const
{
    setModel(model);

    if (wireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        setShade(0.85f);
        mesh.draw();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        return;
    }

    // Pass 1: filled surface, pushed slightly back so the edges stay visible.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    setShade(shade);
    mesh.draw();
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Pass 2: darker outline edges on top so the shape reads without lighting.
    setShade(shade * 0.4f);
    mesh.drawEdges();
}

void Renderer::drawSolid(const Mesh& mesh, const Mat4& model, float shade) const
{
    setModel(model);
    if (wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    setShade(wireframe ? 0.85f : shade);
    mesh.draw();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::drawLines(const Mesh& mesh, const Mat4& model) const
{
    setModel(model);
    setShade(0.26f);
    mesh.draw();
}
