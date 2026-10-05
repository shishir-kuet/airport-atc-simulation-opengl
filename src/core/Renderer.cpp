#include "core/Renderer.h"

#include <iostream>

// ---- Shader sources --------------------------------------------------------
//
// All three programs take the same uniforms, so the draw code does not care
// which one is bound. The lighting formula is the Phong reflection model:
//
//   colour = material * (ambient + diffuse * max(dot(N, L), 0))   <- body
//          +          specular * max(dot(V, R), 0) ^ shininess    <- highlight
//
// with N the surface normal, L the direction to the sun, V the direction to
// the eye and R = reflect(-L, N). The highlight is added as white light, not
// multiplied by the material, so it also shows up on dark surfaces.

// Unlit: the material colour straight out. Also used for the outline
// edges, the flames and the smoke, which must not be shaded.
static const char* flatVS = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

void main()
{
    gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);
}
)";

static const char* flatFS = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 uColor;

void main()
{
    FragColor = vec4(uColor, 1.0);
}
)";

// Gouraud: the lighting is computed once per vertex and the two resulting
// terms are interpolated across the triangle. Cheap, but a highlight that
// falls between two vertices is smeared out or lost completely.
static const char* gouraudVS = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

uniform vec3  uLightDir;
uniform vec3  uViewPos;
uniform float uAmbient;
uniform float uDiffuse;
uniform float uSpecular;
uniform float uShininess;
uniform float uGloss;

out vec2 vLight;   // x = body (ambient + diffuse), y = white highlight

void main()
{
    vec3 worldPos = vec3(uModel * vec4(aPos, 1.0));
    vec3 N = normalize(uNormalMatrix * aNormal);
    vec3 L = normalize(uLightDir);
    vec3 V = normalize(uViewPos - worldPos);
    vec3 R = reflect(-L, N);

    float diff = max(dot(N, L), 0.0);
    float spec = diff > 0.0 ? pow(max(dot(V, R), 0.0), uShininess) : 0.0;

    vLight = vec2(uAmbient + uDiffuse * diff, uSpecular * uGloss * spec);
    gl_Position = uProjection * uView * vec4(worldPos, 1.0);
}
)";

static const char* gouraudFS = R"(
#version 330 core
in vec2 vLight;
out vec4 FragColor;

uniform vec3 uColor;

void main()
{
    vec3 c = uColor * vLight.x + vec3(vLight.y);   // interpolated, already lit
    FragColor = vec4(clamp(c, 0.0, 1.0), 1.0);
}
)";

// Phong: the vertex shader only hands over the position and the normal; the
// same formula then runs for every pixel. Curved surfaces and highlights
// come out smooth, at the cost of much more work per frame.
static const char* phongVS = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

out vec3 vNormal;
out vec3 vWorldPos;

void main()
{
    vWorldPos = vec3(uModel * vec4(aPos, 1.0));
    vNormal = uNormalMatrix * aNormal;        // normalized per fragment
    gl_Position = uProjection * uView * vec4(vWorldPos, 1.0);
}
)";

static const char* phongFS = R"(
#version 330 core
in vec3 vNormal;
in vec3 vWorldPos;
out vec4 FragColor;

uniform vec3 uColor;
uniform vec3  uLightDir;
uniform vec3  uViewPos;
uniform float uAmbient;
uniform float uDiffuse;
uniform float uSpecular;
uniform float uShininess;
uniform float uGloss;

void main()
{
    vec3 N = normalize(vNormal);              // interpolation shortens it
    vec3 L = normalize(uLightDir);
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 R = reflect(-L, N);

    float diff = max(dot(N, L), 0.0);
    float spec = diff > 0.0 ? pow(max(dot(V, R), 0.0), uShininess) : 0.0;

    vec3 c = uColor * (uAmbient + uDiffuse * diff) + vec3(uSpecular * uGloss * spec);
    FragColor = vec4(clamp(c, 0.0, 1.0), 1.0);
}
)";

// ---- Compiling -------------------------------------------------------------

const char* shadingName(Shading mode)
{
    switch (mode)
    {
    case Shading::Flat:    return "Flat (no lighting)";
    case Shading::Gouraud: return "Gouraud (per vertex)";
    case Shading::Phong:   return "Phong (per pixel)";
    }
    return "";
}

Vec3 sunDirection(float azimuthDeg, float elevationDeg)
{
    float a = radians(azimuthDeg), e = radians(elevationDeg);
    return normalize(Vec3(std::cos(e) * std::cos(a), std::sin(e), std::cos(e) * std::sin(a)));
}

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

bool Renderer::build(Program& p, const char* vertexSrc, const char* fragmentSrc)
{
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);

    p.id = glCreateProgram();
    glAttachShader(p.id, vs);
    glAttachShader(p.id, fs);
    glLinkProgram(p.id);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(p.id, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[1024];
        glGetProgramInfoLog(p.id, sizeof(log), nullptr, log);
        std::cout << "Shader link error:\n" << log << "\n";
        return false;
    }

    // Uniforms a program does not use simply come back as -1 and are skipped.
    p.model      = glGetUniformLocation(p.id, "uModel");
    p.view       = glGetUniformLocation(p.id, "uView");
    p.projection = glGetUniformLocation(p.id, "uProjection");
    p.color      = glGetUniformLocation(p.id, "uColor");
    p.normalMat  = glGetUniformLocation(p.id, "uNormalMatrix");
    p.lightDir   = glGetUniformLocation(p.id, "uLightDir");
    p.viewPos    = glGetUniformLocation(p.id, "uViewPos");
    p.ambient    = glGetUniformLocation(p.id, "uAmbient");
    p.diffuse    = glGetUniformLocation(p.id, "uDiffuse");
    p.specular   = glGetUniformLocation(p.id, "uSpecular");
    p.shininess  = glGetUniformLocation(p.id, "uShininess");
    p.gloss      = glGetUniformLocation(p.id, "uGloss");
    return true;
}

bool Renderer::init()
{
    if (!build(flat, flatVS, flatFS))
        return false;
    if (!build(gouraud, gouraudVS, gouraudFS))
        return false;
    if (!build(phong, phongVS, phongFS))
        return false;

    setLight(light);
    return true;
}

void Renderer::destroy()
{
    for (Program* p : {&flat, &gouraud, &phong})
    {
        glDeleteProgram(p->id);
        p->id = 0;
    }
}

// ---- Per-frame state -------------------------------------------------------

void Renderer::setCamera(const Mat4& view, const Mat4& projection)
{
    // The eye position is read back out of the view matrix (which is
    // rotation * translate(-eye)), so the specular term knows where we look
    // from without the caller having to pass it in.
    eye = Vec3(-(view.m[0] * view.m[12] + view.m[1] * view.m[13] + view.m[2]  * view.m[14]),
               -(view.m[4] * view.m[12] + view.m[5] * view.m[13] + view.m[6]  * view.m[14]),
               -(view.m[8] * view.m[12] + view.m[9] * view.m[13] + view.m[10] * view.m[14]));

    for (const Program* p : {&flat, &gouraud, &phong})
    {
        glUseProgram(p->id);
        glUniformMatrix4fv(p->view, 1, GL_FALSE, view.data());
        glUniformMatrix4fv(p->projection, 1, GL_FALSE, projection.data());
        if (p->viewPos >= 0)
            glUniform3f(p->viewPos, eye.x, eye.y, eye.z);
    }
}

void Renderer::setLight(const Light& value)
{
    light = value;
    Vec3 dir = normalize(light.direction);

    for (const Program* p : {&gouraud, &phong})
    {
        glUseProgram(p->id);
        glUniform3f(p->lightDir, dir.x, dir.y, dir.z);
        glUniform1f(p->ambient, light.ambient);
        glUniform1f(p->diffuse, light.diffuse);
        glUniform1f(p->specular, light.specular);
        glUniform1f(p->shininess, light.shininess);
    }
}

// ---- Drawing ---------------------------------------------------------------

const Renderer::Program& Renderer::current() const
{
    switch (shading)
    {
    case Shading::Gouraud: return gouraud;
    case Shading::Phong:   return phong;
    default:               return flat;
    }
}

void Renderer::begin(const Program& p, const Mat4& model) const
{
    glUseProgram(p.id);
    glUniformMatrix4fv(p.model, 1, GL_FALSE, model.data());
    if (p.normalMat >= 0)
    {
        Mat3 n = normalMatrix(model);
        glUniformMatrix3fv(p.normalMat, 1, GL_FALSE, n.data());
    }
}

void Renderer::setMaterial(const Program& p, const Material& m) const
{
    glUniform3f(p.color, m.color.x, m.color.y, m.color.z);
    if (p.gloss >= 0)
        glUniform1f(p.gloss, m.gloss);
}

void Renderer::drawPart(const Mesh& mesh, const Mat4& model, const Material& material) const
{
    if (wireframe)
    {
        begin(flat, model);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        setMaterial(flat, grey(0.85f));
        mesh.draw();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        return;
    }

    // Pass 1: the lit surface, pushed slightly back so the edges stay visible.
    const Program& lit = current();
    begin(lit, model);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    setMaterial(lit, material);
    mesh.draw();
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Pass 2: unlit outline edges on top, so the shape still reads clearly
    // (and stays the same whichever shading model is selected).
    begin(flat, model);
    setMaterial(flat, material.color * 0.4f);
    mesh.drawEdges();
}

void Renderer::drawSolid(const Mesh& mesh, const Mat4& model, const Color& color) const
{
    // Flames, smoke and the instrument needles glow by themselves: they are
    // drawn unlit, so they do not go dark when the sun moves behind them.
    begin(flat, model);
    if (wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    setMaterial(flat, wireframe ? grey(0.85f) : color);
    mesh.draw();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::drawLines(const Mesh& mesh, const Mat4& model) const
{
    begin(flat, model);
    setMaterial(flat, grey(0.26f));
    mesh.draw();
}
