// Shader programs + the helpers that draw one part of a model.
//
// Three shading modes share one lighting formula and differ only in *where*
// it is evaluated:
//   Flat     - no lighting at all, the material colour on its own;
//   Gouraud  - the formula runs in the vertex shader, the result is
//              interpolated across the triangle;
//   Phong    - the normal is interpolated and the formula runs per fragment.
// The user switches between them while the program runs, so the difference
// is visible on the same scene.
#pragma once

#include "core/Math3D.h"
#include "core/Mesh.h"

enum class Shading
{
    Flat,
    Gouraud,
    Phong,
};

const char* shadingName(Shading mode);

// One directional light (the sun): parallel rays, so only its direction
// matters, not its position.
struct Light
{
    Vec3 direction{0.45f, 0.78f, 0.44f};   // unit vector pointing TOWARDS the sun
    float ambient   = 0.28f;   // light that reaches every surface
    float diffuse   = 0.85f;   // Lambert term, strongest facing the sun
    float specular  = 0.65f;   // highlight strength
    float shininess = 36.0f;   // highlight size: larger = smaller and sharper
};

// Direction towards the sun from its azimuth (around Y) and elevation.
Vec3 sunDirection(float azimuthDeg, float elevationDeg);

// What a surface is made of: its own colour and how glossy it is (how strong
// its highlight is compared with the light's specular setting). Grass barely
// reflects the sun, glass and polished metal reflect it sharply.
struct Material
{
    Color color{0.80f, 0.80f, 0.80f};
    float gloss = 1.0f;

    constexpr Material() = default;
    constexpr Material(const Color& color, float gloss = 1.0f) : color(color), gloss(gloss) {}

    // Darker or lighter variant of the same material.
    constexpr Material operator*(float s) const { return {color * s, gloss}; }
};

class Renderer
{
public:
    bool init();
    void destroy();

    void setCamera(const Mat4& view, const Mat4& projection);
    void setLight(const Light& light);

    // Draws a mesh with the given model matrix (solid + edges, or wireframe).
    // The material gives the surface its colour and gloss; the outline edges
    // are drawn in a darker version of the colour. Passing a single number
    // instead means that plain grey.
    void drawPart(const Mesh& mesh, const Mat4& model, const Material& material) const;
    void drawPart(const Mesh& mesh, const Mat4& model, float shade = 0.80f) const
    {
        drawPart(mesh, model, Material(grey(shade)));
    }

    // Draws only the filled surface, unlit (flames, smoke, instrument needles).
    void drawSolid(const Mesh& mesh, const Mat4& model, const Color& color) const;
    void drawSolid(const Mesh& mesh, const Mat4& model, float shade) const
    {
        drawSolid(mesh, model, grey(shade));
    }

    // Draws a line mesh (e.g. the ground grid) in a plain grey.
    void drawLines(const Mesh& mesh, const Mat4& model) const;

    bool wireframe = false;
    Shading shading = Shading::Phong;

private:
    // One compiled program and the uniform locations it uses. The lighting
    // uniforms stay at -1 in the flat program, which simply ignores them.
    struct Program
    {
        GLuint id = 0;
        GLint model = -1, view = -1, projection = -1, color = -1;
        GLint normalMat = -1, lightDir = -1, viewPos = -1;
        GLint ambient = -1, diffuse = -1, specular = -1, shininess = -1, gloss = -1;
    };

    bool build(Program& p, const char* vertexSrc, const char* fragmentSrc);
    const Program& current() const;   // program for the selected shading mode
    void begin(const Program& p, const Mat4& model) const;
    void setMaterial(const Program& p, const Material& m) const;

    Program flat, gouraud, phong;
    Vec3 eye;        // camera position in world space, for the specular term
    Light light;
};
