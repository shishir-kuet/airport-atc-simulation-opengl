// Shader program + a helper that draws one part of a model.
//
// Week 1: no colour and no lighting. Parts are drawn in a single neutral
// shade with dark edges (or as pure wireframe) so the 3D shape is readable.
#pragma once

#include "Math3D.h"
#include "Mesh.h"

class Renderer
{
public:
    bool init();
    void destroy();

    void setCamera(const Mat4& view, const Mat4& projection);

    // Draws a mesh with the given model matrix (solid + edges, or wireframe).
    // `shade` is a grey level (0 = black, 1 = white); edges are drawn darker.
    void drawPart(const Mesh& mesh, const Mat4& model, float shade = 0.80f) const;

    // Draws only the filled surface, without outline edges (flames, smoke).
    void drawSolid(const Mesh& mesh, const Mat4& model, float shade) const;

    // Draws a line mesh (e.g. the ground grid) in a plain grey.
    void drawLines(const Mesh& mesh, const Mat4& model) const;

    bool wireframe = false;

private:
    void setModel(const Mat4& model) const;
    void setShade(float value) const;

    GLuint program = 0;
    GLint locModel = -1, locView = -1, locProjection = -1, locShade = -1;
};
