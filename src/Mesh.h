// Mesh storage (VAO/VBO/EBO) and generators for the basic primitive shapes.
// Every aircraft is built by transforming these unit-sized primitives.
//
// Vertex layout: position (location 0) + normal (location 1).
// Normals are not used yet; they are generated now so that lighting can be
// added next week without touching the geometry code.
#pragma once

#include <glad/gl.h>
#include <vector>

struct MeshData
{
    std::vector<float> vertices;      // x, y, z, nx, ny, nz per vertex
    std::vector<unsigned> indices;    // triangles (or lines for a line mesh)
    std::vector<unsigned> edges;      // outline edges as index pairs (no triangle diagonals)
};

struct Mesh
{
    GLuint vao = 0, vbo = 0, ebo = 0;
    GLsizei indexCount = 0;
    GLsizei edgeCount = 0;
    GLenum mode = GL_TRIANGLES;

    void draw() const;        // the surface
    void drawEdges() const;   // the outline edges only
    void destroy();
};

Mesh uploadMesh(const MeshData& data, GLenum mode = GL_TRIANGLES);

// ---- Primitive generators --------------------------------------------------
// All shapes fit inside a unit cube centred on the origin, so the scale
// applied in the model matrix equals the final size of the part.

// Box from -0.5 to 0.5 on every axis.
MeshData makeCube();

// UV sphere of diameter 1.
MeshData makeSphere(int slices = 24, int stacks = 16);

// Cylinder along the Y axis from y = -0.5 to y = 0.5, bottom diameter 1.
// topRatio scales the top radius: 1 = cylinder, 0 = cone, between = frustum.
MeshData makeCylinder(float topRatio = 1.0f, int slices = 24);

// Ring (thick tube) along the Y axis from y = -0.5 to 0.5, outer diameter 1.
// innerRatio = inner diameter / outer diameter.
MeshData makeTube(float innerRatio, int slices = 32);

// Tapered, swept slab used for wings, stabilizers and fins.
//   chord along X (leading edge at +X), thickness along Y, span along +Z (0..1).
//   tipRatio: tip chord / root chord.  sweep: how far the tip is moved
//   backwards (-X), measured in root chords.
MeshData makeWing(float tipRatio, float sweep);

// Flat grid of lines on the XZ plane (used as a ground reference).
MeshData makeGrid(float halfSize, float step);
