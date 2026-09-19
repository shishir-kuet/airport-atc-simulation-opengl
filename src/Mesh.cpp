#include "Mesh.h"
#include "Math3D.h"

// ---- GPU upload / draw -----------------------------------------------------

Mesh uploadMesh(const MeshData& data, GLenum mode)
{
    Mesh mesh;
    mesh.mode = mode;
    mesh.indexCount = static_cast<GLsizei>(data.indices.size());
    mesh.edgeCount = static_cast<GLsizei>(data.edges.size());

    // One element buffer holds the triangle indices followed by the edge indices.
    std::vector<unsigned> elements = data.indices;
    elements.insert(elements.end(), data.edges.begin(), data.edges.end());

    glGenVertexArrays(1, &mesh.vao);
    glGenBuffers(1, &mesh.vbo);
    glGenBuffers(1, &mesh.ebo);

    glBindVertexArray(mesh.vao);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, data.vertices.size() * sizeof(float),
                 data.vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, elements.size() * sizeof(unsigned),
                 elements.data(), GL_STATIC_DRAW);

    const GLsizei stride = 6 * sizeof(float);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    return mesh;
}

void Mesh::draw() const
{
    glBindVertexArray(vao);
    glDrawElements(mode, indexCount, GL_UNSIGNED_INT, nullptr);
}

void Mesh::drawEdges() const
{
    if (edgeCount == 0)
        return;
    glBindVertexArray(vao);
    glDrawElements(GL_LINES, edgeCount, GL_UNSIGNED_INT,
                   (void*)(static_cast<size_t>(indexCount) * sizeof(unsigned)));
}

void Mesh::destroy()
{
    glDeleteBuffers(1, &ebo);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    vao = vbo = ebo = 0;
}

// ---- Helpers ---------------------------------------------------------------

static unsigned addVertex(MeshData& d, const Vec3& p, const Vec3& n)
{
    unsigned index = static_cast<unsigned>(d.vertices.size() / 6);
    d.vertices.insert(d.vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
    return index;
}

// Adds a flat quad (a, b, c, d in order around the edge). The normal is made
// to point away from `center`, so the vertex order does not matter.
static void addQuad(MeshData& d, Vec3 a, Vec3 b, Vec3 c, Vec3 e, const Vec3& center)
{
    Vec3 n = normalize(cross(b - a, c - a));
    Vec3 mid = (a + b + c + e) * 0.25f;
    if (dot(n, mid - center) < 0.0f)
        n = n * -1.0f;

    unsigned i0 = addVertex(d, a, n);
    unsigned i1 = addVertex(d, b, n);
    unsigned i2 = addVertex(d, c, n);
    unsigned i3 = addVertex(d, e, n);
    d.indices.insert(d.indices.end(), {i0, i1, i2, i0, i2, i3});
    d.edges.insert(d.edges.end(), {i0, i1, i1, i2, i2, i3, i3, i0});
}

// Adds a slab from 8 corners: bottom face b[0..3] and top face t[0..3],
// both listed in the same order around the edge.
static void addSlab(MeshData& d, const Vec3 b[4], const Vec3 t[4])
{
    Vec3 center;
    for (int i = 0; i < 4; ++i)
        center = center + (b[i] + t[i]) * 0.125f;

    addQuad(d, b[0], b[1], b[2], b[3], center);   // bottom
    addQuad(d, t[0], t[1], t[2], t[3], center);   // top
    for (int i = 0; i < 4; ++i)                   // four sides
    {
        int j = (i + 1) % 4;
        addQuad(d, b[i], b[j], t[j], t[i], center);
    }
}

// ---- Primitives ------------------------------------------------------------

MeshData makeCube()
{
    MeshData d;
    const float h = 0.5f;
    Vec3 bottom[4] = {{-h, -h, -h}, {h, -h, -h}, {h, -h, h}, {-h, -h, h}};
    Vec3 top[4]    = {{-h,  h, -h}, {h,  h, -h}, {h,  h, h}, {-h,  h, h}};
    addSlab(d, bottom, top);
    return d;
}

MeshData makeSphere(int slices, int stacks)
{
    MeshData d;
    const float r = 0.5f;

    for (int i = 0; i <= stacks; ++i)
    {
        float phi = PI * i / stacks;                 // 0 (top) .. PI (bottom)
        for (int j = 0; j <= slices; ++j)
        {
            float theta = 2.0f * PI * j / slices;
            Vec3 n(std::sin(phi) * std::cos(theta),
                   std::cos(phi),
                   std::sin(phi) * std::sin(theta));
            addVertex(d, n * r, n);
        }
    }

    for (int i = 0; i < stacks; ++i)
        for (int j = 0; j < slices; ++j)
        {
            unsigned a = i * (slices + 1) + j;
            unsigned b = a + slices + 1;
            d.indices.insert(d.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
        }

    // Outline: a few meridians and parallels, like a globe.
    for (int i = 0; i < stacks; ++i)
        for (int j = 0; j < slices; j += 3)
        {
            unsigned a = i * (slices + 1) + j;
            d.edges.insert(d.edges.end(), {a, a + slices + 1});
        }
    for (int i = 2; i < stacks - 1; i += 2)
        for (int j = 0; j < slices; ++j)
        {
            unsigned a = i * (slices + 1) + j;
            d.edges.insert(d.edges.end(), {a, a + 1});
        }
    return d;
}

MeshData makeCylinder(float topRatio, int slices)
{
    MeshData d;
    const float rBottom = 0.5f;
    const float rTop = 0.5f * topRatio;
    const float h = 0.5f;

    // Side surface. For a cone the normal leans upward by (rBottom - rTop) / height.
    float slope = rBottom - rTop;
    unsigned sideStart = 0;
    for (int j = 0; j <= slices; ++j)
    {
        float theta = 2.0f * PI * j / slices;
        float c = std::cos(theta), s = std::sin(theta);
        Vec3 n = normalize(Vec3(c, slope, s));
        addVertex(d, Vec3(rBottom * c, -h, rBottom * s), n);
        addVertex(d, Vec3(rTop * c, h, rTop * s), n);
    }
    for (int j = 0; j < slices; ++j)
    {
        unsigned b0 = sideStart + 2 * j, t0 = b0 + 1;
        unsigned b1 = b0 + 2, t1 = b0 + 3;
        d.indices.insert(d.indices.end(), {b0, t0, b1, b1, t0, t1});

        // Outline: bottom and top circles, plus some lines along the side.
        d.edges.insert(d.edges.end(), {b0, b1});
        if (rTop > 0.0f)
            d.edges.insert(d.edges.end(), {t0, t1});
        if (slices < 16 || j % 3 == 0)
            d.edges.insert(d.edges.end(), {b0, t0});
    }

    // Caps (the top cap is skipped for a pointed cone).
    auto addCap = [&](float y, float r, float ny)
    {
        unsigned center = addVertex(d, Vec3(0, y, 0), Vec3(0, ny, 0));
        for (int j = 0; j <= slices; ++j)
        {
            float theta = 2.0f * PI * j / slices;
            addVertex(d, Vec3(r * std::cos(theta), y, r * std::sin(theta)), Vec3(0, ny, 0));
        }
        for (int j = 0; j < slices; ++j)
            d.indices.insert(d.indices.end(), {center, center + 1 + j, center + 2 + j});
    };
    addCap(-h, rBottom, -1.0f);
    if (rTop > 0.0f)
        addCap(h, rTop, 1.0f);

    return d;
}

MeshData makeTube(float innerRatio, int slices)
{
    MeshData d;
    const float ro = 0.5f;
    const float ri = 0.5f * innerRatio;
    const float h = 0.5f;

    // Each surface is a strip of quads between two circles.
    auto addStrip = [&](float r0, float y0, float r1, float y1, float nSign, bool radialNormal)
    {
        unsigned start = static_cast<unsigned>(d.vertices.size() / 6);
        for (int j = 0; j <= slices; ++j)
        {
            float theta = 2.0f * PI * j / slices;
            float c = std::cos(theta), s = std::sin(theta);
            Vec3 n = radialNormal ? Vec3(c, 0, s) * nSign : Vec3(0, nSign, 0);
            addVertex(d, Vec3(r0 * c, y0, r0 * s), n);
            addVertex(d, Vec3(r1 * c, y1, r1 * s), n);
        }
        for (int j = 0; j < slices; ++j)
        {
            unsigned a = start + 2 * j;
            d.indices.insert(d.indices.end(), {a, a + 1, a + 2, a + 2, a + 1, a + 3});
            if (radialNormal)   // outline: the circles at both ends of each wall
                d.edges.insert(d.edges.end(), {a, a + 2, a + 1, a + 3});
        }
    };

    addStrip(ro, -h, ro, h, 1.0f, true);     // outer wall
    addStrip(ri, -h, ri, h, -1.0f, true);    // inner wall
    addStrip(ri, h, ro, h, 1.0f, false);     // top face
    addStrip(ri, -h, ro, -h, -1.0f, false);  // bottom face
    return d;
}

MeshData makeWing(float tipRatio, float sweep)
{
    MeshData d;
    const float t = 0.5f;               // half thickness at the root
    const float tt = 0.5f * tipRatio;   // half thickness at the tip (tapers too)

    float tipFront = -sweep + tipRatio * 0.5f;
    float tipBack  = -sweep - tipRatio * 0.5f;

    // Corners ordered: root-front, root-back, tip-back, tip-front.
    Vec3 bottom[4] = {{0.5f, -t, 0}, {-0.5f, -t, 0}, {tipBack, -tt, 1}, {tipFront, -tt, 1}};
    Vec3 top[4]    = {{0.5f,  t, 0}, {-0.5f,  t, 0}, {tipBack,  tt, 1}, {tipFront,  tt, 1}};
    addSlab(d, bottom, top);
    return d;
}

MeshData makeGrid(float halfSize, float step)
{
    MeshData d;
    Vec3 up(0, 1, 0);
    for (float v = -halfSize; v <= halfSize + 0.001f; v += step)
    {
        unsigned a = addVertex(d, Vec3(v, 0, -halfSize), up);
        unsigned b = addVertex(d, Vec3(v, 0, halfSize), up);
        unsigned c = addVertex(d, Vec3(-halfSize, 0, v), up);
        unsigned e = addVertex(d, Vec3(halfSize, 0, v), up);
        d.indices.insert(d.indices.end(), {a, b, c, e});
    }
    return d;
}
