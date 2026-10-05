// Minimal 3D math: vectors and 4x4 matrices (column-major, OpenGL style).
// Provides the transformations used in the project: translate, rotate, scale,
// perspective projection and the lookAt view matrix.
#pragma once

#include <cmath>

constexpr float PI = 3.14159265358979f;

inline float radians(float deg) { return deg * PI / 180.0f; }

struct Vec3
{
    float x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    // Per-channel product: used to tint one colour with another.
    constexpr Vec3 operator*(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }
};

// A colour is just a Vec3 of red, green and blue in 0..1.
using Color = Vec3;

// Plain grey of the given brightness.
constexpr Color grey(float level) { return {level, level, level}; }

// Mixes two colours: t = 0 gives a, t = 1 gives b.
constexpr Color mix(const Color& a, const Color& b, float t) { return a * (1.0f - t) + b * t; }

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline Vec3 normalize(const Vec3& v)
{
    float len = std::sqrt(dot(v, v));
    return len > 0.0f ? v * (1.0f / len) : v;
}

// Column-major 4x4 matrix: element (row, col) is stored at m[col * 4 + row].
struct Mat4
{
    float m[16] = {0};

    static Mat4 identity()
    {
        Mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }

    Mat4 operator*(const Mat4& b) const
    {
        Mat4 r;
        for (int col = 0; col < 4; ++col)
            for (int row = 0; row < 4; ++row)
            {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                    sum += m[k * 4 + row] * b.m[col * 4 + k];
                r.m[col * 4 + row] = sum;
            }
        return r;
    }

    const float* data() const { return m; }
};

// 3x3 matrix (column-major), used for the normal matrix.
struct Mat3
{
    float m[9] = {0};
    const float* data() const { return m; }
};

// Normal matrix: the inverse transpose of the upper-left 3x3 of a model
// matrix. Normals must be transformed with it, not with the model matrix
// itself: a non-uniform scale (a thin box, a stretched cylinder) would
// otherwise tilt them away from the real surface direction and the lighting
// would be wrong. For a pure rotation it is the rotation itself.
inline Mat3 normalMatrix(const Mat4& a)
{
    // Upper-left 3x3 of the model matrix, written out as rows.
    const float u[3][3] = {
        {a.m[0], a.m[4], a.m[8] },
        {a.m[1], a.m[5], a.m[9] },
        {a.m[2], a.m[6], a.m[10]},
    };
    float det = u[0][0] * (u[1][1] * u[2][2] - u[1][2] * u[2][1])
              - u[0][1] * (u[1][0] * u[2][2] - u[1][2] * u[2][0])
              + u[0][2] * (u[1][0] * u[2][1] - u[1][1] * u[2][0]);

    Mat3 r;
    if (std::fabs(det) < 1e-12f)      // degenerate (zero scale): leave normals as they are
    {
        r.m[0] = r.m[4] = r.m[8] = 1.0f;
        return r;
    }

    // inverse(M)^T is the matrix of cofactors divided by the determinant.
    float invDet = 1.0f / det;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
        {
            int r1 = (row + 1) % 3, r2 = (row + 2) % 3;
            int c1 = (col + 1) % 3, c2 = (col + 2) % 3;
            r.m[col * 3 + row] = (u[r1][c1] * u[r2][c2] - u[r1][c2] * u[r2][c1]) * invDet;
        }
    return r;
}

// Applies a matrix to a point (w = 1).
inline Vec3 transformPoint(const Mat4& a, const Vec3& p)
{
    return {a.m[0] * p.x + a.m[4] * p.y + a.m[8]  * p.z + a.m[12],
            a.m[1] * p.x + a.m[5] * p.y + a.m[9]  * p.z + a.m[13],
            a.m[2] * p.x + a.m[6] * p.y + a.m[10] * p.z + a.m[14]};
}

// ---- Transformation matrices ----------------------------------------------

inline Mat4 translate(float x, float y, float z)
{
    Mat4 r = Mat4::identity();
    r.m[12] = x;
    r.m[13] = y;
    r.m[14] = z;
    return r;
}

inline Mat4 scale(float x, float y, float z)
{
    Mat4 r = Mat4::identity();
    r.m[0] = x;
    r.m[5] = y;
    r.m[10] = z;
    return r;
}

// Rotation of `deg` degrees around an arbitrary axis (Rodrigues' formula).
inline Mat4 rotate(float deg, Vec3 axis)
{
    axis = normalize(axis);
    float c = std::cos(radians(deg));
    float s = std::sin(radians(deg));
    float t = 1.0f - c;
    float x = axis.x, y = axis.y, z = axis.z;

    Mat4 r = Mat4::identity();
    r.m[0] = t * x * x + c;     r.m[4] = t * x * y - s * z; r.m[8]  = t * x * z + s * y;
    r.m[1] = t * x * y + s * z; r.m[5] = t * y * y + c;     r.m[9]  = t * y * z - s * x;
    r.m[2] = t * x * z - s * y; r.m[6] = t * y * z + s * x; r.m[10] = t * z * z + c;
    return r;
}

inline Mat4 rotateX(float deg) { return rotate(deg, {1, 0, 0}); }
inline Mat4 rotateY(float deg) { return rotate(deg, {0, 1, 0}); }
inline Mat4 rotateZ(float deg) { return rotate(deg, {0, 0, 1}); }

// ---- Camera matrices -------------------------------------------------------

inline Mat4 perspective(float fovyDeg, float aspect, float zNear, float zFar)
{
    float f = 1.0f / std::tan(radians(fovyDeg) / 2.0f);
    Mat4 r;
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (zFar + zNear) / (zNear - zFar);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
    return r;
}

inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up)
{
    Vec3 f = normalize(center - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);

    Mat4 r = Mat4::identity();
    r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
    r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -dot(s, eye);
    r.m[13] = -dot(u, eye);
    r.m[14] = dot(f, eye);
    return r;
}
