// Small building blocks shared by the three vehicle models.
// Internal header: only the files in src/models/ include it.
#pragma once

#include "models/Models.h"

// Rotations that turn a Y-axis primitive (cylinder/cone) to lie along X.
inline Mat4 alongPlusX()  { return rotateZ(-90.0f); }   // local +Y -> world +X
inline Mat4 alongMinusX() { return rotateZ(90.0f); }    // local +Y -> world -X

// The paint scheme of the three vehicles. One name per material, so a part is
// recoloured here once and changes wherever it is drawn.
namespace Livery
{
    // Airliner: white body, grey wings, blue tail. Painted metal is glossy,
    // so the highlight runs along the fuselage as it turns.
    constexpr Material PLANE_BODY  {{0.93f, 0.94f, 0.96f}, 2.40f};
    constexpr Material PLANE_WING  {{0.70f, 0.73f, 0.78f}, 1.10f};
    constexpr Material PLANE_TAIL  {{0.10f, 0.29f, 0.60f}, 1.20f};
    constexpr Material NACELLE     {{0.82f, 0.83f, 0.86f}, 2.20f};
    constexpr Material EXHAUST     {{0.22f, 0.22f, 0.24f}, 0.70f};

    // Helicopter: rescue red with a white trim.
    constexpr Material HELI_BODY   {{0.74f, 0.16f, 0.12f}, 2.20f};
    constexpr Material HELI_TRIM   {{0.90f, 0.90f, 0.89f}, 1.10f};
    constexpr Material HELI_ENGINE {{0.45f, 0.47f, 0.50f}, 1.00f};
    constexpr Material ROTOR       {{0.17f, 0.18f, 0.20f}, 0.80f};

    // Rocket: white stages, dark interstage and engine bay.
    constexpr Material ROCKET_BODY {{0.90f, 0.90f, 0.92f}, 2.20f};
    constexpr Material INTERSTAGE  {{0.15f, 0.16f, 0.18f}, 0.50f};
    constexpr Material NOZZLE      {{0.34f, 0.35f, 0.38f}, 1.40f};
    constexpr Material FIN         {{0.26f, 0.27f, 0.30f}, 0.70f};

    // Shared by all three.
    constexpr Material METAL       {{0.60f, 0.62f, 0.66f}, 2.00f};   // struts, legs, skids
    constexpr Material TYRE        {{0.09f, 0.09f, 0.10f}, 0.30f};
    constexpr Color    FLAME       {1.00f, 0.63f, 0.18f};             // exhaust (drawn unlit)
}
