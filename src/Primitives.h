// The shared set of unit primitive meshes. Every vehicle and every airport
// structure is built by transforming (translate / rotate / scale) these.
#pragma once

#include "Mesh.h"

struct Primitives
{
    Mesh cube, sphere, cylinder, cone, frustum, frustumWide;
    Mesh octagon;              // 8-sided prism (launch pad)
    Mesh ringThin, ringThick;  // tubes (helipad circle, launch mount)
    Mesh wing, fin;            // tapered slabs
    Mesh smoke;                // low-detail sphere for smoke puffs (drawn in large numbers)

    void create();
    void destroy();
};
