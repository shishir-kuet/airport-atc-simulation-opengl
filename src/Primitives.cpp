#include "Primitives.h"

void Primitives::create()
{
    cube        = uploadMesh(makeCube());
    sphere      = uploadMesh(makeSphere());
    cylinder    = uploadMesh(makeCylinder(1.0f));
    cone        = uploadMesh(makeCylinder(0.0f));
    frustum     = uploadMesh(makeCylinder(0.5f));
    frustumWide = uploadMesh(makeCylinder(0.8f));
    octagon     = uploadMesh(makeCylinder(1.0f, 8));
    ringThin    = uploadMesh(makeTube(0.9f));
    ringThick   = uploadMesh(makeTube(0.6f));
    wing        = uploadMesh(makeWing(0.35f, 1.0f));   // swept, tapered
    fin         = uploadMesh(makeWing(0.40f, 0.3f));   // short rocket fin
    smoke       = uploadMesh(makeSphere(12, 8));
}

void Primitives::destroy()
{
    for (Mesh* m : {&cube, &sphere, &cylinder, &cone, &frustum, &frustumWide,
                    &octagon, &ringThin, &ringThick, &wing, &fin, &smoke})
        m->destroy();
}
