// What is drawn each frame, in the order it is drawn: the airport, the three
// vehicles, their smoke, and the cockpit frame on top when we sit inside one.
#pragma once

#include "core/Mesh.h"
#include "core/Primitives.h"
#include "core/Renderer.h"
#include "sim/Simulation.h"

// Ground, reference grid, runway and taxiways, terminal, tower, helipad,
// launch pad and the rocket base. radarAngle turns the radar on the tower.
void drawWorld(const Renderer& r, const Primitives& p, const Simulation& sim,
               const Mesh& grid, float radarAngle);

// Airplane, helicopter, the flying rocket (or its separated second stage),
// the spent parts falling away and the two recovered rockets at the base.
// cockpitVehicle: the vehicle we sit in (-1 = none); its body is left out.
void drawVehicles(const Renderer& r, const Primitives& p, const Simulation& sim, int cockpitVehicle);

void drawSmoke(const Renderer& r, const Primitives& p, const Simulation& sim);

// Cockpit frame and instrument panel of `vehicle`, drawn over the scene
// (unlit, so the gauges stay readable whichever shading model is selected).
void drawCockpitOverlay(Renderer& r, const Primitives& p, const Simulation& sim, int vehicle);
