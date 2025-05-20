#pragma once

#include "geometry.h"
#include "concave_geometry.h"

void resolve_fluid_collision(Sphere* p, Sphere* q, glm::vec3 penetration);

void resolve_collision(Polyhedron* p, Polyhedron* q, glm::vec3 penetration);