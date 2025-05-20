#pragma once

#include "geometry.h"

ConvexPolyhedron* generate_convex_polyhedron(float density, unsigned int num_seed_points, float scale = 1.0f, bool equidistant = false);

ConvexPolyhedron* generate_cube(float density, glm::vec3 min, glm::vec3 max);