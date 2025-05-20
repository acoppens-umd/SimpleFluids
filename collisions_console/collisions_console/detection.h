#pragma once

#include "geometry.h"
#include "concave_geometry.h"
#include <random>

struct detection {
	bool detected;
	glm::vec3 penetration;
};

typedef std::vector<glm::vec3> Simplex;

bool tetrahedron_contains_origin(Simplex& simplex);

bool triangle_contains_origin_projection(Simplex& simplex, float& distance);
bool triangle_contains_origin_projection(Simplex& simplex);

glm::vec3 displacement_to_triangle(Simplex& simplex);

bool nearest_simplex(Simplex& simplex, glm::vec3& normal);

detection intersects(Polyhedron* p, Polyhedron* q);

bool gjk(Polyhedron& p, Polyhedron& q, Simplex& simplex, glm::vec3 direction = glm::vec3(rand_float(), rand_float(), rand_float()));

glm::vec3 penetration_angle(Polyhedron& p, Polyhedron& q, Simplex& simplex);