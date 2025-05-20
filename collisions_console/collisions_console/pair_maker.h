#pragma once

#include "geometry.h"

class PairMaker {

public:
	virtual void update() = 0;

	virtual void find_edge_striking_polys(std::vector<std::pair<Polyhedron*, glm::vec3>>& edge_striking) = 0;

	virtual void find_potential_colliding_pairs(std::vector<std::pair<Polyhedron*, Polyhedron*>>& pairs) = 0;
};
