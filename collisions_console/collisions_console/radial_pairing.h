#pragma once

#include "pair_maker.h"

class RadialPairing : public PairMaker {
private:
	class RadialPairingItem {
	public:
		ConvexPolyhedron* poly;
		int neg[3], pos[3];

		RadialPairingItem(ConvexPolyhedron* poly);
	};

public:
	std::vector<RadialPairingItem> polyhedra;
	glm::vec3 min;
	glm::vec3 max;

	RadialPairing(std::vector<ConvexPolyhedron*>& polyhedra, glm::vec3 min, glm::vec3 max);

	virtual void update();

	virtual void find_edge_striking_polys(std::vector<std::pair<ConvexPolyhedron*, glm::vec3>>& edge_striking);

	virtual void find_potential_colliding_pairs(std::vector<std::pair<ConvexPolyhedron*, ConvexPolyhedron*>>& pairs);
};