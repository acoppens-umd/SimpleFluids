#include "radial_pairing.h"

RadialPairing::RadialPairingItem::RadialPairingItem(ConvexPolyhedron* poly) : poly(poly) {
	neg[0] = neg[1] = neg[2] = 0;
	pos[0] = pos[1] = pos[2] = 0;
}

RadialPairing::RadialPairing(std::vector<ConvexPolyhedron*>& polyhedra, glm::vec3 min, glm::vec3 max) : 
	polyhedra(std::vector<RadialPairingItem>()), min(min), max(max) {
	for (ConvexPolyhedron* poly : polyhedra) {
		this->polyhedra.push_back(RadialPairingItem(poly));
	}
}

void RadialPairing::update() {
	//Do nothing
}

void RadialPairing::find_edge_striking_polys(std::vector<std::pair<ConvexPolyhedron*, glm::vec3>>& edge_striking) {
	edge_striking.clear();

	for (int axis = 0; axis < 3; axis++) {
		for (RadialPairingItem& rpi : this->polyhedra) {
			glm::vec3 direction = glm::vec3(0.0f);
			direction[axis] = -1.0f;

			glm::vec3 support_vector = rpi.poly->support_function(direction, rpi.neg[axis], rpi.neg[axis]);

			if (support_vector[axis] < this->min[axis]) {
				glm::vec3 offset = glm::vec3(0.0f);
				offset[axis] = this->min[axis] - support_vector[axis] + 1e-4;

				edge_striking.push_back(std::pair<ConvexPolyhedron*, glm::vec3>(rpi.poly, offset));
			}
		}

		for (RadialPairingItem& rpi : this->polyhedra) {
			glm::vec3 direction = glm::vec3(0.0f);
			direction[axis] = 1.0f;

			glm::vec3 support_vector = rpi.poly->support_function(direction, rpi.pos[axis], rpi.pos[axis]);

			if (support_vector[axis] > this->max[axis]) {
				glm::vec3 offset = glm::vec3(0.0f);
				offset[axis] = this->max[axis] - support_vector[axis] - 1e-4;

				edge_striking.push_back(std::pair<ConvexPolyhedron*, glm::vec3>(rpi.poly, offset));
			}
		}
	}
}

void RadialPairing::find_potential_colliding_pairs(std::vector<std::pair<ConvexPolyhedron*, ConvexPolyhedron*>>& pairs) {
	pairs.clear();

	for (int i = 0; i < this->polyhedra.size() - 1; i++) {
		for (int k = i + 1; k < this->polyhedra.size(); k++) {
			if (glm::length(this->polyhedra[i].poly->displacement - this->polyhedra[k].poly->displacement) < this->polyhedra[i].poly->radius + this->polyhedra[k].poly->radius)
				pairs.push_back(std::pair<ConvexPolyhedron*, ConvexPolyhedron*>(this->polyhedra[i].poly, this->polyhedra[k].poly));
		}
	}
}