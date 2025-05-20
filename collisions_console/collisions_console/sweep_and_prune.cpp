#include "sweep_and_prune.h"
#include <unordered_set>
#include <unordered_map>

SweepAndPrune::SweepAndPrune(std::vector<Polyhedron*>& polyhedra, glm::vec3 min, glm::vec3 max) : 
	min(min), max(max), x_sweep(std::vector<SorterEntry*>()), y_sweep(std::vector<SorterEntry*>()), z_sweep(std::vector<SorterEntry*>()) {
	for (Polyhedron* poly : polyhedra) {
		this->x_sweep.push_back(new SorterEntry(poly, true, 0, 0.0f));
		this->x_sweep.push_back(new SorterEntry(poly, false, 0, 0.0f));
		this->y_sweep.push_back(new SorterEntry(poly, true, 0, 0.0f));
		this->y_sweep.push_back(new SorterEntry(poly, false, 0, 0.0f));
		this->z_sweep.push_back(new SorterEntry(poly, true, 0, 0.0f));
		this->z_sweep.push_back(new SorterEntry(poly, false, 0, 0.0f));
	}
}

void SweepAndPrune::update() {
	//Update AABBs
	for (int axis = 0; axis < 3; axis++) {
		glm::vec3 direction = glm::vec3(0.0f);
		direction[axis] = 1.0f;

		std::vector<SorterEntry*>& sweep = axis == 0 ? this->x_sweep : (axis == 1 ? this->y_sweep : this->z_sweep);

		for (SorterEntry* entry : sweep) {
			glm::vec3 extent = entry->poly->support_function((entry->start ? -1.0f : 1.0f) * direction, 
															 entry->last_vertex_index, entry->last_vertex_index);

			entry->value = extent[axis];
		}
	}

	//Sort AABB values using insertion sort
	//Insertion sort has good performance for almost sorted lists
	for (int axis = 0; axis < 3; axis++) {
		std::vector<SorterEntry*>& sweep = axis == 0 ? this->x_sweep : (axis == 1 ? this->y_sweep : this->z_sweep);

		for (int i = 0; i < sweep.size(); i++) {
			SorterEntry* entry = sweep[i];

			int j = i;
			while (j > 0 && sweep[j - 1]->value > entry->value) {
				sweep[j] = sweep[j - 1];
				j--;
			}

			sweep[j] = entry;
		}
	}
}

void SweepAndPrune::find_edge_striking_polys(std::vector<std::pair<Polyhedron*, glm::vec3>>& edge_striking) {
	edge_striking.clear();

	for (int axis = 0; axis < 3; axis++) {
		std::vector<SorterEntry*>& sweep = axis == 0 ? this->x_sweep : (axis == 1 ? this->y_sweep : this->z_sweep);

		for (int i = 0; i < sweep.size(); i++) {
			if (sweep[i]->value > this->min[axis]) {
				break;
			}

			glm::vec3 offset = glm::vec3(0.0f);
			offset[axis] = this->min[axis] - sweep[i]->value + 1e-4;

			edge_striking.push_back(std::pair<Polyhedron*, glm::vec3>(sweep[i]->poly, offset));
		}

		for (int i = sweep.size() - 1; i >= 0; i--) {
			if (sweep[i]->value < this->max[axis]) {
				break;
			}

			glm::vec3 offset = glm::vec3(0.0f);
			offset[axis] = this->max[axis] - sweep[i]->value - 1e-4;
			edge_striking.push_back(std::pair<Polyhedron*, glm::vec3>(sweep[i]->poly, offset));
		}
	}
}

void SweepAndPrune::find_potential_colliding_pairs(std::vector<std::pair<Polyhedron*, Polyhedron*>>& pairs) {
	pairs.clear();

	std::unordered_set<Polyhedron*> active_polys = std::unordered_set<Polyhedron*>();
	std::unordered_map<Polyhedron*, std::unordered_set<Polyhedron*>> potential_pairs_x =
		std::unordered_map<Polyhedron*, std::unordered_set<Polyhedron*>>();
	//X direction check
	for (SorterEntry* entry : this->x_sweep) {
		//Encounter polyhedron
		if (entry->start) {
			for (Polyhedron* other_poly : active_polys) {
				if (potential_pairs_x.count(other_poly) == 0) {
					potential_pairs_x[other_poly] = std::unordered_set<Polyhedron*>();
				}

				potential_pairs_x[other_poly].insert(entry->poly);
			}

			active_polys.insert(entry->poly);
		}
		//End of polyhedron
		else {
			active_polys.erase(entry->poly);
		}
	}

	std::unordered_map<Polyhedron*, std::unordered_set<Polyhedron*>> potential_pairs_y =
		std::unordered_map<Polyhedron*, std::unordered_set<Polyhedron*>>();
	//Y direction check
	for (SorterEntry* entry : this->y_sweep) {
		//Encounter polyhedron
		if (entry->start) {
			for (Polyhedron* other_poly : active_polys) {
				//Check if pair intersects in x direction first
				if ((potential_pairs_x.count(other_poly) == 0 || potential_pairs_x[other_poly].count(entry->poly) == 0) &&
					(potential_pairs_x.count(entry->poly) == 0 || potential_pairs_x[entry->poly].count(other_poly) == 0))
					continue;

				if (potential_pairs_y.count(other_poly) == 0) {
					potential_pairs_y[other_poly] = std::unordered_set<Polyhedron*>();
				}

				potential_pairs_y[other_poly].insert(entry->poly);
			}

			active_polys.insert(entry->poly);
		}
		//End of polyhedron
		else {
			active_polys.erase(entry->poly);
		}
	}

	//Z direction check
	for (SorterEntry* entry : this->z_sweep) {
		//Encounter polyhedron
		if (entry->start) {
			for (Polyhedron* other_poly : active_polys) {
				//Check if pair intersects in x and y directions first
				if ((potential_pairs_y.count(other_poly) == 0 || potential_pairs_y[other_poly].count(entry->poly) == 0) &&
					(potential_pairs_y.count(entry->poly) == 0 || potential_pairs_y[entry->poly].count(other_poly) == 0))
					continue;

				pairs.push_back(std::pair<Polyhedron*, Polyhedron*>(entry->poly, other_poly));
			}

			active_polys.insert(entry->poly);
		}
		//End of polyhedron
		else {
			active_polys.erase(entry->poly);
		}
	}
}

SweepAndPrune::~SweepAndPrune() {
	for (SorterEntry* entry : x_sweep) {
		delete entry;
	}
	for (SorterEntry* entry : y_sweep) {
		delete entry;
	}
	for (SorterEntry* entry : z_sweep) {
		delete entry;
	}
}

SweepAndPrune::SorterEntry::SorterEntry(Polyhedron* poly, bool start, int last_vertex_index, float value) :
	poly(poly), start(start), last_vertex_index(last_vertex_index), value(value) {
}