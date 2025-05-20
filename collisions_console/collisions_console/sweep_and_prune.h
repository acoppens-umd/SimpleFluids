#pragma once

#include "pair_maker.h"


class SweepAndPrune : public PairMaker {

	class SorterEntry {
	public:
		Polyhedron* poly;
		bool start;
		int last_vertex_index;
		float value;

		SorterEntry(Polyhedron* poly, bool start, int last_vertex_index, float value);
	};
public:
	glm::vec3 min;
	glm::vec3 max;
	
	SweepAndPrune(std::vector<Polyhedron*>& polyhedra, glm::vec3 min, glm::vec3 max);

	void update();

	void find_edge_striking_polys(std::vector<std::pair<Polyhedron*, glm::vec3>>& edge_striking);

	void find_potential_colliding_pairs(std::vector<std::pair<Polyhedron*, Polyhedron*>>& pairs);

	~SweepAndPrune();

private:
	std::vector<SorterEntry*> x_sweep;
	std::vector<SorterEntry*> y_sweep;
	std::vector<SorterEntry*> z_sweep;
};

