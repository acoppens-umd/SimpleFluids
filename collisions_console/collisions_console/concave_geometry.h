#pragma once

#include "geometry.h"

class ConcavePolyhedron : public Polyhedron {
public:
	float volume;
	glm::vec3 centroid;

	std::vector<ConvexPolyhedron*> decomposition;

	ConcavePolyhedron(float density, std::vector<std::string> &object_files);
	~ConcavePolyhedron();

	glm::vec3 support_function(glm::vec3 direction, int& found_index, int start_vertex_index = 0);

	void time_step(double time_step) override;

	void insert_vertices(std::vector<float>& vertices) override;
	void insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) override;

	void recenter(glm::vec3 target = glm::vec3(0));

	void displace(glm::vec3 offset) override;

private:
	void calculate_center_of_mass(void);
	

	void align_subs(void);
};
