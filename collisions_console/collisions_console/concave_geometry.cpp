#include "concave_geometry.h"

ConcavePolyhedron::ConcavePolyhedron(float density, std::vector<std::string>& object_files) : 
	Polyhedron(),
	volume(0.0), centroid(glm::vec3(0.0)) {

	for (std::string &object_file : object_files) {
		ConvexPolyhedron * sub_poly = new ConvexPolyhedron(density, object_file, false);
		this->decomposition.push_back(sub_poly);
		this->mass += sub_poly->mass;
		this->volume += sub_poly->volume;
		this->num_triangles += sub_poly->num_triangles;
	}

	this->calculate_center_of_mass();
	this->recenter();

	this->align_subs();
}

ConcavePolyhedron::~ConcavePolyhedron() {
	for (ConvexPolyhedron* sub_poly : decomposition) {
		delete sub_poly;
	}
}

glm::vec3 ConcavePolyhedron::support_function(glm::vec3 direction, int& found_index, int start_vertex_index) {
	int max_found_index = 0;
	glm::vec3 max_vector = this->decomposition[0]->support_function(direction, max_found_index, start_vertex_index);
	float max_dot = glm::dot(max_vector, direction);

	for (int i = 1; i < this->decomposition.size(); i++) {
		int sub_found_index = 0;
		glm::vec3 sub_vector = this->decomposition[i]->support_function(direction, sub_found_index, start_vertex_index);
		float sub_dot = glm::dot(sub_vector, direction);

		if (max_dot < sub_dot) {
			max_found_index = sub_found_index;
			max_vector = sub_vector;
			max_dot = sub_dot;
		}
	}

	found_index = max_found_index;
	return max_vector;
}

void ConcavePolyhedron::time_step(double time_step) {
	Polyhedron::time_step(time_step);

	this->align_subs();
}

void ConcavePolyhedron::displace(glm::vec3 offset) {
	for (ConvexPolyhedron* sub_poly : decomposition) {
		sub_poly->displace(offset);
	}

	Polyhedron::displace(offset);
}

void ConcavePolyhedron::calculate_center_of_mass(void) {
	glm::vec3 summation = glm::vec3(0.0f);

	for (ConvexPolyhedron* sub_poly : decomposition) {
		summation += sub_poly->centroid * sub_poly->volume;
	}

	this->centroid = summation / this->volume;	
}

void ConcavePolyhedron::recenter(glm::vec3 target) {
	for (ConvexPolyhedron* sub_poly : decomposition) {
		sub_poly->recenter(sub_poly->centroid - this->centroid + target);
	}

	this->centroid = target;
}

void ConcavePolyhedron::align_subs(void) {
	for (ConvexPolyhedron* sub_poly : decomposition) {
		sub_poly->displacement = this->displacement;
		sub_poly->rotation = this->rotation;
	}
}

void ConcavePolyhedron::insert_vertices(std::vector<float>& vertices_flat) {
	for (ConvexPolyhedron* sub_poly : decomposition) {
		sub_poly->insert_vertices(vertices_flat);
	}
}

void ConcavePolyhedron::insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) {
	for (ConvexPolyhedron* sub_poly : decomposition) {
		sub_poly->insert_indices(indices_flat, polyhedron_offset);
	}
}