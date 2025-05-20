#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <random>
#include <functional>

float rand_float();

glm::vec3 unit_surface_point(float phi, float theta);

glm::vec3 random_unit_surface_point();

class Polyhedron {
public:
	unsigned int num_triangles;

	float mass;

	glm::vec3 displacement;
	glm::mat3 rotation;

	glm::vec3 velocity;
	float rotation_rate;
	glm::vec3 rotation_axis;

	glm::vec3 force_balance;

	glm::vec4 color;

	float radius;

	bool active;

	static std::function<glm::vec3(Polyhedron*)> external_force;

	Polyhedron();

	virtual glm::vec3 support_function(glm::vec3 direction, int& found_index, int start_vertex_index = 0) = 0;
	virtual glm::vec3 support_function(glm::vec3 direction, int start_vertex_index = 0);
	virtual void time_step(double time_step);
	virtual void insert_vertices(std::vector<float> & vertices_flat) = 0;
	virtual void insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) = 0;

	virtual void displace(glm::vec3 offset);
};


class Sphere : public Polyhedron {
public:
	Sphere(float radius, float density);

	glm::vec3 support_function(glm::vec3 direction, int& found_index, int start_vertex_index = 0) override;

	void insert_vertices(std::vector<float>& vertices) override;
	void insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) override;

private:

	std::vector<glm::vec3> vertices;
	std::vector<unsigned int> triangles;
	void initialize_geometry(unsigned int n_lat, unsigned int n_long);
};


class ConvexPolyhedron : public Polyhedron {
public:
	class ConvexPolyhedronFace {
	public:
		std::vector<unsigned int> vertex_indices;
		unsigned int num_vertices;
		std::vector<unsigned int> triangle_indices;
		unsigned int num_triangles;

		ConvexPolyhedronFace(unsigned int* vertex_indices, unsigned int num_vertices);
		ConvexPolyhedronFace();
	};

	class InternalTetrahedron {
	public:
		glm::vec3 v1, v2, v3, centroid;
		float signed_volume;

		InternalTetrahedron(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, bool direction);
	};

	std::vector<glm::vec3> vertices;
	unsigned int num_vertices;

	ConvexPolyhedronFace* faces;
	unsigned int num_faces;

	std::vector<std::vector<unsigned int>> adjacency_list;

	std::vector<InternalTetrahedron> internal_tetrahedrons;

	float volume;

	glm::vec3 centroid;

	ConvexPolyhedron(float density, glm::vec3* vertices, unsigned int num_vertices, ConvexPolyhedronFace* faces, unsigned int num_faces, bool recenter = true);
	ConvexPolyhedron(float density, std::string& object_file, bool recenter = true);

	glm::vec3 support_function(glm::vec3 direction, int& found_index, int start_vertex_index = 0);
	void time_step(double time_step) override;

	void insert_vertices(std::vector<float>& vertices_flat) override;
	void insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) override;

	void recenter(glm::vec3 target = glm::vec3(0));

private:
	void geometry_init(float density, bool recenter);

	void count_triangles(void);

	void produce_adjacency_list(void);

	void produce_internal_tetrahedrons(void);

	void calculate_volume(void);

	void calculate_center_of_mass(void);
};
