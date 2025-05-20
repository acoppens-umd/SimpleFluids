#include "geometry.h"
#include "load_obj.h"
#include <algorithm>

std::function<glm::vec3(Polyhedron*)> Polyhedron::external_force = [](Polyhedron* ptr) { return glm::vec3(0.0f); };

float rand_float() {
	return 2.0 * ((double)rand() / (RAND_MAX)) - 1.0;
}

glm::vec3 unit_surface_point(float phi, float theta) {
	return glm::vec3(glm::cos(phi) * glm::sin(theta), glm::sin(phi), glm::cos(phi) * glm::cos(theta));
}


glm::vec3 random_unit_surface_point() {
	float theta = ((double)rand() / (RAND_MAX)) * 2.0 * glm::pi<float>();
	float pre_phi = rand_float();
	float phi = glm::asin(pre_phi);
	return unit_surface_point(phi, theta);
}

Polyhedron::Polyhedron() :
	num_triangles(0),
	displacement(glm::vec3(0.0f)), rotation(glm::mat3(1.0f)),
	velocity(glm::vec3(0.0f)), rotation_rate(0.0f), rotation_axis(glm::vec3(1.0f)), 
	color(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)), mass(0), radius(0), active(false), force_balance(glm::vec3(0.0f)) {
}

void Polyhedron::time_step(double time_step) {
	glm::vec3 acceleration = this->force_balance / this->mass;

	this->displacement += this->velocity * (float)time_step + (0.5f) * acceleration * (float)glm::pow((float)time_step, 2);
	this->velocity += acceleration * (float)time_step;

	this->force_balance = Polyhedron::external_force(this);

	this->rotation = glm::rotate(glm::mat4(this->rotation), (float)(this->rotation_rate * time_step), this->rotation_axis);
}

glm::vec3 Polyhedron::support_function(glm::vec3 direction, int start_vertex_index) {
	int found_index = 0;
	return this->support_function(direction, found_index, start_vertex_index);
}

void Polyhedron::displace(glm::vec3 offset) {
	this->displacement += offset;
}

Sphere::Sphere(float radius, float density) :
	Polyhedron(), 
	vertices(std::vector<glm::vec3>()), triangles(std::vector<unsigned int>()) {

	this->radius = radius;
	this->mass = (4/3) * glm::pi<float>() * glm::pow(radius, 3) * density;

	this->initialize_geometry(6, 8);

	this->num_triangles = this->triangles.size() / 3;
}

glm::vec3 Sphere::support_function(glm::vec3 direction, int& found_index, int start_vertex_index) {
	return direction * this->radius + this->displacement;
}

void Sphere::initialize_geometry(unsigned int n_lat, unsigned int n_long) {
	float pi = glm::pi<float>();
	this->vertices.push_back(glm::vec3(0, -this->radius, 0)); //North pole
	this->vertices.push_back(glm::vec3(0, this->radius, 0)); //South pole

	for (int i = 0; i < n_lat; i++) {
		if (i < n_lat - 1) {
			float phi = pi * ((float)(i + 1) / n_lat) - pi / 2.0f;

			for (int k = 0; k < n_long; k++) {
				float theta = 2 * pi * ((float)k / n_long);

				this->vertices.push_back(this->radius * unit_surface_point(phi, theta));
			}
		}

		for (int k = 0; k < n_long; k++) {
			if (i == 0) {
				this->triangles.push_back(0);
				this->triangles.push_back(this->vertices.size() - n_long + (k % n_long));
				this->triangles.push_back(this->vertices.size() - n_long + ((k + 1) % n_long));
			}
			else if (i == n_lat - 1) {
				this->triangles.push_back(1);
				this->triangles.push_back(this->vertices.size() - n_long + (k % n_long));
				this->triangles.push_back(this->vertices.size() - n_long + ((k + 1) % n_long));
			}
			else {
				this->triangles.push_back(this->vertices.size() - 2 * n_long + (k % n_long));
				this->triangles.push_back(this->vertices.size() - 2 * n_long + ((k + 1) % n_long));
				this->triangles.push_back(this->vertices.size() - n_long + (k % n_long));
				this->triangles.push_back(this->vertices.size() - 2 * n_long + ((k + 1) % n_long));
				this->triangles.push_back(this->vertices.size() - n_long + ((k + 1) % n_long));
				this->triangles.push_back(this->vertices.size() - n_long + (k % n_long));
			}
		}
	}
}

void Sphere::insert_vertices(std::vector<float>& vertices_flat) {
	for (int i = 0; i < this->vertices.size(); i++) {
		vertices_flat.push_back(this->vertices[i][0]);
		vertices_flat.push_back(this->vertices[i][1]);
		vertices_flat.push_back(this->vertices[i][2]);
	}
}

void Sphere::insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) {
	for (int i = 0; i < this->triangles.size(); i++) {
		indices_flat.push_back(triangles[i] + polyhedron_offset);
	}

	polyhedron_offset += this->vertices.size();
}

void ConvexPolyhedron::geometry_init(float density, bool recenter) {
	count_triangles();
	produce_adjacency_list();

	produce_internal_tetrahedrons();

	calculate_volume();
	calculate_center_of_mass();

	if (recenter)
		this->recenter();

	for (glm::vec3& vert : this->vertices) {
		float R = glm::length(vert);
		if (R > radius) {
			this->radius = R;
		}
	}

	this->mass = this->volume * density;
}

ConvexPolyhedron::ConvexPolyhedron(float density, glm::vec3* vertices, unsigned int num_vertices, 
								   ConvexPolyhedron::ConvexPolyhedronFace* faces, unsigned int num_faces, bool recenter) :
	Polyhedron(),
	vertices(std::vector<glm::vec3>()), num_vertices(num_vertices), faces(faces), num_faces(num_faces),
	adjacency_list(std::vector<std::vector<unsigned int>>()),
	internal_tetrahedrons(std::vector<InternalTetrahedron>()), volume(0.0) {
	for (int i = 0; i < num_vertices; i++) {
		this->vertices.push_back(vertices[i]);
	}

	this->geometry_init(density, recenter);
}

ConvexPolyhedron::ConvexPolyhedron(float density, std::string& object_file, bool recenter) :
	Polyhedron(), 
	vertices(std::vector<glm::vec3>()), num_vertices(0), faces(nullptr), num_faces(0),
	adjacency_list(std::vector<std::vector<unsigned int>>()),
	internal_tetrahedrons(std::vector<InternalTetrahedron>()), volume(0.0) {
	std::vector<glm::ivec3> file_faces = std::vector<glm::ivec3>();

	loadOBJ(object_file, this->vertices, file_faces);

	this->num_vertices = this->vertices.size();

	this->faces = new ConvexPolyhedronFace[file_faces.size()];
	this->num_faces = file_faces.size();

	for (int i = 0; i < file_faces.size(); i++) {
		glm::ivec3 index_vector = file_faces[i];
		unsigned int index_vector_array[3] = {index_vector[0], index_vector[1], index_vector[2]};
		this->faces[i] = ConvexPolyhedronFace(index_vector_array, 3);
	}

	this->geometry_init(density, recenter);
}

//Hill climbing approach
glm::vec3 ConvexPolyhedron::support_function(glm::vec3 direction, int& found_index, int start_vertex_index) {
	if (start_vertex_index >= this->num_vertices) {
		start_vertex_index = 0;
	}

	direction = glm::inverse(this->rotation) * direction;

	int max_vertex_index = start_vertex_index;
	float max_projection = glm::dot(this->vertices[max_vertex_index], direction);

	bool consider_vertex = true;
	while (consider_vertex) {
		consider_vertex = false;

		for (unsigned int other_vertex_index : this->adjacency_list[start_vertex_index]) {
			float projection = glm::dot(this->vertices[other_vertex_index], direction);

			if (projection > max_projection) {
				consider_vertex = true;
				max_vertex_index = other_vertex_index;
				max_projection = projection;
			}
		}

		start_vertex_index = max_vertex_index;
	}

	found_index = max_vertex_index;
	return this->rotation * this->vertices[max_vertex_index] + this->displacement;
}

void ConvexPolyhedron::time_step(double time_step) {
	Polyhedron::time_step(time_step);
}

void ConvexPolyhedron::count_triangles(void) {
	for (int i = 0; i < num_faces; i++) {
		this->num_triangles += faces[i].num_triangles;
	}
}

void ConvexPolyhedron::produce_adjacency_list(void) {
	for (int i = 0; i < num_vertices; i++) {
		this->adjacency_list.push_back(std::vector<unsigned int>());
	}

	for (int i = 0; i < num_faces; i++) {
		ConvexPolyhedronFace& face = faces[i];

		unsigned int prev_index = face.vertex_indices[face.num_vertices - 1];

		for (int k = 0; k < face.num_vertices; k++) {
			unsigned int next_index = face.vertex_indices[k];

			if (std::find(this->adjacency_list[prev_index].begin(), this->adjacency_list[prev_index].end(), next_index) == this->adjacency_list[prev_index].end())
				this->adjacency_list[prev_index].push_back(next_index);

			if (std::find(this->adjacency_list[next_index].begin(), this->adjacency_list[next_index].end(), prev_index) == this->adjacency_list[next_index].end())
				this->adjacency_list[next_index].push_back(prev_index);

			prev_index = next_index;
		}
	}
}

void ConvexPolyhedron::produce_internal_tetrahedrons(void) {
	for (int i = 0; i < num_faces; i++) {
		ConvexPolyhedronFace& face = faces[i];

		glm::vec3 face_internal_point = (1 / 3.0f) * (this->vertices[face.vertex_indices[0]] +
			this->vertices[face.vertex_indices[1]] +
			this->vertices[face.vertex_indices[2]]);

		glm::vec3 leg = this->vertices[face.vertex_indices[0]] - face_internal_point;
		glm::vec3 edge = this->vertices[face.vertex_indices[1]] - this->vertices[face.vertex_indices[0]];

		bool direction = glm::dot(glm::cross(leg, edge), face_internal_point) > 0;

		for (int k = 0; k < face.num_triangles; k++) {
			this->internal_tetrahedrons.push_back(InternalTetrahedron(
				this->vertices[face.vertex_indices[0]],
				this->vertices[face.vertex_indices[k + 1]],
				this->vertices[face.vertex_indices[k + 2]],
				direction));
		}
	}
}


void ConvexPolyhedron::calculate_volume(void) {
	for (InternalTetrahedron& tet : this->internal_tetrahedrons) {
		this->volume += tet.signed_volume;
	}
}

void ConvexPolyhedron::calculate_center_of_mass(void) {
	glm::vec3 summation = glm::vec3(0.0f);

	for (InternalTetrahedron& tet : this->internal_tetrahedrons) {
		summation += tet.signed_volume * tet.centroid;
	}

	this->centroid = summation / this->volume;
}

//Displace all vertices so that the center of mass of the polyhedron is the target vector
void ConvexPolyhedron::recenter(glm::vec3 target) {
	for (int i = 0; i < num_vertices; i++) {
		this->vertices[i] += (target - this->centroid);
	}

	this->centroid = target;
}

ConvexPolyhedron::InternalTetrahedron::InternalTetrahedron(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, bool direction) :
	v1(v1), v2(v2), v3(v3), centroid((v1 + v2 + v3) / 4.0f), signed_volume(0) {
	glm::vec3 cr = glm::cross(v1, v2);
	float dt = glm::dot(cr, v3);
	this->signed_volume = dt / (direction ? 6 : -6);
}

ConvexPolyhedron::ConvexPolyhedronFace::ConvexPolyhedronFace(unsigned int* vertex_indices, unsigned int num_vertices) :
	vertex_indices(std::vector<unsigned int>()), num_vertices(num_vertices),
	triangle_indices(std::vector<unsigned int>()), num_triangles(num_vertices - 2) {

	for (int i = 0; i < num_vertices; i++) {
		this->vertex_indices.push_back(vertex_indices[i]);
	}

	unsigned int source_index = this->vertex_indices[0];

	for (int i = 2; i < num_vertices; i++) {
		this->triangle_indices.push_back(source_index);
		this->triangle_indices.push_back(this->vertex_indices[i - 1]);
		this->triangle_indices.push_back(this->vertex_indices[i]);
	}
}

ConvexPolyhedron::ConvexPolyhedronFace::ConvexPolyhedronFace() :
	vertex_indices(std::vector<unsigned int>()), num_vertices(0),
	triangle_indices(std::vector<unsigned int>()), num_triangles(0) {
}

void ConvexPolyhedron::insert_vertices(std::vector<float>& vertices_flat) {
	for (int i = 0; i < this->num_vertices; i++) {
		vertices_flat.push_back(this->vertices[i][0]);
		vertices_flat.push_back(this->vertices[i][1]);
		vertices_flat.push_back(this->vertices[i][2]);
	}
}

void ConvexPolyhedron::insert_indices(std::vector<unsigned int>& indices_flat, int& polyhedron_offset) {
	for (int i = 0; i < this->num_faces; i++) {
		ConvexPolyhedron::ConvexPolyhedronFace& face = this->faces[i];

		for (unsigned int vertex_index : this->faces[i].triangle_indices) {
			indices_flat.push_back(vertex_index + polyhedron_offset);
		}
	}

	polyhedron_offset += this->num_vertices;
}
