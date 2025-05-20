#include "generation.h"
#include <unordered_map>

struct facet {
	unsigned int vertex_indices[3];
	glm::vec3 normal;

	facet(std::vector<glm::vec3>& polytope, glm::vec3 center, int i0, int i1, int i2) {
		this->vertex_indices[0] = i0;
		this->vertex_indices[1] = i1;
		this->vertex_indices[2] = i2;

		this->normal = glm::normalize(glm::cross(polytope[i1] - polytope[i0], polytope[i2] - polytope[i0]));
		float distance = glm::dot(normal, polytope[i0] - center);
		if (distance < 0) {
			normal = -normal;
		}
	}
};

void add_unique_facet_edge(std::vector<std::pair<int, int>>& edges, int i0, int i1) {
	auto find_reverse = std::find(edges.begin(), edges.end(), std::make_pair(i1, i0));

	if (find_reverse != edges.end()) {
		edges.erase(find_reverse);
	}
	else {
		edges.push_back(std::make_pair(i0, i1));
	}
}

bool facet_sees_point(std::vector<glm::vec3>& points, facet& f, int point_ptr) {
	return glm::dot(f.normal, points[point_ptr] - points[f.vertex_indices[0]]) > 0;
}

void find_extrema(std::vector<glm::vec3>& points, int& min_x, int& max_x, int& min_y, int& max_y, int& min_z, int& max_z) {
	min_x = 0; max_x = 0; min_y = 0; max_y = 0; min_z = 0; max_z = 0;

	for (int i = 1; i < points.size(); i++) {
		if (points[i].x < points[min_x].x)
			min_x = i;
		if (points[i].x > points[max_x].x)
			max_x = i;
		if (points[i].y < points[min_x].y)
			min_y = i;
		if (points[i].y > points[max_x].y)
			max_y = i;
		if (points[i].z < points[min_x].z)
			min_z = i;
		if (points[i].z > points[max_x].z)
			max_z = i;
	}
}

void produce_starting_tetrahedron(std::vector<glm::vec3>& points, int& min_x, int& max_x, int& min_y, int& max_y, int& min_z, int& max_z,
								  int& p0, int& p1, int& p2, int& p3) {
	p0 = min_x; p1 = max_x;

	float dny, dxy, dnz, dxz;

	dny = glm::length(glm::cross(points[p1] - points[p0], points[min_y] - points[p0]));
	dxy = glm::length(glm::cross(points[p1] - points[p0], points[max_y] - points[p0]));
	dnz = glm::length(glm::cross(points[p1] - points[p0], points[min_z] - points[p0]));
	dxz = glm::length(glm::cross(points[p1] - points[p0], points[max_z] - points[p0]));

	if (dny > dxy) {
		if (dny > dnz) {
			if (dny > dxz) {
				p2 = min_y;
			} else {
				p2 = max_z;
			}
		} else {
			if (dnz > dxz) {
				p2 = min_z;
			}
			else {
				p2 = max_z;
			}
		}
	} else {
		if (dxy > dnz) {
			if (dxy > dxz) {
				p2 = max_y;
			}
			else {
				p2 = max_z;
			}
		}
		else {
			if (dnz > dxz) {
				p2 = min_z;
			}
			else {
				p2 = max_z;
			}
		}
	}

	glm::vec3 normal = glm::cross(points[p2] - points[p0], points[p1] - points[p0]);

	dny = abs(glm::dot(normal, points[min_y] - points[p0]));
	dxy = abs(glm::dot(normal, points[max_y] - points[p0]));
	dnz = abs(glm::dot(normal, points[min_z] - points[p0]));
	dxz = abs(glm::dot(normal, points[max_z] - points[p0]));

	if (dny > dxy) {
		if (dny > dnz) {
			if (dny > dxz) {
				p3 = min_y;
			}
			else {
				p3 = max_z;
			}
		}
		else {
			if (dnz > dxz) {
				p3 = min_z;
			}
			else {
				p3 = max_z;
			}
		}
	}
	else {
		if (dxy > dnz) {
			if (dxy > dxz) {
				p3 = max_y;
			}
			else {
				p3 = max_z;
			}
		}
		else {
			if (dnz > dxz) {
				p3 = min_z;
			}
			else {
				p3 = max_z;
			}
		}
	}
}

//Use a convex hull to produce a convex polyhedron from a random point cloud
ConvexPolyhedron* generate_convex_polyhedron(float density, unsigned int num_seed_points, float scale, bool equidistant) {
	std::vector<glm::vec3> points = std::vector<glm::vec3>();

	for (int i = 0; i < num_seed_points; i++) {
		if (equidistant) {
			float phi = glm::acos(rand_float()) - glm::pi<float>() / 2.0f;
			float lambda = glm::pi<float>() * (rand_float() + 1);
			points.push_back(scale * glm::vec3(glm::cos(phi)*glm::cos(lambda), glm::cos(phi) * glm::sin(lambda), glm::sin(phi)));
		} else {
			points.push_back(glm::vec3(scale * rand_float(), scale * rand_float(), scale * rand_float()));
		}
	}

	int min_x, max_x, min_y, max_y, min_z, max_z;
	find_extrema(points, min_x, max_x, min_y, max_y, min_z, max_z);

	int p0, p1, p2, p3;
	produce_starting_tetrahedron(points, min_x, max_x, min_y, max_y, min_z, max_z, p0, p1, p2, p3);

	glm::vec3 inside_pt = (points[p0] + points[p1] + points[p2] + points[p3]) / 4.0f;

	std::vector<facet> facets = std::vector<facet>();
	facets.push_back(facet(points, inside_pt, p0, p1, p2 ));
	facets.push_back(facet(points, inside_pt, p0, p2, p3 ));
	facets.push_back(facet(points, inside_pt, p0, p3, p1 ));
	facets.push_back(facet(points, inside_pt, p1, p3, p2 ));

	for (int i = 0; i < num_seed_points; i++) {
		if (i == p0 || i == p1 || i == p2 || i == p3) {
			continue;
		}

		std::vector<std::pair<int, int>> edges;

		for (int f = 0; f < facets.size(); f++) {
			if (facet_sees_point(points, facets[f], i)) {
				add_unique_facet_edge(edges, facets[f].vertex_indices[0], facets[f].vertex_indices[1]);
				add_unique_facet_edge(edges, facets[f].vertex_indices[1], facets[f].vertex_indices[2]);
				add_unique_facet_edge(edges, facets[f].vertex_indices[2], facets[f].vertex_indices[0]);

				facets[f] = facets.back();
				facets.pop_back();

				f--;
			}
		}

		for (std::pair<int, int> edge : edges) {
			facets.push_back(facet(points, inside_pt, edge.first, edge.second, i));
		}
	}

	//Remove points that don't end up in the hull to prevent issues with support function hill climbing later
	std::unordered_map<int, int> old_index_to_new_index = std::unordered_map<int, int>();
	std::vector<glm::vec3> reduced_vertices = std::vector<glm::vec3>();
	int next_unused_index = 0;
	for (facet &f : facets) {
		for (int i = 0; i < 3; i++) {
			if (old_index_to_new_index.count(f.vertex_indices[i]) == 0) {
				old_index_to_new_index[f.vertex_indices[i]] = next_unused_index;
				reduced_vertices.push_back(points[f.vertex_indices[i]]);
				next_unused_index++;
			}

			f.vertex_indices[i] = old_index_to_new_index[f.vertex_indices[i]];
		}
	}
	
	//If we have a degenerate case, we need to start over.
	if (reduced_vertices.size() < 4) {
		return generate_convex_polyhedron(num_seed_points, scale, equidistant);
	}

	ConvexPolyhedron::ConvexPolyhedronFace* faces = new ConvexPolyhedron::ConvexPolyhedronFace[facets.size()];

	for (int i = 0; i < facets.size(); i++) {
		faces[i] = ConvexPolyhedron::ConvexPolyhedronFace(facets[i].vertex_indices, 3);
	}

	ConvexPolyhedron* poly = new ConvexPolyhedron(density, reduced_vertices.data(), reduced_vertices.size(), faces, facets.size(), !equidistant);

	return poly;
}

ConvexPolyhedron* generate_cube(float density, glm::vec3 min, glm::vec3 max) {
	std::vector<glm::vec3> cube_vertices;
	cube_vertices.push_back(glm::vec3(min.x, min.y, min.z));
	cube_vertices.push_back(glm::vec3(min.x, min.y, max.z));
	cube_vertices.push_back(glm::vec3(min.x, max.y, max.z));
	cube_vertices.push_back(glm::vec3(min.x, max.y, min.z));
	cube_vertices.push_back(glm::vec3(max.x, min.y, min.z));
	cube_vertices.push_back(glm::vec3(max.x, min.y, max.z));
	cube_vertices.push_back(glm::vec3(max.x, max.y, max.z));
	cube_vertices.push_back(glm::vec3(max.x, max.y, min.z));

	unsigned int cube_face_indices[] = { 0, 1, 2, 3,
										 4, 5, 6, 7,
										 0, 1, 5, 4,
										 3, 2, 6, 7,
										 0, 3, 7, 4,
										 1, 2, 6, 5 };

	ConvexPolyhedron::ConvexPolyhedronFace* cube_faces = new ConvexPolyhedron::ConvexPolyhedronFace[6];

	cube_faces[0] = ConvexPolyhedron::ConvexPolyhedronFace(cube_face_indices, 4);
	cube_faces[1] = ConvexPolyhedron::ConvexPolyhedronFace(&(cube_face_indices[4]), 4);
	cube_faces[2] = ConvexPolyhedron::ConvexPolyhedronFace(&(cube_face_indices[8]), 4);
	cube_faces[3] = ConvexPolyhedron::ConvexPolyhedronFace(&(cube_face_indices[12]), 4);
	cube_faces[4] = ConvexPolyhedron::ConvexPolyhedronFace(&(cube_face_indices[16]), 4);
	cube_faces[5] = ConvexPolyhedron::ConvexPolyhedronFace(&(cube_face_indices[20]), 4);

	return new ConvexPolyhedron(density, cube_vertices.data(), 8, cube_faces, 6);
}

/*
	std::vector<glm::vec3> octo_vertices;
	octo_vertices.push_back(glm::vec3(0, 0, .25));
	octo_vertices.push_back(glm::vec3(0, 0, -.25));
	octo_vertices.push_back(glm::vec3(0, .25, 0));
	octo_vertices.push_back(glm::vec3(0, -.25, 0));
	octo_vertices.push_back(glm::vec3(.25, 0, 0));
	octo_vertices.push_back(glm::vec3(-.25, 0, 0));

	unsigned int octo_face_indices[] = { 0, 2, 4,
										 1, 2, 4,
										 0, 3, 4,
										 1, 3, 4,
										 0, 2, 5,
										 1, 2, 5,
										 0, 3, 5,
										 1, 3, 5 };
	ConvexPolyhedron::ConvexPolyhedronFace octo_faces[] = { ConvexPolyhedron::ConvexPolyhedronFace(octo_face_indices, 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[3]), 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[6]), 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[9]), 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[12]), 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[15]), 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[18]), 3),
															ConvexPolyhedron::ConvexPolyhedronFace(&(octo_face_indices[21]), 3) };

	ConvexPolyhedron octo = ConvexPolyhedron(octo_vertices.data(), 6, octo_faces, 8);
	octo.displacement = glm::vec3(1.0f, -.25f, 0.0f);
	octo.rotation_rate = glm::radians(30.0f);

	polyhedra.push_back(&cube1);
	polyhedra.push_back(&cube2);
	polyhedra.push_back(&octo);
	*/