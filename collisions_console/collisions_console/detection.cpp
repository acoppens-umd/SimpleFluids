#include "detection.h"
#include <queue>
#include <tuple>
#include <limits>


bool tetrahedron_contains_origin(Simplex& simplex) {
	bool sign1 = glm::dot(glm::cross(simplex[0], simplex[1]), simplex[2]) > 0;
	bool sign2 = glm::dot(glm::cross(simplex[0], simplex[2]), simplex[3]) > 0;
	bool sign3 = glm::dot(glm::cross(simplex[0], simplex[3]), simplex[1]) > 0;
	bool sign4 = glm::dot(glm::cross(simplex[1], simplex[3]), simplex[2]) > 0;

	return sign1 == sign2 && sign2 == sign3 && sign3 == sign4;
}

bool triangle_contains_origin_projection(Simplex& simplex, float& distance) {
	glm::vec3 AB = simplex[1] - simplex[0];
	glm::vec3 AC = simplex[2] - simplex[0];
	glm::vec3 AO = -simplex[0];

	float ABdAO = glm::dot(AB, AO);
	float ABdAB = glm::dot(AB, AB);
	float ACdAO = glm::dot(AC, AO);
	float ABdAC = glm::dot(AB, AC);
	float ACdAC = glm::dot(AC, AC);

	float v = ABdAO * ABdAB + ACdAO * ABdAC;
	float u = ABdAO * ABdAC + ACdAO * ACdAC;

	distance = glm::length(AO - v * AB - u * AC);

	return v > 0 && u > 0 && 1 - v - u > 0;
}

bool triangle_contains_origin_projection(Simplex& simplex) {
	float distance;
	return triangle_contains_origin_projection(simplex, distance);
}

glm::vec3 displacement_to_triangle(Simplex& simplex) {
	glm::vec3 AB = simplex[1] - simplex[0];
	glm::vec3 AC = simplex[2] - simplex[0];
	glm::vec3 AO = -simplex[0];

	float d1 = glm::dot(AB, AO);
	float d2 = glm::dot(AC, AO);
	if (d1 <= 0.f && d2 <= 0.f) return simplex[0]; //#1

	glm::vec3 BO = simplex[1];
	float d3 = glm::dot(AB, BO);
	float d4 = glm::dot(AC, BO);
	if (d3 >= 0.f && d4 <= d3) return simplex[1]; //#2

	glm::vec3 CO = - simplex[2];
	float d5 = dot(AB, CO);
	float d6 = dot(AC, CO);
	if (d6 >= 0.f && d5 <= d6) return simplex[2]; //#3

	float VC = d1 * d4 - d3 * d2;
	if (VC <= 0.f && d1 >= 0.f && d3 <= 0.f)
	{
		const float v = d1 / (d1 - d3);
		return simplex[0] + v * AB; //#4
	}

	const float VB = d5 * d2 - d1 * d6;
	if (VB <= 0.f && d2 >= 0.f && d6 <= 0.f)
	{
		const float v = d2 / (d2 - d6);
		return simplex[0] + v * AC; //#5
	}

	const float VA = d3 * d6 - d5 * d4;
	if (VA <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
	{
		const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return simplex[1] + v * (simplex[2] - simplex[1]); //#6
	}

	const float denom = 1.f / (VA + VB + VC);
	const float v = VB * denom;
	const float w = VC * denom;
	return simplex[0] + v * AB + w * AC; //#0


}

//Returns true if the simplex contains the origin
bool nearest_simplex(Simplex& simplex, glm::vec3& normal) {
	if (simplex.size() == 4) {
		if (tetrahedron_contains_origin(simplex)) {
			return true;
		}

		glm::vec3 v3v2v1 = glm::cross(simplex[2] - simplex[3], simplex[1] - simplex[3]);
		glm::vec3 v3v1v0 = glm::cross(simplex[1] - simplex[3], simplex[0] - simplex[3]);
		glm::vec3 v3v0v2 = glm::cross(simplex[0] - simplex[3], simplex[2] - simplex[3]);
		glm::vec3 v3O = -simplex[3];

		if (glm::dot(v3O, v3v2v1) > 0) {
			simplex = { simplex[1], simplex[2], simplex[3] };
			nearest_simplex(simplex, normal);

			return false;
		}

		if (glm::dot(v3O, v3v1v0) > 0) {
			simplex = { simplex[0], simplex[1], simplex[3] };
			nearest_simplex(simplex, normal);

			return false;
		}

		if (glm::dot(v3O, v3v0v2) > 0) {
			simplex = { simplex[2], simplex[0], simplex[3] };
			nearest_simplex(simplex, normal);

			return false;
		}
		
		return true;
	} else if (simplex.size() == 3) {
		glm::vec3 v2v0 = simplex[0] - simplex[2];
		glm::vec3 v2v1 = simplex[1] - simplex[2];
		glm::vec3 v2v1cv2v0 = glm::cross(v2v1, v2v0);
		glm::vec3 v2O = -simplex[2];

		if (glm::dot(glm::cross(v2v1cv2v0, v2v0), v2O) > 0) {
			if (glm::dot(v2v0, v2O) > 0) {
				simplex = { simplex[0], simplex[2] };
				normal = glm::cross(glm::cross(v2v0, v2O), v2v0);
			} else {
				if (glm::dot(v2v1, v2O) > 0) {
					simplex = { simplex[1], simplex[2] };
					normal = glm::cross(glm::cross(v2v1, v2O), v2v1);
				} else {
					simplex = { simplex[2] };
					normal = v2O;
				}
			}
		} else {
			if (glm::dot(glm::cross(v2v1, v2v1cv2v0), v2O) > 0) {
				if (glm::dot(v2v1, v2O) > 0) {
					simplex = { simplex[1], simplex[2] };
					normal = glm::cross(glm::cross(v2v1, v2O), v2v1);
				} else {
					simplex = { simplex[2] };
					normal = v2O;
				}
			} else {
				if (glm::dot(v2v1cv2v0, v2O) > 0) {
					normal = v2v1cv2v0;
				} else {
					simplex = { simplex[1], simplex[0], simplex[2] };
					normal = -v2v1cv2v0;
				}
			}
		}

		return false;
			
	} else if (simplex.size() == 2) {
		glm::vec3 AB = simplex[1] - simplex[0];
		glm::vec3 AO = -simplex[0];

		float u = glm::dot(AO, AB) / glm::dot(AB, AB);

		if (u > 1) {
			glm::vec3 pnt = simplex[1];
			simplex = { pnt }; normal = -pnt;
		} else {
			glm::vec3 proj = u * AB;
			normal = AO - proj;
		}
	}

	return false;
}

detection intersects(Polyhedron* p, Polyhedron* q) {
	if (typeid(*p) == typeid(Sphere) && typeid(*q) == typeid(Sphere)) {
		glm::vec3 dist = p->displacement - q->displacement;
		float length = glm::length(dist);

		if (length < (p->radius + q->radius)) {
			return { true, glm::normalize(dist) * (p->radius + q->radius - length) };
		}
		else {
			return { false, dist };
		}
	}
	else if ((typeid(*p) == typeid(Sphere) && typeid(*q) == typeid(ConvexPolyhedron)) ||
		(typeid(*p) == typeid(ConvexPolyhedron) && typeid(*q) == typeid(Sphere))) {
		Simplex simplex = std::vector<glm::vec3>();

		if (gjk(*p, *q, simplex)) {
			return { true, penetration_angle(*p, *q, simplex) };
		}
		else {
			return { false, glm::vec3(0.0f) };
		}
	}
	else if (typeid(*p) == typeid(Sphere) && typeid(*q) == typeid(ConcavePolyhedron)) {
		for (ConvexPolyhedron* sub_poly : ((ConcavePolyhedron *)q)->decomposition) {
			detection sub_detection = intersects(p, sub_poly);
			if (sub_detection.detected) {
				return sub_detection;
			}
		}
		return { false, glm::vec3(0.0f) };
	} 
	else if (typeid(p) == typeid(ConcavePolyhedron) && typeid(q) == typeid(Sphere)) {
		for (ConvexPolyhedron* sub_poly : ((ConcavePolyhedron*)p)->decomposition) {
			detection sub_detection = intersects(sub_poly, q);
			if (sub_detection.detected) {
				return sub_detection;
			}
		}
		return { false, glm::vec3(0.0f) };
	}
	//Case of unsupported collision, i.e. Concave-Concave collision
	else {
		return { false, glm::vec3(0.0f) };
	}

}

//Uses the GJK algorithm
bool gjk(Polyhedron& p, Polyhedron& q, Simplex& simplex, glm::vec3 direction) {
	glm::vec3 support_vector = p.support_function(direction) - q.support_function(-direction);
	simplex.push_back(support_vector);
	direction = -support_vector;

	int iterations = 0;
	while (true) {
		iterations++;
		if (iterations > 100) {
			//iterations = 0;
			break;
		}
		support_vector = p.support_function(direction) - q.support_function(-direction);

		if (glm::dot(support_vector, direction) < 0) {
			return false;
		}

		simplex.push_back(support_vector);

		if (nearest_simplex(simplex, direction)) {
			return true;
		}
	}

	return false;
}

struct face {
	int vertex_indices[3];
	glm::vec3 normal;
	float distance;

	face(std::vector<glm::vec3>& polytope, int i0, int i1, int i2) {
		this->vertex_indices[0] = i0;
		this->vertex_indices[1] = i1;
		this->vertex_indices[2] = i2;

		this->normal = glm::normalize(glm::cross(polytope[i1] - polytope[i0], polytope[i2] - polytope[i0]));
		this->distance = glm::dot(normal, polytope[i0]);
		if (distance < 0) {
			normal = -normal;
			distance = -distance;
		}
	}
};

void add_unique_edge(std::vector<std::pair<int, int>>& edges, int i0, int i1) {
	auto find_reverse = std::find(edges.begin(), edges.end(), std::make_pair(i1, i0));

	if (find_reverse != edges.end()) {
		edges.erase(find_reverse);
	} else {
		edges.push_back(std::make_pair(i0, i1));
	}
}

//Uses expanding polytope algorithm
glm::vec3 penetration_angle(Polyhedron& p, Polyhedron& q, Simplex& simplex) {
	std::vector<glm::vec3> polytope(simplex.begin(), simplex.end());

	std::vector<face> faces = {
		face(polytope, 0, 1, 2),
		face(polytope, 0, 2, 3),
		face(polytope, 0, 3, 1),
		face(polytope, 1, 3, 2)
	};

	face* min_face = nullptr;

	int iterations = 0;
	while (true) {
		iterations++;
		if (iterations > 100) {
			//iterations = 0;
			break;
		}
		min_face = nullptr;
		for (face& f : faces) {
			if (min_face == nullptr || f.distance < min_face->distance) {
				min_face = &f;
			}
		}

		glm::vec3 support_vector = p.support_function(min_face->normal) - q.support_function(-(min_face->normal));

		float support_distance = glm::dot(min_face->normal, support_vector);

		if (abs(support_distance - min_face->distance) < 1e-5) {
			break;
		}

		std::vector<std::pair<int, int>> edges;

		for (int i = 0; i < faces.size(); i++) {
			if (glm::dot(faces[i].normal, support_vector) > 0) {
				add_unique_edge(edges, faces[i].vertex_indices[0], faces[i].vertex_indices[1]);
				add_unique_edge(edges, faces[i].vertex_indices[1], faces[i].vertex_indices[2]);
				add_unique_edge(edges, faces[i].vertex_indices[2], faces[i].vertex_indices[0]);

				faces[i] = faces.back();
				faces.pop_back();

				i--;
			}
		}

		polytope.push_back(support_vector);
		for (std::pair<int, int> edge : edges) {
			faces.push_back(face(polytope, edge.first, edge.second, polytope.size() - 1));
		}
	}

	return (min_face->distance + .001f) * min_face->normal;
}