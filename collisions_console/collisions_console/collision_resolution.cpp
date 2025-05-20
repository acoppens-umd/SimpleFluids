#include "collision_resolution.h"
#include <iostream>

float ideal_penetration = .085;
float incompressibility_stiffness = .3;
float viscous_damping = .02;

void resolve_fluid_collision(Sphere* p, Sphere* q, glm::vec3 penetration) {
	glm::vec3 penetration_direction = glm::normalize(penetration);
	float penetration_difference = glm::length(penetration) - ideal_penetration;

	glm::vec3 incompressibility_force = -incompressibility_stiffness * penetration_difference * penetration_direction;

	float distance = glm::length(p->displacement - q->displacement);

	glm::vec3 relative_velocity = p->velocity - q->velocity;

	glm::vec3 viscous_force = - viscous_damping * relative_velocity;

	glm::vec3 total_force = incompressibility_force + viscous_force;

	p->force_balance += total_force;
	q->force_balance -= total_force;
}

float restitution = .5;
//No force applied to polyhedron as polyhedron emulates interactive element like player model
void resolve_fluid_rigid_collision(Sphere* particle, ConcavePolyhedron* poly, glm::vec3 penetration) {
	glm::vec3 penetration_direction = glm::normalize(penetration);

	glm::vec3 relative_velocity = particle->velocity - poly->velocity;

	glm::vec3 relative_velocity_pen_direction = glm::dot(relative_velocity, penetration_direction) * penetration_direction;

	relative_velocity -= relative_velocity_pen_direction * (1 + restitution);

	particle->velocity = relative_velocity + poly->velocity;
	particle->displace(penetration);
}

void resolve_collision(Polyhedron* p, Polyhedron* q, glm::vec3 penetration) {
	if (typeid(*p) == typeid(Sphere) && typeid(*q) == typeid(Sphere)) {
		resolve_fluid_collision((Sphere*)p, (Sphere*)q, penetration);
	}
	else if ((typeid(*p) == typeid(Sphere) && typeid(*q) == typeid(ConvexPolyhedron)) ||
		(typeid(*p) == typeid(ConvexPolyhedron) && typeid(*q) == typeid(Sphere))) {

	}
	else if (typeid(*p) == typeid(Sphere) && typeid(*q) == typeid(ConcavePolyhedron)) {
		resolve_fluid_rigid_collision((Sphere*)p, (ConcavePolyhedron*)q, penetration);
	}
	else if (typeid(p) == typeid(ConcavePolyhedron) && typeid(q) == typeid(Sphere)) {
		resolve_fluid_rigid_collision((Sphere*)q, (ConcavePolyhedron*)p, -penetration);
	}
	else {
		//Do Nothing
	}
}