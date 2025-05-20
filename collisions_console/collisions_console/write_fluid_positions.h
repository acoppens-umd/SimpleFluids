#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include "geometry.h"

bool writeParticlePositions(const std::string& filename, const std::vector<Polyhedron*>& polyhedra) {
    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    outFile << "[";

    bool first = true;

    for (int i = 0; i < polyhedra.size(); i++) {
        Polyhedron* polyhedron = polyhedra[i];

        if (typeid(*polyhedron) != typeid(Sphere))
            continue;

        outFile << (first ? "\n" : ",\n");

        first = false;
        outFile << "[\n";
        outFile << polyhedron->displacement.x << ",\n";
        outFile << polyhedron->displacement.y << ",\n";
        outFile << polyhedron->displacement.z << "\n";
        outFile << "]";
    }
    
    outFile << "\n]\n";

    return true;
}