#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

bool loadOBJ(const std::string& filename,
    std::vector<glm::vec3>& outVertices,
    std::vector<glm::ivec3>& outFaces) {
    std::ifstream inFile(filename);
    if (!inFile) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            outVertices.push_back(vertex);
        }
        else if (prefix == "f") {
            glm::ivec3 face;
            std::string v1, v2, v3;

            iss >> v1 >> v2 >> v3;

            auto parseIndex = [](const std::string& token) -> int {
                std::istringstream s(token);
                std::string idxStr;
                std::getline(s, idxStr, '/'); // get vertex index before '/'
                return std::stoi(idxStr) - 1; // convert to 0-based
                };

            face[0] = parseIndex(v1);
            face[1] = parseIndex(v2);
            face[2] = parseIndex(v3);
            outFaces.push_back(face);
        }
    }

    return true;
}