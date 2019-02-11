#pragma once
#include "Mesh.h"

// CubeMesh is equivalent to Mesh except that it enforces what gotes into the
// contstructors vertices, normals, and texture coordinates
class CubeMesh: public Mesh {
public:
    CubeMesh(std::vector<Texture>& texs);
};
