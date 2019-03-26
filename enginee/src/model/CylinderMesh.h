#pragma once
#include "Mesh.h"

// CylinderMesh is equivalent to Mesh except that it creates what goes into the
// constructors vertices, normals, and texture coordinates
class CylinderMesh: public Mesh {
public:
    CylinderMesh(
        std::vector<Texture>& texs,
        unsigned resolution = 18,
        float radius = 1.f
    );
};
