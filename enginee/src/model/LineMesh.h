#pragma once
#include "Mesh.h"

class LineMesh: public Mesh {
public:
    LineMesh(const Mesh& m);
    LineMesh(
        const std::vector<Vertex>& endpoints,
        const std::vector<GLuint>& lines
    );
    void set_material(
        Shader* material,
        float shininess,
        glm::vec3 ambient = glm::vec3(0),
        glm::vec3 diffuse = glm::vec3(0),
        glm::vec3 specular = glm::vec3(0)
    ) override;
    void draw() override;
};
