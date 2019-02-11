#include "LineMesh.h"
#include "util/debug.h"

#include <iostream>

LineMesh::LineMesh(const Mesh& m): Mesh(m.vertices, m.indices, {}) {
    vertices = m.vertices;
    indices = m.indices;
    textures = {};
    vao = m.vao;
    vbo = m.vbo;
    ibo = m.ibo;
    mtl = m.mtl;
    _shininess = 0;
    _ambient = m._ambient;
    _diffuse = glm::vec3();
    _specular = glm::vec3();
}

LineMesh::LineMesh(
    const std::vector<Vertex>& endpoints,
    const std::vector<GLuint>& lines
):
    Mesh(endpoints, lines, {}) {
    gen();
    glBindVertexArray(vao);
    bind();
    attrib();
    buffer();
    glBindVertexArray(0);

    _type = Type::LINE;
}

void LineMesh::set_material(
    Shader* material,
    float sh,
    glm::vec3 ambient,
    glm::vec3 d,
    glm::vec3 s
) {
    UNUSED(sh, d, s);
    mtl = material;
    _ambient = ambient;
    _shininess = 0;
    _diffuse = glm::vec3(0);
    _specular = glm::vec3(0);
}

void LineMesh::draw() {
    mtl->use();
    glBindVertexArray(vao);
    mtl->set("material.shininess", _shininess);
    mtl->set("material.color.ambient", _ambient);
    mtl->set("material.color.diffuse", _diffuse);
    mtl->set("material.color.specular", _specular);
    glDrawElements(
        GL_LINES, static_cast<int>(indices.size()), GL_UNSIGNED_INT, nullptr
    );
    glBindVertexArray(0);
}
