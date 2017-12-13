#include "Mesh.h"

#include <string>
#include <cstddef>

using namespace std;

Mesh::Mesh(const vector<Vertex>& verts,
        const vector<GLuint>& idxs,
        const vector<Texture>& texs)
        : vertices(verts), indices(idxs), textures(texs) {
    gen();//need some bufs
    glBindVertexArray(vao); //remember this sequence
    bind();//use the generated bufs
    attrib();//expect the data to be like this
    buffer();//here is some (static) data; different for skinning
    glBindVertexArray(0);

    _type = Type::ANY;
}

Mesh::~Mesh() {
}

void Mesh::gen() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ibo);
}

void Mesh::bind() {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
}

void Mesh::attrib() {
    //enable position data
    glVertexAttribPointer(0, 
        3, GL_FLOAT, GL_FALSE,
        sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);

    //enable normal data
    glVertexAttribPointer(1,
        3, GL_FLOAT, GL_FALSE,
        sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, norm)));
    glEnableVertexAttribArray(1);

    //enable texture-uv data
    glVertexAttribPointer(2,
        2, GL_FLOAT, GL_FALSE,
        sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, tex)));
    glEnableVertexAttribArray(2);
}

void Mesh::buffer() {
    glBufferData(GL_ARRAY_BUFFER, 
        static_cast<long int>(vertices.size() * sizeof(Vertex)), &vertices[0],
        GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        static_cast<long int>(indices.size() * sizeof(GLuint)), &indices[0],
        GL_STATIC_DRAW);
}

string texmap_to_string(Texmap t) {
    switch(t) {
    case Texmap::other:
        return "other";
    case Texmap::ambient:
        return "ambient";
    case Texmap::diffuse:
        return "diffuse";
    case Texmap::specular:
        return  "specular";
    default:
        return "";
    }
}

void Mesh::bind_textures() {
    unsigned map_count[4] = {0};
    for (GLuint i = 0; i < textures.size(); ++i) {
        glActiveTexture(GL_TEXTURE0 + i);
        Texmap t = textures[i].type;
        unsigned count = ++map_count[static_cast<unsigned>(t)];
        mtl->set("material." + texmap_to_string(t) + to_string(count),
            static_cast<GLint>(i));
        glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }
    glActiveTexture(GL_TEXTURE0);
}

void Mesh::unbind_textures() {
    for (GLuint i = 0; i < textures.size(); ++i) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    glActiveTexture(GL_TEXTURE0);
}

void Mesh::set_material(Shader* material, float shininess,
        glm::vec3 ambient, glm::vec3 diffuse, glm::vec3 specular) {
     mtl = material;
    _shininess = shininess;
    if (textures.size() == 0) {
        _ambient = ambient;
        _diffuse = diffuse;
        _specular = specular;
    } else {
        _specular = _diffuse = _ambient = glm::vec3(0);
    }
}

void Mesh::draw() {
    mtl->use();
    glBindVertexArray(vao);
    bind_textures();
    mtl->set("material.shininess", _shininess);
    mtl->set("material.color.ambient", _ambient);
    mtl->set("material.color.diffuse", _diffuse);
    mtl->set("material.color.specular", _specular);
    glDrawElements(GL_TRIANGLES, static_cast<int>(indices.size()), GL_UNSIGNED_INT, nullptr);
    unbind_textures();
    glBindVertexArray(0);
}
