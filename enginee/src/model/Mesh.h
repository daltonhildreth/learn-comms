#pragma once
#include "light/Shader.h"
#include <glad.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <memory>
#include <vector>

struct Vertex {
    glm::vec3 pos;
    glm::vec3 norm;
    glm::vec2 tex;
};

enum class Texmap { other, ambient, diffuse, specular };
std::string texmap_to_string(Texmap t);

struct Texture {
    GLuint id;
    Texmap type;
};

class Mesh {
public:
    Mesh(
        const std::vector<Vertex>& vertices,
        const std::vector<GLuint>& indices,
        const std::vector<Texture>& textures
    );
    virtual void set_material(
        Shader* material,
        float shininess,
        glm::vec3 ambient = glm::vec3(0),
        glm::vec3 diffuse = glm::vec3(0),
        glm::vec3 specular = glm::vec3(0)
    );
    virtual void draw();

    virtual ~Mesh();

    enum class Type { ANY, CUBE, LINE, CYLINDER };
    Type _type;

    // protected:
    // helpers
    void gen();
    void bind();
    void bind_textures();
    void unbind_textures();
    void attrib();
    void buffer();

    // mesh
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Texture> textures;

    // OpenGL buffers and ids
    GLuint vao;
    GLuint vbo;
    GLuint ibo;

    Shader* mtl;
    float _shininess;
    glm::vec3 _ambient;
    glm::vec3 _diffuse;
    glm::vec3 _specular;
};
