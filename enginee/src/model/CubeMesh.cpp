#include "CubeMesh.h"

// clang-format off
const std::vector<Vertex> verts = {
    //top (0 - 3)
    {glm::vec3( 0.5,  0.5,  0.5), glm::vec3(0, 1, 0), glm::vec2(0, 0)},
    {glm::vec3( 0.5,  0.5, -0.5), glm::vec3(0, 1, 0), glm::vec2(1, 0)},
    {glm::vec3(-0.5,  0.5,  0.5), glm::vec3(0, 1, 0), glm::vec2(0, 1)},
    {glm::vec3(-0.5,  0.5, -0.5), glm::vec3(0, 1, 0), glm::vec2(1, 1)},

    //bottom (4 - 7)
    {glm::vec3( 0.5, -0.5,  0.5), glm::vec3(0, -1, 0), glm::vec2(0, 0)},
    {glm::vec3( 0.5, -0.5, -0.5), glm::vec3(0, -1, 0), glm::vec2(1, 0)},
    {glm::vec3(-0.5, -0.5,  0.5), glm::vec3(0, -1, 0), glm::vec2(0, 1)},
    {glm::vec3(-0.5, -0.5, -0.5), glm::vec3(0, -1, 0), glm::vec2(1, 1)},

    //right (8 - 11)
    {glm::vec3( 0.5,  0.5,  0.5), glm::vec3(1, 0, 0), glm::vec2(0, 0)},
    {glm::vec3( 0.5,  0.5, -0.5), glm::vec3(1, 0, 0), glm::vec2(1, 0)},
    {glm::vec3( 0.5, -0.5,  0.5), glm::vec3(1, 0, 0), glm::vec2(0, 1)},
    {glm::vec3( 0.5, -0.5, -0.5), glm::vec3(1, 0, 0), glm::vec2(1, 1)},

    //left (12 - 15)
    {glm::vec3(-0.5,  0.5,  0.5), glm::vec3(-1, 0, 0), glm::vec2(0, 0)},
    {glm::vec3(-0.5,  0.5, -0.5), glm::vec3(-1, 0, 0), glm::vec2(1, 0)},
    {glm::vec3(-0.5, -0.5,  0.5), glm::vec3(-1, 0, 0), glm::vec2(0, 1)},
    {glm::vec3(-0.5, -0.5, -0.5), glm::vec3(-1, 0, 0), glm::vec2(1, 1)},

    //forward (16 - 19)
    {glm::vec3( 0.5,  0.5,  0.5), glm::vec3(0, 0, 1), glm::vec2(0, 0)},
    {glm::vec3(-0.5,  0.5,  0.5), glm::vec3(0, 0, 1), glm::vec2(1, 0)},
    {glm::vec3( 0.5, -0.5,  0.5), glm::vec3(0, 0, 1), glm::vec2(0, 1)},
    {glm::vec3(-0.5, -0.5,  0.5), glm::vec3(0, 0, 1), glm::vec2(1, 1)},

    //backward (20 - 23)
    {glm::vec3( 0.5,  0.5, -0.5), glm::vec3(0, 0, -1), glm::vec2(0, 0)},
    {glm::vec3(-0.5,  0.5, -0.5), glm::vec3(0, 0, -1), glm::vec2(1, 0)},
    {glm::vec3( 0.5, -0.5, -0.5), glm::vec3(0, 0, -1), glm::vec2(0, 1)},
    {glm::vec3(-0.5, -0.5, -0.5), glm::vec3(0, 0, -1), glm::vec2(1, 1)}
};
const std::vector<GLuint> idxs = {
    //top
     0,  1,  3,
     0,  3,  2,
    //bottom
     4,  6,  7, 
     4,  7,  5,
    //right
     8, 10, 11,
     8, 11,  9,
    //left
    12, 13, 15,
    12, 15, 14,
    //forward
    16, 17, 19,
    16, 19, 18,
    //backward
    20, 22, 23,
    20, 23, 21
};
// clang-format on

CubeMesh::CubeMesh(std::vector<Texture>& texs): Mesh(verts, idxs, texs) {
    _type = Type::CUBE;
}
