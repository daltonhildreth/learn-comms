#include "CylinderMesh.h"
#include <glm/gtc/constants.hpp>

const std::vector<Vertex> _verts(unsigned resolution, float radius) {
    std::vector<Vertex> vertices;
    // bottom center // -0
    vertices.push_back(Vertex{{0, 0, 0}, {0, -1, 0}, {0.5, 0.5}});
    // top center    // +1
    vertices.push_back(Vertex{{0, 1, 0}, {0, 1, 0}, {0.5, 0.5}});

    for (unsigned i = 0; i < resolution; ++i) {
        float elem = 1 / static_cast<float>(resolution);
        float ratio = i * elem;
        float pi2 = 2.f * glm::pi<float>();
        float angle = ratio * pi2;
        float nx = std::cos(angle);
        float ny = std::sin(angle);
        float x = radius * nx;
        float y = radius * ny;

        for (int h = 0; h <= 1; ++h) {
            float side = h * 2 - 1;

            glm::vec3 vec{x, h, y};
            // cap
            // -2, +5, -8, +11,
            vertices.push_back(Vertex{
                vec,
                {0, side, 0},
                {side * x / 2 + 1, side * y / 2 + 1},
            });

            // wall
            // -3, +6, -9, +12,
            vertices.push_back(Vertex{vec, {nx, 0, ny}, {ratio, h}});

            // -4, +7, -10, +13,
            float next = angle + pi2 * elem;
            vertices.push_back(Vertex{
                {radius * std::cos(next), h, radius * std::sin(next)},
                {nx, 0, ny},
                {ratio, h},
            });
        }
    }
    return vertices;
}
const std::vector<GLuint> _idxs(unsigned resolution) {
    std::vector<GLuint> indices;
    unsigned verts_sz = 6 * resolution;
    indices.reserve(3 * 4 * resolution);
    for (unsigned i = 0; i < resolution; ++i) {
        // faces
        unsigned tr = 6u + 6 * i;
        unsigned tl = 7u + 6 * i;
        unsigned br = 3u + 6 * i;
        unsigned bl = 4u + 6 * i;
        indices.insert(indices.end(), {bl, tr, tl});
        indices.insert(indices.end(), {br, tr, bl});
    }
    for (unsigned i = 0; i < resolution; ++i) {
        // caps
        // 0, 2, 8
        // 1, 5, 11
        unsigned current = 2u + 6 * i;
        unsigned next = (2u + 6 * (i + 1)) % verts_sz;
        indices.insert(indices.end(), {current, next, 0});
        indices.insert(indices.end(), {current + 3, 1, next + 3});
    }
    return indices;
}

CylinderMesh::CylinderMesh(
    std::vector<Texture>& texs,
    unsigned resolution,
    float radius
):
    Mesh(_verts(resolution, radius), _idxs(resolution), texs) {
    _type = Type::CYLINDER;
}
