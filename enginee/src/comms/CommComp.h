#ifndef COMM_COMP_H
#define COMM_COMP_H

#include <glm/vec3.hpp>

struct CommComp {
    glm::vec2 c;
    glm::vec2 c_buf;
    glm::vec2 v_buf;
    glm::vec2 facing;
    void buf_in(glm::vec4 v) {
        for (int i = 0; i < 4; ++i) {
            v[i] = glm::clamp<float>(v[i], -1, 1);
        }
        c_buf = glm::vec2(v[0], v[1]);
        v_buf = glm::vec2(v[2], v[3]);
    };
    void swap() {
        c = c_buf;
        v_buf = glm::vec2(0);
    };
    glm::vec2 right() {
        return glm::vec2(facing.y, -facing.x);
    }
};
#endif
