#ifndef COMM_COMP_H
#define COMM_COMP_H

#include <glm/vec3.hpp>

struct CommComp {
    float c;
    float c_buf;
    glm::vec2 v_buf;
    glm::vec2 facing;
    void buf_in(glm::vec3 v) {
        c_buf = v[0];
        v_buf = glm::vec2(v[1], v[2]);
    };
    void swap() {
        c = c_buf;
        v_buf = glm::vec2(0);
    };
    glm::vec2 right() {
        return glm::vec2(-facing.y, facing.x);
    }
};
#endif
