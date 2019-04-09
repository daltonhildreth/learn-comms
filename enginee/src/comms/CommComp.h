#pragma once
#include <array>
#include <glm/vec3.hpp>

namespace comm {
#define NCOMM 2
typedef glm::vec<NCOMM, float> vecc;
}; // namespace comm

struct CommComp {
    comm::vecc c; // NCOMM
    comm::vecc c_buf; // NCOMM
    glm::vec2 f_buf;
    glm::vec2 facing;
    void buf_in(comm::vecc c_prime, glm::vec2 f_prime) {
        // distributed non-linearity
        for (int i = 0; i < NCOMM; ++i) {
            c_buf[i] = glm::clamp(c_prime[i], -1.f, 1.f);
        }
        // max force
        for (int i = 0; i < 2; ++i) {
            f_buf[i] = glm::clamp(f_prime[i], -1.f, 1.f);
        }
    };
    void swap() {
        c = c_buf;
        f_buf = glm::vec2(0);
    };
    glm::vec2 right() { return glm::vec2(facing.y, -facing.x); }
};
