#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <glm/vec3.hpp>

namespace comm {
#define NCOMM 3
typedef Eigen::Matrix<float, NCOMM, 1> vecc;
} // namespace comm

struct CommComp {
    comm::vecc c;
    comm::vecc c_buf;
    glm::vec2 f_buf;
    glm::vec2 facing;
    void buf_in(comm::vecc c_prime, Eigen::Vector2f f_prime) {
        // distributed non-linearity
        for (int i = 0; i < NCOMM; ++i) {
            const float x = c_prime[i];
            c_buf[i] = // x / (1.f + std::fabs(x));
                x / (1.f + std::fabs(2.f * x)) + 0.5f;
            // glm::two_over_pi<float>() * std::atan(glm::half_pi<float>() * x);
            // glm::one_over_pi<float>() * std::atan(glm::pi<float>() * x) +
            // 0.5f;
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
