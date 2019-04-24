#pragma once
#include <array>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <glm/vec3.hpp>

namespace comm {
#define NCOMM 3
typedef glm::vec<NCOMM, float> vecc;
} // namespace comm

struct CommComp {
    comm::vecc c;
    comm::vecc c_buf;
    glm::vec2 f_buf;
    glm::vec2 facing;
    void buf_in(comm::vecc c_prime, glm::vec2 f_prime) {
        // distributed non-linearity
        for (int i = 0; i < NCOMM; ++i) {
            // const float y = .5f;
            const float b = .5f;
            const float x = c_prime[i]; // y
            c_buf[i] =
                // atan "wrong"
                // glm::two_over_pi<float>() * std::atan(x);
                // tanh
                // std::tanh(x);
                // Softsign (multiply denom x by 2 for 0..1)
                // x / (1.f + std::fabs(2.f * x));
                // atan "correct" (approximates y=x for small x)
                // remove two's to make 0..1
                // glm::one_over_pi<float>() * std::atan(glm::pi<float>() * x);
                // ISRU (multiply denom x by 4 for 0..1)
                x / std::sqrt(1.f + 4.f * x * x);
            // Soft.... ?
            // 4.f * x / (4.f + x * x);
            // clamp
            // glm::clamp(x, -1.f, 1.f);
            // sigmoid 0..1
            // 1.f / (1.f + std::exp(-x));

            // TODO: optimize p over
            // Soft Lp-normalized
            // x / std::pow(1.f + std::pow(std::fabs(x), p)), 1.f/p);

            // c_buf[i] *= y;
            c_buf[i] += b;
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
