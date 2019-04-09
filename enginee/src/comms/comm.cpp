#include "comm.h"
#include "Pool.h"
#include "ai/physics.h"
#include "demo/demo.h"
#include "io.h"
#include <fstream>
#include <sstream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

namespace comm {
// x,y + mag
// #define NORM_REL
// y + mag
#define COSINE_REL // NOT AT THE SAME TIME AS NORM_REL (which requires a 4x9 M)
// 1 / p_mag
#define PROX // with either rel, not by itself
// g / (1 + g)
#define SOFTSIGN_GOAL

// one (c+2)x(c+(5 or 7)) split into multiple for easy multiplication
//     +
//   c c v v v v v
// + c c v v v v v
//   f f
//   f f         0
//
//   M_c M_v
//   M_f   0
//
//   M_c = cxc
//   M_v = cx2; (cx1;) cx2; (cx1;) cx1
//   M_f = 2xc
//

#if NCOMM != 1
glm::mat<NCOMM, NCOMM, float> M_c; // c input/output
#else
float M_c;
#endif

#ifdef NORM_REL
vecc M_vx; // relative velocity input
#endif
vecc M_vy; // relative velocity input
vecc M_s; // magnitude of the relative velocity
#ifdef NORM_REL
vecc M_px; // relative position input
#endif
vecc M_py; // relative position input
vecc M_d; // distance of the relative position of neighbor
vecc M_g; // goal distance input

#if NCOMM != 1
glm::mat<NCOMM, 2, float> M_f;
#else
glm::vec2 M_f;
#endif

std::string data_dir;

void init(std::string data_dir_) {
    comm::data_dir = data_dir_;
#ifndef NO_COMM
    std::string config_file = comm::data_dir + "/comms.config";
    std::string config_str = *read_file(config_file);

    std::stringstream ss(config_str);
    std::string line;
    for (int row = 0; row < NCOMM; ++row) {
        std::getline(ss, line, '\n');
        std::stringstream sl(line);
        std::string item;

        // M_c
        int col = 0;
        for (; col < NCOMM; ++col) {
            std::getline(sl, item, ' ');
#    if NCOMM != 1
            M_c[col][row] = std::stof(item);
            printf("%f ", M_c[col][row]);
#    else
            M_c = std::stof(item);
            printf("%f ", M_c);
#    endif
        }

#    ifdef NORM_REL
        std::array<vecc*, 7> M_v = {
            &M_vx, &M_vy, &M_s, &M_px, &M_py, &M_d, &M_g};
#    else
        std::array<vecc*, 5> M_v = {&M_vy, &M_s, &M_py, &M_d, &M_g};
#    endif
        // (M_vx;) M_vy; M_s; (M_px;) M_py; M_d; M_g
        for (int col = 0; col < static_cast<int>(M_v.size()); ++col) {
            std::getline(sl, item, ' ');
            (*M_v[col])[row] = std::stof(item);
            printf("%f ", (*M_v[col])[row]);
        }
        printf("\n");
    }

    // Fill in M_f
    for (int row = 0; row < 2; ++row) {
        std::getline(ss, line, '\n');
        std::stringstream sl(line);
        std::string item;
        for (int col = 0; col < NCOMM; ++col) {
            std::getline(sl, item, ' ');
#    if NCOMM != 1
            M_f[col][row] = std::stof(item);
            printf("%f ", M_f[col][row]);
#    else
            M_f[row] = std::stof(item);
            printf("%f ", M_f[row]);
#    endif
        }
        printf("\n");
    }
#endif
}

void terminate() {
    std::ofstream results;
    results.open(comm::data_dir + "/comms.result");
    float group_time = demo::run_avg_time + demo::run_std_time * 3;
    results << physics::avg_velocity << "\n" << group_time << "\n";
    results.close();
}

void run() {
    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        // if, for some reason, a nearest neighbor is not found, treat them as
        // silent, not moving, and not distant.
        glm::vec2 closest_pos{0};
        glm::vec2 closest_vel{0};
        vecc closest_c{0};
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        float min_dist2 = std::numeric_limits<float>::max();

        // this is *really* inefficient, but oh well.
        POOL.for_<CommComp>([&](CommComp& o_c, Entity& e_other) {
            if (e_c.id == e_other.id)
                return;

            Dynamics* d_other = POOL.get<Dynamics>(e_other);
            glm::vec3 diff = d_other->pos - d.pos;
            float dist2 = glm::length2(diff);
            if (dist2 < min_dist2) {
                min_dist2 = dist2;
                closest_pos = d_other->pos;
                closest_vel = d_other->vel;
                closest_c = o_c.c;
            }
        });

        // w.r.t. local coordinate frame
        glm::mat2x2 frame = glm::transpose(glm::mat2x2(c.right(), c.facing));

        // this assumes v_forward/v_right are normalized
        glm::vec2 diff_v = closest_vel - glm::vec2(d.vel.x, d.vel.z);
        glm::vec2 rel_v = frame * diff_v;
#if defined(COSINE_REL) || defined(NORM_REL)
        float mag_rv = glm::length(rel_v);
        if (mag_rv > 0) {
            rel_v /= mag_rv;
        }
#endif

        glm::vec2 diff_p = closest_pos - glm::vec2(d.pos.x, d.pos.z);
        glm::vec2 rel_p = frame * diff_p;
#if defined(COSINE_REL) || defined(NORM_REL)
        float mag_rp = glm::length(rel_p);
        if (mag_rp > 0) {
            rel_p /= mag_rp;
#    ifdef PROX
            mag_rp = 1 / mag_rp;
#    endif
        }
#endif

        Agent& a = *POOL.get<Agent>(e_c);
        float g = a.goal_dist;
#ifdef SOFTSIGN_GOAL
        g /= 1 + g;
#endif

        c.buf_in(
            M_c * closest_c //
#ifdef NORM_REL
                + M_vx * rel_v.x //
#endif
                + M_vy * rel_v.y //
#if defined(COSINE_REL) || defined(NORM_REL)
                + M_s * mag_rv //
#endif
#ifdef NORM_REL
                + M_px * rel_p.x
#endif
                + M_py * rel_p.y //
#if defined(COSINE_REL) || defined(NORM_REL)
                + M_d * mag_rp //
#endif
                + M_g * g,
            M_f * closest_c
        );
    });

    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        glm::vec2 diff = c.right() * c.f_buf.x + c.facing * c.f_buf.y;

        Agent* a = POOL.get<Agent>(e_c);
        if (a && !a->done()) {
            d.force += glm::vec3(diff.x, 0.f, diff.y);
        }

        if ( //
            glm::length(d.vel) > 0 //
            && glm::length(a->local_goal - a->start) > 0
        ) {
            c.facing = glm::normalize(a->local_goal - a->start);
        }
        c.swap();
    });
}

} // namespace comm
