#include "comm.h"
#include "Pool.h"
#include "ai/physics.h"
#include "demo/demo.h"
#include "io.h"
#include <fstream>
#include <sstream>
#define GLM_ENABLE_EXPERIMENTAL
#include <Eigen/Dense>
#include <glm/gtx/norm.hpp>

namespace comm {
// x,y + mag
// #define NORM_REL
// y + mag
#define COSINE_REL // NOT AT THE SAME TIME AS NORM_REL (which requires a 4x9 M)
// 1 / p_mag
#define PROX // with either rel, not by itself
// g / (1 + |g|) (but g is always >0 so g/(1 + g))
#define SOFTSIGN_GOAL

// one (c+2)x(c+(5 or 7)) split into multiple for easy multiplication w/ glm
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

Eigen::Matrix<float, NCOMM, NCOMM> M_c; // c input/output

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

Eigen::Matrix<float, 2, NCOMM> M_f;

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
            M_c(row, col) = std::stof(item);
        }

#    ifdef NORM_REL
        std::array<vecc*, 7> M_v = {
            &M_vx, &M_vy, &M_s, &M_px, &M_py, &M_d, &M_g};
#    else
        std::array<vecc*, 5> M_v = {&M_vy, &M_s, &M_py, &M_d, &M_g};
#    endif
        // (M_vx;) M_vy; M_s; (M_px;) M_py; M_d; M_g
#    ifdef NORM_REL
        for (col = 0; col < 7; ++col) {
#    else
        for (col = 0; col < 5; ++col) {
#    endif
            std::getline(sl, item, ' ');
            (*M_v[static_cast<size_t>(col)])(row) = std::stof(item);
        }
    }

    // Fill in M_f
    for (int row = 0; row < 2; ++row) {
        std::getline(ss, line, '\n');
        std::stringstream sl(line);
        std::string item;
        for (int col = 0; col < NCOMM; ++col) {
            std::getline(sl, item, ' ');
            M_f(row, col) = std::stof(item);
        }
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

std::array<float, 7> x(
    Agent& a,
    CommComp& c,
    Dynamics& d,
    glm::vec2 vj,
    glm::vec2 pj
) {
    // w.r.t. local coordinate frame
    glm::mat2x2 frame = glm::transpose(glm::mat2x2(c.right(), c.facing));

    // this assumes v_forward/v_right are normalized
    glm::vec2 diff_v = vj - glm::vec2(d.vel.x, d.vel.z);
    glm::vec2 rel_v = frame * diff_v;
#if defined(COSINE_REL) || defined(NORM_REL)
    float mag_rv = glm::length(rel_v);
    if (mag_rv > 0) {
        rel_v /= mag_rv;
    }
#endif

    glm::vec2 diff_p = pj - glm::vec2(d.pos.x, d.pos.z);
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

    float g = a.goal_dist;
#ifdef SOFTSIGN_GOAL
    g /= 1 + g;
#endif

    return {rel_v.x, rel_v.y, mag_rv, rel_p.x, rel_p.y, mag_rp, g};
}

// template <size_t K>
// std::array<Entity*, K> KNN(Entity& i) {
Entity* NN(Entity& i) {
    Dynamics& d = *POOL.get<Dynamics>(i);
    glm::vec2 p_i = glm::vec2(d.pos.x, d.pos.z);
    float min_dist2 = std::numeric_limits<float>::max();
    Entity* closest = nullptr;

    // this is *really* inefficient, but oh well.
    POOL.for_<CommComp>([&](CommComp&, Entity& j) {
        if (i.id == j.id)
            return;
        Dynamics& d_j = *POOL.get<Dynamics>(j);
        glm::vec2 p_j = glm::vec2(d_j.pos.x, d_j.pos.z);

        glm::vec2 diff = p_j - p_i;
        float dist2 = glm::length2(diff);
        if (dist2 < min_dist2) {
            min_dist2 = dist2;
            closest = &j;
        }
    });
    return closest;
}

void run() {
    POOL.for_<CommComp>([&](CommComp& c_i, Entity& i) {
        // if, for some reason, a nearest neighbor is not found, treat them as
        // silent, not moving, and not distant.
        // glm::vec2 closest_pos{0};
        // glm::vec2 closest_vel{0};
        // vecc closest_c = vecc::Zero();
        // Entity* closest = NN(e_c);
        // if (closest) {
        //     closest_pos = POOL.get<Dynamics>(closest)->pos;
        //     closest_vel = POOL.get<Dynamics>(closest)->vel;
        //     closest_c = POOL.get<CommComp>(closest)->c;
        // }

        Agent& a_i = *POOL.get<Agent>(i);
        Dynamics& d_i = *POOL.get<Dynamics>(i);
        vecc sum_c = vecc::Zero();
        Eigen::Vector2f sum_f = Eigen::Vector2f::Zero();
        POOL.for_<CommComp>([&](CommComp& c_j, Entity& j) {
            if (i.id == j.id)
                return;

            Dynamics& d_j = *POOL.get<Dynamics>(j);
            glm::vec2 p_j = glm::vec2(d_j.pos.x, d_j.pos.z);
            glm::vec2 v_j = glm::vec2(d_j.vel.x, d_j.vel.z);
            auto x_i = x(a_i, c_i, d_i, v_j, p_j);

            glm::vec2 diff = p_j - glm::vec2(d_i.pos.x, d_i.pos.z);
            float proximity = 1.f / glm::dot(diff, diff);
            sum_c += proximity
                * (M_c * c_j.c //
#ifdef NORM_REL
                   + M_vx * x_i[0] //
#endif
                   + M_vy * x_i[1] //
#if defined(COSINE_REL) || defined(NORM_REL)
                   + M_s * x_i[2] //
#endif
#ifdef NORM_REL
                   + M_px * x_i[3]
#endif
                   + M_py * x_i[4] //
#if defined(COSINE_REL) || defined(NORM_REL)
                   + M_d * x_i[5] //
#endif
                   + M_g * x_i[6]);

            sum_f += proximity * (static_cast<Eigen::Vector2f>(M_f * c_j.c));
        });
        c_i.buf_in(sum_c, sum_f);
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
