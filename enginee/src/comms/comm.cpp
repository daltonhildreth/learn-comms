#include "comm.h"
#include "Pool.h"
#include "ai/physics.h"
#include "demo/demo.h"
#include "io.h"
#include <fstream>
#include <sstream>

namespace comm {
#define COSINE_REL // NOT AT THE SAME TIME AS NORM_REL (which requires a 4x9 M)
#define PROX // with either rel, not by itself
#define SOFTSIGN_GOAL

// one 4x7 or 9 matrix I split into many for easy multiplication later
glm::mat2x4 M_c; // c input/output
glm::mat2x4 M_relv; // relative velocity input
#ifdef NORM_REL
glm::vec4 M_magv; // magnitude of the relative velocity
#endif
glm::mat2x4 M_relp; // relative position input
#ifdef NORM_REL
glm::vec4 M_distp; // distance of the relative position of neighbor
#endif
glm::vec4 M_dist; // goal distance input

std::string data_dir;

void init(std::string data_dir_) {
    comm::data_dir = data_dir_;
#ifndef NO_COMM
    std::string config_file = comm::data_dir + "/comms.config";
    std::string config_str = *read_file(config_file);

    std::stringstream ss(config_str);
    std::string line;
    std::getline(ss, line, '\n');
    for (int row = 0; std::getline(ss, line, '\n'); ++row) {
        std::stringstream sl(line);
        std::string item;
        for (int col = 0; std::getline(sl, item, ' '); ++col) {
            if (col == 0 || col == 1)
                M_c[col][row] = std::stof(item);
            else if (col == 2 || col == 3)
                M_relv[col - 2][row] = std::stof(item);
#    ifdef NORM_REL
            else if (col == 4)
                M_magv[row] = std::stof(item);
            else if (col == 5 || col == 6)
                M_relp[col - 5][row] = std::stof(item);
            else if (col == 7)
                M_distp[row] = std::stof(item);
#    else
            else if (col == 4 || col == 5)
                M_relp[col - 4][row] = std::stof(item);
#    endif
            else
                M_dist[row] = std::stof(item);
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

void run() {
    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        // if, for some reason, a nearest neighbor is not found, treat them as
        // silent, not moving, and not distant.
        glm::vec2 closest_pos{0, 0};
        glm::vec2 closest_vel{0, 0};
        glm::vec2 closest_c{0, 0};
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        float dist2 = std::numeric_limits<float>::max();

        // this is *really* inefficient, but oh well.
        POOL.for_<CommComp>([&](CommComp& o_c, Entity& e_other) {
            if (e_c.id == e_other.id)
                return;

            Dynamics* d_other = POOL.get<Dynamics>(e_other);
            glm::vec3 diff = d_other->pos - d.pos;
            float dot = glm::dot(diff, diff);
            if (dot < dist2) {
                dist2 = dot;
                closest_pos = d_other->pos;
                closest_vel = d_other->vel;
                closest_c = o_c.c;
            }
        });

        // w.r.t. local coordinate frame
        glm::vec2 v_front = c.facing;
        glm::vec2 v_right = c.right();

        // this assumes v_forward/v_right are normalized
        glm::vec2 diff_v = closest_vel - glm::vec2(d.vel.x, d.vel.z);
        glm::vec2 rel_v(glm::dot(v_right, diff_v), glm::dot(v_front, diff_v));

#if defined(COSINE_REL) || defined(NORM_REL)
        float mag_rv = glm::length(rel_v);
        if (mag_rv > 0) {
            rel_v /= mag_rv;
        }
#    ifdef COSINE_REL
        float dot_rv = rel_v.y;
        rel_v = glm::vec2(dot_rv, mag_rv);
#    endif
#endif

        glm::vec2 diff_p = closest_pos - glm::vec2(d.pos.x, d.pos.z);
        glm::vec2 rel_p(glm::dot(v_right, diff_p), glm::dot(v_front, diff_p));
#if defined(COSINE_REL) || defined(NORM_REL)
        float mag_rp = glm::length(rel_p);
        if (mag_rp > 0) {
            rel_p /= mag_rp;
#    ifdef PROX
            mag_rp = 1 / mag_rp;
#    endif
        }
#    ifdef COSINE_REL
        float dot_rp = rel_p.y;
        rel_p = glm::vec2(dot_rp, mag_rp);
#    endif
#endif

        Agent& a = *POOL.get<Agent>(e_c);
        // TODO: compute this where I INTENTIONALLY ignore the 0 block
        c.buf_in(
            M_c * closest_c //
            + M_relv * rel_v //
#ifdef NORM_REL
            + M_magv * mag_rv
#endif
            + M_relp * rel_p //
            + M_dist * a.goal_dist
#ifdef SOFTSIGN_GOAL
                / (1 + a.goal_dist)
#endif
#ifdef NORM_REL
            + M_distp * mag_rp //
#endif
        );
    });

    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        glm::vec2 diff = c.facing * c.v_buf.y + c.right() * c.v_buf.x;

        Agent* a = POOL.get<Agent>(e_c);
        if (a && !a->done())
            d.force += glm::vec3(diff.x, 0.f, diff.y);

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
