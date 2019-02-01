#include "comm.h"
#include "io.h"
#include "Pool.h"
#include <sstream>
#include <iostream>

namespace comm {
//one 4x7 matrix I split into four for easy multiplication later
glm::mat2x4 M_c; // c input/output
glm::mat2x4 M_relv; // relative velocity input
glm::mat2x4 M_relp; // relative position input
glm::vec4 M_dist; // relative distance input

/*
static void print_v3as2(glm::vec3 v, char c) {
    std::cout << c << " " << v.x << " "<<v.z << "\n";
}
static void print_v2(glm::vec2 v, char c) {
    std::cout << c << " " << v.x << " " << v.y << "\n";
}
static void print_v3(glm::vec3 v, char c) {
    std::cout << c << " " << v.x << " " << v.y << " " << v.z << "\n";
}
*/

void init() {
    std::string config_file = std::string(DATA_DIR) + "/comms.config";
    std::string config_str = *read_file(config_file);

    std::stringstream ss(config_str);
    std::string line;
    std::getline(ss, line, '\n');
    for (int row = 0; std::getline(ss, line, '\n'); ++row) {
        std::stringstream sl(line);
        std::string item;
        for (int col = 0; std::getline(sl, item, ' '); ++col) {
            if (col == 0)
                M_c[col][row] = std::stof(item);
            else if (col == 2 || col == 3)
                M_relv[col - 2][row] = std::stof(item);
            else if (col == 4 || col == 5)
                M_relp[col - 4][row] = std::stof(item);
            else
                M_dist[row] = std::stof(item);
        }
    }
}

void run() {
    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        Dynamics* closest_d = nullptr;
        CommComp* closest_c = nullptr;
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
                closest_d = d_other;
                closest_c = &o_c;
            }
        });

        glm::vec2 v_forward = closest_c->facing;
        glm::vec2 v_right = closest_c->right();

        // this assumes v_forward/v_right are normalized
        glm::vec2 diff_v = closest_d->vel - d.vel;
        glm::vec2 rel_v(glm::dot(v_right, diff_v), glm::dot(v_forward, diff_v));
        glm::vec2 diff_p = closest_d->pos - d.pos;
        glm::vec2 rel_p(glm::dot(v_right, diff_p), glm::dot(v_forward, diff_p));


        Agent& a = *POOL.get<Agent>(e_c);
        c.buf_in(
            M_c * closest_c->c
            + M_relv * rel_v
            + M_relp * rel_p
            + M_dist * a.goal_dist
        );
    });

    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        glm::vec2 diff = c.facing * c.v_buf.y + c.right() * c.v_buf.x;

        Agent* a = POOL.get<Agent>(e_c);
        if (a && !a->done())
            d.vel += glm::vec3(diff.x, 0.f, diff.y);

        if (glm::length(d.vel) > 0 && glm::length(a->local_goal - a->start) > 0) {
            c.facing = glm::normalize(a->local_goal - a->start);
        }
        c.swap();
    });
}

}
