#include "comm.h"
#include "io.h"
#include "Pool.h"
#include <sstream>
#include <iostream>

namespace comm {
//one 3x5 matrix I split into two for easy multiplication later
glm::vec3 M_c;
glm::mat2x3 M_relv;
glm::mat2x3 M_relp;

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
    std::cout << config_file << "\n";
    std::string config_str = *read_file(config_file);

    std::stringstream ss(config_str);
    std::string line;
    std::getline(ss, line, '\n');
    int row = 0;
    while (std::getline(ss, line, '\n')) {
        int col = 0;
        std::stringstream sl(line);
        std::string item;
        while (std::getline(sl, item, ' ')) {
            if (col == 0)
                M_c[row] = std::stof(item);
            else if (col == 1 || col == 2)
                M_relv[col-1][row] = std::stof(item);
            else
                M_relp[col-3][row] = std::stof(item);
            col++;
        }
        row++;
    }
}

void run() {
    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c){
        Dynamics* best_d = nullptr;
        CommComp* best_c = nullptr;
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        float dist2 = std::numeric_limits<float>::max();

        //this is inefficient, but oh well.
        POOL.for_<CommComp>([&](CommComp& o_c, Entity& e_other){
            if (e_c.id == e_other.id)
                return;
            Dynamics* d_other = POOL.get<Dynamics>(e_other);
            glm::vec3 diff = d_other->pos - d.pos;
            float dot = glm::dot(diff, diff);
            if (dot < dist2) {
                dist2 = dot;
                best_d = d_other;
                best_c = &o_c;
            }
        });

        float c_c = best_c->c;
        glm::vec2 vel_forward = best_c->facing;
        glm::vec2 vel_right = best_c->right();
        glm::vec2 diffv = best_d->vel - d.vel;
        glm::vec2 relv(glm::dot(vel_right, diffv), glm::dot(vel_forward, diffv));
        glm::vec2 diffp = best_d->pos - d.pos;
        glm::vec2 relp(glm::dot(vel_right, diffp), glm::dot(vel_forward, diffp));

        glm::vec3 result =
            M_c * c_c
            + M_relv * relv
            + M_relp * relp;
        for (int i =0; i < 3; ++i)
            result[i] = glm::clamp<float>(result[i], -1, 1);
        c.buf_in(result);
    });

    POOL.for_<CommComp>([&](CommComp& c, Entity& e_c) {
        Dynamics& d = *POOL.get<Dynamics>(e_c);
        glm::vec2 vel_forward = c.facing;

        glm::vec2 vel_right = c.right();
        glm::vec2 diff = vel_forward * c.v_buf.y + vel_right * c.v_buf.x;
        Agent* a = POOL.get<Agent>(e_c);
        if (a && !a->done())
            d.vel += glm::vec3(diff.x, 0.f, diff.y);
        //if (!a.done()) {
        if (glm::length(d.vel) > 0 && glm::length(a->local_goal - a->start) > 0) {
            c.facing = glm::normalize(a->local_goal - a->start);
                //glm::normalize(glm::vec2(d.vel.x, d.vel.z));
        }
        c.swap();
    });
}

}
