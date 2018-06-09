#include "physics.h"
#include "ai.h"
#include "Pool.h"
#include "util/debug.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#undef GLM_ENABLE_EXPERIMENTAL

namespace physics {
float avg_velocity;
uint64_t vel_count;

void init() {}

void simulate(float dt) {
    POOL.for_<Dynamics>([&](Dynamics& d, const Entity& e) {
        //halp-step integration, I think....
        glm::vec3 next_a = d.force/d.mass;

        glm::vec3 half_a = (d.acc + next_a) * .5f;
        glm::vec3 next_v = d.vel + dt * half_a;

        glm::vec3 half_v = (next_v + d.vel) * .5f;
        //glm::vec3 half_v = d.vel + (dt * .5f) * d.acc;
        if (glm::length2(half_v) > 1.f) {
            half_v = glm::normalize(half_v);
            next_v = glm::normalize(next_v);
            next_a = glm::vec3(0);
        }
        glm::vec3 next_p = d.pos + dt * (half_v);

        //detect collisions; no momentum, also not CCD, sadly. Extrusion would
        //be very expensive
        BoundVolume** bv = POOL.get<BoundVolume*>(e);
        if (bv) {
            (*bv)->_o = glm::vec2(next_p.x, next_p.z);

            std::vector<Entity*> in_dyn = ai::dynamic_bvh->query(*bv);
            std::vector<Entity*> in_st = ai::static_bvh->query(*bv);

            //collision! undo the motion at the collision!
            if (in_dyn.size() > 1 || in_st.size() > 0) {
                //next_p = d.pos - dt * half_v;
                //next_v = -next_v;
                //next_a = -next_a;
            }
        }

        d.pos = next_p;
        d.vel = next_v;
        Agent* a = POOL.get<Agent>(e);
        if (a && !a->done()) {
            avg_velocity = (avg_velocity * static_cast<float>(vel_count)
                + glm::length(d.vel))
                / (static_cast<float>(vel_count) + 1.f);
        }
        d.acc = next_a;
        d.force = glm::vec3(0);
    });
}
}
