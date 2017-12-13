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
/*
static float find_collision_point(BoundVolume** mover_bv, glm::vec3 next_p,
        Entity* candidate) {
    BoundVolume** c_bv = POOL.get<BoundVolume*>(*candidate);
    //skip if same
    if (mover_bv == c_bv)
        return std::numeric_limits<float>::max();

    //if moving the complete distance would cause an intermediate collision,
    //only move so far as the collision point; no momentum because that's prone
    //to explosions...
    glm::vec2 next_p_2d(next_p.x, next_p.z);
    glm::vec2 motion(next_p_2d - (*mover_bv)->_o);
    float len2 = glm::dot(motion, motion);
    return (*c_bv)->line_of_sight((*mover_bv)->_o, next_p, motion, len2);
}
*/

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

        #ifdef STOP_COLLISION
        //detect collisions; no momentum, also not CCD, sadly. Extrusion would
        //be very expensive
        BoundVolume** bv = POOL.get<BoundVolume*>(e);
        if (bv) {
            (*bv)->_o = glm::vec2(next_p.x, next_p.z);
            //float first = std::numeric_limits<float>::max();

            std::vector<Entity*> in_dyn = ai::dynamic_bvh->query(*bv);
            //for (Entity* c : in_dyn) {
                //first = std::min(first, find_collision_point(bv, next_p, c));
            //}

            std::vector<Entity*> in_st = ai::static_bvh->query(*bv);
            //for (Entity* c : in_st) {

            //    first = std::min(first, find_collision_point(bv, next_p, c));
            //}

            //collision! stop the motion at the collision!
            //if (first < std::numeric_limits<float>::max()) {
            if (in_dyn.size() > 1 || in_st.size() > 0) {
                next_p = d.pos;
                //completely deflect motion to the RIGHT
                //next_v = glm::vec3(-next_v.z, next_v.y, next_v.x);
                //next_a = glm::vec3(-next_a.z, next_a.y, next_a.x);
            }
        }
        #else
        UNUSED(e);
        #endif

        d.pos = next_p;
        d.vel = next_v;
        avg_velocity = (avg_velocity * static_cast<float>(vel_count) + glm::length(d.vel))
            / (static_cast<float>(vel_count) + 1.f);
        d.acc = next_a;
        d.force = glm::vec3(0);
    });
}
}
