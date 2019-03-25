#include "physics.h"
#include "Pool.h"
#include "ai.h"
#include "util/debug.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#undef GLM_ENABLE_EXPERIMENTAL

namespace physics {
float avg_velocity;
uint64_t vel_count;

void init() {}

void prewarm(float dt) {
    // half-step leap-frog
    POOL.for_<Dynamics>([&](Dynamics& d, const Entity&) {
        d.acc = d.force / d.mass;
        d.vel_forehalf = d.vel + dt / 2.f * d.acc;
    });
}

static void push_away(Dynamics& d, glm::vec2 collision) {
    glm::vec3 away(collision.x, 0, collision.y);
    printf("%f %f\n", collision.x, collision.y);

    // move object out of collision with some buffer.
    if (glm::length2(away) > 0.f) {
        d.pos += away + 0.001f * glm::normalize(away);
    }

    // stop motion into object
    glm::vec3 vel_proj = away * glm::dot(away, d.vel_forehalf);
    d.vel_forehalf += vel_proj;
    glm::vec3 acc_proj = away * glm::dot(away, d.acc);
    d.acc += acc_proj;
}

void simulate(float dt) {
    POOL.for_<Dynamics>([&](Dynamics& d, const Entity& e) {
        // leap-frog
        d.vel_backhalf = d.vel_forehalf;
        d.acc = d.force / d.mass;
        d.vel_forehalf = d.vel_backhalf + dt * d.acc;
        // force velocity to max out at 1m/s
        if (glm::length2(d.vel_forehalf) > 1.f) {
            d.vel_forehalf = glm::normalize(d.vel_forehalf);
            d.acc = glm::vec3(0);
        }
        d.vel = d.vel_forehalf / 2.f + d.vel_backhalf / 2.f;
        d.pos += d.vel_forehalf * dt;
        d.force = glm::vec3(0);

        // detect collisions; no momentum
        BoundVolume** bv = POOL.get<BoundVolume*>(e);
        if (bv) {
            (*bv)->_o = glm::vec2(d.pos.x, d.pos.z);

            auto in_dyn = ai::dynamic_bvh->query(*bv);

            auto in_st = ai::static_bvh->query(*bv);

            // collision! undo the motion at the collision!
            for (auto coll : in_dyn) {
                push_away(d, coll.second);
            }

            for (auto coll : in_st) {
                push_away(d, coll.second);
            }
        }

        Agent* a = POOL.get<Agent>(e);
        if (a && !a->done()) {
            float v = static_cast<float>(vel_count);
            avg_velocity = (avg_velocity * v + glm::length(d.vel)) / (v + 1.f);
        }
    });
}
} // namespace physics
