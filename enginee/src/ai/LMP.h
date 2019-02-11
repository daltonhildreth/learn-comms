#pragma once
#include "Agent.h"
#include "BVH.h"
#include "BoundVolume.h"
#include "Dynamics.h"
#include "PRM.h"

#include <glm/glm.hpp>

namespace LMP {
float ttc(BoundVolume& i, glm::vec2 iv, BoundVolume& j, glm::vec2 jv);
float ttc_(Circ& i, glm::vec2 iv, Circ& j, glm::vec2 jv);
float ttc_(Rect& i, glm::vec2 iv, Rect& j, glm::vec2 jv);
float ttc_(Circ& i, glm::vec2 iv, Rect& j, glm::vec2 jv);

glm::vec2 lookahead(Agent& a, BoundVolume& bv);

glm::vec2 ttc_forces_(double ttc, glm::vec2 dir);
glm::vec2 ttc_forces(
    Dynamics& da,
    BoundVolume& bva,
    BoundVolume& bvb,
    float ttc
);
glm::vec2 ttc_forces(
    Dynamics& da,
    BoundVolume& bva,
    Dynamics& db,
    BoundVolume& bvb,
    float ttc
);
glm::vec2 calc_sum_force(
    Entity* e,
    BVH* static_bvh,
    BVH* dynamic_bvh,
    std::vector<Entity*> statics, // shouldn't be here; temp hack
    std::vector<Entity*> dynamics
); // shouldn't be here; temp hack
} // namespace LMP
