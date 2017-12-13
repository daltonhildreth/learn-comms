#ifndef AI_H
#define AI_H

#include "Agent.h"
#include "PRM.h"
#include "BoundVolume.h"
#include "GMP.h"
#include "LMP.h"
#include "Dynamics.h"
#include "BVH.h"

namespace ai {
    extern Cspace2d* std_cspace;
    extern PRM* std_prm;
    extern BVH* static_bvh;
    extern BVH* dynamic_bvh;
    void init();
    void update_agents();
}

#endif//AI_H
