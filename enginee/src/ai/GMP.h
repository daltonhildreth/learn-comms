#pragma once
#include "Agent.h"
#include "PRM.h"
#include <vector>

namespace GMP {
// custom A* search for a PRM graph. Note: I would like to generalize this to
// any graph & cost function.
PointPath* find_path_astar(
    float e,
    Graph<glm::vec2>* roadmap,
    NodeId start,
    NodeId goal
);
// Custom uniform cost search (aka Djikstra's search) for a PRM Graph
// simplifciation of A*. (h = 0)
PointPath* find_path_ucs(Graph<glm::vec2>* roadmap, NodeId start, NodeId goal);
// plan the given agent
void plan_one(Agent&);
} // namespace GMP
