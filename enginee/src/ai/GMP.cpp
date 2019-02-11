#include "GMP.h"
//#include "AI.h"
#include <queue>
#include <tuple>

namespace GMP {
PointPath* find_path_astar(
    float e,
    Graph<glm::vec2>* roadmap,
    NodeId s,
    NodeId g
) {
    // for readability
    using namespace std;
    typedef glm::vec2 Point;
    typedef float Cost;
    typedef tuple<Point, Cost, NodeId> PQItem;
    Point start = *roadmap->data(s);
    Point goal = *roadmap->data(g);

    // init PQ
    // normally <
    auto cmp = [](PQItem l, PQItem r) {
        return std::get<1>(l) > std::get<1>(r);
    };
    std::priority_queue<PQItem, vector<PQItem>, decltype(cmp)> pq(cmp);
    pq.push(PQItem(start, 0.f, s));

    // init parent and g_cost (also used as a visited set)
    unordered_map<NodeId, NodeId> parent;
    unordered_map<NodeId, Cost> g_cost;
    parent[s] = s;
    g_cost[s] = 0.f;

    // while there are nodes to process, minimize cost to goal
    while (!pq.empty()) {
        PQItem item = pq.top();
        pq.pop();
        NodeId cur = std::get<2>(item);
        glm::vec2 cur_v = *roadmap->data(cur);

        // quit if we hit the goal, we have a path
        if (cur == g) {
            // reconstruct path to return
            PointPath* path = new PointPath();
            for (NodeId v = g; v != s; v = parent[v]) {
                path->insert(path->begin(), *roadmap->data(v));
            }
            return path;
        }

        // try each adjacency
        for (NodeId adj : roadmap->edges(cur)) {
            glm::vec2 adj_v = *roadmap->data(adj);
            // new potential path = path to here + edge-cost to adjacent
            float g_alt = g_cost[cur] + glm::distance(adj_v, cur_v);

            // if new path is better than current path to adjacent
            if (!g_cost.count(adj) || g_alt < g_cost[adj]) {
                g_cost[adj] = g_alt;
                float fcost = g_alt + e * glm::distance(goal, adj_v);
                // it's okay if we get multiple nodes on the PQ with diff fcost
                // whichever has the lowest will show up first and the later
                // ones will have no effect as they have the same g_cost
                pq.push(PQItem(adj_v, fcost, adj));
                parent[adj] = cur;
            }
        }
    }

    // g was not reached so a path was not found.
    return new PointPath();
}

PointPath* find_path_ucs(Graph<glm::vec2>* roadmap, NodeId start, NodeId goal) {
    return find_path_astar(0, roadmap, start, goal);
}

// just do an LoS over every node
static void connect_to_all(Graph<glm::vec2>* rm, NodeId v, Cspace2d* cspace) {
    rm->for_vertex([&](NodeId u) {
        if (v != u && cspace->line_of_sight(*rm->data(v), *rm->data(u))) {
            rm->add_edge(v, u);
        }
    });
}

void plan_one(Agent& a) {
    // this is to fix the one hack in A*
    // temporarily add to the roadmap the start/goal nodes of agent
    Graph<glm::vec2>* rm = a.prm->_roadmap.get();

    NodeId start = rm->add_vertex(a.start);
    NodeId goal = rm->add_vertex(a.final_goal);
    connect_to_all(rm, start, a.cspace);
    connect_to_all(rm, goal, a.cspace);

    // PATH PLANNING METHOD
    a.plan = find_path_astar(1.f, rm, start, goal);

    rm->del_vertex(start);
    rm->del_vertex(goal);

    a.num_done = 0;
}
} // namespace GMP
