#include "ai.h"
#include "Pool.h"
#include "model/CubeMesh.h"
#include "model/LineMesh.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#undef GLM_ENABLE_EXPERIMENTAL

namespace ai {
Cspace2d* std_cspace;
PRM* std_prm;
BVH* static_bvh;
BVH* dynamic_bvh;

static void rebuild_sbvh(std::vector<Entity*> statics) {
    static_bvh = new BVH(statics);
}
static void rebuild_dbvh(std::vector<Entity*> dynamics) {
    dynamic_bvh = new BVH(dynamics);
}

//todo: figure out how to handle varying size Cspace
void init() {
    static const float root2 = static_cast<float>(sqrt(2));
    std::vector<Entity*> statics;
    std::vector<Entity*> dynamics;
    POOL.for_<BoundVolume*>([&](BoundVolume*&, const Entity& e) {
        if (POOL.get<Agent>(e) && POOL.get<Dynamics>(e)) {
            dynamics.push_back(&const_cast<Entity&>(e));
        } else {
            statics.push_back(&const_cast<Entity&>(e));
        }
    });

    rebuild_sbvh(statics);
    rebuild_dbvh(dynamics);

    if (dynamics.size() > static_cast<size_t>(0)) {
        //cspace
        std::vector<BoundVolume*> obs_bv;
        for (Entity* s : statics) {
            obs_bv.push_back(*POOL.get<BoundVolume*>(*s));
        }
        std::unique_ptr<Cspace2d> cs(
            new Cspace2d(obs_bv, &**POOL.get<BoundVolume*>(*dynamics[0])));
        std_cspace = cs.get();

        //prm
        glm::vec2 center_2d(0, 0);//org
        glm::vec2 dim(50/2, 50/2);//w,h
        dim *= 1;//cellsize
        std_prm = new PRM(
            std::move(cs),
            root2*6.f,
            0.f,
            glm::vec2(3.f, 3.f),
            2,
            center_2d - dim,
            center_2d + dim,
            1.f);//1.f

        #ifdef PRM_DEBUG
        auto& rm = std_prm->_roadmap;
        std::vector<Vertex> endpoints(rm->vertex_num());
        std::vector<GLuint> lines(2 * rm->edge_num());

        rm->for_vertex([&](NodeId u) {
            Entity& v = POOL.spawn_entity();
            uint16_t tid = POOL.create<Transform>(Transform(nullptr));
            std::vector<Texture> tex = {};
            uint16_t mid = POOL.create<Mesh>(CubeMesh(tex));
            glm::vec2 v_pos = *rm->data(u);
            auto& t = *POOL.get<Transform>(tid);
            glm::mat4 scale(.3f);
            scale[3][3] = 1.f;
            t.set_mat(scale);
            t.set_pos(glm::vec3(v_pos.x, 0, v_pos.y));
            POOL.attach<Transform>(v, tid);
            POOL.attach<Mesh>(v, mid);

            glm::vec3 pos(v_pos.x, 0, v_pos.y);
            Vertex end;
            end.pos = pos;
            end.norm = glm::vec3(0);
            end.tex = glm::vec2(0);
            endpoints[u] = end;
        });

        rm->for_edge([&](NodeId u, NodeId v) {
            lines.push_back(u);
            lines.push_back(v);
        });
        {
            Entity& debug_map = POOL.spawn_entity();
            uint16_t mid = POOL.create<Mesh>(LineMesh(endpoints, lines));
            POOL.attach<Mesh>(debug_map, mid);
        }
        #endif

        //planners
        POOL.for_<Agent>([&](Agent& a, Entity&) {
            a.num_done = 0;
            a.cspace = std_cspace;
            a.prm = std_prm;
            a.local_goal = glm::vec2(0, 0);
            a.plan = nullptr;

            GMP::plan_one(a);
        });
    }
}

//move to AI/planner --- this is a force-based LMP
void update_agents() {
    std::vector<Entity*> statics;
    std::vector<Entity*> dynamics;
    POOL.for_<BoundVolume*>([&](BoundVolume*&, const Entity& e) {
        if (POOL.get<Agent>(e) && POOL.get<Dynamics>(e)) {
            dynamics.push_back(&const_cast<Entity&>(e));
        } else {
            statics.push_back(&const_cast<Entity&>(e));
        }
    });

    //probably very slow
    delete static_bvh;
    rebuild_sbvh(statics);
    //TODO: fix PRM

    delete dynamic_bvh;
    rebuild_dbvh(dynamics);

    POOL.for_<Agent>([&](Agent& a, Entity& e){
        float dist = glm::length2(a.final_goal - a.start);
        BoundVolume* bv = *POOL.get<BoundVolume*>(e);
        size_t next = static_cast<size_t>(a.num_done);
        bool visible_next = a.cspace->line_of_sight(bv->_o, (*a.plan)[next]);
        if (!a.has_plan()
            || (a.done() && dist > 1.f)
            // TODO: fix Lookahead/LoS checks instead of a.plan->size() > 1
            || (!a.done() && !visible_next && a.plan->size() > 1)
        ) {
            GMP::plan_one(a);
        }

        glm::vec2 f2d = LMP::calc_sum_force(&e, static_bvh, dynamic_bvh,
            statics, dynamics);
        POOL.get<Dynamics>(e)->force += glm::vec3(f2d.x, 0, f2d.y);
    });
}
}//
