#include "demo.h"
#include "model/CubeMesh.h"
#include "light/PointLight.h"
#include "light/DirLight.h"
#include "util/debug.h"
#include "Pool.h"
#include "render.h"
#include "util/Seeder.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>

#include <iostream>
#include <string>
#include <vector>

using namespace std;

namespace demo {
const int SCENE = 7;
//0: circle radius 10; 30 agents; 30s
//1: circle radius 20; 90 agents; 70s
//2: circle radius 10; 30 agents; 40 .1m x .1m posts in radius 8; 40s
//3: circle radius 10; 30 agents; 1 1m radius pillar at center; 60s
//4: intersection 14 wide; 112 agents; 40s
//5: intersection 14 wide; 112 agents; swapped goals (-pos); 70s
//6: intersection 14 wide; 112 agents; curbed; 70s

//7: clogged doorway with 5 running in, and 25 escaping.

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

constexpr unsigned num_robos() {
    switch (SCENE) {
    case 0:
    case 2:
    case 3:
    case 7:
        return 60;
    case 1:
        return 60;
    case 4:
    case 5:
    case 6:
        return 56;
    }
}

constexpr static unsigned num_walls() {
    switch (SCENE) {
        case 0:
        case 1:
        case 4:
        case 5:
            return 0;
        case 2:
            return 60;
        case 3:
            return 1;
        case 6:
            return 13;
        case 7:
            return 30;
    }
}

constexpr unsigned NUM_ROBOS = num_robos();
constexpr unsigned NUM_WALLS = num_walls();

static BoundVolume* set_wall_size(glm::vec2 pos) {
    switch (SCENE) {
    case 0:
    case 1:
    case 4:
    case 5:
        return nullptr;
    case 2:
        return new Rect(pos, .1f, .1f);
    case 3:
        return new Rect(pos, 4.f, 4.f);
    case 6:
    case 7:
        return new Rect(pos, 2.f, 2.f);
    }
}

static float set_wall_scale() {
    switch (SCENE) {
    case 0:
    case 1:
    case 4:
    case 5:
        return 0;
    case 2:
        return .1f;
    case 3:
        return 4.f;
    case 6:
    case 7:
        return 2.f;
    }
}

static void set_goal_scene(glm::vec2 &goal, glm::vec2 pos) {
    switch (SCENE) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 5: {
        goal = -glm::vec2(pos.x, pos.y);
        break;
    }
    case 4:
    case 6: {
        int axis = sgn<float>(abs(pos.x) - abs(pos.y));
        axis = (axis == 0 ? 1 : axis);
        float faxis = static_cast<float>(axis);
        goal = glm::vec2(-faxis * pos.x, faxis * pos.y);
        break;
    }
    case 7: {
        goal = glm::vec2(-pos.x, pos.y);
        break;
    }
    }
}

static glm::vec2 agents_pos(unsigned i) {
    float rad = static_cast<float>(i)/static_cast<float>(NUM_ROBOS)
        * (2.f * glm::pi<float>());
    switch (SCENE) {
    case 0:
    case 2:
    case 3:
        return 10.f * glm::vec2(cos(rad), sin(rad));
    case 1:
        return 20.f * glm::vec2(cos(rad), sin(rad));
    case 4:
    case 5:
    case 6: {
        float side = (i%2 ? -1 : 1);
        unsigned k = (i-i%2) / 14;
        if (i >= NUM_ROBOS/2) {
            k = (i - i%2 - NUM_ROBOS/2) / 14;
        }
        unsigned j = (i-i%2) % 14;
        float x = side * (static_cast<float>(k) + 7);
        float y = static_cast<float>(j)/2.f - 3;
        if (i >= NUM_ROBOS/2) {
            float t = x;
            x = y;
            y = t;
        }
        return glm::vec2(x, y);
    }
    case 7: {
        unsigned col = i % 5;
        unsigned row = i / 5;
        float side = (row < 1 ? -3.f : 3.f);
        float x = static_cast<float>(row) * 1.f + side;
        float y = static_cast<float>(col) * 1.f - 2.f;
        return glm::vec2(x, y);
    }
    }
}

static glm::vec2 set_walls_pos(unsigned i) {
    switch (SCENE) {
    case 0:
    case 1:
    case 4:
    case 5: {
        return glm::vec2(1000.f);
    }
    case 2: {
        Seeder s;
        typedef uniform_real_distribution<float> UFD;
        UFD map(-7.f, 7.f);
        return glm::vec2(map(s.gen()), map(s.gen()));
    }
    case 3:
        return glm::vec2(0.f);
    case 6:
        switch (i) {
        case 0: return glm::vec2(-3.f, -3.f);
        case 11: return glm::vec2(-4.f, -4.f);
        case 12: return glm::vec2(-6.f, -5.f);

        case 1: return glm::vec2(-3.f, 3.f);
        case 9: return glm::vec2(-4.f, 4.f);
        case 10: return glm::vec2(-6.f, 5.f);

        case 2: return glm::vec2(3.f, -3.f);
        case 6: return glm::vec2(4.f, -4.f);
        case 7: return glm::vec2(6.f, -5.f);

        case 3: return glm::vec2(3.f, 3.f);
        case 4: return glm::vec2(4.f, 4.f);
        case 8: return glm::vec2(6.f, 5.f);
        }
    case 7:
        float x = 0;
        float y = 2 * (static_cast<float>(i) - 15) + 1.f * (i > 14);
        return glm::vec2(x, y);
    }
    return glm::vec2(1000.f);
}

static float set_demo_length() {
    switch (SCENE) {
    case 0:
        return 30.f;
    case 2:
    case 3:
    case 4:
    case 6:
        return 40.f;
    case 1:
    case 5:
    case 7:
        return 70.f;
    }
}


static void create_floor(Entity& e, vector<Texture> texs) {
    #ifdef NO_RENDER
    UNUSED(texs);
    #endif
    uint16_t tid = POOL.create<Transform>(Transform(nullptr));
    #ifndef NO_RENDER
    uint16_t mid = POOL.create<Mesh>(CubeMesh(texs));
    #endif
    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(50.f);
    scale[1][1] = 1.f;
    scale[3][3] = 1.f;
    t.set_mat(scale);
    t.set_pos(glm::vec3(0, -1, 0));

    POOL.attach<Transform>(e, tid);
    #ifndef NO_RENDER
    POOL.attach<Mesh>(e, mid);
    #endif
}

static void create_wall(Entity& e, vector<Texture> texs, glm::vec2 pos) {
    #ifdef NO_RENDER
    UNUSED(texs);
    #endif
    uint16_t tid = POOL.create<Transform>(Transform(nullptr));
    #ifndef NO_RENDER
    uint16_t mid = POOL.create<Mesh>(CubeMesh(texs));
    #endif
    uint16_t bvid = POOL.create<BoundVolume*>(set_wall_size(pos));

    Seeder s;
    typedef uniform_real_distribution<float> UFD;
    UFD tall(1.f, 3.f);

    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(set_wall_scale());
    scale[1][1] = tall(s.gen());
    scale[3][3] = 1.f;
    t.set_mat(scale);
    t.set_pos(glm::vec3(pos.x, 0, pos.y));

    POOL.attach<Transform>(e, tid);
    #ifndef NO_RENDER
    POOL.attach<Mesh>(e, mid);
    #endif
    POOL.attach<BoundVolume*>(e, bvid);
}

static void create_robo(Entity& e, vector<Texture> texs, glm::vec2 pos) {
    #ifdef NO_RENDER
    UNUSED(texs);
    #endif
    uint16_t tid = POOL.create<Transform>(Transform(nullptr));
    #ifndef NO_RENDER
    uint16_t mid = POOL.create<Mesh>(CubeMesh(texs));
    #endif
    uint16_t did = POOL.create<Dynamics>(Dynamics());
    uint16_t bvid = POOL.create<BoundVolume*>(new Rect(pos, .3f, .3f));
    uint16_t aid = POOL.create<Agent>(Agent());
    uint16_t cid = POOL.create<CommComp>(CommComp());

    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(.3f);
    scale[1][1] = 1.f;
    scale[3][3] = 1.f;
    t.set_mat(scale);

    auto& d = *POOL.get<Dynamics>(did);
    d.pos = glm::vec3(pos.x, 0, pos.y);
    d.mass = 1.f;

    auto& a = *POOL.get<Agent>(aid);
    set_goal_scene(a.final_goal, pos);

    auto& c = *POOL.get<CommComp>(cid);
    c.c = 0;
    c.c_buf = 0;
    c.v_buf = glm::vec2(0);
    c.facing = a.final_goal - glm::vec2(d.pos.x, d.pos.z);

    POOL.attach<Transform>(e, tid);
    #ifndef NO_RENDER
    POOL.attach<Mesh>(e, mid);
    #endif
    POOL.attach<Dynamics>(e, did);
    POOL.attach<BoundVolume*>(e, bvid);
    POOL.attach<Agent>(e, aid);
    POOL.attach<CommComp>(e, cid);
}

void init() {
    string pwd(PROJECT_SRC_DIR);
    //so, normally I'd only want one mesh shared amongst many entities, but the
    //renderer does a for_<Mesh> so I can't do that.
    vector<Texture> robo_tex = {
    };
    vector<Texture> floor_tex = {
    #ifndef NO_RENDER
        {render::create_tex(pwd + "/res/stone.jpg"), Texmap::diffuse}
    #endif
    };
    vector<Texture> wall_tex = {
    #ifndef NO_RENDER
        {render::create_tex(pwd + "/res/container2.png"), Texmap::diffuse},
        {render::create_tex(pwd + "/res/container2_specular.png"), Texmap::specular}
    #endif
    };

    Entity& floors = POOL.spawn_entity();
    create_floor(floors, floor_tex);

    for (unsigned i = 0; i < NUM_ROBOS; ++i) {
        create_robo(POOL.spawn_entity(), robo_tex, agents_pos(i));
    }

    #pragma GCC diagnostic ignored "-Wtype-limits"
    for (unsigned i = 0; i < NUM_WALLS; ++i) {
        create_wall(POOL.spawn_entity(), wall_tex, set_walls_pos(i));
    }

    #ifndef NO_RENDER
    {
        render::dir_lights.push_back(make_unique<DirLight>());
        render::dir_lights.back()->dir(glm::vec3(-1, -2, -1));
        render::dir_lights.back()->ambient(glm::vec3(.1f));
        render::dir_lights.back()->diffuse(glm::vec3(.5f));
        render::dir_lights.back()->specular(glm::vec3(1.f));
    }

    Seeder s;
    typedef uniform_real_distribution<float> UFD;
    UFD y_dist(3, 10);
    UFD tweak(-.3f, .3f);
    UFD map(-25.f, 25.f);
    for (unsigned i = 0; i < 8; ++i) {
        render::point_lights.push_back(make_unique<PointLight>());
        render::point_lights.back()->pos(glm::vec3(
            map(s.gen()), y_dist(s.gen()), map(s.gen())));
        render::point_lights.back()->att_to_dist(1000);
        render::point_lights.back()->ambient(glm::vec3(0.f));
        render::point_lights.back()->diffuse(glm::vec3(.5f)
            + glm::vec3(tweak(s.gen()), tweak(s.gen()), tweak(s.gen())));
        render::point_lights.back()->specular(glm::vec3(.3f));
    }
    #endif
}

float run_avg_time = 0;
float run_std_time = 0;
float run_sum_times_sq = 0;
int total_num_done = 0;

static void update_runs(float time) {
    float ftotal = static_cast<float>(++total_num_done);
    run_avg_time += (time - run_avg_time) / ftotal;
    run_sum_times_sq += time * time;
    run_std_time = sqrt(run_sum_times_sq * ftotal - run_avg_time * run_avg_time) / ftotal;
}

bool run(double dt, double time, unsigned frame_count) {
    UNUSED(dt);
    bool all_done = true;
    int num_done = 0;
    static double last_s = 0;
    POOL.for_<Agent>([&](Agent& ai, Entity&) {
        if (!ai.done()) {
            all_done = false;
        } else {
            ++num_done;
        }
    });
    if (time - last_s >= 1) {
        std::cout << "NUM DONE: " << num_done << "\n";
        std::cout << "FRAMES: " << frame_count << "\n";
        std::cout.flush();
        last_s = time;
    }
    for (int i = total_num_done; i < num_done; ++i, ++total_num_done) {
         update_runs(time);
    }
    if (time > set_demo_length()) {
        POOL.for_<Agent>([&](Agent& ai, Entity&){
            if (!ai.done()) {
                float finish_time = 2*glm::length(ai.final_goal - ai.start) + time;
                update_runs(finish_time);
            }
        });
        all_done = true;
    }
    return all_done;
}
}
