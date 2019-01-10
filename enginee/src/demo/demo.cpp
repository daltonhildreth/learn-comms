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
#include <array>

using namespace std;

namespace demo {
template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

static glm::vec2 opposite_goal(glm::vec2 pos, unsigned) {return -pos;}
static glm::vec2 mirror_x_goal(glm::vec2 pos, unsigned) {return glm::vec2(-pos.x, pos.y);}
static glm::vec2 axial_goal(glm::vec2 pos, unsigned) {
    int axis = sgn<float>(abs(pos.x) - abs(pos.y));
    axis = (axis == 0 ? 1 : axis);
    float faxis = static_cast<float>(axis);
    return glm::vec2(-faxis * pos.x, faxis * pos.y);
}

static BoundVolume* null_shape(glm::vec2, unsigned) { return nullptr; }
static auto make_square_shape(float sz) {
    return [sz](glm::vec2 pos, unsigned) { return new Rect(pos, sz, sz); };
}

static auto make_radial_bots(unsigned num, float radius) {
    return [num, radius](unsigned i) {
        float radian = static_cast<float>(i)/static_cast<float>(num)
            * 2.f * glm::pi<float>();
        return radius * glm::vec2(cos(radian), sin(radian));
    };
}

struct Regiment {
    glm::vec2 facing;
    glm::vec2 head;
    glm::vec2 spacing;
    unsigned cols;
    unsigned n;
};
static glm::vec2 right_face(Regiment r) {
    return glm::cross(glm::vec3(r.facing, 0), glm::vec3(0, 0, 1));
}
static Regiment center(Regiment r) {
    float half = (r.spacing.x * static_cast<float>(r.cols - 1)) / 2.f;
    r.head -= right_face(r) * half;
    return r;
}

template <size_t N> static auto make_regimented_bots(std::array<Regiment, N> regs) {
    for (Regiment r : regs) {
        r.facing /= glm::length(r.facing);
    }

    return [regs](unsigned i) {
        unsigned current = 0;
        unsigned last = regs[current].n;
        unsigned j = i;
        while (j >= last) {
            j -= last;
            last = regs[++current].n;
        }

        unsigned col = j % regs[current].cols;
        unsigned row = j / regs[current].cols;
        float u = regs[current].spacing.x * static_cast<float>(col);
        float v = regs[current].spacing.y * static_cast<float>(row);
        glm::vec2 x = right_face(regs[current]) * u;
        glm::vec2 y = -regs[current].facing * v;
        return x + y + regs[current].head;
    };
}
static auto make_intersecting_bots(unsigned n_per, unsigned cols, float dist) {
    std::array<Regiment, 4> regiments{
        center({glm::vec2(1, 0), glm::vec2(-dist, 0), glm::vec2(1.f), cols, n_per}),
        center({glm::vec2(-1, 0), glm::vec2(dist, 0), glm::vec2(1.f), cols, n_per}),
        center({glm::vec2(0, 1), glm::vec2(0, -dist), glm::vec2(1.f), cols, n_per}),
        center({glm::vec2(0, -1), glm::vec2(0, dist), glm::vec2(1.f), cols, n_per})
    };
    return make_regimented_bots(regiments);
}
static glm::vec2 ignore_wall(unsigned) { return glm::vec2(1000.0f); }
static auto make_hole_wall(float gap, float scale) {
    return [gap, scale](unsigned i) {
        float x = 0;
        float side = ((i % 2) == 0 ? 1.f : -1.f);
        float height = static_cast<float>(i / 2);
        float y = side * (scale * (height + .5f) + gap/2.f);
        return glm::vec2(x, y);
    };
}

struct Scene {
    unsigned num_robos;
    unsigned num_walls = 0u;
    float wall_scale = 0.f;
    float max_duration;
    float cam_dist = 1.f;
    std::function<glm::vec2(unsigned)> pos_of;
    std::function<glm::vec2(glm::vec2, unsigned)> goal_of;
    std::function<BoundVolume*(glm::vec2, unsigned)> wall_shape_of = null_shape;
    std::function<glm::vec2(unsigned)> wall_pos_of = ignore_wall;
};

static Scene make_scene(unsigned scn) {
    Scene s;

    switch(scn) {
    case 0: // circle radius 10; 30 agents; 30s
        s.num_robos = 60u;
        s.max_duration = 30.f;
        s.pos_of = make_radial_bots(s.num_robos, 10.f);
        s.goal_of = opposite_goal;
        break;

    case 1: // circle radius 20; 90 agents 70s
        s.num_robos = 60u;
        s.max_duration = 70.f;
        s.cam_dist = 2.f;
        s.pos_of = make_radial_bots(s.num_robos, 20.f);
        s.goal_of = opposite_goal;
        break;

    case 2: // circle radius 10; 30 agent; 40 .3x.3m posts in radius 8; 40 s
        s.num_robos = 60u;
        s.num_walls = 60u;
        s.wall_scale = 0.3f;
        s.max_duration = 40.f;
        s.pos_of = make_radial_bots(s.num_robos, 10.f);
        s.goal_of = opposite_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned) {
            Seeder seed;
            uniform_real_distribution<float> map(-7.f, 7.f);
            return glm::vec2(map(seed.gen()), map(seed.gen()));
        };
        break;

    case 3: // circle radius 10; 30 agents 1 4x4m pillar at center; 60s
        s.num_robos = 60u;
        s.num_walls = 1u;
        s.wall_scale = 4.0f;
        s.max_duration = 40.f;
        s.pos_of = make_radial_bots(s.num_robos, 10.f);
        s.goal_of = opposite_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned) { return glm::vec2(0.f); };
        break;

    case 4: // interesection 14 wide; 112 agents; 40s
        s.num_robos = 56u;
        s.max_duration = 40.f;
        s.cam_dist = 1.2f;
        s.pos_of = make_intersecting_bots(s.num_robos/4, 7, 6.f);
        s.goal_of = axial_goal;
        break;

    case 5: // interesection 14 wide; 112 agents; swapped goals; 70s
        s.num_robos = 56u;
        s.max_duration = 70.f;
        s.cam_dist = 1.2f;
        s.pos_of = make_intersecting_bots(s.num_robos/4, 7, 6.f);
        s.goal_of = opposite_goal;
        break;

    case 6: //intersection 14 wide; 112 agents; curbed; 70s
        s.num_robos = 56u;
        s.num_walls = 12u;
        s.wall_scale = 2.0f;
        s.max_duration = 40.f;
        s.cam_dist = 1.2f;
        s.pos_of = make_intersecting_bots(s.num_robos/4, 7, 6.f);
        s.goal_of = axial_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned i) {
            switch (i) {
            case 0:  return glm::vec2(-3.f, -3.f); // wall 1
            case 10: return glm::vec2(-4.f, -4.f);
            case 11: return glm::vec2(-6.f, -5.f);
            case 1:  return glm::vec2(-3.f,  3.f); // wall 2
            case 8:  return glm::vec2(-4.f,  4.f);
            case 9:  return glm::vec2(-6.f,  5.f);
            case 2:  return glm::vec2( 3.f, -3.f); // wall 3
            case 5:  return glm::vec2( 4.f, -4.f);
            case 6:  return glm::vec2( 6.f, -5.f);
            case 3:  return glm::vec2( 3.f,  3.f); // wall 4
            case 4:  return glm::vec2( 4.f,  4.f);
            case 7:  return glm::vec2( 6.f,  5.f);
            default: assert(false);
            }
            exit(1);
        };
        break;

    case 7: // clogged doorway with 1 each way
        s.num_robos = 2u;
        s.num_walls = 30u;
        s.wall_scale = 2.0f;
        s.max_duration = 70.f;
        s.pos_of = [](unsigned i) {
            return (i==0 ? glm::vec2(-3.f, 0) : glm::vec2(3.f, 0));
        };
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        break;

    case 8: { // clogged doorway qith 2 running in and 2 out
        s.num_robos = 4u;
        s.num_walls = 30u;
        s.wall_scale = 2.0f;
        s.max_duration = 30.f;
        std::array<Regiment, 2> r{
            center({glm::vec2(1, 0), glm::vec2(-3, 0), glm::vec2(1.f, 1.f), 2, 2}),
            center({glm::vec2(-1, 0), glm::vec2(3, 0), glm::vec2(1.f, 1.f), 2, 2})
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        break;
    }

    case 9: { // clogged doorway with 20 running in, and 20 escaping.
        s.num_robos = 40u;
        s.num_walls = 30u;
        s.wall_scale = 2.0f;
        s.max_duration = 90.f;
        std::array<Regiment, 2> r{
            center({glm::vec2(1, 0), glm::vec2(-3, 0), glm::vec2(1.f, 1.f), 5, 20}),
            center({glm::vec2(-1, 0), glm::vec2(3, 0), glm::vec2(1.f, 1.f), 5, 20})
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        break;
    }

    case 10: { // clogged doorway with 5 running in, and 35 escaping
        s.num_robos = 40u;
        s.num_walls = 30u;
        s.wall_scale = 2.0f;
        s.max_duration = 60.f;
        std::array<Regiment, 2> r{
            center({glm::vec2(1, 0), glm::vec2(-3, 0), glm::vec2(1.f, 1.f), 5, 5}),
            center({glm::vec2(-1, 0), glm::vec2(3, 0), glm::vec2(1.f, 1.f), 5, 35})
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        break;
    }

    case 11: // escaping room
        assert(false);
//        s.num_robos = 40u;
//        s.num_walls = 120u;
//        s.wall_scale = 2.0f;
//        s.max_duration = ;
//        s.pos_of = ;
//        s.goal_of = ;
//        s.wall_shape_of = ;
//        s.wall_pos_of = ;
        break;

    case 12: // crowd
        assert(false);
 //       s.num_robos = 100u;
 //       s.num_walls = 0u;
 //       s.wall_scale = 0.0f;
 //       s.max_duration = ;
 //       s.pos_of = ;
 //       s.goal_of = ;
 //       s.wall_shape_of = ;
 //       s.wall_pos_of = ;
        break;

    case 13: // two tight formations of 20 going down a hall
        assert(false);
//        s.num_robos = 40u;
//        s.num_walls = 30u;
//        s.wall_scale = 2.0f;
//        s.max_duration = ;
//        s.pos_of = ;
//        s.goal_of = ;
//        s.wall_shape_of = ;
//        s.wall_pos_of = ;
        break;

    case 14: // staggered 1 on 1
        s.num_robos = 2u;
        s.max_duration = 30.f;
        s.pos_of = [](unsigned i) {
            return (i==0 ? glm::vec2(-5.f, .1f) : glm::vec2(3.f, 0));
        };
        s.goal_of = mirror_x_goal;
        break;

    case 15:  // staggered 1 on 2
        s.num_robos = 3u;
        s.max_duration = 40.f;
        s.pos_of = [](unsigned i) {
            switch (i) {
            case 0: return glm::vec2(-5.f, .1f);
            case 1: return glm::vec2(3.f, 0);
            case 2: return glm::vec2(7.f, 0);
            default: assert(false);
            }
            exit(1);
        };
        s.goal_of = mirror_x_goal;
        break;

    case 16: // staggered 2 on 2
        s.num_robos = 4u;
        s.max_duration = 40.f;
        s.pos_of = [](unsigned i) {
            switch (i) {
            case 0: return glm::vec2(-5.f, .1f);
            case 1: return glm::vec2(3.f, 0);
            case 2: return glm::vec2(7.f, 0);
            case 3: return glm::vec2(-2.f, .1f);
            default: assert(false);
            }
            exit(1);
        };
        s.goal_of = mirror_x_goal;
        break;

    case 17: { // staggered equal 2 on equal 2
        s.num_robos = 4u;
        s.max_duration = 45.f;
        std::array<Regiment, 2> r{
            center({glm::vec2(1, 0), glm::vec2(-5, .1f), glm::vec2(1.f), 2, 2}),
            center({glm::vec2(-1, 0), glm::vec2(3, 0), glm::vec2(1.f), 2, 2})
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        break;
    }
    }
    return s;
}

Scene scn;

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
    uint16_t bvid = POOL.create<BoundVolume*>(scn.wall_shape_of(pos, 0));

    Seeder s;
    typedef uniform_real_distribution<float> UFD;
    UFD tall(1.f, 3.f);

    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(scn.wall_scale);
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
    #ifndef NO_COMM
    uint16_t cid = POOL.create<CommComp>(CommComp());
    #endif

    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(.3f);
    scale[1][1] = 1.f;
    scale[3][3] = 1.f;
    t.set_mat(scale);

    auto& d = *POOL.get<Dynamics>(did);
    d.pos = glm::vec3(pos.x, 0, pos.y);
    d.mass = 1.f;

    auto& a = *POOL.get<Agent>(aid);
    a.final_goal = scn.goal_of(pos, 0);
    a.start = pos;

    #ifndef NO_COMM
    auto& c = *POOL.get<CommComp>(cid);
    c.c = 0;
    c.c_buf = 0;
    c.v_buf = glm::vec2(0);
    c.facing = a.final_goal - a.start;
    #endif

    POOL.attach<Transform>(e, tid);
    #ifndef NO_RENDER
    POOL.attach<Mesh>(e, mid);
    #endif
    POOL.attach<Dynamics>(e, did);
    POOL.attach<BoundVolume*>(e, bvid);
    POOL.attach<Agent>(e, aid);
    #ifndef NO_COMM
    POOL.attach<CommComp>(e, cid);
    #endif
}


void init(unsigned scn_i) {
    string pwd(PROJECT_SRC_DIR);

    scn = make_scene(scn_i);

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

    for (unsigned i = 0; i < scn.num_robos; ++i) {
        create_robo(POOL.spawn_entity(), robo_tex, scn.pos_of(i));
    }

    #pragma GCC diagnostic ignored "-Wtype-limits"
    for (unsigned i = 0; i < scn.num_walls; ++i) {
        create_wall(POOL.spawn_entity(), wall_tex, scn.wall_pos_of(i));
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

    render::cam_dist = scn.cam_dist;
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
         update_runs(static_cast<float>(time));
    }
    if (time > scn.max_duration) {
        POOL.for_<Agent>([&](Agent& ai, Entity&){
            if (!ai.done()) {
                float finish_time = 2*glm::length(ai.final_goal - ai.start)
                    + static_cast<float>(time);
                update_runs(finish_time);
            }
        });
        all_done = true;
    }
    return all_done;
}
}
