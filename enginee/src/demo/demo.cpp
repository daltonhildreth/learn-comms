#include "demo.h"
#include "Pool.h"
#include "light/DirLight.h"
#include "light/PointLight.h"
#include "model/CubeMesh.h"
#include "model/CylinderMesh.h"
#include "render.h"
#include "util/Seeder.h"
#include "util/debug.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

namespace demo {
std::ofstream paths;

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

static glm::vec2 opposite_goal(glm::vec2 pos, unsigned) { return -pos; }
static auto make_translate_goal(glm::vec2 t) {
    return [t](glm::vec2 pos, unsigned) { return pos + t; };
}
static glm::vec2 mirror_x_goal(glm::vec2 pos, unsigned) {
    return glm::vec2(-pos.x, pos.y);
}
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
        float radian = static_cast<float>(i) / static_cast<float>(num) * 2.f
            * glm::pi<float>();
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

template <size_t N>
static auto make_regimented_bots(std::array<Regiment, N> regs) {
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
static auto make_intersecting_bots(unsigned n_per, unsigned cols, float d) {
    std::array<Regiment, 4> regiments{
        center({{1, 0}, {-d, 0}, {1.f, 1.f}, cols, n_per}),
        center({{-1, 0}, {d, 0}, {1.f, 1.f}, cols, n_per}),
        center({{0, 1}, {0, -d}, {1.f, 1.f}, cols, n_per}),
        center({{0, -1}, {0, d}, {1.f, 1.f}, cols, n_per}),
    };
    return make_regimented_bots(regiments);
}
static glm::vec2 ignore_wall(unsigned) { return glm::vec2(1000.0f); }
static auto make_hole_wall(float gap, float scale) {
    return [gap, scale](unsigned i) {
        float x = 0;
        float side = ((i % 2) == 0 ? 1.f : -1.f);
        float height = static_cast<float>(i / 2);
        float y = side * (scale * (height + .5f) + gap / 2.f);
        return glm::vec2(x, y);
    };
}
static auto make_hall(float gap, float scale) {
    return [gap, scale](unsigned i) {
        float side = ((i % 2) == 0 ? 1.f : -1.f);
        float width = static_cast<float>((i + 2 % 4) / 4);
        float xside = (i % 4 < 2 ? 1.f : -1.f);
        float x = xside * scale * width;
        float y = side * .5f * (gap + scale);
        return glm::vec2(x, y);
    };
}

// TODO: compose/union/intersect

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

// TODO: replace with reading a file with data
static Scene make_scene(unsigned scn) {
    Scene s;

    switch (scn) {
    case 0: // circle radius 10; 30 agents; 45s
        s.num_robos = 60u;
        s.max_duration = 45.f;
        s.cam_dist = .85f;
        s.pos_of = make_radial_bots(s.num_robos, 10.f);
        s.goal_of = opposite_goal;
        break;

    case 1: // circle radius 20; 90 agents 70s
        s.num_robos = 90u;
        s.max_duration = 70.f;
        s.cam_dist = 1.1f;
        s.pos_of = make_radial_bots(s.num_robos, 13.f);
        s.goal_of = opposite_goal;
        break;

    case 2: // circle radius 10; 30 agent; 40 .3x.3m posts in radius 8; 40 s
        s.num_robos = 60u;
        s.num_walls = 60u;
        s.wall_scale = 0.3f;
        s.cam_dist = .85f;
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
        s.cam_dist = .85f;
        s.max_duration = 40.f;
        s.pos_of = make_radial_bots(s.num_robos, 10.f);
        s.goal_of = opposite_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned) { return glm::vec2(0.f); };
        break;

    case 4: // interesection 14 wide; 112 agents; 40s
        s.num_robos = 56u;
        s.max_duration = 40.f;
        s.cam_dist = .6f;
        s.pos_of = make_intersecting_bots(s.num_robos / 4, 7, 6.f);
        s.goal_of = axial_goal;
        break;

    case 5: // interesection 14 wide; 112 agents; swapped goals; 70s
        s.num_robos = 56u;
        s.max_duration = 70.f;
        s.cam_dist = .6f;
        s.pos_of = make_intersecting_bots(s.num_robos / 4, 7, 6.f);
        s.goal_of = opposite_goal;
        break;

    case 6: // intersection 14 wide; 112 agents; curbed; 70s
        s.num_robos = 56u;
        s.num_walls = 12u;
        s.wall_scale = 2.0f;
        s.max_duration = 40.f;
        s.cam_dist = .6f;
        s.pos_of = make_intersecting_bots(s.num_robos / 4, 7, 6.f);
        s.goal_of = axial_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned i) {
            switch (i) {
            case 0: return glm::vec2(-3.f, -3.f); // wall 1
            case 10: return glm::vec2(-4.f, -4.f);
            case 11: return glm::vec2(-6.f, -5.f);
            case 1: return glm::vec2(-3.f, 3.f); // wall 2
            case 8: return glm::vec2(-4.f, 4.f);
            case 9: return glm::vec2(-6.f, 5.f);
            case 2: return glm::vec2(3.f, -3.f); // wall 3
            case 5: return glm::vec2(4.f, -4.f);
            case 6: return glm::vec2(6.f, -5.f);
            case 3: return glm::vec2(3.f, 3.f); // wall 4
            case 4: return glm::vec2(4.f, 4.f);
            case 7: return glm::vec2(6.f, 5.f);
            default: assert(false);
            }
            exit(1);
        };
        break;

    case 7: // clogged doorway with 1 each way
        s.num_robos = 2u;
        s.num_walls = 30u;
        s.wall_scale = 2.0f;
        s.cam_dist = .3f;
        s.max_duration = 70.f;
        s.pos_of = [](unsigned i) {
            return (i == 0 ? glm::vec2(-3.f, 0) : glm::vec2(3.f, 0));
        };
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        break;

    case 8: { // clogged doorway qith 2 running in and 2 out
        s.num_robos = 4u;
        s.num_walls = 30u;
        s.wall_scale = 2.0f;
        s.cam_dist = .3f;
        s.max_duration = 30.f;
        std::array<Regiment, 2> r{
            center({{1, 0}, {-3, 0}, {1.f, 1.f}, 2, 2}),
            center({{-1, 0}, {3, 0}, {1.f, 1.f}, 2, 2}),
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
        s.cam_dist = .6f;
        s.max_duration = 100.f;
        std::array<Regiment, 2> r{
            center({{1, 0}, {-3, 0}, {1.f, 1.f}, 5, 20}),
            center({{-1, 0}, {3, 0}, {1.f, 1.f}, 5, 20}),
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
        s.cam_dist = .8f;
        s.max_duration = 100.f;
        std::array<Regiment, 2> r{
            center({{1, 0}, {-3, 0}, {1.f, 1.f}, 5, 5}),
            center({{-1, 0}, {3, 0}, {1.f, 1.f}, 5, 35}),
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        break;
    }

    // escaping room
    case 11: {
        s.num_robos = 70u;
        s.num_walls = 32u;
        s.wall_scale = 2.0f;
        s.max_duration = 100.0f;
        std::array<Regiment, 2> r{
            center({{0, 1}, {1, 6}, {2.f, 1.5f}, 3, 6}),
            center({{-1, 0}, {-4, -2}, {1.f, 1.f}, 8, 64}),
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = make_translate_goal({-20.f, 0.f});
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned i) -> glm::vec2 {
            switch (i) {
            case 0: return {0.f, 8.f}; // bottom
            case 1: return {2.f, 8.f};
            case 2: return {4.f, 8.f};
            case 3: return {8.f, 8.f};
            case 4: return {-6.f, 8.f};
            case 5: return {-4.f, 8.f};
            case 6: return {-2.f, 8.f};

            case 7: return {-8.f, 0.f}; // left
            case 8: return {-8.f, 2.f};
            case 9: return {-8.f, 4.f};
            case 10: return {-8.f, 6.f};
            case 11: return {-8.f, 8.f};
            case 12: return {-8.f, -6.f};
            case 13: return {-8.f, -4.f};
            case 14: return {-8.f, -2.f};

            case 15: return {-7.4f, -8.f}; // top
            case 16: return {1.4f, -8.f};
            case 17: return {3.4f, -8.f};
            case 18: return {5.4f, -8.f};
            case 19: return {-5.4f, -8.f};
            case 20: return {-3.4f, -8.f};
            case 21: return {-1.4f, -8.f};

            case 22: return {8.f, 0.f}; // right
            case 23: return {8.f, 2.f};
            case 24: return {8.f, 4.f};
            case 25: return {8.f, 6.f};
            case 26: return {7.4f, -8.f};
            case 27: return {8.f, -6.f};
            case 28: return {8.f, -4.f};
            case 29: return {8.f, -2.f};

            case 30: return {6.f, 3.f};
            case 31: return {4.f, 3.f};

            default: return {0.f, 0.f};
            }
        };
        break;
    }

    // dense crowd & groups
    case 12: {
        s.num_robos = 200u;
        s.num_walls = 0u;
        s.max_duration = 30.f;
        Seeder sd;
        typedef uniform_real_distribution<float> UFD;
        s.pos_of = [&sd](unsigned) {
            UFD inside(-10.f, 10.f);
            return glm::vec2(inside(sd.gen()), inside(sd.gen()));
        };
        s.goal_of = [&sd](glm::vec2, unsigned) {
            UFD inside(-10.f, 10.f);
            return glm::vec2(inside(sd.gen()), inside(sd.gen()));
        };
        break;
    }

    case 13: { // two tight formations of 20 going down a hall
        s.num_robos = 40u;
        s.num_walls = 18u;
        s.wall_scale = 2.0f;
        s.cam_dist = .6f;
        s.max_duration = 90.0f;
        std::array<Regiment, 2> r{
            center({{1, 0}, {-3, 0}, {.5f, .7f}, 5, 20}),
            center({{-1, 0}, {3, 0}, {.5f, .7f}, 5, 20}),
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = make_hall(3.f, s.wall_scale);
        break;
    }

    case 14: // staggered 1 on 1
        s.num_robos = 2u;
        s.cam_dist = .6f;
        s.max_duration = 30.f;
        s.pos_of = [](unsigned i) {
            return (i == 0 ? glm::vec2(-5.f, .1f) : glm::vec2(3.f, 0));
        };
        s.goal_of = mirror_x_goal;
        break;

    case 15: // staggered 1 on 2
        s.num_robos = 3u;
        s.cam_dist = .6f;
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
        s.cam_dist = .6f;
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
        s.cam_dist = .6f;
        s.max_duration = 45.f;
        std::array<Regiment, 2> r{
            center({{1, 0}, {-5, .1f}, {1.f, 1.f}, 2, 2}),
            center({{-1, 0}, {3, 0}, {1.f, 1.f}, 2, 2}),
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = mirror_x_goal;
        break;
    }

    case 18: { // two-door
        s.num_robos = 65u;
        s.num_walls = 20u;
        s.wall_scale = 2.0f;
        s.cam_dist = .9f;
        s.max_duration = 60.f;
        std::array<Regiment, 2> r{
            center({{-1, 0}, {3, 3}, {1.f, 1.f}, 5, 5}),
            center({{-1, 0}, {3, -6}, {1.f, 1.f}, 10, 60}),
        };
        s.pos_of = make_regimented_bots(r);
        s.goal_of = make_translate_goal({-15.f, 0.f});
        s.wall_shape_of = make_square_shape(s.wall_scale);
        s.wall_pos_of = [](unsigned i) {
            float y = 10.f;
            switch (i) {
            case 0: y = +1.0f; break;
            case 1: y = -1.0f; break;
            case 2: y = +3.0f; break;
            case 3: y = -3.0f; break;
            case 4: y = +6.0f; break;
            case 5: y = -5.0f; break;
            case 6: y = +8.0f; break;
            case 7: y = -8.0f; break;
            case 8: y = +10.0f; break;
            case 9: y = -10.0f; break;
            case 10: y = +12.0f; break;
            case 11: y = -12.0f; break;
            case 12: y = +14.0f; break;
            case 13: y = -14.0f; break;
            case 14: y = +16.0f; break;
            case 15: y = -16.0f; break;
            case 16: y = +18.0f; break;
            case 17: y = -18.0f; break;
            case 18: y = +20.0f; break;
            case 19: y = -20.0f; break;
            }
            return glm::vec2(0.f, y);
        };
        break;
    }

    default: { // variable num_robos version of s9
        if (scn > 100) {
            s.num_robos = 40u + (scn - 900);
            s.num_walls = 30u;
            s.wall_scale = 2.0f;
            s.cam_dist = .6f;
            s.max_duration = 100.f;
            unsigned even_size = s.num_robos / 2;
            unsigned odd_size = even_size + s.num_robos % 2;
            std::array<Regiment, 2> r{
                center({{1, 0}, {-3, 0}, {1.f, 1.f}, 5, odd_size}),
                center({{-1, 0}, {3, 0}, {1.f, 1.f}, 5, even_size}),
            };
            s.pos_of = make_regimented_bots(r);
            s.goal_of = mirror_x_goal;
            s.wall_shape_of = make_square_shape(s.wall_scale);
            s.wall_pos_of = make_hole_wall(1.f, s.wall_scale);
        }
    }
    }
    return s;
}

Scene scn;

static void create_wall(Entity& e, vector<Texture> texs, glm::vec2 pos) {
#ifdef NO_RENDER
    UNUSED(texs);
#endif
    uint16_t tid = POOL.create<Transform>(Transform(nullptr));
#ifndef NO_RENDER
    uint16_t mid = POOL.create<Mesh>(CubeMesh(texs));
#endif
    uint16_t bvid = POOL.create<BoundVolume*>(scn.wall_shape_of(pos, 0));

    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(scn.wall_scale);
    scale[1][1] = 1.f;
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
    float s = .3f;
#ifdef NO_RENDER
    UNUSED(texs);
#endif
    uint16_t tid = POOL.create<Transform>(Transform(nullptr));
#ifndef NO_RENDER
    uint16_t mid = POOL.create<Mesh>(CylinderMesh(texs, 18, .5f));
    // uint16_t mid = POOL.create<Mesh>(CubeMesh(texs));
#endif
    uint16_t did = POOL.create<Dynamics>(Dynamics());
    // uint16_t bvid = POOL.create<BoundVolume*>(new Rect(pos, s, s));
    uint16_t bvid = POOL.create<BoundVolume*>(new Circ(pos, s * .5f));
    uint16_t aid = POOL.create<Agent>(Agent());
#ifndef NO_COMM
    uint16_t cid = POOL.create<CommComp>(CommComp());
#endif

    auto& t = *POOL.get<Transform>(tid);
    glm::mat4 scale(s);
    scale[1][1] = 1.f;
    scale[3][3] = 1.f;
    t.set_mat(scale);

    auto& d = *POOL.get<Dynamics>(did);
    d.force = glm::vec3(0);
    d.acc = glm::vec3(0);
    d.vel = glm::vec3(0);
    d.pos = glm::vec3(pos.x, 0, pos.y);
    d.mass = 1.f;

    auto& a = *POOL.get<Agent>(aid);
    a.final_goal = scn.goal_of(pos, 0);
    a.start = pos;

#ifndef NO_COMM
    auto& c = *POOL.get<CommComp>(cid);
    c.c = glm::vec2(0);
    c.c_buf = glm::vec2(0);
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

void init(unsigned scn_i, std::string data) {
    string pwd(PROJECT_SRC_DIR);
    // TODO: open buffer to {this_scene}/{mode}.paths
    // id, groupID, x, y, vx, vy, radius, timeSTAMP

    paths = std::ofstream(data + "/agent_paths.csv", std::ios::binary);
    scn = make_scene(scn_i);

    // so, normally I'd only want one mesh shared amongst many entities, but the
    // renderer does a for_<Mesh> so I can't do that.
    vector<Texture> robo_tex = {};
    vector<Texture> wall_tex = {
#ifndef NO_RENDER
        {render::create_tex(pwd + "/res/outline.png"), Texmap::diffuse},
#endif
    };

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
        render::dir_lights.back()->dir(glm::vec3(0, -1, 0));
        render::dir_lights.back()->ambient(glm::vec3(1.f));
        render::dir_lights.back()->diffuse(glm::vec3(0.f));
        render::dir_lights.back()->specular(glm::vec3(0.f));
    }

    render::cam_dist = scn.cam_dist;
#endif
}

int total_num_done = 0;
float run_avg_time = 0;
float M2 = 0;
float run_std_time = 0;

static void update_runs(float overhead) {
    float ftotal = static_cast<float>(++total_num_done);
    float delta = overhead - run_avg_time;
    run_avg_time += delta / ftotal;
    float delta2 = overhead - run_avg_time;
    M2 += delta * delta2;
    run_std_time = sqrt(M2 / ftotal);
    printf("t:%f u:%f s:%f\n", overhead, run_avg_time, run_std_time);
}

bool run(double dt, double time, unsigned frame_count) {
    bool all_done = true;
    static double last_s = 0;
    float fake_time = static_cast<float>(dt) * static_cast<float>(frame_count);
    POOL.for_<Agent>([&](Agent& ai, Entity& e) {
        if (!ai.done()) {
            all_done = false;
        } else if (!ai.overhead_counted) {
            update_runs(fake_time - ai.min_time);
            ai.overhead_counted = true;
        }
        Dynamics& d = *POOL.get<Dynamics>(e);
        Transform& t = *POOL.get<Transform>(e);
        paths //
            << e.id << ",0," //
            << d.pos.x << "," << d.pos.z << "," //
            << d.vel.x << "," << d.vel.z << "," //
            << t.mat()[0][0] * .5f << "," // .5f for circles
            << static_cast<float>(dt) * static_cast<float>(frame_count - 1) //
            << "\n";
    });
    if (time - last_s >= 1) {
        std::clog << "NUM DONE: " << total_num_done << "\n";
        last_s = time;
    }

    if (fake_time > scn.max_duration) {
        POOL.for_<Agent>([&](Agent& ai, Entity&) {
            if (!ai.done()) {
                float finish_time = 2 * glm::length(ai.final_goal - ai.start)
                    + fake_time - ai.min_time;
                update_runs(finish_time);
            }
        });
        all_done = true;
    }
    return all_done;
}

void terminate() {
    POOL.for_<BoundVolume*>([&](BoundVolume*& bv, Entity&) {
        if (bv)
            delete bv;
    });
    paths.close();
}
} // namespace demo
