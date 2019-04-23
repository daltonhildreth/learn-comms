#ifdef WIN32
// avoid including <windows.h> in glad
#    define APIENTRY __stdcall
#endif
#include <glad.h>
#ifdef _WINDOWS_
#    error windows.h was included
#endif
#ifdef WIN32
#    undef near
#    undef far
#    define NOMINMAX
#endif
#include "Pool.h"
#include "ai/ai.h"
#include "ai/physics.h" // TOOD: move physics into its own directory :p
#include "comms/comm.h"
#include "demo/demo.h"
#include "io.h"
#include "render.h"
#include "ui.h"
#include "util/Seeder.h"
#include "util/Timer.h"
#include "util/debug.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <glm/vec2.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <fstream>

using namespace std;

void prewarm_game(bool& all_done);

int main(int argc, char** argv) {
    // register CLI options
    std::string data_dir = "";
    unsigned split_offset = 0;
    uint64_t seed = 0;
    //unsigned c_func_id = comm::f_id::CLAMP;

    // register CLI positionals
    unsigned scene;

    // read CLI
    std::string prog(argv[0]);
    cli::parse(
        argc,
        argv,
        {
            {"help", cli::opt(nullptr)}, // uses hack
            {"record", cli::opt(&render::is_record)},
            {"data", cli::opt(nullptr, &data_dir, I(forward<string>), "dir")},
            {"split", cli::opt(nullptr, &split_offset, I(stoi), "n")},
            {"seed", cli::opt(nullptr, &seed, I(stoull), "n")},
            //{"comm_func", cli::opt(nullptr, &c_func_id, I(stoull), "f")},
        },
        {cli::pos(scene, I(stoi), "scene")}
    );

    // argument doing
    data_dir = std::string(PROJECT_DIR) + "data/" + data_dir;
    Seeder s;
    s.seed(seed);

    // system initialization
#ifndef NO_RENDER
    auto size = render::create_context_window(prog, split_offset);
#endif
    demo::init(scene, data_dir);
    // we still want to log times even on nocomm_render...
    comm::init(data_dir); //, c_func_id
    ai::init();
    physics::init();
#ifndef NO_RENDER
    ui::init_callbacks(render::window);
    render::init(size, data_dir);
#endif

    // main loop initialization
    Timer init_time;
    Timer frame_time;
    unsigned total_frames = 0;
    unsigned fps = 0;
    auto last_s = init_time.time();
    bool all_done = false;
    physics::prewarm(1.f / 60.f);
    POOL.all_sync();

    // main loop
    while (
        !all_done
#ifndef NO_RENDER
        && !glfwWindowShouldClose(render::window)
#endif
    ) {
#ifndef NO_RENDER
        ////UI: would iterate over controllers, but it just handles specific
        // entities for now
        ui::handle_input(render::window, frame_time.delta_s());
        if (ui::paused) {
            continue;
        }
#endif

        frame_time.tick();
        ++total_frames;
        ++fps;

        // FPS recorder
        if (1.f <= frame_time - last_s) {
            clog << "\nFPS: " << fps / (frame_time - last_s) << "\n";
            clog << "FRAMES: " << total_frames << "\n";
            clog << flush;
            fps = 0;
            last_s = frame_time.time();
        }

        double total_time = frame_time - init_time;
        // frame_time.delta_s()
        all_done = demo::run(1.f / 60.f, total_time, total_frames);

#ifndef NO_COMM
        comm::run();
#endif

        ////AI: iterates over agents, which often depend on boundvolumes,
        /// dynamics and transforms.
        ai::update_agents();

        ////Physics: iterates over dynamics, which often depend on boundvolumes,
        // and transforms.
        physics::simulate(1.f / 60.f); // (frame_time.delta_s()));

        ////sync: currently some components have redundant information that
        // needs to be synced every frame.
        POOL.all_sync();

#ifndef NO_RENDER
        ////Render: iterates over meshes, which often depend on transforms.
        render::draw();
#endif
    }

    demo::terminate();
    comm::terminate(); // we still want to log times even on nocomm_render...
    ai::terminate();
#ifndef NO_RENDER
    render::terminate();
#endif
    return EXIT_SUCCESS;
}
