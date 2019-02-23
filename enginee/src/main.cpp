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
#include <any>
#include <cstdint>
#include <cstdlib>
#include <glm/vec2.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <fstream>

using namespace std;

void prewarm_game(bool& all_done, unsigned total_frames);
void glfw_error(int err, const char* msg);
void monitor_connect(GLFWmonitor*, int event);
void keymap_input(GLFWwindow*);

void cli_error(std::string usage) {
    fprintf(stderr, "%s\n", usage.c_str());
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {
    // register CLI options
    bool is_record = false;
    bool is_split = false;
    std::string data_dir = "";
    unsigned split_offset = 0;
    uint64_t seed = 0;

    // register CLI positionals
    unsigned scene;

    // read CLI
    std::string prog(argv[0]);
    cli::parse(
        argc,
        argv,
        {
            {"help", cli::opt(nullptr)}, // uses hack
            {"record", cli::opt(&is_record)},
            {"data", cli::opt(nullptr, &data_dir, I(forward<string>), "dir")},
            {"split", cli::opt(&is_split, &split_offset, I(stoi), "n")},
            {"seed", cli::opt(nullptr, &seed, I(stoull), "n")},
        },
        {cli::pos(scene, I(stoi), "scene")}
    );

    // argument doing
    data_dir = std::string(PROJECT_DIR) + "data/" + data_dir;
    if (is_split) {
        ui::paused = false;
    }
    Seeder s;
    s.seed(seed);

    // create context, window, viewport
#ifndef NO_RENDER
    // Setup pre-init glfw
    glfwSetErrorCallback(glfw_error);

    // initialize glfw, output version.
    if (!glfwInit()) {
        cerr << "gg! Failed to init glfw; exiting.\n";
        return EXIT_FAILURE;
    } else {
        clog //
            << "gg. GLFW compiled as v" //
            << GLFW_VERSION_MAJOR << "." //
            << GLFW_VERSION_MINOR << "." //
            << GLFW_VERSION_REVISION << "\n";
        int major, minor, revision;
        glfwGetVersion(&major, &minor, &revision);
        clog //
            << "gg. GLFW running as v" //
            << major << "." << minor << "." << revision << "\n" //
            << "gg. GLFW version string " << glfwGetVersionString() << "\n";
    }

    // obtain primary monitor
    int num_monitor;
    GLFWmonitor** monitors = glfwGetMonitors(&num_monitor);
    if (!monitors) {
        cerr << "gg! No monitor found.\n";
        glfwTerminate();
        return EXIT_FAILURE;
    } else {
        clog << "gg. Found " << num_monitor << " monitors. Using primary.\n";
    }
    // primary monitor is 0; so, mode0 is the primary video mode.
    const GLFWvidmode* mode0 = glfwGetVideoMode(monitors[0]);
    glfwSetMonitorCallback(monitor_connect);

    // initialize window and OpenGL context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for Mac OSX to work
    glm::vec<2, int> size(min(640, mode0->width), min(480, mode0->height));
    GLFWwindow* window =
        glfwCreateWindow(size.x, size.y, prog.c_str(), nullptr, nullptr);
    if (!window) {
        cerr << "gg! Failed to create window context.\n";
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwSetWindowPos(
        window,
        mode0->width / 2 - split_offset * size.x,
        mode0->height / 2 - size.y / 2
    );
    glfwSetFramebufferSizeCallback(window, render::framebuffer_resize);
    glfwMakeContextCurrent(window);

    // load glad in window context
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        cerr << "gg! Failed to load OpenGL context.\n";
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glViewport(0, 0, size.x, size.y);
#endif

    // system initialization
    demo::init(scene);
#ifndef NO_COMM
    comm::init(data_dir);
#endif
    ai::init();
    physics::init();
#ifndef NO_RENDER
    ui::init_callbacks(window);
    render::init(size);
#endif

    // main loop initialization
    Timer init_time;
    Timer frame_time;
    unsigned total_frames = 0;
    unsigned fps = 0;
    auto last_s = init_time.time();
    bool all_done = false;
    prewarm_game(all_done, total_frames);

    // main loop
    while (
        !all_done
#ifndef NO_RENDER
        && !glfwWindowShouldClose(window)
#endif
    ) {
#ifndef NO_RENDER
        ////UI: would iterate over controllers, but it just handles specific
        // entities for now
        ui::handle_input(window, frame_time.delta_s());
        if (ui::paused) {
            continue;
        }
#endif

        frame_time.tick();
        ++total_frames;

        // FPS recorder
        if (1.f <= frame_time - last_s) {
            clog << "FPS: " << fps << "\n";
            fps = 0;
            last_s = frame_time.time();
        } else {
            ++fps;
        }

        double total_time = frame_time - init_time;
        // frame_time.delta_s()
        all_done = demo::run(1.f / 60.f, total_time, total_frames);

#ifndef NO_COMM
        comm::run();
#endif

        ////AI: iterates over agents, which often depend on boundvolumes,
        /// dynamics
        // and transforms.
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
        // double buffer
        glfwSwapBuffers(window);
#endif
    }

    comm::terminate();

#ifndef NO_RENDER
    // free all memory and libraries
    glfwTerminate();
#endif
    return EXIT_SUCCESS;
}

void prewarm_game(bool& all_done, unsigned total_frames) {
    all_done = demo::run(
        1.f / 60.f, static_cast<float>(total_frames) / 60.f, total_frames
    );

#ifndef NO_COMM
    comm::run();
#endif
    ai::update_agents();
    physics::prewarm(1.f / 60.f);
    POOL.all_sync();
}

void glfw_error(int err, const char* msg) {
    cerr << "gg! GLFW error: #" << err << " " << msg << "\n";
}

void monitor_connect(GLFWmonitor* m, int event) {
    if (event == GLFW_CONNECTED) {
        clog << "gg. Monitor " << glfwGetMonitorName(m) << " connected.\n";
    } else if (event == GLFW_DISCONNECTED) {
        clog << "gg. Monitor " << glfwGetMonitorName(m) << " disconnected.\n";
    }
}
