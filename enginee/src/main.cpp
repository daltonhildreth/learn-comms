#ifdef WIN32
    //avoid including <windows.h> in glad
    #define APIENTRY __stdcall
#endif
#include <glad.h>
#ifdef _WINDOWS_
    #error windows.h was included
#endif
#ifdef WIN32
    #undef near
    #undef far
    #define NOMINMAX
#endif
#include <GLFW/glfw3.h>
#include <glm/vec2.hpp>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <string>

#include "util/Timer.h"
#include "render.h"
#include "io.h"
#include "ui.h"
#include "ai/ai.h"
//TOOD: move physics into its own directory :p
#include "ai/physics.h"
#include "Pool.h"
#include "util/debug.h"

#include "demo/demo.h"
#include "comms/comm.h"
#include "util/Seeder.h"

#include <fstream>

using namespace std;

void glfw_error(int err, const char* msg);
void monitor_connect(GLFWmonitor*, int event);
void keymap_input(GLFWwindow*);

int main(int argc, char** argv) {
    #ifndef NO_RENDER
    //Setup pre-init glfw
    glfwSetErrorCallback(glfw_error);

    //initialize glfw, output version.
    if (!glfwInit()) {
        cerr << "gg! Failed to init glfw; exiting.\n";
        return EXIT_FAILURE;
    } else {
        clog << "gg. GLFW compiled as v"
             << GLFW_VERSION_MAJOR << "."
             << GLFW_VERSION_MINOR << "."
             << GLFW_VERSION_REVISION << "\n";
        int major, minor, revision;
        glfwGetVersion(&major, &minor, &revision);
        clog << "gg. GLFW running as v"
             << major << "." << minor << "." << revision << "\n";
        clog << "gg. GLFW version string " << glfwGetVersionString() << "\n";
    }

    //obtain primary monitor
    int num_monitor;
    GLFWmonitor** monitors = glfwGetMonitors(&num_monitor);
    if (!monitors) {
        cerr << "gg! No monitor found.\n";
        glfwTerminate();
        return EXIT_FAILURE;
    } else {
        clog << "gg. Found " << num_monitor << " monitors. Using primary.\n";
    }
    //primary monitor is 0; so, mode0 is the primary video mode.
    const GLFWvidmode* mode0 = glfwGetVideoMode(monitors[0]);
    glfwSetMonitorCallback(monitor_connect);


    //initialize window and OpenGL context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); //for Mac OSX to work
    glm::vec<2, int> size(min(640, mode0->width), min(480, mode0->height));
    GLFWwindow* window;
    if (argc > 3) {
        window = glfwCreateWindow(size.x, size.y, argv[3], nullptr, nullptr);
    } else {
        window = glfwCreateWindow(size.x, size.y, "gg", nullptr, nullptr);
    }
    if (!window) {
        cerr << "gg! Failed to create window context.\n";
        glfwTerminate();
        return EXIT_FAILURE;
    }
    //center window
    int nudge = -1;
    if (argc > 3) {
        nudge = std::stoi(argv[3]);
        #ifndef NO_RENDER
        ui::paused = false;
        #endif
    }
    glfwSetWindowPos(
        window,
        mode0->width/2 - nudge * size.x,
        mode0->height/2 - size.y/2
    );
    glfwSetFramebufferSizeCallback(window, render::framebuffer_resize);
    glfwMakeContextCurrent(window);

    //load glad in window context
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        cerr << "gg! Failed to load OpenGL context.\n";
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glViewport(0, 0, size.x, size.y);
    #endif

    if (argc > 2) {
        Seeder s;
        s.seed(std::stoull(argv[2]));
    }
    Seeder s;
    if (argc == 1) {
        cerr << "gg! Failed to create demo.\n";
        #ifndef NO_RENDER
        glfwTerminate();
        #endif
        return EXIT_FAILURE;
    }
    int scn_i = std::stoi(argv[1]);
    if (scn_i < 0) {
        cerr << "gg! Invalid demo ID.\n";
        #ifndef NO_RENDER
        glfwTerminate();
        #endif
        return EXIT_FAILURE;
    }
    demo::init(static_cast<unsigned>(scn_i));
    #ifndef NO_COMM
    comm::init();
    #endif
    ai::init();
    physics::init();
    #ifndef NO_RENDER
    ui::init_callbacks(window);
    render::init(size);
    #endif

    //main loop
    Timer init_time;
    Timer frame_time;
    unsigned total_frames = 0;
    unsigned fps = 0;
    auto last_s = init_time.time();
    bool all_done = false;
    while(
        !all_done
        #ifndef NO_RENDER
        && !glfwWindowShouldClose(window)
        #endif
    ) {
        #ifndef NO_RENDER
        ////UI: would iterate over controllers, but it just handles specific
        //entities for now
        if (ui::paused) {
            glfwWaitEvents();
        } else {
            glfwPollEvents();
        }
        //input handling
        ui::handle_input(window, frame_time.delta_s());

        if (ui::paused) {
            continue;
        }
        #endif

        frame_time.tick();
        ++total_frames;

        //FPS recorder
        if (1.f <= frame_time - last_s) {
            clog << "FPS: " << fps << "\n";
            fps = 0;
            last_s = frame_time.time();
        } else {
            ++fps;
        }

        double total_time = frame_time - init_time;
        UNUSED(total_time);
        //frame_time.delta_s()
        all_done = demo::run(1.f/60.f, static_cast<float>(total_frames)/60.f, total_frames);
        #ifndef NO_RENDER
        glfwSetWindowShouldClose(window, all_done);
        #endif

        #ifndef NO_COMM
        comm::run();
        #endif

        ////AI: iterates over agents, which often depend on boundvolumes, dynamics
        //and transforms.
        ai::update_agents();

        ////Physics: iterates over dynamics, which often depend on boundvolumes,
        //and transforms.
        physics::simulate(1.f/60.f);//static_cast<float>(frame_time.delta_s()));

        ////sync: currently some components have redundant information that
        //needs to be synced every frame.
        POOL.all_sync();

        #ifndef NO_RENDER
        ////Render: iterates over meshes, which often depend on transforms.
        render::draw();
        //double buffer
        glfwSwapBuffers(window);
        #endif
    }
    //double total_time = static_cast<double>(total_frames)/60.0;//frame_time - init_time;
    double avg_velocity = physics::avg_velocity;

    ofstream results;
    results.open(std::string(DATA_DIR)+"/"+std::string(PRE_R)+"comms.result");
    float group_time = demo::run_avg_time + demo::run_std_time * 3;
    results << avg_velocity << "\n" << group_time << "\n";
    results.close();

    #ifndef NO_RENDER
    //free all memory and libraries
    glfwTerminate();
    #endif
    return EXIT_SUCCESS;
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
