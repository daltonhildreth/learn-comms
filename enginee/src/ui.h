#pragma once
#include <GLFW/glfw3.h>
#include <functional>
#include <glm/vec2.hpp>
#include <vector>

// todo: make it a class :P
namespace ui {
extern bool paused;
extern glm::vec2 d_cursor_pos;
extern glm::vec2 cursor_pos;
extern float cursor_sensitivity;
extern float d_scroll;
extern int key_map[512];
extern int prior_key_map[512];
extern std::vector<std::function<void(GLFWwindow*, double)>> handlers;

// setup GLFW callback functions
void init_callbacks(GLFWwindow* w);
// call input per frame
void handle_input(GLFWwindow* w, double delta_s);
// add custom handler for handle_input, it can take a delta-time
void add_handler(std::function<void(GLFWwindow*, double)> input_handler);

bool edge_up(int key);
bool edge_down(int key);
// callback for GLFW key events
void key_callback(
    GLFWwindow* window,
    int key,
    int scancode,
    int action,
    int mode
);
// help function to center mouse on screen
void center_mouse(GLFWwindow* window);
// callback for GLFW mouse events
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
// callback for GLFW scrolling events
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
} // namespace ui
