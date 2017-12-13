#include "ui.h"
#include <iostream>

namespace ui {
float cursor_sensitivity = 1.f;
glm::vec2 d_cursor_pos = glm::vec2(0.f, 0.f);
glm::vec2 cursor_pos = glm::vec2(0.f, 0.f);
float d_scroll = 0.f;
int key_map[512] = {0};
int prior_key_map[512] = {0};
std::vector<std::function<void(GLFWwindow*, double)>> handlers;

bool edge_up(int key) {
    return key_map[key] && !prior_key_map[key];
}
bool edge_down(int key) {
    return !key_map[key] && prior_key_map[key];
}

void init_callbacks(GLFWwindow* w) {
    glfwSetKeyCallback(w, key_callback);
    #ifndef NO_RENDER
    glfwSetInputMode(w, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    #endif
    glfwSetInputMode(w, GLFW_STICKY_KEYS, 1);
    glfwSetCursorPosCallback(w, mouse_callback);
    glfwSetScrollCallback(w, scroll_callback);
}

void handle_input(GLFWwindow* w, double delta_s) {
    //global handlers
    //esc -> close application
    if (key_map[GLFW_KEY_ESCAPE]) {
        glfwSetWindowShouldClose(w, true);
        return;
    }

    for (const auto& f : handlers) {
        f(w, delta_s);
    }
    d_scroll = 0.f;
    d_cursor_pos = glm::vec2(0, 0);

    memcpy(prior_key_map, key_map, 512 * sizeof(int));
}

void add_handler(std::function<void(GLFWwindow*, double)> handle) {
    handlers.push_back(handle);
}

void key_callback(GLFWwindow*, int key, int, int action, int) {
    key_map[key] = action;
}

void center_mouse(GLFWwindow* w) {
    int height, width;
    glfwGetWindowSize(w, &width, &height);
    glfwSetCursorPos(w, width/2, height/2);
    cursor_pos = glm::vec2(width, height)/2.f;
}

void mouse_callback(GLFWwindow* w, double xpos, double ypos) {
    static bool focus = false;
    if (!focus) {
        focus = true;
        center_mouse(w);
        return;
    }
    d_cursor_pos = cursor_sensitivity * (glm::vec2(xpos, ypos) - cursor_pos);

    cursor_pos = glm::vec2(xpos, ypos);
}

void scroll_callback(GLFWwindow*, double, double yoffset) {
    d_scroll = static_cast<float>(yoffset);
}
}//UI::
