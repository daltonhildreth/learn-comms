#pragma once
#include <GLFW/glfw3.h>
#include <glad.h>
#include <glm/vec2.hpp>
#include <memory>
#include <string>
#include <vector>

class Shader;
class DirLight;
class PointLight;
class SpotLight;
class Mesh;
class Camera;

namespace render {
extern float cam_dist;
extern GLFWwindow* window;
// practically static: DO NOT CHANGE AFTER INIT
extern bool is_record;
enum VisF { XYZ2LAB, X2RG, Y2RG, Z2RG, XYZ2LUV };
extern unsigned comm_vis_f;
// currently going to do one shader for all meshes in the pool,
// just because I don't have a better way right this moment that I've made.
// There are many ways I have thought of though.
extern std::unique_ptr<Shader> mtl;
extern std::vector<std::unique_ptr<DirLight>> dir_lights;
extern std::vector<std::unique_ptr<PointLight>> point_lights;
extern std::vector<std::unique_ptr<SpotLight>> spot_lights;
extern std::unique_ptr<Camera> cam;

GLuint create_tex(std::string path);
// TODO: combine create and init
glm::vec<2, int> create_context_window(std::string prog, unsigned offset);
// TODO: remove init in favor of SceneGraphs
void init(glm::vec<2, int>, std::string);
void draw();
void terminate();

void framebuffer_resize(GLFWwindow* w, int width, int height);
// input handler
void input_key(GLFWwindow* w, double dt);
void input_cursor(GLFWwindow* w, double dt);
void input_scroll(GLFWwindow* w, double dt);
} // namespace render
