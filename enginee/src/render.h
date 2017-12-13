#ifndef RENDER_H
#define RENDER_H

#include <glad.h>
#include <glm/vec2.hpp>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>
#include <vector>

class Shader;
class DirLight;
class PointLight;
class SpotLight;
class Mesh;
class Camera;

namespace render {
//currently going to do one shader for all meshes in the pool,
//just because I don't have a better way right this moment that I've made.
//There are many ways I have thought of though.
extern std::unique_ptr<Shader> mtl;
extern std::vector<std::unique_ptr<DirLight>> dir_lights;
extern std::vector<std::unique_ptr<PointLight>> point_lights;
extern std::vector<std::unique_ptr<SpotLight>> spot_lights;
extern std::unique_ptr<Camera> cam;

GLuint create_tex(std::string path);
//TODO: remove init in favor of SceneGraphs
void init(glm::vec<2, int>);
void draw();

void framebuffer_resize(GLFWwindow* w, int width, int height);
//input handler
void input_key(GLFWwindow* w, double dt);
void input_cursor(GLFWwindow* w, double dt);
void input_scroll(GLFWwindow* w, double dt);
}

#endif//RENDER_H
