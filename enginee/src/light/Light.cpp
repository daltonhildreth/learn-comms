#include "Light.h"

Light::Light()
    : _ambient(glm::vec3(0, 0, 0)),
    _diffuse(glm::vec3(0, 0, 0)),
    _specular(glm::vec3(0, 0, 0)) {
}

void Light::pass_to(Shader& shader, const std::string& id) {
    shader.use();
    shader.set(id + "color.ambient", _ambient);
    shader.set(id + "color.diffuse", _diffuse);
    shader.set(id + "color.specular", _specular);
}

glm::vec3 Light::ambient() {
    return _ambient;
}
void Light::ambient(glm::vec3 ambient) {
    _ambient = ambient;
}
glm::vec3 Light::diffuse() {
    return _diffuse;
}
void Light::diffuse(glm::vec3 diffuse) {
    _diffuse = diffuse;
}
glm::vec3 Light::specular() {
    return _specular;
}
void Light::specular(glm::vec3 specular) {
    _specular = specular;
}
