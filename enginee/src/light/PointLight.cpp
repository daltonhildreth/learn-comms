#include "PointLight.h"

PointLight::PointLight(): _pos(glm::vec3(0, 0, 0)) { att_to_dist(30.); }

void PointLight::pass_to(Shader& shader, const std::string& id) {
    Light::pass_to(shader, id);
    shader.set(id + "pos", _pos);
    shader.set(id + "att_k", _att_k);
    shader.set(id + "att_x", _att_x);
    shader.set(id + "att_x2", _att_x2);
}

void PointLight::att_to_dist(float dist) {
    // www.desmos.com/calculator/q4xbz0yf7
    // these values are regressed on OGRE's distance based attenuation:
    // wiki.ogre3d.org/tiki-index.php?page=-Point+Light+Attenuation
    float dist2 = dist * dist;
    _att_k = 1.f;
    _att_x = 5.00404f / dist2 + 4.16793f / dist + 0.00178546f;
    _att_x2 = 97.7941f / dist2 - 1.48474f / dist + 0.00972348f;
}

glm::vec3 PointLight::pos() { return _pos; }
void PointLight::pos(glm::vec3 pos) { _pos = pos; }
float PointLight::att_constant() { return _att_k; }
float PointLight::att_linear() { return _att_x; }
float PointLight::att_quadratic() { return _att_x2; }
void PointLight::att(float k, float x, float x2) {
    _att_k = k;
    _att_x = x;
    _att_x2 = x2;
}
