#ifndef POINT_LIGHT_H
#define POINT_LIGHT_H

#include "Light.h"

class PointLight : public Light {
public:
    PointLight();

    virtual void pass_to(Shader& shader, const std::string& id);

    //attenuate so that the light has some noticeable effect until dist units
    void att_to_dist(float dist);

    //get/set
    glm::vec3 pos();
    void pos(glm::vec3);
    float att_constant();
    float att_linear();
    float att_quadratic();
    void att(float k, float x, float x2);
private:
    glm::vec3 _pos;

    //attenuation parameters
    //constant
    float _att_k;
    //linear
    float _att_x;
    //quadratic
    float _att_x2;
};

#endif//POINT_LIGHT_H
