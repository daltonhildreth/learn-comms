#pragma once
#include "Light.h"

class DirLight: public Light {
public:
    DirLight();

    virtual void pass_to(Shader& shader, const std::string& id);

    glm::vec3 dir();
    void dir(glm::vec3);

private:
    glm::vec3 _dir;
};
