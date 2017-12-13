#ifndef CAMERA_H
#define CAMERA_H

#include <glm/vec3.hpp>
#include "Transform.h"
#ifdef WIN32
#undef near
#undef far
#endif

class Shader;

/* Manipulates the current view and projection matrix. */
class Camera {
public:
    Camera();

    //set/get
    void fov(float);
    float fov();
    void aspect(float);
    float aspect();
    void near(float);
    float near();
    void far(float);
    float far();

    //place the camera in its local frame
    void set_pos(glm::vec3);
    //set orientation based on direction
    void set_rot(glm::vec3 look_dir, glm::vec3 up);
    //set orientation based on focus
    //void set_lookat(glm::vec3 lookat, glm::vec3 up);

    glm::vec3 look_dir();
    glm::vec3 up();
    glm::vec3 right();

    //move the camera by vec3 in its local Transform
    void move(glm::vec3);
    //rotate the cameras viewing axis by quat. (alternate to setting)
    void rotate(glm::quat);
    //zoom in/out. 1 is default.
    void zoom(float z);
    float zoom();

    // init materials to use projection
    void apply_proj(Shader& shader);
    // reapply camera view every time its dirty (frame)
    void apply_view(Shader& shader);

private:
    std::unique_ptr<Transform> _view;
    glm::vec3 _pos;
    glm::vec3 _look_dir;
    glm::vec3 _up;
    glm::mat4 _proj;
    bool _dirty;
    const float _def_fov = 45.f;
    float _fov;
    const float _def_aspect = 4.f/3.f;
    float _aspect;
    const float _def_near = .1f;
    float _near;
    const float _def_far = 100.f;
    float _far;
};

#endif//CAMERA_H
