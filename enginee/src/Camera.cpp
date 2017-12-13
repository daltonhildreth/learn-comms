#include "light/Shader.h"
#include "Camera.h"
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_access.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#undef GLM_ENABLE_EXPERIMENTAL

Camera::Camera()
    : _view(new Transform(nullptr)), _proj(glm::mat4()), _dirty(false)
    , _fov(45.f), _aspect(4.f/3.f), _near(.1f), _far(100.f) {
}

void Camera::fov(float fov) {
    _fov = fov;
    _dirty = true;
}
float Camera::fov() {
    return _fov;
}

void Camera::aspect(float aspect) {
    _aspect = aspect;
    _dirty = true;
}
float Camera::aspect() {
    return _aspect;
}

void Camera::near(float near) {
    _near = near;
    _dirty = true;
}
float Camera::near() {
    return _near;
}

void Camera::far(float far) {
    _far = far;
    _dirty = true;
}
float Camera::far() {
    return _far;
}

void Camera::set_pos(glm::vec3 p) {
    //_view->set_pos(p);
    _pos = p;
    _view->set_mat(glm::lookAt(_pos, _pos + _look_dir, _up));
}

void Camera::set_rot(glm::vec3 look_dir, glm::vec3 up) {
    _look_dir = glm::normalize(look_dir);
    _up = glm::cross(glm::cross(_look_dir, glm::normalize(up)), _look_dir);
    //TODO: this is an inverse transform!! eeeeek
    _view->set_mat(glm::lookAt(_pos, _pos + _look_dir, _up));
}

glm::vec3 Camera::look_dir() {
    return _look_dir;
}
glm::vec3 Camera::up() {
    return _up;
}
glm::vec3 Camera::right() {
    return glm::cross(_look_dir, _up);
}

void Camera::move(glm::vec3 p) {
    //_view->set_pos(p + _view->pos());
    glm::vec3 right = glm::cross(_look_dir, _up);
    _pos = p.z*_look_dir + p.y*_up + p.x*right + _pos;
    _view->set_mat(glm::lookAt(_pos, _pos + _look_dir, _up));
}

void Camera::rotate(glm::quat q) {
    //todo: move into Transform in a better fashion
    glm::mat4 rot = glm::toMat4(q);
    _look_dir = glm::vec3(rot * glm::vec4(_look_dir, 1.f));
    _up = glm::vec3(rot * glm::vec4(_look_dir, 1.f));
    _view->set_mat(glm::lookAt(_pos, _pos + _look_dir, _up));
}

void Camera::zoom(float z) {
    //the clamp is needed for an FPS camera
    _fov = glm::clamp(glm::degrees(static_cast<float>(atan2(1, z))), 1.f, 90.f);
    _dirty = true;
}
float Camera::zoom() {
    return 1.f / static_cast<float>(tan(glm::radians(_fov)));
}

void Camera::apply_proj(Shader& shader) {
    if (_dirty) {
        _proj = glm::perspective(glm::radians(_fov), _aspect, _near, _far);
        _dirty = false;
        shader.use();
        shader.set("proj", _proj);
    }
}

void Camera::apply_view(Shader& shader) {
    shader.use();
    shader.set("view", _view->global_mat());
    shader.set("eye_pos", _pos);
}

