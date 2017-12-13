#include "Transform.h"
#include <glm/gtc/matrix_access.hpp>

using namespace std;
using namespace glm;

Transform::Transform(const Transform* p) 
    : _parent(p), _children(0)
    , _transform(glm::mat4(1.f)) {
}

void Transform::set_parent(const Transform& p) {
    _parent = &p;
}

void Transform::add_child(Transform* child) {
    _children.push_back(child);
}

vector<Transform*>& Transform::children() {
    return _children;
}

//void del_chld(int idx) {}

mat4 Transform::mat() const {
    return _transform;
}

void Transform::set_mat(mat4 m) {
    _transform = m;
}

mat4 Transform::global_mat() const {
    if (_parent == nullptr) {
        //This transform is the root so global == local
        return _transform;
    }

    return _transform * _parent->global_mat();
}

void Transform::set_global_mat(mat4 gm) {
    if (_parent == nullptr) {
        //This transform is the root so global == local
        _transform = gm;
    }

    _transform = gm * inverse(_parent->global_mat());
}

//void wrt_mat(int num_up);
//void set_wrt_mat(glm::mat4, int num_up);

//glm::vec3 scale();
//void set_scale(glm::vec3);
//glm::quat rot();
//void set_rot(glm::quat);

glm::vec3 Transform::pos() const {
    return glm::column(_transform, 3);
}

void Transform::set_pos(glm::vec3 p) {
    _transform = glm::column(_transform, 3, glm::vec4(p, 1));
}
