#pragma once
#include <glm/glm.hpp>
#include <memory>
#include <vector>

// template <class T>
class Transform {
public:
    Transform(const Transform* parent);

    void set_parent(const Transform& parent);
    void add_child(Transform* child);
    std::vector<Transform*>& children();
    // I may want this at some point
    // void del_chld(int idx);

    // get local transform
    glm::mat4 mat() const;
    // set local transform
    void set_mat(glm::mat4);
    // get global (to root) transform
    glm::mat4 global_mat() const;
    // set local based on global (to root) transform
    void set_global_mat(glm::mat4);

    // get global (to height) transform
    // void wrt_mat(int num_up);
    // set local w.r.t. some higher transform
    // void set_wrt_mat(glm::mat4, int num_up);

    // It may be useful at some point to manipulate components
    // glm::vec3 scale() const;
    // void set_scale(glm::vec3);

    // get quaternion rotation
    // glm::quat rot() const;
    // set rotation
    // void set_rot(glm::quat);

    // get translation
    glm::vec3 pos() const;
    // set translation
    void set_pos(glm::vec3);

private:
    const Transform* _parent;
    std::vector<Transform*> _children;

    glm::mat4 _transform;

    // it may be useful at some point to know components
    // glm::vec3 scale;
    // glm::quat rot;
    // glm::vec3 pos;

    // it may be useful for traversal to know what this is attached to
    // node/thing which is being transformed
    // std::shared_ptr<T> socket;
};
