#ifndef DYNAMICS_H
#define DYNAMICS_H

struct Dynamics {
    //TODO: remove redundancies that need to be synced.
    glm::vec3 pos;
    glm::vec3 vel;
    glm::vec3 acc;
    glm::vec3 force;
    float mass;
};

#endif//DYNAMICS_H
