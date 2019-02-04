#ifndef PHYSICS_H
#define PHYSICS_H

namespace physics {
extern float avg_velocity;
void init();
void prewarm(float dt);
void simulate(float dt);
}

#endif//PHYSICS_H
