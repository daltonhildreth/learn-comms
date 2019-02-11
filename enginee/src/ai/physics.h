#pragma once
namespace physics {
extern float avg_velocity;
void init();
void prewarm(float dt);
void simulate(float dt);
} // namespace physics
