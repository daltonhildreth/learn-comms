#pragma once
#include <string>

namespace demo {
void init(unsigned scn_i, std::string data_dir);
bool run(double dt, double time, unsigned frame_count);
void terminate();

extern float run_avg_time;
extern float run_std_time;
extern float run_sum_times_sq;
extern const int SCENE;
} // namespace demo
