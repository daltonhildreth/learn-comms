#ifndef DEMO_H
#define DEMO_H

namespace demo {
void init();
bool run(double dt, double time, unsigned frame_count);

extern float run_avg_time;
extern float run_std_time;
extern float run_sum_times_sq;
}
#endif
