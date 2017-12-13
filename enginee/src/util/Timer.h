#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer {
public:
    //create a timer which starts ticking now
    Timer();

    //advance clock to now
    void tick();

    std::chrono::duration<double> delta();
    double delta_ms();
    double delta_s();
    std::chrono::time_point<std::chrono::steady_clock> time();

    double operator-(Timer& timer);
    double operator-(std::chrono::time_point<std::chrono::steady_clock> t);
private:
    // Time of this frame; steady_clock so we can measure intervals
    std::chrono::time_point<std::chrono::steady_clock> _time;
    // Difference in time between this frame and the last frame
    std::chrono::duration<double> _delta;
};

#endif//TIMER_H
