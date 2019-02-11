#include "Timer.h"

using namespace std::chrono;

Timer::Timer() {
    _time = steady_clock::now();
    _delta = duration<double>::zero();
}

void Timer::tick() {
    auto last = _time;
    _time = steady_clock::now();
    _delta = _time - last;
}

duration<double> Timer::delta() { return _delta; }

double Timer::delta_ms() {
    return duration<double, std::milli>(_delta).count();
}

double Timer::delta_s() { return _delta.count(); }

time_point<steady_clock> Timer::time() { return _time; }

double Timer::operator-(Timer& timer) {
    return duration<double>(time() - timer.time()).count();
}

double Timer::operator-(time_point<steady_clock> t) {
    return duration<double>(time() - t).count();
}
