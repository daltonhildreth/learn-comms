#ifndef SEEDER_H
#define SEEDER_H

#include <random>
#include <chrono>
#include <cstdint>

typedef std::chrono::high_resolution_clock HRClock;

//singleton seed generator
class Seeder {
public:
    Seeder();

    void reseed();
    uint64_t seed();
    void seed(uint64_t s);
    std::default_random_engine& gen();
private:
    static uint64_t _seed;
    static HRClock::time_point _first;
    static std::default_random_engine _gen;
};
#endif//SEEDER_H
