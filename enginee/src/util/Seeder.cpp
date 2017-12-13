#include "Seeder.h"

typedef std::chrono::high_resolution_clock hrclock;

uint64_t Seeder::_seed = 0;
HRClock::time_point Seeder::_first = HRClock::now();
std::default_random_engine Seeder::_gen = std::default_random_engine();


Seeder::Seeder() {
    static bool seeded = false;
    if (!seeded) {
        _first = hrclock::now();
        reseed();
    }
}

void Seeder::reseed() {
    seed(static_cast<uint64_t>(_first.time_since_epoch().count()));
}

uint64_t Seeder::seed() {
    return _seed;
}

void Seeder::seed(uint64_t s) {
    _seed = s;
    _gen.seed(_seed);
}

std::default_random_engine& Seeder::gen() {
    return _gen;
}
