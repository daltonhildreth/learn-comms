#pragma once
#include <cstdint>
#include <string>
#include <utility>

const unsigned N_COMP = 6;

struct Entity {
    Entity(uint16_t i): id(i), comp{} {};

    // the only important thing is the ID
    /*const*/ uint16_t id;
    // reverse pointers to the components attached to this entity; maybe these
    // are unnecessary or wasteful
    uint16_t comp[N_COMP];
};
