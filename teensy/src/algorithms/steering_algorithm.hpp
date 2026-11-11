#pragma once
#include "sfr.hpp"

class SteeringAlgorithm {
public:
    SteeringAlgorithm();
    void execute();

private:
    uint8_t step_index;      // NOTE: this is part of the fake algorithm.
    uint32_t last_step_time; // NOTE: this is part of the fake algorithm.
    uint32_t last_update_time;
};
