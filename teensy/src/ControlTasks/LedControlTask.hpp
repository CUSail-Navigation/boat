#pragma once
#include "sfr.hpp"

class LedControlTask {
public:
    LedControlTask();
    void execute();

private:
    uint8_t num_flashes;
    uint32_t code_length;
    uint32_t code_start_time;
};
