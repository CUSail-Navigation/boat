#pragma once
#include "sfr.hpp"

class SailTrimAlgo {
public:
    SailTrimAlgo();
    void execute();

private:
    uint32_t last_update_time;
};
