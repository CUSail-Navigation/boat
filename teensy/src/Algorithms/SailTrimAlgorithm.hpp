#pragma once
#include "sfr.hpp"

class SailTrimAlgorithm {
public:
    SailTrimAlgorithm();
    void execute();

private:
    uint32_t last_update_time;
};
