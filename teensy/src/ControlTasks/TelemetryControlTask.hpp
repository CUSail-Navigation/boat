#pragma once
#include "sfr.hpp"

class TelemetryControlTask {
public:
    TelemetryControlTask();
    void execute();

private:
    uint32_t last_telemetry_send_time;
};
