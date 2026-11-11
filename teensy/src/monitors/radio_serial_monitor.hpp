#pragma once
#include "serial_monitor_base.hpp"

class RadioSerialMonitor : public SerialMonitorBase {
public:
    void execute() override;

private:
    uint8_t temp_buffer[constants::serial::RADIO_BUFFER_LEN] = {};
};
