#pragma once
#include "monitors/anemometer_monitor.hpp"
#include "monitors/radio_serial_monitor.hpp"
#include "monitors/usb_serial_monitor.hpp"
#include "sailing/sail_trim_algo.hpp"
#include "sailing/steering_algo.hpp"
#include "control_tasks/servo_control_task.hpp"
#include "control_tasks/led_control_task.hpp"
#include "control_tasks/telemetry_control_task.hpp"

class MainControlLoop {
public:
    MainControlLoop();
    void execute();

protected:
    AnemometerMonitor anemometer_monitor;
    RadioSerialMonitor radio_serial_monitor;
    USBSerialMonitor usb_serial_monitor;
    SailTrimAlgo sail_trim_algo;
    SteeringAlgo steering_algo;
    ServoControlTask servo_control_task;
    LedControlTask led_control_task;
    TelemetryControlTask telemetry_control_task;
};
