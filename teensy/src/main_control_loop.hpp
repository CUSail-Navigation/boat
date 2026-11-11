#pragma once
#include "monitors/anemometer_monitor.hpp"
#include "monitors/radio_serial_monitor.hpp"
#include "monitors/usb_serial_monitor.hpp"
#include "algorithms/sail_trim_algorithm.hpp"
#include "algorithms/steering_algorithm.hpp"
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
    SailTrimAlgorithm sail_trim_algorithm;
    SteeringAlgorithm steering_algorithm;
    ServoControlTask servo_control_task;
    LedControlTask led_control_task;
    TelemetryControlTask telemetry_control_task;
};
