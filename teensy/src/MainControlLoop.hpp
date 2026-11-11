#pragma once
#include "Monitors/AnemometerMonitor.hpp"
#include "Monitors/RadioSerialMonitor.hpp"
#include "Monitors/USBSerialMonitor.hpp"
#include "Algorithms/SailTrimAlgorithm.hpp"
#include "Algorithms/SteeringAlgorithm.hpp"
#include "ControlTasks/ServoControlTask.hpp"
#include "ControlTasks/LedControlTask.hpp"
#include "ControlTasks/TelemetryControlTask.hpp"

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
