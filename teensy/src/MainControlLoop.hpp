#pragma once
#include "Monitors/AnemometerMonitor.hpp"
#include "Monitors/RadioSerialMonitor.hpp"
#include "Monitors/USBSerialMonitor.hpp"
#include "ControlTasks/LedControlTask.hpp"
#include "ControlTasks/ServoControlTask.hpp"
#include "ControlTasks/TelemetryControlTask.hpp"
#include "sfr.hpp"

class MainControlLoop {
public:
    MainControlLoop();
    void execute();

protected:
    AnemometerMonitor anemometer_monitor;
    RadioSerialMonitor radio_serial_monitor;
    USBSerialMonitor usb_serial_monitor;
    LedControlTask led_control_task;
    ServoControlTask servo_control_task;
    TelemetryControlTask telemetry_control_task;
};
