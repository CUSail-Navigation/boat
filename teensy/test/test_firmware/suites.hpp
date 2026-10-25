#pragma once

/**
 * This file is the test suite registry. It acts like a header file for @c main.cpp, in a way.
 *
 * PlatformIO builds each directory under @c test/ into one binary with a single @c main(), so all of these
 * tests link together. They are still split across files by topic; each file keeps its own tests file-local,
 * @c static, and exposes just the one function below, which @c main.cpp calls.
 */
void run_serial_framing_tests();
void run_servo_control_tests();
void run_telemetry_tests();
void run_anemometer_tests();
