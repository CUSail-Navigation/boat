/**
 * ANEMOMETER TESTS -- AnemometerMonitor.
 * This file tests the properties of the AnemometerMonitor that the rest of the boat relies on: the reading is a compass
 * bearing with 0 as head-to-wind, it never wraps past a full circle, and it tracks the sensor in the right direction.
 */
#include "test_support.h"
#include "suites.hpp"
#include "monitors/anemometer_monitor.hpp"


// Constants and helper functions.
static constexpr int ADC_MAX = 1023;
static constexpr uint16_t FULL_CIRCLE_DEGREES = 360;

/** With the ADC staged at @c raw, take one reading. */
static uint16_t read_wind_angle_for(AnemometerMonitor& monitor, const int raw) {
    mock_set_analog(constants::anemometer::ANEMOMETER_PIN, raw);
    monitor.execute();
    return sfr::anemometer::wind_angle;
}


// Tests.
static void test_zero_reading_is_dead_downwind() {
    AnemometerMonitor monitor;
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(180, read_wind_angle_for(monitor, 0),
                                     "A zero ADC reading (the sensor's 0) should be dead downwind (180)");
}

static void test_half_scale_reading_is_head_to_wind() {
    AnemometerMonitor monitor;
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, read_wind_angle_for(monitor, (ADC_MAX + 1) / 2),
                                     "A half-scale ADC reading (the sensor's 180) should be head-to-wind (0)");
}

static void test_every_reading_stays_within_one_revolution() {
    AnemometerMonitor monitor;

    for (int raw = 0; raw <= ADC_MAX; raw++) {
        const uint16_t angle = read_wind_angle_for(monitor, raw);
        if (angle >= FULL_CIRCLE_DEGREES) {
            char message[128];
            snprintf(message, sizeof(message), "ADC reading %d produced %u degrees, which is not a valid bearing",
                     raw, static_cast<unsigned>(angle));
            TEST_FAIL_MESSAGE(message);
        }
    }
}

static void test_angle_rises_with_sensor_reading_wrapping_once() {
    AnemometerMonitor monitor;
    std::vector<uint32_t> below_wrap, above_wrap;

    // The 180-degree shift wraps 359 -> 0 exactly at half scale; the angle must rise on either side of it.
    for (int raw = 0; raw <= ADC_MAX; raw += 16) {
        (raw < (ADC_MAX + 1) / 2 ? below_wrap : above_wrap).push_back(read_wind_angle_for(monitor, raw));
    }

    assert_non_decreasing(below_wrap, "wind angle (below half scale)");
    assert_non_decreasing(above_wrap, "wind angle (above half scale)");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(above_wrap.front(), below_wrap.back(),
                                            "The angle should wrap around past 359 back to 0 at half scale");
}

static void test_full_scale_reading_lands_just_short_of_dead_downwind() {
    AnemometerMonitor monitor;
    const uint16_t angle = read_wind_angle_for(monitor, ADC_MAX);

    TEST_ASSERT_LESS_THAN_UINT16(180, angle);
    TEST_ASSERT_GREATER_THAN_UINT16_MESSAGE(180 - 5, angle,
                                            "A full-scale reading should map to just short of dead downwind");
}

static void test_reading_vane_isolated() {
    AnemometerMonitor monitor;
    read_wind_angle_for(monitor, ADC_MAX / 2);

    TEST_ASSERT_EQUAL_UINT8(0, sfr::serial::dropped_packets);
    TEST_ASSERT_FALSE(sfr::serial::update_servos_usb);
    TEST_ASSERT_FALSE(sfr::serial::update_servos_radio);
    TEST_ASSERT_EQUAL_UINT32(0, sfr::servo::rudder_pwm);
}


// Runner.
void run_anemometer_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_zero_reading_is_dead_downwind);
    RUN_TEST(test_half_scale_reading_is_head_to_wind);
    RUN_TEST(test_every_reading_stays_within_one_revolution);
    RUN_TEST(test_angle_rises_with_sensor_reading_wrapping_once);
    RUN_TEST(test_full_scale_reading_lands_just_short_of_dead_downwind);
    RUN_TEST(test_reading_vane_isolated);
}
