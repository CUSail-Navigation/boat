/**
 * LED TESTS -- LedControlTask.
 * Tests in this file assert the logic behind status codes: which code shows for which state, a code always plays to the
 * end once started (state changes are only picked up by the next code), and that the task never blocks the main loop.
 */
#include "test_support.h"
#include "suites.hpp"
#include "ControlTasks/LedControlTask.hpp"


// The codes are only useful if a person can tell them apart.
static_assert(constants::led::RC_FLASHES != constants::led::ROS_ALGO_FLASHES &&
              constants::led::RC_FLASHES != constants::led::DETERM_ALGO_FLASHES &&
              constants::led::ROS_ALGO_FLASHES != constants::led::DETERM_ALGO_FLASHES, "Every mode code must differ");
static_assert(constants::led::STALE_BLINK_MS != constants::led::FLASH_ON_MS, "A stale blink must not look like a flash");


// Helper functions.
static uint32_t flash_code_length(const uint8_t flashes) {
    return flashes * (constants::led::FLASH_ON_MS + constants::led::FLASH_GAP_MS) + constants::led::FLASH_PAUSE_MS;
}

static uint32_t stale_code_length() {
    return 2 * constants::led::STALE_BLINK_MS;
}

/** What the LED showed over a time period: how many times it turned on and how long it was on in total. */
namespace {
    struct Observation {
        int flashes;
        uint32_t on_ms;
    };
}

/** Run the task once per simulated millisecond over @code [start, start + duration)@endcode, as the main loop does. */
static Observation observe(LedControlTask& task, const uint32_t start, const uint32_t duration) {
    Observation seen{.flashes = 0, .on_ms = 0};
    int previous = mock_pin_level(constants::led::LED_PIN);
    for (uint32_t t = start; t < start + duration; ++t) {
        mock_set_millis(t);
        task.execute();
        const int level = mock_pin_level(constants::led::LED_PIN);
        if (level == HIGH) {
            ++seen.on_ms;
            if (previous == LOW) ++seen.flashes;
        }
        previous = level;
    }
    return seen;
}

static void assert_flash_code(const Observation& seen, const uint8_t flashes, const char* message) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(flashes, seen.flashes, message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(flashes * constants::led::FLASH_ON_MS, seen.on_ms, message);
}

static void assert_stale_blink(const Observation& seen, const char* message) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, seen.flashes, message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::led::STALE_BLINK_MS, seen.on_ms, message);
}


// Tests.
static void test_execute_never_blocks() {
    LedControlTask task;
    mock_set_millis(0);
    task.execute(); // Stale.

    sfr::serial::last_radio_packet_time = millis();
    LedControlTask fresh_task;
    fresh_task.execute(); // RC in control and fresh.

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, millis(), "The LED task must never wait inside `execute()`");
}

static void test_boot_shows_stale_blink() {
    LedControlTask task;
    assert_stale_blink(observe(task, 0, stale_code_length()), "No packets received yet: input is stale");
}

static void test_rc_in_control_and_fresh_shows_rc_code() {
    sfr::control::mode = constants::control::RC;
    sfr::serial::last_radio_packet_time = 0;

    LedControlTask task;
    assert_flash_code(observe(task, 0, flash_code_length(constants::led::RC_FLASHES)),
                      constants::led::RC_FLASHES, "A fresh radio in control should show the RC code");
}

static void test_ros_in_control_and_fresh_shows_ros_code() {
    sfr::control::mode = constants::control::ROS_ALGO;
    sfr::serial::last_usb_packet_time = 0;

    LedControlTask task;
    assert_flash_code(observe(task, 0, flash_code_length(constants::led::ROS_ALGO_FLASHES)),
                      constants::led::ROS_ALGO_FLASHES, "A fresh USB input in control should show the ROS code");
}

static void test_determ_in_control_shows_determ_code_even_with_no_serial_input() {
    sfr::control::mode = constants::control::DETERM_ALGO; // Both serial inputs are stale (nothing received since boot).

    LedControlTask task;
    assert_flash_code(observe(task, 0, flash_code_length(constants::led::DETERM_ALGO_FLASHES)),
                      constants::led::DETERM_ALGO_FLASHES, "The deterministic algorithm is never stale");
}

static void test_only_the_input_in_control_counts() {
    sfr::control::mode = constants::control::RC;  // RC is in control...
    sfr::serial::last_usb_packet_time = 0; // but only USB is sending.

    LedControlTask task;
    assert_stale_blink(observe(task, 0, stale_code_length()),
                       "A fresh USB input must not hide that the radio, which is in control, is stale");
}

static void test_flash_code_is_not_cut_off_when_input_goes_stale() {
    constexpr uint32_t start = constants::serial::RX_STALE_AFTER_MS;
    const uint32_t usb_code_length = flash_code_length(constants::led::ROS_ALGO_FLASHES);
    sfr::control::mode = constants::control::ROS_ALGO;
    sfr::serial::last_usb_packet_time = start - constants::serial::RX_STALE_AFTER_MS + 1; // Stale 1 ms into the code.

    LedControlTask task;
    assert_flash_code(observe(task, start, usb_code_length), constants::led::ROS_ALGO_FLASHES,
                      "A code that started fresh must play in full, even if the input goes stale partway through");
    assert_stale_blink(observe(task, start + usb_code_length, stale_code_length()),
                       "The next code should then show that the input went stale");
}

static void test_stale_blink_is_not_cut_off_when_input_becomes_fresh() {
    sfr::control::mode = constants::control::RC;

    LedControlTask task;
    const Observation first_ms = observe(task, 0, 1); // The stale blink starts.
    sfr::serial::last_radio_packet_time = 1;                         // A radio packet arrives 1 ms in.
    const Observation rest = observe(task, 1, stale_code_length() - 1);

    TEST_ASSERT_EQUAL_UINT32(1, first_ms.on_ms);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, rest.flashes, "The stale blink must not restart or flicker partway through");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::led::STALE_BLINK_MS - 1, rest.on_ms,
                                     "The stale blink must play in full, even if fresh input arrives partway through");
}


// Runner.
void run_led_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_execute_never_blocks);
    RUN_TEST(test_boot_shows_stale_blink);
    RUN_TEST(test_rc_in_control_and_fresh_shows_rc_code);
    RUN_TEST(test_ros_in_control_and_fresh_shows_ros_code);
    RUN_TEST(test_determ_in_control_shows_determ_code_even_with_no_serial_input);
    RUN_TEST(test_only_the_input_in_control_counts);
    RUN_TEST(test_flash_code_is_not_cut_off_when_input_goes_stale);
    RUN_TEST(test_stale_blink_is_not_cut_off_when_input_becomes_fresh);
}
