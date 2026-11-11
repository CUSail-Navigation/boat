#include "SteeringAlgorithm.hpp"

// NOTE: THIS IS A PLACEHOLDER. An actual algorithm can be written when GPS/IMU data comes into the Teensy.
namespace {
    constexpr uint8_t SEQUENCE[] = {constants::servo::RUDDER_MID_ANGLE - 10, constants::servo::RUDDER_MID_ANGLE,
                                    constants::servo::RUDDER_MID_ANGLE + 10, constants::servo::RUDDER_MID_ANGLE};
    constexpr uint8_t SEQUENCE_LEN = sizeof(SEQUENCE);
    constexpr uint32_t STEP_MS = 2000;
}

SteeringAlgorithm::SteeringAlgorithm() : step_index(0), last_step_time(0), last_update_time(0) {}

/**
 * Computes an angle to set the rudder to, effectively steering the boat, and writes
 * this goal angle to the SFR (staged in @c sfr::algo), every @c UPDATE_PERIOD_MS .
 */
void SteeringAlgorithm::execute() {
    const uint32_t now = millis();
    if (now - last_update_time < constants::algo::UPDATE_PERIOD_MS) return;
    last_update_time = now;

    // FAKE algorithm behavior (for testing): set rudder on repeat to -10, 0, 10, 0, changing every 2 seconds.
    if (now - last_step_time >= STEP_MS) {
        step_index = (step_index + 1) % SEQUENCE_LEN;
        last_step_time = now;
    }
    sfr::algo::rudder_angle = SEQUENCE[step_index];

    sfr::algo::update_servos_algo = true;
}
