#include "sail_trim_algo.hpp"

SailTrimAlgo::SailTrimAlgo() : last_update_time(0) {}

/**
 * Computes goal angles to set the sails to based on the current wind direction
 * and writes them to the SFR (staged in @c sfr::algo), every @c UPDATE_PERIOD_MS .
 */
void SailTrimAlgo::execute() {
    const uint32_t now = millis();
    if (now - last_update_time < constants::algo::UPDATE_PERIOD_MS) return;
    last_update_time = now;

    uint16_t wind_angle = sfr::anemometer::wind_angle;
    sfr::algo::jib_side_flag = wind_angle >= 180 ? constants::servo::JIB_SIDE_PORT : constants::servo::JIB_SIDE_STB;

    // No longer care about side: map the wind to one side and compute the goal sail angle magnitude from that.
    if (wind_angle > 180) wind_angle = 360 - wind_angle;
    if (wind_angle < constants::algo::NO_GO_HALF_WIDTH) {
        sfr::algo::mainsail_angle = constants::servo::MAINSAIL_MIN_ANGLE;
        sfr::algo::jib_angle = constants::servo::JIB_MIN_ANGLE;
    } else {
        sfr::algo::mainsail_angle = static_cast<uint8_t>(map(wind_angle, constants::algo::NO_GO_HALF_WIDTH, 180,
                                        constants::servo::MAINSAIL_MIN_ANGLE, constants::servo::MAINSAIL_MAX_ANGLE));
        sfr::algo::jib_angle = static_cast<uint8_t>(map(wind_angle, constants::algo::NO_GO_HALF_WIDTH, 180,
                                   constants::servo::JIB_MIN_ANGLE, constants::servo::JIB_MAX_ANGLE));
    }
    sfr::algo::update_servos_algo = true;
}
