#include "AnemometerMonitor.hpp"

AnemometerMonitor::AnemometerMonitor() {
    pinMode(constants::anemometer::ANEMOMETER_PIN, INPUT);
}

/**
 * Reads the anemometer's analog voltage and converts it to a wind angle in degrees.
 * Our convention: an angle of @c 0 represents the boat being head-to-wind; an angle of @c 180 represents a run.
 */
void AnemometerMonitor::execute() {
    // Note that 0.3515625 = 360/1024 -- maps the analogRead() range of 0-1023 onto 0-359 degrees.
    const auto raw_angle = static_cast<uint16_t>(0.3515625 * analogRead(constants::anemometer::ANEMOMETER_PIN));
    // The sensor reads 180 when head-to-wind; map this so that 0 is head-to-wind instead.
    sfr::anemometer::wind_angle = (raw_angle + 180) % 360;
}
