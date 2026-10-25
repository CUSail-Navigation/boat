#include "TelemetryControlTask.hpp"

TelemetryControlTask::TelemetryControlTask() : last_telemetry_send_time(0) {}

/** Sends a telemetry packet of SFR data to the computer connected via @c Serial, every @c TX_PERIOD_MS . */
void TelemetryControlTask::execute() {
    if (const uint32_t now = millis(); now - last_telemetry_send_time >= constants::serial::TX_PERIOD_MS) {
        const uint8_t data[] = {
            constants::serial::TX_START_FLAG,
            static_cast<uint8_t>(sfr::anemometer::wind_angle >> 8),
            static_cast<uint8_t>(sfr::anemometer::wind_angle & 0xFF),
            sfr::servo::mainsail_angle,
            sfr::servo::rudder_angle,
            sfr::servo::jib_angle,
            sfr::servo::jib_side_flag,
            sfr::serial::dropped_packets,
            constants::serial::TX_END_FLAG
        };
        Serial.write(data, sizeof(data));
        last_telemetry_send_time = now;
    }
}
