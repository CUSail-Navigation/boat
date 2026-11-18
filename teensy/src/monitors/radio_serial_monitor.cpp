#include "radio_serial_monitor.hpp"

/** Reads and assembles incoming radio packets from @c Serial2 into @c radio_buffer . */
void RadioSerialMonitor::execute() {
    if (read_packet(Serial2, temp_buffer, sfr::serial::radio_buffer, constants::serial::RADIO_BUFFER_LEN)) {
        sfr::serial::update_servos_radio = true;
        sfr::serial::last_radio_packet_time = millis();
        sfr::serial::radio_set_mode = sfr::serial::radio_buffer[0];
    }
}
