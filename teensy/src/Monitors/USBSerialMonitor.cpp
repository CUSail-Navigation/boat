#include "USBSerialMonitor.hpp"

/** Reads and assembles incoming packets from the computer connected via @c Serial into @c usb_buffer . */
void USBSerialMonitor::execute() {
    if (read_packet(Serial, temp_buffer, sfr::serial::usb_buffer, constants::serial::USB_BUFFER_LEN)) {
        sfr::serial::update_servos_usb = true;
        sfr::serial::last_usb_packet_time = millis();
    }
}
