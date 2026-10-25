#include "LedControlTask.hpp"

LedControlTask::LedControlTask() : num_flashes(0), code_length(0), code_start_time(0) {
    pinMode(constants::led::LED_PIN, OUTPUT);
}

/**
 * Blinks a certain status LED code to indicate which serial input is in control, if that input is not stale (we
 * say that the input is stale if we haven't received a packet from it in the last \code RX_STALE_AFTER_MS\endcode).
 *
 * The different types of status LED codes are:
 * - Radio in control (and not stale): 1 flash, then a pause.
 * - USB in control (and not stale): 2 flashes, then a pause.
 * - Otherwise (whichever input in control is stale): constant slow blink.
 *
 * The LED should always blink while the Teensy is running: not blinking means the Teensy is not running.
 */
void LedControlTask::execute() {
    const uint32_t now = millis();

    // The previous status code has finished; decide what the next one should be.
    if (now - code_start_time >= code_length) {
        const bool radio_mode = sfr::serial::radio_flag != 0;
        const bool stale = now - (radio_mode ? sfr::serial::last_radio_packet_time : sfr::serial::last_usb_packet_time)
                           >= constants::serial::RX_STALE_AFTER_MS;

        if (stale) {
            num_flashes = 0;
            code_length = 2 * constants::led::STALE_BLINK_MS;
        } else {
            num_flashes = radio_mode ? constants::led::RADIO_FLASHES : constants::led::USB_FLASHES;
            code_length = num_flashes * (constants::led::FLASH_ON_MS + constants::led::FLASH_GAP_MS)
                                      + constants::led::FLASH_PAUSE_MS;
        }
        code_start_time = now;
    }

    // Figure out where in the current code we are and update the LED to match.
    bool on;
    const uint32_t elapsed = now - code_start_time;
    if (num_flashes == 0) {
        on = elapsed < constants::led::STALE_BLINK_MS;
    } else {
        constexpr uint32_t slot = constants::led::FLASH_ON_MS + constants::led::FLASH_GAP_MS;
        on = elapsed / slot < num_flashes && elapsed % slot < constants::led::FLASH_ON_MS;
    }

    digitalWrite(constants::led::LED_PIN, on ? HIGH : LOW);
}
