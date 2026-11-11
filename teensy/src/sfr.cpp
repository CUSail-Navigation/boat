#include "sfr.hpp"

/** Initialize all fields here; these values will be changed immediately when relevant. */
namespace sfr {
    namespace anemometer {
        uint16_t wind_angle = 0;
    }
    namespace servo {
        uint8_t mainsail_angle = 0;
        uint8_t rudder_angle = 0;
        uint8_t jib_angle = 0;
        uint8_t jib_side_flag = 0;

        uint32_t mainsail_pwm = 0;
        uint32_t rudder_pwm = 0;
        uint32_t jib_port_pwm = 0;
        uint32_t jib_stb_pwm = 0;
    }
    namespace serial {
        bool update_servos_usb = false;
        bool update_servos_radio = false;
        uint8_t usb_buffer[constants::serial::USB_BUFFER_LEN] = {};
        uint8_t radio_buffer[constants::serial::RADIO_BUFFER_LEN] = {};
        uint8_t radio_set_mode = constants::control::RC;
        uint8_t dropped_packets = 0;
        uint32_t last_usb_packet_time = -constants::serial::RX_STALE_AFTER_MS;
        uint32_t last_radio_packet_time = -constants::serial::RX_STALE_AFTER_MS;
    }
    namespace algo {
        uint8_t mainsail_angle = constants::servo::MAINSAIL_MAX_ANGLE;
        uint8_t rudder_angle = constants::servo::RUDDER_MID_ANGLE;
        uint8_t jib_angle = constants::servo::JIB_MAX_ANGLE;
        uint8_t jib_side_flag = constants::servo::JIB_SIDE_PORT;
        bool update_servos_algo = false;
    }
    namespace control {
        uint8_t mode = constants::control::RC;
    }
}
