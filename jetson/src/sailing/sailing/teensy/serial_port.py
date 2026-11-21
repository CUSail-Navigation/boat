import serial

from sailing.constants import PHYSICAL, SERIAL


class SerialPort:
    """Handle serial commands and telemetry for the Teensy."""

    def __init__(self, port):
        """Open the serial connection."""
        self.port = port
        self.buffer = []
        self.packet_started = False
        self.serial = serial.Serial(self.port, baudrate=SERIAL.BAUD_RATE)

    def send_command(self, mainsail_angle, rudder_angle, jib_angle, jib_side_flag):
        """
        Send a properly formatted command packet to the Teensy.
        Return 0 on success or 1 if encoding or writing fails.
        :param mainsail_angle: new mainsail angle to set (integer). Should be in range [0, 90].
        :param rudder_angle: new rudder angle to set (integer). Should be in range [-45, 45].
        :param jib_angle: new jib angle to set (integer). Should be in range [10, 80].
        :param jib_side_flag: side to set the jib on (0 = port, 1 = starboard).
        """
        try:
            # Check bounds. For the sails, this is just defensive.
            mainsail_angle = max(min(mainsail_angle, 127), -128)
            rudder_angle = rudder_angle - PHYSICAL.RUDDER_MIN_ANGLE
            jib_angle = max(min(jib_angle, 127), -128)

            # Convert control values to 8-bit integers (bytes). The else check is just defensive.
            mainsail_byte = (
                mainsail_angle & 0xFF
                if mainsail_angle >= 0
                else (mainsail_angle + 256) & 0xFF
            )
            rudder_byte = (
                rudder_angle & 0xFF
                if rudder_angle >= 0
                else (rudder_angle + 256) & 0xFF
            )
            jib_angle_byte = (
                jib_angle & 0xFF if jib_angle >= 0 else (jib_angle + 256) & 0xFF
            )
            jib_side_byte = jib_side_flag & 0xFF

            # Command payload format between start/end flags:
            # [mainsail_angle, rudder_angle, jib_angle, jib_side_flag]
            command_packet = bytearray(
                [
                    SERIAL.RX_START_FLAG,
                    mainsail_byte,
                    rudder_byte,
                    jib_angle_byte,
                    jib_side_byte,
                    SERIAL.RX_END_FLAG,
                ]
            )

            # Send the packet over serial.
            self.serial.write(command_packet)
            return 0
        except Exception:
            return 1

    def read_telemetry(self, data):
        """Read a telemetry packet into data; return 0 on success, otherwise 1.

        Firmware must send exactly SERIAL.TX_PACKET_LEN payload bytes between
        SERIAL.TX_START_FLAG (0xFF) and SERIAL.TX_END_FLAG (0xEE). Neither flag value may
        occur anywhere in the payload, including wind bytes and counters.
        Payload order: [wind_hi, wind_lo, mainsail_angle, rudder_angle,
        jib_angle, jib_side_flag, dropped_packets].

        Partial packets persist between calls. After a successful read,
        queued serial input is discarded, matching the original driver.
        """
        # Check for waiting serial data.
        while self.serial.in_waiting > 0:
            incoming_byte = self.serial.read()

            # If we see a packet start byte: set flags, clear buffer.
            if incoming_byte == SERIAL.TX_START_FLAG.to_bytes(1, "big"):
                self.packet_started = True
                self.buffer = []
            # If we see a packet end byte and the buffer is full, process the buffer and store into ``data``.
            elif (
                self.packet_started
                and incoming_byte == SERIAL.TX_END_FLAG.to_bytes(1, "big")
                and len(self.buffer) == SERIAL.TX_PACKET_LEN
            ):
                self.packet_started = False
                (
                    data["wind_angle"],
                    data["mainsail_angle"],
                    data["rudder_angle"],
                    data["jib_angle"],
                    data["jib_side_flag"],
                    data["dropped_packets"],
                ) = self._parse_packet(self.buffer)
                self.serial.reset_input_buffer()  # Clear buffer.
                return 0
            # If we have previously seen a packet start byte, add data to our buffer.
            elif self.packet_started:
                self.buffer.append(int.from_bytes(incoming_byte, "big"))
        return 1

    def _parse_packet(self, packet):
        """Decode the seven-byte telemetry payload, excluding framing flags."""
        wind_angle = (packet[0] << 8) | packet[1]

        # uint8_t to int8_t conversion for negative values.
        mainsail_angle = packet[2] - 256 if packet[2] >= 128 else packet[2]
        mainsail_angle = (
            PHYSICAL.MAINSAIL_MAX_ANGLE - mainsail_angle
        )  # invert for mechanical swap
        rudder_angle = packet[3] - 256 if packet[3] >= 128 else packet[3]
        rudder_angle -= -PHYSICAL.RUDDER_MIN_ANGLE  # undo the command offset
        jib_angle = packet[4] - 256 if packet[4] >= 128 else packet[4]
        jib_angle = (
            PHYSICAL.JIB_MAX_ANGLE + PHYSICAL.JIB_MIN_ANGLE - jib_angle
        )  # invert for mechanical swap

        jib_side_flag = packet[5]
        dropped_packets = packet[6]

        return (
            wind_angle,
            mainsail_angle,
            rudder_angle,
            jib_angle,
            jib_side_flag,
            dropped_packets,
        )

    def close(self):
        """Close the serial connection."""
        if hasattr(self, "serial"):
            self.serial.close()
