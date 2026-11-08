import serial
from sailing.constants import PHYSICAL, SERIAL


class TeensySerialDriver:
    """A helper class that handles serial commands and telemetry for the Teensy."""

    def __init__(self, port):
        """Open the serial connection."""
        self.buffer = []
        self.packet_started = False
        self.serial = serial.Serial(port, baudrate=SERIAL.BAUD_RATE)

    def send_command(self, mainsail_angle, rudder_angle, jib_angle, jib_side_flag):
        """
        Send a properly formatted command packet to the Teensy.
        Each value passed in must be valid as defined by ``constants.py``
        (and must therefore fit into a byte).

        :param mainsail_angle: new mainsail angle to set (integer).
        :param rudder_angle: new rudder angle to set (integer).
        :param jib_angle: new jib angle to set (integer).
        :param jib_side_flag: new side to set the jib on (0 or 1).
        :raises serial.SerialException: if writing to the port fails.
        """
        # Command payload format between start/end flags: [mainsail_angle, rudder_angle, jib_angle, jib_side_flag]
        command_packet = [
            SERIAL.RX_START_FLAG,
            mainsail_angle,
            rudder_angle - PHYSICAL.RUDDER_MIN_ANGLE,  # Offset for the Teensy's 0-90 range.
            jib_angle,
            jib_side_flag,
            SERIAL.RX_END_FLAG,
        ]
        self.serial.write(bytes(command_packet))

    def read_telemetry(self, data):
        """
        Reads a telemetry packet into the dictionary ``data``. This function
        expects the Teensy to send exactly ``SERIAL.TX_PACKET_LEN`` payload
        bytes between ``SERIAL.TX_START_FLAG`` and ``SERIAL.TX_END_FLAG``.

        If several packets are staged, ``data`` will always reflect the most
        recent complete one. A packet that is still in progress when this
        function returns will be completed on the next call.

        :param data: the dictionary to fill.
        :return: ``True`` if at least one complete packet was read, ``False`` otherwise.
        """
        received = False
        while self.serial.in_waiting > 0:
            incoming_byte = self.serial.read()[0]

            # If we see START_FLAG, prepare for reading in a new packet.
            if incoming_byte == SERIAL.TX_START_FLAG:
                self.packet_started = True
                self.buffer = []
            # If we are currently processing a packet, add data to the buffer.
            elif self.packet_started and incoming_byte != SERIAL.TX_END_FLAG:
                if len(self.buffer) < SERIAL.TX_PACKET_LEN:
                    self.buffer.append(incoming_byte)
                else:  # Drop packet: buffer is full, but we have not reached END_FLAG.
                    self.packet_started = False
            # If we see END_FLAG, either process a full buffer into ``data``, or drop the packet.
            elif self.packet_started and incoming_byte == SERIAL.TX_END_FLAG:
                self.packet_started = False
                if len(self.buffer) == SERIAL.TX_PACKET_LEN:
                    data["wind_angle"] = (self.buffer[0] << 8) | self.buffer[1]
                    data["mainsail_angle"] = self.buffer[2]
                    data["rudder_angle"] = self.buffer[3] + PHYSICAL.RUDDER_MIN_ANGLE  # Undo the Teensy's offset.
                    data["jib_angle"] = self.buffer[4]
                    data["jib_side_flag"] = self.buffer[5]
                    data["dropped_packets"] = self.buffer[6]
                    received = True

        return received

    def close(self):
        """Close the serial connection."""
        self.serial.close()
