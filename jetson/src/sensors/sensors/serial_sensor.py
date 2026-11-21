"""Background line reader shared by the migrated serial sensors."""

import threading
import logging

import serial


class SerialSensor:
    """Keep the latest complete sample without blocking the ROS executor."""

    def __init__(self, port, baud_rate=115200):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sample = self.initial_sample
        self.ser = serial.Serial(port, baud_rate, timeout=0.1)
        self._thread = threading.Thread(target=self._read_serial, daemon=True)
        self._thread.start()

    def _read_serial(self):
        pending = bytearray()
        discarding = False
        try:
            while not self._stop.is_set():
                chunk = self.ser.read(min(max(self.ser.in_waiting, 1), 4096))
                for byte in chunk:
                    if byte == 10:
                        if not discarding:
                            try:
                                sample = self.parse(pending.decode("ascii").strip())
                            except (UnicodeError, ValueError, IndexError):
                                pass
                            else:
                                with self._lock:
                                    self._sample = sample
                        pending.clear()
                        discarding = False
                    elif not discarding:
                        pending.append(byte)
                        if len(pending) > 4096:
                            pending.clear()
                            discarding = True
        except (OSError, serial.SerialException) as exc:
            logging.getLogger(__name__).warning(
                "Sensor serial read failed; retaining last sample: %s", exc
            )

    def read_sample(self):
        """Return initial values or the last sample, even after silence/disconnect."""
        with self._lock:
            return self._sample

    def close(self):
        """Stop the reader before closing its serial port; safe to call twice."""
        self._stop.set()
        self._thread.join()
        self.ser.close()
