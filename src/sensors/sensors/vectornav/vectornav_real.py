"""VectorNav parsing migrated from sailbot's sailboat_sensors package."""

import math
from dataclasses import dataclass

from sensors.serial_sensor import SerialSensor


@dataclass(frozen=True)
class NavigationSample:
    yaw: float
    pitch: float
    roll: float
    latitude: float
    longitude: float


class SailVectorNav(SerialSensor):
    # Match sailbot startup values (raw yaw zero converts to 90 degrees).
    initial_sample = NavigationSample(90.0, 0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def parse(line):
        """Use the boat's existing ASCII layout: YPR and lat/lon at fields 4–8.

        The sensor must already be configured to stream this layout, as in
        sailbot. This is not a decoder for arbitrary VectorNav output modes.
        """
        fields = line.split(",")
        yaw, pitch, roll, latitude, longitude = map(float, fields[4:9])
        if not all(map(math.isfinite, (yaw, pitch, roll, latitude, longitude))):
            raise ValueError("Non-finite navigation sample")
        # Preserve sailbot main's heading convention: 0=east, 90=north.
        return NavigationSample((450.0 - yaw) % 360.0, pitch, roll, latitude, longitude)
