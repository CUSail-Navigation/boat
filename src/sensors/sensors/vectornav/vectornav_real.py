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
    # Compass heading in degrees: north=0, east=90, increasing clockwise.
    initial_sample = NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0)

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
        # Preserve the sensor compass convention, normalizing signed yaw to [0, 360).
        return NavigationSample(yaw % 360.0, pitch, roll, latitude, longitude)
