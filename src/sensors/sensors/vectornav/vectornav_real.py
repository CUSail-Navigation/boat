"""VectorNav parsing migrated from sailbot's sailboat_sensors package."""

import logging
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
    # Heading in [-180, 180): east=0, north=90; counter-clockwise positive.
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
        # Convert the sensor compass bearing once, at the hardware boundary.
        logging.getLogger(__name__).debug("Raw VectorNav compass heading: %s", yaw)
        return NavigationSample((90.0 - yaw + 180.0) % 360.0 - 180.0, pitch, roll, latitude, longitude)
