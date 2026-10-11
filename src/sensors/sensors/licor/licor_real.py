"""LI-COR ASCII parsing migrated from sailbot's sailboat_sensors package."""

import math
from dataclasses import dataclass

from sensors.serial_sensor import SerialSensor


@dataclass(frozen=True)
class WindSample:
    speed: float
    direction: int


class LicorAnemometer(SerialSensor):
    # Match sailbot startup wind values.
    initial_sample = WindSample(0.0, 0)

    @staticmethod
    def parse(line):
        """Read the existing ``S 00.16 D 305 U ... V ...`` output format."""
        tokens = line.split()
        speed = float(tokens[tokens.index("S") + 1])
        direction = int(tokens[tokens.index("D") + 1])
        if not math.isfinite(speed):
            raise ValueError("Invalid wind sample")
        return WindSample(speed, direction)
