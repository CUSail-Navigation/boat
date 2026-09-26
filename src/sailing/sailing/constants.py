"""Boat limits, node settings, and serial protocol constants."""

from dataclasses import dataclass


@dataclass(frozen=True)
class _Physical:
    """Actuator limits in degrees and jib-side conventions."""

    RUDDER_MIN_ANGLE: int = -45
    RUDDER_MAX_ANGLE: int = 45
    MAINSAIL_MIN_ANGLE: int = 0
    MAINSAIL_MAX_ANGLE: int = 90
    JIB_MIN_ANGLE: int = 0
    JIB_MAX_ANGLE: int = 90
    JIB_SIDE_PORT: int = 0
    JIB_SIDE_STB: int = 1


@dataclass(frozen=True)
class _Serial:
    """Serial protocol values; TX/RX are from the Teensy's perspective."""

    TX_START_FLAG: int = 0xFF
    TX_END_FLAG: int = 0xEE
    TX_PERIOD_MS: int = 500
    TX_PACKET_LEN: int = 7
    RX_START_FLAG: int = 0xFF
    RX_END_FLAG: int = 0xEE
    RX_PERIOD_MS: int = 500
    BAUD_RATE: int = 9600


@dataclass(frozen=True)
class _Teensy:
    """Default settings and initial actuator positions for the Teensy node."""

    DEFAULT_PORT: str = "/dev/ttyACM0"
    DEFAULT_TELEMETRY_POLL_PERIOD_SECONDS: float = 0.5
    INITIAL_MAINSAIL_ANGLE: int = 0
    INITIAL_RUDDER_ANGLE: int = 0
    INITIAL_JIB_ANGLE: int = 0
    WARNING_THROTTLE_SECONDS: float = 5.0


PHYSICAL = _Physical()
SERIAL = _Serial()
TEENSY = _Teensy()

RL_INFERENCE_PERIOD_SECONDS = 0.2
