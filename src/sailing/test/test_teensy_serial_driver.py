"""Regression tests for the migrated serial driver's legacy behavior.

Cover framing, rudder wire center 45, command encoding, error handling, and
cleanup using a fake serial connection. Telemetry payloads must exclude start
and end flag values.
These tests preserve the migration's behavior; they do not verify hardware calibration.
"""

import pytest
from sailing.constants import PHYSICAL
from sailing.teensy import teensy_serial_driver


def frame(wind=300, mainsail=20, rudder=45, jib=30, side=1, dropped=7):
    return bytes([0xFF, wind >> 8, wind & 0xFF, mainsail, rudder, jib, side, dropped, 0xEE])


def test_open_uses_configured_port_and_baudrate(transport):
    teensy_serial_driver.TeensySerialDriver("/dev/test-teensy")
    args, kwargs = teensy_serial_driver.serial.Serial.call_args
    assert (args[0] if args else kwargs["port"]) == "/dev/test-teensy"
    assert kwargs["baudrate"] == 9600


def test_open_failure_is_visible(transport):
    teensy_serial_driver.serial.Serial.side_effect = OSError("disconnected")
    with pytest.raises(OSError, match="disconnected"):
        teensy_serial_driver.TeensySerialDriver("/dev/test-teensy")


def test_empty_read_leaves_data_unchanged(driver):
    data = {"previous": 123}
    assert not driver.read_telemetry(data)
    assert data == {"previous": 123}


def test_decode_telemetry(driver, transport):
    transport.incoming.extend(frame())
    data = {}
    assert driver.read_telemetry(data)
    assert data == dict(
        wind_angle=300,
        mainsail_angle=20,
        rudder_angle=0,
        jib_angle=30,
        jib_side_flag=1,
        dropped_packets=7,
    )


@pytest.mark.parametrize("split", range(1, 9))
def test_fragmented_frame(driver, transport, split):
    packet = frame()
    transport.incoming.extend(packet[:split])
    data = {"previous": 123}
    assert not driver.read_telemetry(data)
    assert data == {"previous": 123}
    transport.incoming.extend(packet[split:])
    assert driver.read_telemetry(data)
    assert data["wind_angle"] == 300


@pytest.mark.parametrize("garbage", [b"noise", frame()[:-1] + b"\x00", b"\xff" + b"\x01" * 64])
def test_resynchronize_after_noise_or_bad_frame(driver, transport, garbage):
    transport.incoming.extend(garbage + frame())
    data = {}
    for _ in range(3):
        if driver.read_telemetry(data):
            break
    assert data.get("wind_angle") == 300


def test_newest_queued_packet_wins(driver, transport):
    transport.incoming.extend(frame(wind=100) + frame(wind=200))
    data = {}
    assert driver.read_telemetry(data)
    assert data["wind_angle"] == 200


def test_short_packet_is_not_merged_with_following_bytes(driver, transport):
    transport.incoming.extend(bytes([0xFF, 1, 44, 20, 0xEE, 45, 30, 1, 7, 0xEE]))
    assert not driver.read_telemetry({})


@pytest.mark.parametrize("position", ["minimum", "neutral", "maximum"])
def test_command_bytes(driver, transport, position):
    rudder = {
        "minimum": PHYSICAL.RUDDER_MIN_ANGLE,
        "neutral": 0,
        "maximum": PHYSICAL.RUDDER_MAX_ANGLE,
    }[position]
    # The migrated protocol uses 45 as the rudder wire center.
    wire = rudder + 45
    side = PHYSICAL.JIB_SIDE_STB
    driver.send_command(20, rudder, 30, side)
    assert transport.written == bytes([255, 20, wire, 30, side, 238])


@pytest.mark.parametrize("args", [(-1, 0, 30, 0), (20, 0, 256, 0), (20, 0, 30, -1)])
def test_value_outside_byte_range_raises(driver, transport, args):
    with pytest.raises(ValueError):
        driver.send_command(*args)
    assert not transport.written


def test_read_failure_is_visible(driver, transport):
    transport.read_error = OSError("unplugged")
    with pytest.raises(OSError, match="unplugged"):
        driver.read_telemetry({})


def test_write_failure_is_visible(driver, transport):
    transport.write_error = OSError("unplugged")
    with pytest.raises(OSError, match="unplugged"):
        driver.send_command(20, 0, 30, 0)
    assert not transport.written


def test_close_closes_connection(driver, transport):
    driver.close()
    assert not transport.is_open
