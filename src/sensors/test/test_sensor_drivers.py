"""Exercise serial parsing, framing, retained data and reader shutdown."""

import queue
import time

import pytest
from sensors import serial_sensor
from sensors.licor.licor_real import LicorAnemometer, WindSample
from sensors.vectornav.vectornav_real import NavigationSample, SailVectorNav


@pytest.mark.parametrize("yaw,expected", [(0, 0), (90, 90), (-90, 270), (180, 180), (-180, 180), (270, 270), (360, 0), (450, 90)])
def test_navigation_mapping(yaw, expected):
    sample = SailVectorNav.parse(f"$VNINS,1,2,3,{yaw},-2,7,42.8,-77.0,100*AB")
    assert sample == NavigationSample(expected, -2, 7, 42.8, -77.0)


def test_wind_mapping():
    assert LicorAnemometer.parse("S 00.16 D 305 U 00.13 V -00.09") == WindSample(
        0.16, 305
    )


@pytest.mark.parametrize(
    "driver,line",
    [
        (LicorAnemometer, "S"),
        (LicorAnemometer, "D 10"),
        (LicorAnemometer, "S nan D 10"),
        (LicorAnemometer, "S 1 D 10.5"),
        (LicorAnemometer, "S 2 D inf"),
        (SailVectorNav, "$VNINS,1,2,3,4"),
        (SailVectorNav, "$VNINS,1,2,3,4,5,bad,42,-77"),
        (SailVectorNav, "$VNINS,1,2,3,nan,5,6,42,-77"),
        (SailVectorNav, "$VNINS,1,2,3,4,5,6,42,nan"),
    ],
)
def test_invalid_samples(driver, line):
    with pytest.raises((ValueError, IndexError)):
        driver.parse(line)


class Stream:
    def __init__(self):
        self.chunks = queue.Queue()
        self.closed = False

    @property
    def in_waiting(self):
        return 1

    def read(self, size):
        try:
            chunk = self.chunks.get(timeout=0.01)
        except queue.Empty:
            return b""
        if isinstance(chunk, Exception):
            raise chunk
        return chunk

    def close(self):
        self.closed = True


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert predicate()


def test_serial_framing_retained_data_disconnect_and_cleanup(monkeypatch):
    stream = Stream()
    monkeypatch.setattr(serial_sensor.serial, "Serial", lambda *a, **k: stream)
    driver = LicorAnemometer("test")
    try:
        assert driver.read_sample() == WindSample(0.0, 0.0)
        stream.chunks.put(b"S 2 D ")
        stream.chunks.put(b"310\r\nS broken\n\xff\n")
        wait_for(lambda: driver.read_sample() == WindSample(2, 310))
        stream.chunks.put(b"x" * 5000 + b"\nS 3 D 20\n")
        wait_for(lambda: driver.read_sample() == WindSample(3, 20))
        # More than the former two-second timeout must not expire the sample.
        time.sleep(2.1)
        assert driver.read_sample() == WindSample(3, 20)
        stream.chunks.put(OSError("unplugged"))
        driver._thread.join(timeout=2)
        assert driver.read_sample() == WindSample(3, 20)
    finally:
        driver.close()
    assert stream.closed
    assert not driver._thread.is_alive()
    driver.close()


def test_vectornav_startup_is_north(monkeypatch):
    stream = Stream()
    monkeypatch.setattr(serial_sensor.serial, "Serial", lambda *a, **k: stream)
    driver = SailVectorNav("test")
    try:
        assert driver.read_sample() == NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0)
    finally:
        driver.close()
