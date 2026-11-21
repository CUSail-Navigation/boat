"""Exercise serial parsing, framing, retained data and reader shutdown."""

import queue
import time

import pytest
from sensors import serial_sensor
from sensors.vectornav.vectornav_real import NavigationSample, SailVectorNav


@pytest.mark.parametrize(
    "yaw,expected",
    [(0, 0), (90, 90), (-90, -90), (180, 180), (-180, -180)],
)
def test_navigation_mapping(yaw, expected):
    sample = SailVectorNav.parse(f"$VNINS,1,2,3,{yaw},-2,7,42.8,-77.0,100*AB")
    assert sample == NavigationSample(expected, -2, 7, 42.8, -77.0)


@pytest.mark.parametrize(
    "driver,line",
    [
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
    driver = SailVectorNav("test")
    try:
        assert driver.read_sample() == NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0)
        stream.chunks.put(b"$VNINS,1,2,3,90,")
        stream.chunks.put(b"-2,7,42.8,-77.0\r\n$VNINS,broken\n\xff\n")
        wait_for(
            lambda: driver.read_sample() == NavigationSample(90, -2, 7, 42.8, -77.0)
        )
        stream.chunks.put(b"x" * 5000 + b"\n$VNINS,1,2,3,180,0,0,42,-77\n")
        wait_for(lambda: driver.read_sample() == NavigationSample(180, 0, 0, 42, -77))
        # More than the former two-second timeout must not expire the sample.
        time.sleep(2.1)
        assert driver.read_sample() == NavigationSample(180, 0, 0, 42, -77)
        stream.chunks.put(OSError("unplugged"))
        driver._thread.join(timeout=2)
        assert driver.read_sample() == NavigationSample(180, 0, 0, 42, -77)
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
