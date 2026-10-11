"""Fake serial connection shared by driver tests."""

from unittest.mock import Mock

import pytest
from sailing.teensy import serial_port


class FakeSerial:
    def __init__(self):
        self.incoming = bytearray()
        self.written = bytearray()
        self.is_open = True
        self.write_error = None
        self.read_error = None

    @property
    def in_waiting(self):
        if self.read_error:
            raise self.read_error
        return len(self.incoming)

    def read(self, size=1):
        data = bytes(self.incoming[:size])
        del self.incoming[:size]
        return data

    def write(self, data):
        if self.write_error:
            raise self.write_error
        self.written.extend(data)
        return len(data)

    def reset_input_buffer(self):
        self.incoming.clear()

    def close(self):
        self.is_open = False


@pytest.fixture
def transport(monkeypatch):
    transport = FakeSerial()
    monkeypatch.setattr(serial_port.serial, "Serial", Mock(return_value=transport))
    return transport


@pytest.fixture
def driver(transport):
    driver = serial_port.SerialPort("/dev/test-teensy")
    yield driver
    driver.close()
