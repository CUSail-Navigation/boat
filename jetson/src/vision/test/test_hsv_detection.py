"""HSV detector on synthetic frames, without a camera."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from vision.hsv_detection import HSVBuoyDetector  # noqa: E402

ORANGE = (0, 128, 255)  # BGR
GREEN = (40, 140, 40)


def frame(background=GREEN):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:] = background
    return image


def test_finds_orange_disc():
    image = frame()
    cv2.circle(image, (300, 200), 40, ORANGE, -1)
    detection, _ = HSVBuoyDetector().process_frame(image)
    assert detection is not None
    cx, cy = detection["center"]
    assert abs(cx - 300) <= 2 and abs(cy - 200) <= 2
    x1, y1, x2, y2 = detection["box"]
    assert abs((x2 - x1) - 80) <= 3 and abs((y2 - y1) - 80) <= 3
    # A disc fills about pi/4 of its box
    assert 0.7 <= detection["confidence"] <= 0.85


def test_ignores_background_and_small_blobs():
    image = frame()
    cv2.circle(image, (300, 200), 4, ORANGE, -1)
    detection, _ = HSVBuoyDetector(min_area=100).process_frame(image)
    assert detection is None


def test_picks_largest_blob():
    image = frame()
    cv2.circle(image, (100, 100), 15, ORANGE, -1)
    cv2.circle(image, (450, 300), 50, ORANGE, -1)
    detection, _ = HSVBuoyDetector().process_frame(image)
    cx, cy = detection["center"]
    assert abs(cx - 450) <= 2 and abs(cy - 300) <= 2
