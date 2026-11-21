"""
Color-threshold (HSV) buoy detector, ported from sailbot's OpenCVBuoyDetector. Needs only
OpenCV and numpy, so it runs where torch and ultralytics are not installed.
"""

import cv2
import numpy as np


class HSVBuoyDetector:
    def __init__(
        self,
        hsv_lower=(0, 127, 63),
        hsv_upper=(20, 255, 255),
        min_area=100,
        show_frames=False,
    ):
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area = min_area  # px, smaller blobs are ignored
        self.show_frames = show_frames

    def increase_contrast(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lightness, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        lab = cv2.merge((clahe.apply(lightness), a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def reduce_glare(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, glare_mask = cv2.threshold(blurred, 220, 255, cv2.THRESH_BINARY)
        return cv2.bitwise_and(frame, frame, mask=cv2.bitwise_not(glare_mask))

    def process_frame(self, frame):
        """
        Return the largest in-range blob as a dict with 'center' (px), 'confidence' and
        'box' (x1, y1, x2, y2 in px), or None, plus the (optionally annotated) frame.
        Confidence is how much of its bounding box the blob fills: about 0.6-0.8 for a
        round buoy, lower for ragged clutter.
        """
        frame = self.increase_contrast(self.reduce_glare(frame))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(largest)
                best = {
                    "center": (x + w // 2, y + h // 2),
                    "confidence": float(area / (w * h)),
                    "box": (x, y, x + w, y + h),  # same edges as YOLO's xyxy
                }

        if self.show_frames:
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            if best is not None:
                x1, y1, x2, y2 = best["box"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.circle(frame, best["center"], 5, (255, 0, 0), -1)
            self.show(frame)
        return best, frame

    def show(self, frame):
        """Display the frame, turning display off when running headless."""
        try:
            cv2.imshow("Camera", frame)
            cv2.waitKey(1)
        except cv2.error:
            self.show_frames = False
