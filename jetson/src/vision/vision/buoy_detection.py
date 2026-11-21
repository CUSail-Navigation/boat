"""YOLO buoy detector. Imports torch and ultralytics, so it only runs where they are installed."""

import cv2
import torch
from ultralytics import YOLO


class YOLOBuoyDetector:
    def __init__(self, model_path, detection_threshold=0.85, show_frames=False):
        # Fall back to CPU so the detector can run in sim and on laptops.
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.device == "cuda":
            torch.cuda.init()
            torch.cuda.synchronize()

        self.model = YOLO(model_path)
        self.detection_threshold = detection_threshold
        self.show_frames = show_frames

    def update_parameters(self, detection_threshold=None):
        if detection_threshold is not None:
            self.detection_threshold = detection_threshold

    def process_frame(self, frame):
        """
        Return the most confident detection as a dict with 'center' (px), 'confidence'
        and 'box' (x1, y1, x2, y2 in px), or None, plus the (optionally annotated) frame.
        """
        best = None
        results = self.model(
            frame,
            stream=True,
            conf=self.detection_threshold,
            device=self.device,
            verbose=False,
        )
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                confidence = float(box.conf)
                if best is None or confidence > best["confidence"]:
                    best = {
                        "center": center,
                        "confidence": confidence,
                        "box": (x1, y1, x2, y2),
                    }
                if self.show_frames:
                    label = f"{result.names[int(box.cls[0])]} {confidence:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
                    cv2.putText(
                        frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_PLAIN,
                        3,
                        (255, 0, 0),
                        3,
                    )
                    cv2.circle(frame, center, 3, (255, 0, 0), -1)
        if self.show_frames:
            self.show(frame)
        return best, frame

    def show(self, frame):
        """Display the frame, turning display off when running headless."""
        try:
            cv2.imshow("Camera", frame)
            cv2.waitKey(1)
        except cv2.error:
            self.show_frames = False
