# vision

Detects buoys on the Intel RealSense color stream and publishes each detection on
`/buoy_detection` (`interface/msg/BuoyDetection`), with its position from the aligned depth stream.
Frames are only processed while `current_mode` is `search`.

## Detectors

The `detector` parameter picks one:

- `yolo` (default): the `buoy2.pt` model. Confidence is YOLO's; `detection_threshold` sets the minimum.
- `hsv`: a color threshold ported from sailbot's old OpenCV detector. It takes the largest blob inside
  `hsv_lower`-`hsv_upper` (orange by default) bigger than `hsv_min_area` pixels. Confidence is how
  much of its box the blob fills (about 0.6-0.8 for a round buoy). It needs no torch or ultralytics,
  so it is useful for testing the search, but it fires on anything orange.

## Jetson-only dependencies

The camera node needs these, which are **not** in `jetson/setup/Dockerfile` (CI builds and tests
the package without them; `vision/depth.py` and, where OpenCV is installed,
`vision/hsv_detection.py` are tested there):

- `pyrealsense2` (librealsense built for ARM64)
- for `detector: yolo` only: `torch` from the Jetson wheel index
  (`https://pypi.jetson-ai-lab.io/jp6/cu126`), `ultralytics` and `numpy<2`

## Model weights

`*.pt` files are gitignored. Copy `buoy2.pt` into `jetson/src/vision/vision/` **before** `colcon build`
so it is installed with the package, or set the `model_path` parameter to its location.

## Topics

| Topic | Type | |
|---|---|---|
| `/buoy_detection` | `interface/msg/BuoyDetection` | Stamped detection: position (x right, y forward, z down), confidence, width |
| `/buoy_position`, `/buoy_distance` | `geometry_msgs/Point`, `std_msgs/Float32` | Unstamped, for the webserver |
| `update_detection_threshold` (sub) | `std_msgs/Float32` | Change the YOLO confidence threshold live (0-1) |
| `current_mode` (sub, latched) | `std_msgs/String` | Only `search` processes frames |
