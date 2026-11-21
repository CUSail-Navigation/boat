# vision

Detects buoys with YOLO on the Intel RealSense color stream and publishes each detection on
`/buoy_detection` (`interface/msg/BuoyDetection`), with its position from the aligned depth stream.
Frames are only processed while `current_mode` is `search`.

## Jetson-only dependencies

The camera node needs these, which are **not** in `jetson/setup/Dockerfile` (CI builds and tests
the package without them; only `vision/depth.py` is tested there):

- `pyrealsense2` (librealsense built for ARM64)
- `torch` from the Jetson wheel index (`https://pypi.jetson-ai-lab.io/jp6/cu126`)
- `ultralytics` and `numpy<2`

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
