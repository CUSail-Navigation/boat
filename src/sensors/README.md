# Sensor nodes

The LI-COR and VectorNav integrations were migrated from
`sailbot/src/sailboat_sensors` (local `origin/main`, `1f69daa`, also present on
`comp_day3_force_fixes`). Both run in the standalone `sensors` ROS 2 package.

This is a temporary setup for the lake test. Run the nodes directly on the
boat's Linux computer with ROS 2 Humble, colcon, and PySerial installed. Hardcode
the settings in the node files below; no launch file or configuration file is
needed.

## Identify and hardcode the serial ports

Do this on the boat computer, with the sensor nodes stopped:

1. Disconnect both sensor USB/serial adapters and list the current devices:

   ```bash
   ls -l /dev/serial/by-id/
   ```

   If the directory does not exist yet, connect the first adapter and try again.

2. Connect only the LI-COR adapter, run the command again, and record the new
   full `/dev/serial/by-id/...` path. Repeat with the VectorNav adapter. Connecting
   one at a time identifies which adapter belongs to which sensor; the displayed
   name may identify the adapter rather than the sensor. Keep each adapter with
   its sensor.

3. Hardcode those paths in the following files, in the `(name, default)` pairs
   near the start of each node's `__init__`:

   | Sensor | File (relative to the repo root) | Value to edit |
   | --- | --- | --- |
   | LI-COR | `src/sensors/sensors/licor/licor_node.py` | The string paired with `"licor_port"` (currently `"/dev/ttyUSB0"`) |
   | VectorNav | `src/sensors/sensors/vectornav/vectornav_node.py` | The string paired with `"vectornav_port"` (currently the old boat's FTDI path) |

   For example, replace the LI-COR pair with your actual path:

   ```python
   ("licor_port", "/dev/serial/by-id/REPLACE_WITH_YOUR_LICOR_ADAPTER"),
   ```

   Use the `/dev/serial/by-id/...` path itself, not the `ttyUSB` target shown
   after the arrow: `ttyUSB` numbering can change after reconnecting or rebooting.
   If the adapters do not expose unique by-id paths, use `/dev/serial/by-path/`
   and keep them in the same USB sockets. As a temporary fallback, compare
   `ls /dev/ttyUSB* /dev/ttyACM*` before and after connecting each adapter and
   hardcode the new device path, then recheck it before the lake test.

4. LI-COR's `baud_rate` parameter defaults to `115200`; VectorNav uses a fixed
   `115200` baud as in sailbot. Other settings are `timer_period` and LI-COR's
   `min_speed` and `alpha`. Ordinary `ros2 run` uses these defaults.

The process must have permission to open both serial devices. On Ubuntu, if you
get a permission error, check `ls -l` on the resolved device and `groups`; serial
devices commonly require membership in `dialout`. If that is the device's group,
run `sudo usermod -aG dialout "$USER"`, then log out and back in.
These drivers read the existing sensor configuration; they do not program it.

## Build and run directly

From the repo root, build after editing the hardcoded settings:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select sensors
source install/setup.bash
```

Run each node in a separate terminal. In **each terminal**, change to the repo
root and source both setup files before running its command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sensors licor_node
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sensors vectornav_node
```

Rebuild and restart the nodes after changing hardcoded settings. Both nodes
always open real serial hardware; simulated sensor mode has been removed.

## Published topics

The commands above publish at the root namespace (`/wind`, `/gps`, etc.).
Run the controller in the same namespace. If you need a namespace, append
`--ros-args -r __ns:=/sailbot` to each node and the controller command; the
topics will then be under `/sailbot/`.

| Topic | ROS message | Meaning |
| --- | --- | --- |
| `wind` | `std_msgs/Int32` | Apparent wind direction, degrees in [0, 360) |
| `wind_speed` | `std_msgs/Float32` | Apparent wind speed, m/s (added for the boat controller) |
| `gps` | `sensor_msgs/NavSatFix` | Latitude/longitude, degrees; empty header and zero altitude, as in sailbot |
| `imu` | `geometry_msgs/Vector3` | x=roll, y=pitch, z=yaw, all in degrees |

LI-COR expects the original whitespace-separated `S 00.16 D 305 U ... V ...`
stream. Wind direction retains sailbot's circular exponential filter (`alpha`,
default 0.3) and holds its last value below `min_speed` (default 1.0 m/s). The
initial held direction is zero. Speed is published without filtering even below
that threshold. Sensor installation determines the relative wind direction zero.

VectorNav expects the original comma-separated ASCII stream with yaw, pitch,
roll, latitude and longitude at zero-based fields 4–8. Other output layouts and
binary packets are unsupported. As in sailbot main, published yaw is
`(450 - sensor_yaw) % 360` (east=0, north=90 for a north-zero clockwise input).
Pitch and roll are unchanged. This deliberately preserves the deployed conversion,
not the conflicting north-zero comment in the old driver. GPS covariance is
unknown, altitude is zero (the old default, not a measured altitude), and INS/GNSS fix status is not decoded by this migration.
The parser retains the old layout assumption and does not validate checksums.

Both nodes accept `timer_period` (default 0.5 seconds, matching sailbot's deployed
launch configuration). As in sailbot, they
publish initial values before data arrives: zero wind speed/direction, zero
latitude/longitude/pitch/roll, and yaw 90 degrees (the conversion of raw yaw zero).
They keep publishing the last valid sample indefinitely during silence or after
a serial read failure. There is no stale-data timeout or automatic controller
stop. Restart a node after reconnecting a failed serial device.

## Remaining differences from sailbot

- Packaging: use `ros2 run sensors licor_node` and `ros2 run sensors vectornav_node`
  instead of the old `sailboat_sensors` package's `licor` and `vectornav` executables.
  This lake-test setup runs directly without the old launch/config files.
- Topic namespaces: GPS and IMU topics are now relative (`gps`, `imu`) instead of
  absolute (`/gps`, `/imu`). Root-namespace behavior is unchanged; namespaced
  consumers must use matching topic names or remappings. Wind was already relative.
- Wind speed: `wind_speed` (`Float32`, m/s) is newly published for the boat controller.
  Supplying it can let the existing RL scaffold reach its unfinished inference
  code once all inputs arrive; that code currently references undefined actuator
  variables and must be completed before using it as a controller.
- Correctness fixes: complete samples are converted before being committed and read
  atomically, preventing mixed fields or malformed numeric strings from reaching
  ROS callbacks. Non-finite values are rejected because they can break wind angle
  conversion or propagate NaNs into navigation. Other field parsing follows the
  old driver: integer LI-COR direction, no added direction wrapping, coordinate
  range filtering, negative-speed filtering, or checksum-suffix stripping.
- Serial cleanup: the shared reader preserves partial lines across read timeouts,
  discards malformed/non-ASCII lines instead of terminating the reader, and bounds
  line buffering at 4096 bytes to prevent unlimited growth on a corrupt stream.
  Reader threads and ports are closed on shutdown. Serial read failures are logged;
  the last sample continues to be published.
- Simulated sensor mode and its random-data generators have been removed. Automated
  unit tests mock the hardware boundary only; there is no runtime simulation path.

GPS headers remain empty, altitude remains zero, VectorNav remains fixed at
115200 baud, and per-publication informational logging matches the old driver.
No additional timer/filter parameter validation is performed by these nodes.

The yaw conversion, roll/pitch mapping, circular wind filter and low-speed hold
remain unchanged. Both ROS nodes still default to 115200 baud. The 0.5-second
publication period and continuous last-sample publication match the old boat setup.

## Verify

```bash
ros2 topic echo /wind
ros2 topic echo /wind_speed
ros2 topic echo /gps
ros2 topic echo /imu
colcon test --packages-select sensors --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

Tests cover parsing, partial serial lines, malformed data, retained readings,
disconnects, shutdown, circular filtering and namespaced ROS delivery.
A dockside hardware check is still required to verify sensor output configuration,
port selection, mounting orientation, and live values.
