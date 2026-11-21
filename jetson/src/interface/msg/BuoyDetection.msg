# One buoy detection from the camera, stamped at frame capture time.

std_msgs/Header header
geometry_msgs/Point position  # camera frame, meters: x = right, y = forward, z = down
float32 confidence            # detector confidence, 0-1
float32 width_m               # estimated physical width of the detected box
