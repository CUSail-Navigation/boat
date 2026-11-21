"""
Search a circular area for a buoy with the camera, then sail to it.

A coverage map records what the camera has looked at, every tick and in every state. The
boat sweeps the area on evenly spaced cross-wind lines, and the map decides which
stretches of each line still need sailing, so water already seen is skipped and a line
cut short by an approach is finished from where the boat left it. Detections build
candidates (one particle filter each) without leaving the leg. Once a candidate has
enough hits and a tight enough estimate, the boat approaches it. A candidate that should
be visible but is not seen is rejected; its place is remembered so it takes more
evidence to pull the boat back there. The next leg is then planned from wherever the
boat is.

Waypoints go to the waypoint service as a whole new queue. The queue is versioned: a set
made with a stale version is rejected, so the search refreshes the version and resends.
"""

import math
import uuid
from collections import deque
from enum import Enum

import rclpy
import utm
from geometry_msgs.msg import Point, Vector3
from interface.msg import BuoyDetection
from interface.msg import Waypoint as WaypointMsg
from interface.srv import Waypoint as WaypointSrv
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int32, String

from mission.search_core import (
    Candidate,
    CoverageMap,
    ParticleFilter,
    RejectedZone,
    compass_to_math,
    measurement_to_point,
    point_visible,
    range_std_for,
)

# Latched topics: late joiners and restarted nodes still get the last value.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

PARAMS = {
    # Search area and coverage
    "search_center_lat": 42.44415534324632,
    "search_center_lon": -76.48367002684182,
    "search_radius": 100.0,  # m, used when set_mode sends no radius
    "leg_spacing": 16.0,  # m, ~2x the lateral range the camera covers well
    "search_direction": 0.0,  # deg, used when wind is unknown
    "wind_aligned_pattern": True,  # legs cross-wind when wind is known
    "waypoint_reached_radius": 5.0,  # m, matches the waypoint follower's pop radius
    # Coverage map and leg planning
    "coverage_cell_size": 2.0,  # m
    "prior_center_std": 0.0,  # m, > 0 searches near the center first
    "detect_rate": 1.0,  # per s a buoy in view is found, at reliable range
    "reliable_detection_range": 10.0,  # m, detect_rate falls to 0 at max_visible_range
    "min_leg_gain": 0.35,  # sail a leg if it finds this much of what a fresh one would
    "min_leg_length": 10.0,  # m, shorter legs are popped before they are sailed
    "no_go_zone": 60.0,  # deg off the wind
    "nominal_speed": 1.0,  # m/s, only for leg timeouts
    "leg_timeout_factor": 3.0,  # replan when a leg takes this many times too long
    # Sensor conventions and camera mounting
    "heading_convention": "compass",  # 'compass' (CW from north) or 'math' (CCW from east)
    "camera_hfov_deg": 85.2,
    "camera_yaw_offset_deg": 0.0,  # camera axis relative to bow, clockwise positive
    "camera_forward_offset": 0.0,  # m, camera ahead of GPS antenna
    "compensate_heel": False,  # verify roll sign on hardware before enabling
    "max_pose_age_sec": 0.5,
    "max_sensor_age_sec": 2.0,
    # Measurement model
    "num_particles": 500,
    "bearing_std_deg": 8.0,
    "range_std_base": 1.0,  # m
    "range_std_quadratic": 0.02,  # m per m^2: 3 m at 10 m, 9 m at 20 m
    "process_std": 0.3,  # m per sqrt(s) of resample jitter
    "min_detection_range": 0.3,  # m, RealSense minimum depth
    "max_detection_range": 25.0,  # m
    "min_confidence": 0.5,
    "association_radius": 8.0,  # m
    # Evidence and recovery
    "min_hits_to_approach": 5,
    "max_spread_to_approach": 6.0,  # m
    "candidate_timeout_sec": 10.0,  # passive candidate dropped after this long unseen
    "max_candidates": 5,
    "min_visible_range": 4.0,  # m, closer than this the buoy leaves the frame
    "max_visible_range": 15.0,  # m, farther than this YOLO is unreliable
    "miss_timeout_sec": 4.0,  # time visible-but-unseen before rejecting
    "reacquire_after_sec": 2.0,
    "reacquire_timeout_sec": 30.0,
    "approach_timeout_sec": 240.0,
    "waypoint_update_distance": 3.0,  # m the estimate must move before resending
    "reject_radius": 10.0,  # m
    "reject_duration_sec": 180.0,  # detections ignored this long; extra hits after
    "buoy_reached_radius": 5.0,  # m, no tighter than the follower's pop radius
    "confirm_range": 8.0,  # m, buoy must be seen this close before it counts as reached
    "confirm_hits": 3,  # hits needed within confirm_range
    "tick_period": 0.2,
}


class SearchState(Enum):
    IDLE = "idle"  # not in search mode
    SEARCHING = "searching"  # sailing search legs, tracking candidates passively
    APPROACHING = "approaching"  # sailing to a candidate with enough evidence
    REACQUIRING = "reacquiring"  # candidate out of view; holding course to its estimate
    DONE = "done"  # buoy reached


class BuoySearch(Node):
    def __init__(self):
        super().__init__("buoy_search")
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMS}

        self.create_subscription(NavSatFix, "/gps", self.gps_callback, 10)
        self.create_subscription(Vector3, "/imu", self.imu_callback, 10)
        self.create_subscription(Int32, "wind_angle", self.wind_callback, 10)
        self.create_subscription(
            BuoyDetection, "/buoy_detection", self.buoy_detection_callback, 10
        )
        # Search area from the mode manager (x = lon, y = lat, z = radius in m, 0 = configured)
        self.create_subscription(
            Point, "search_center_point", self.search_center_callback, LATCHED_QOS
        )
        self.current_mode = "manual"
        self.create_subscription(
            String, "current_mode", self.mode_callback, LATCHED_QOS
        )

        self.status_pub = self.create_publisher(String, "search_status", LATCHED_QOS)
        self.coverage_pub = self.create_publisher(Float32, "search_coverage", 10)

        self.waypoint_client = self.create_client(WaypointSrv, "mutate_waypoint_queue")
        self.pending_waypoints = None  # [(lat, lon)] waiting to be sent
        self.sent_waypoints = None  # [(lat, lon)] in the request in flight
        self.waypoint_future = None
        self.queue_version = None  # last version the waypoint service reported

        # Sensor state. Positions are UTM (easting, northing) in the search area's zone.
        self.wind_dir = None
        self.position = None
        self.heading = None  # math convention
        self.gps_time = None
        self.imu_time = None
        self.pose_history = deque(maxlen=100)  # (t, easting, northing)
        self.heading_history = deque(maxlen=200)  # (t, heading, roll)

        self.set_area(
            self.p["search_center_lat"],
            self.p["search_center_lon"],
            self.p["search_radius"],
        )
        self.state = SearchState.IDLE
        self.reset_search()

        self.create_timer(self.p["tick_period"], self.tick)
        self.get_logger().info("Buoy search started")

    # ------------------------------------------------------------------ helpers

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, state: SearchState, reason: str = ""):
        if state == self.state:
            return
        suffix = f": {reason}" if reason else ""
        self.get_logger().info(f"{self.state.value} -> {state.value}{suffix}")
        self.state = state
        self.status_pub.publish(String(data=state.value))

    def set_area(self, lat: float, lon: float, radius: float):
        x, y, self.zone_number, self.zone_letter = utm.from_latlon(lat, lon)
        self.center = (x, y)
        self.search_radius = max(radius, self.p["coverage_cell_size"])

    def to_latlon(self, x: float, y: float):
        return utm.to_latlon(x, y, self.zone_number, self.zone_letter)

    def sensors_fresh(self, t: float) -> bool:
        max_age = self.p["max_sensor_age_sec"]
        return (
            self.position is not None
            and self.heading is not None
            and t - self.gps_time <= max_age
            and t - self.imu_time <= max_age
        )

    @staticmethod
    def nearest_in_history(history, t: float):
        return min(history, key=lambda entry: abs(entry[0] - t)) if history else None

    # ---------------------------------------------------------------- callbacks

    def gps_callback(self, msg):
        x, y, zone_number, _ = utm.from_latlon(msg.latitude, msg.longitude)
        if zone_number != self.zone_number:
            self.get_logger().warn(
                "GPS fix is outside the search area UTM zone; ignoring.",
                throttle_duration_sec=5.0,
            )
            return
        self.position = (x, y)
        self.gps_time = self.now()
        self.pose_history.append((self.gps_time, x, y))

    def wind_callback(self, msg):
        self.wind_dir = msg.data

    def imu_callback(self, msg):
        """IMU data: z = yaw, x = roll, degrees."""
        if self.p["heading_convention"] == "compass":
            self.heading = compass_to_math(msg.z)
        else:
            self.heading = msg.z % 360
        self.imu_time = self.now()
        self.heading_history.append((self.imu_time, self.heading, msg.x))

    def search_center_callback(self, msg):
        """An all-zero center keeps the configured center; a zero radius the configured radius."""
        lat, lon = msg.y, msg.x
        if lat == 0.0 and lon == 0.0:
            lat, lon = self.p["search_center_lat"], self.p["search_center_lon"]
        radius = msg.z if msg.z > 0 else self.p["search_radius"]
        self.set_area(lat, lon, radius)
        self.get_logger().info(
            f"Search area set to {self.search_radius:.0f} m around ({lat}, {lon})."
        )
        if self.state == SearchState.SEARCHING:
            self.start_search()

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode == self.current_mode:
            return
        if self.current_mode == "search":
            self.reset_search()
            self.set_state(SearchState.IDLE, f"mode changed to '{new_mode}'")
        self.current_mode = new_mode
        if self.current_mode == "search":
            self.start_search()

    def buoy_detection_callback(self, msg: BuoyDetection):
        """Turn a camera-frame detection into a global measurement for a candidate."""
        if self.current_mode != "search" or self.state in (
            SearchState.IDLE,
            SearchState.DONE,
        ):
            return
        if msg.confidence < self.p["min_confidence"]:
            return

        # Camera frame: x right, y forward, z down
        x, forward, z = msg.position.x, msg.position.y, msg.position.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        t = stamp if stamp > 0 else self.now()

        pose = self.nearest_in_history(self.pose_history, t)
        attitude = self.nearest_in_history(self.heading_history, t)
        max_age = self.p["max_pose_age_sec"]
        if (
            pose is None
            or attitude is None
            or abs(pose[0] - t) > max_age
            or abs(attitude[0] - t) > max_age
        ):
            self.get_logger().warn(
                "Dropping detection: no boat pose near its capture time.",
                throttle_duration_sec=5.0,
            )
            return
        _, boat_x, boat_y = pose
        _, heading, roll = attitude

        if self.p["compensate_heel"]:
            r = math.radians(roll)
            x = x * math.cos(r) - z * math.sin(r)
        forward += self.p["camera_forward_offset"]

        observed_range = math.hypot(x, forward)
        if not (
            self.p["min_detection_range"]
            <= observed_range
            <= self.p["max_detection_range"]
        ):
            return

        # Relative bearing is clockwise from the bow; math heading is counter-clockwise.
        relative_bearing = (
            math.degrees(math.atan2(x, forward)) + self.p["camera_yaw_offset_deg"]
        )
        bearing = (heading - relative_bearing) % 360
        px, py = measurement_to_point(boat_x, boat_y, bearing, observed_range)

        if any(
            zone.contains(px, py) and t < zone.ignore_until
            for zone in self.rejected_zones
        ):
            return
        self.add_measurement(
            t, boat_x, boat_y, bearing, observed_range, px, py, msg.confidence
        )

    # ------------------------------------------------------------- estimation

    def add_measurement(
        self, t, boat_x, boat_y, bearing, observed_range, px, py, confidence
    ):
        """Associate a measurement with the nearest candidate, or start a new one."""
        bearing_std = self.p["bearing_std_deg"]
        range_std = range_std_for(
            observed_range, self.p["range_std_base"], self.p["range_std_quadratic"]
        )

        best = None
        best_distance = None
        for candidate in self.candidates:
            gate = max(self.p["association_radius"], 3 * candidate.filter.spread())
            distance = candidate.distance_to(px, py)
            if distance <= gate and (best_distance is None or distance < best_distance):
                best, best_distance = candidate, distance

        if best is not None and best.filter.update(
            bearing, boat_x, boat_y, observed_range, bearing_std, range_std
        ):
            dt = max(t - best.last_update, 1e-3)
            best.filter.resample(self.p["process_std"] * math.sqrt(dt))
            best.last_update = t
            # Judged from the filtered estimate, not this one noisy range
            close = best.distance_to(boat_x, boat_y) <= self.p["confirm_range"]
            best.add_hit(t, confidence, observed_range, close)
            return

        spread = max(range_std, observed_range * math.radians(bearing_std))
        candidate = Candidate(
            ParticleFilter(self.p["num_particles"], px, py, spread),
            t,
            confidence,
            observed_range,
        )
        self.candidates.append(candidate)
        if len(self.candidates) > self.p["max_candidates"]:
            # Drop the stalest candidate that is not the current target
            stale = [c for c in self.candidates if c is not self.target]
            self.candidates.remove(min(stale, key=lambda c: c.last_seen))
        self.get_logger().info(
            f"New buoy candidate at range {observed_range:.1f} m "
            f"({len(self.candidates)} tracked)."
        )

    # ----------------------------------------------------------------- control

    def tick(self):
        """Periodic decision step."""
        self.retry_pending_waypoints()
        if self.current_mode != "search" or self.state in (
            SearchState.IDLE,
            SearchState.DONE,
        ):
            return

        t = self.now()
        dt = t - self.last_tick if self.last_tick is not None else 0.0
        self.last_tick = t
        if not self.sensors_fresh(t):
            self.get_logger().warn(
                "GPS or heading is stale; holding search decisions.",
                throttle_duration_sec=5.0,
            )
            return
        self.observe_coverage(min(dt, 1.0))

        if self.state == SearchState.SEARCHING:
            self.tick_searching(t)
        else:
            self.tick_approaching(t, dt)

    def tick_searching(self, t: float):
        # Drop passive candidates that have gone quiet
        self.candidates = [
            c
            for c in self.candidates
            if t - c.last_seen <= self.p["candidate_timeout_sec"]
        ]

        ready = [
            c
            for c in self.candidates
            if c.hits >= self.hits_required(c)
            and c.filter.spread() <= self.p["max_spread_to_approach"]
        ]
        if ready:
            self.target = max(ready, key=lambda c: c.confidence_sum)
            self.approach_started = t
            self.last_sent_estimate = None
            self.set_state(
                SearchState.APPROACHING,
                f"candidate has {self.target.hits} hits, "
                f"spread {self.target.filter.spread():.1f} m",
            )
            self.send_target_waypoint()
            return

        # Plan the next leg once this one is sailed, or if it is taking far too long
        if self.leg is None:
            self.plan_next_leg()
            return
        end_x, end_y = self.leg[1]
        boat_x, boat_y = self.position
        if (
            math.hypot(end_x - boat_x, end_y - boat_y)
            <= self.p["waypoint_reached_radius"]
        ):
            self.plan_next_leg()
        elif t - self.leg_started > self.leg_timeout:
            self.get_logger().info("Search leg is taking too long; replanning.")
            self.plan_next_leg()

    def hits_required(self, candidate: Candidate) -> int:
        """More evidence for every time the boat already investigated this place."""
        strikes = sum(
            zone.strikes
            for zone in self.rejected_zones
            if zone.contains(*candidate.estimate())
        )
        return self.p["min_hits_to_approach"] * (1 + strikes)

    def tick_approaching(self, t: float, dt: float):
        target = self.target
        tx, ty = target.estimate()
        boat_x, boat_y = self.position
        distance = math.hypot(tx - boat_x, ty - boat_y)
        since_seen = t - target.last_seen

        if distance <= self.p["buoy_reached_radius"] or (
            since_seen <= self.p["reacquire_after_sec"]
            and target.last_range <= self.p["buoy_reached_radius"]
        ):
            if target.close_hits < self.p["confirm_hits"]:
                # A real buoy stays visible until close in; this one vanished early
                self.reject_target(
                    t,
                    f"reached, but seen only {target.close_hits} times within "
                    f"{self.p['confirm_range']:.0f} m",
                )
                return
            self.set_state(
                SearchState.DONE, f"buoy reached ({distance:.1f} m from estimate)"
            )
            self.get_logger().info(f"Buoy found at {self.to_latlon(tx, ty)}.")
            return

        if t - self.approach_started > self.p["approach_timeout_sec"]:
            self.reject_target(t, "approach timed out")
            return

        camera_heading = self.heading - self.p["camera_yaw_offset_deg"]
        visible = point_visible(
            boat_x,
            boat_y,
            camera_heading,
            tx,
            ty,
            self.p["camera_hfov_deg"],
            self.p["min_visible_range"],
            self.p["max_visible_range"],
        )
        if visible and since_seen > self.p["tick_period"]:
            target.missed_visible_time += dt
            if target.missed_visible_time >= self.p["miss_timeout_sec"]:
                self.reject_target(
                    t,
                    f"not seen for {target.missed_visible_time:.1f} s while in view",
                )
                return

        if since_seen > self.p["reacquire_after_sec"] and not visible:
            if since_seen > self.p["reacquire_timeout_sec"]:
                # Not disproven, so no ignore period, but it takes more evidence to return
                self.mark_investigated(tx, ty, ignore_until=t)
                self.drop_target(f"lost for {since_seen:.0f} s")
                return
            self.set_state(SearchState.REACQUIRING, "candidate out of view")
        elif since_seen <= self.p["reacquire_after_sec"]:
            self.set_state(SearchState.APPROACHING, "candidate back in view")

        self.send_target_waypoint()

    def send_target_waypoint(self):
        """Send the target estimate, but only when it has moved enough to matter."""
        tx, ty = self.target.estimate()
        if self.last_sent_estimate is not None:
            lx, ly = self.last_sent_estimate
            if math.hypot(tx - lx, ty - ly) < self.p["waypoint_update_distance"]:
                return
        self.last_sent_estimate = (tx, ty)
        self.set_waypoints([self.to_latlon(tx, ty)])

    def reject_target(self, t: float, reason: str):
        """Mark the target a false positive, ignore its area a while, plan the next leg."""
        tx, ty = self.target.estimate()
        zone = self.mark_investigated(
            tx, ty, ignore_until=t + self.p["reject_duration_sec"]
        )
        self.candidates = [
            c
            for c in self.candidates
            if c is self.target or not zone.contains(*c.estimate())
        ]
        self.get_logger().info(f"Rejecting candidate: {reason}.")
        self.drop_target(reason)

    def mark_investigated(
        self, x: float, y: float, ignore_until: float
    ) -> RejectedZone:
        """Remember a place the boat checked; a second visit to it adds a strike."""
        for zone in self.rejected_zones:
            if zone.contains(x, y):
                zone.strikes += 1
                zone.ignore_until = max(zone.ignore_until, ignore_until)
                return zone
        zone = RejectedZone(x, y, self.p["reject_radius"], ignore_until)
        self.rejected_zones.append(zone)
        return zone

    def drop_target(self, reason: str):
        if self.target in self.candidates:
            self.candidates.remove(self.target)
        self.target = None
        self.set_state(SearchState.SEARCHING, reason)
        self.plan_next_leg()

    # --------------------------------------------------------------- coverage

    def reset_search(self):
        """Reset candidates, the coverage map and the leg plan."""
        self.candidates = []
        self.target = None
        self.rejected_zones = []
        self.coverage = CoverageMap(
            self.center[0],
            self.center[1],
            self.search_radius,
            self.p["coverage_cell_size"],
            self.p["prior_center_std"],
        )
        self.leg = None
        self.leg_started = None
        self.leg_timeout = None
        self.pattern_pass = 0
        self.last_sent_estimate = None
        self.approach_started = None
        self.last_tick = None

    def start_search(self):
        self.reset_search()
        self.set_state(SearchState.SEARCHING, "search started")
        self.plan_next_leg()

    def wind_from(self):
        """
        Absolute direction the wind comes from (math convention), or None. wind_angle is
        apparent wind "from", clockwise from the bow, so it is subtracted from the
        counter-clockwise math heading.
        """
        if self.wind_dir is None or self.heading is None:
            return None
        return (self.heading - self.wind_dir) % 360

    def pattern_direction(self) -> float:
        """Cross-wind legs when wind is known, else the configured direction."""
        wind_from = self.wind_from()
        if self.p["wind_aligned_pattern"] and wind_from is not None:
            return wind_from
        return self.p["search_direction"]

    def observe_coverage(self, dt: float):
        """Credit what the camera sees now. Runs in every state, so detours count too."""
        camera_heading = self.heading - self.p["camera_yaw_offset_deg"]
        self.coverage.observe(
            self.position[0],
            self.position[1],
            camera_heading,
            self.p["camera_hfov_deg"],
            self.p["min_visible_range"],
            self.p["reliable_detection_range"],
            self.p["max_visible_range"],
            self.p["detect_rate"],
            dt,
        )
        self.coverage_pub.publish(Float32(data=self.coverage.coverage()))

    def plan_leg(self):
        here = self.position if self.position is not None else self.center
        upwind = self.wind_from()  # sailing toward where the wind comes from
        return self.coverage.plan_leg(
            here[0],
            here[1],
            self.pattern_direction(),
            self.p["leg_spacing"],
            self.p["min_leg_length"],
            self.p["reliable_detection_range"],
            self.p["min_leg_gain"],
            0.5 if self.pattern_pass % 2 else 0.0,
            upwind,
            self.p["no_go_zone"],
        )

    def plan_next_leg(self):
        """
        Send the next stretch of the sweep that still needs sailing. When the whole area
        is covered for this pass, start another pass on lines between the earlier ones.
        """
        leg = self.plan_leg()
        if leg is None:
            self.pattern_pass += 1
            self.get_logger().info(
                f"Search area {100 * self.coverage.coverage():.0f}% covered; "
                f"starting pass {self.pattern_pass + 1} between the earlier legs."
            )
            self.coverage.start_pass()
            leg = self.plan_leg()
        if leg is None:
            self.get_logger().error(
                "No search leg could be planned; check the search radius."
            )
            return
        start, end, cost = leg
        self.leg = (start, end)
        self.leg_started = self.now()
        self.leg_timeout = max(
            60.0, self.p["leg_timeout_factor"] * cost / self.p["nominal_speed"]
        )
        self.set_waypoints([self.to_latlon(*start), self.to_latlon(*end)])

    # -------------------------------------------------------------- waypoints

    def set_waypoints(self, waypoints):
        """
        Replace the waypoint queue. Never blocks; if the service is busy or not ready,
        the latest request is kept and retried on the next tick.
        """
        if self.current_mode != "search":
            return
        self.pending_waypoints = waypoints
        self.retry_pending_waypoints()

    def retry_pending_waypoints(self):
        if self.pending_waypoints is None:
            return
        if self.waypoint_future is not None and not self.waypoint_future.done():
            return
        if not self.waypoint_client.service_is_ready():
            self.get_logger().warn(
                "Waypoint service not available; will retry.",
                throttle_duration_sec=5.0,
            )
            return

        if self.queue_version is None:
            # A set needs the current queue version; fetch it first.
            request = WaypointSrv.Request(command="get")
            self.sent_waypoints = None
        else:
            request = WaypointSrv.Request(
                command="set",
                expected_version=self.queue_version,
                waypoints=[
                    WaypointMsg(
                        id=f"search-{uuid.uuid4().hex}",
                        latitude=float(lat),
                        longitude=float(lon),
                    )
                    for lat, lon in self.pending_waypoints
                ],
            )
            self.sent_waypoints = self.pending_waypoints
            self.pending_waypoints = None
        self.waypoint_future = self.waypoint_client.call_async(request)
        self.waypoint_future.add_done_callback(
            lambda future, sent_version=request.expected_version: (
                self.waypoint_response_callback(future, request.command, sent_version)
            )
        )

    def waypoint_response_callback(self, future, command, sent_version):
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - log any service failure and move on
            self.get_logger().error(f"Waypoint service call failed: {error}")
            if command == "set" and self.pending_waypoints is None:
                self.pending_waypoints = self.sent_waypoints
            return

        self.queue_version = response.version
        if command != "set":
            return
        if response.success:
            self.get_logger().info("Waypoints set.")
        elif response.version != sent_version and self.pending_waypoints is None:
            # The queue changed (e.g. a waypoint was popped); resend with the new version.
            self.pending_waypoints = self.sent_waypoints
        else:
            self.get_logger().error(f"Failed to set waypoints: {response.error}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BuoySearch()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
